#include "RealDist.h"
#include <cmath>
#include "esp_log.h"

RealDist real_dist;

void sort_points(int *xarr, int *yarr, int n)
{
    for (int i = 1; i < n; ++i)
    {
        for (int j = i; j > 0; --j)
        {
            if (xarr[j] < xarr[j - 1])
            {
                std::swap(xarr[j], xarr[j - 1]);
                std::swap(yarr[j], yarr[j - 1]);
            }
            else
                break;
        }
    }
}

int linear_interpolate(int x, int *xs, int *ys, int n)
{
    if (x < xs[0])
        return 0;
    if (x > xs[n - 1])
        return n;
        
    int l = 0, r = n - 2;

    for (int i = 0; i < n - 2; ++i){
        if (x >= xs[i]){
            l = r = i;
        }
    }

    // while (l < r)
    // {
    //     int val = (l + r) / 2;

    //     if (x < xs[val])
    //         r = val - 1;
    //     else if (x > xs[val + 1])
    //         l = val + 1;
    //     else
    //     {
    //         l = r = val;
    //         break;
    //     }
    // }

    int y;
    if (xs[r + 1] == xs[r])
        y = (ys[r + 1] + ys[r]) / 2;
    else
        y = ys[r] + (x - xs[r]) * (ys[r + 1] - ys[r]) / (xs[r + 1] - xs[r]);

    return y;
}

int RealDist::angle_to_sector(int angle)
{
    if (angle < 0)
        angle += 360;
    return angle / (360 / DIST_CALIB_SECTORS);
}

void RealDist::save2NVS()
{
    nvs_handle_t nvs_handle;
    ESP_ERROR_CHECK(nvs_open(REAL_DIST_NVS, NVS_READWRITE, &nvs_handle));

    ESP_ERROR_CHECK(nvs_set_blob(nvs_handle, "xs", xs, sizeof(xs)));
    ESP_ERROR_CHECK(nvs_set_blob(nvs_handle, "ys", ys, sizeof(ys)));
    ESP_ERROR_CHECK(nvs_set_blob(nvs_handle, "pcount", pcount, sizeof(pcount)));

    nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
}

void RealDist::restoreNVS()
{
    for (int i = 0; i < DIST_CALIB_SECTORS; ++i)
        pcount[i] = 0;
    for (int i = 0; i < DIST_CALIB_SECTORS; ++i)
        for (int j = 0; j < DIST_CALIB_MAX_POINTS; ++j)
            xs[i][j] = ys[i][j] = 0;
    save2NVS();
}

void RealDist::init()
{

    // читаем массивы xs, ys и pcount из NVS

    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(REAL_DIST_NVS, NVS_READONLY, &nvs_handle);

    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        restoreNVS();
        esp_restart();
    }
    else
        ESP_ERROR_CHECK(err);

    esp_err_t err_mass[3];
    size_t xs_size = sizeof(xs);
    size_t ys_size = sizeof(ys);
    size_t pcount_size = sizeof(pcount);
    err_mass[0] = nvs_get_blob(nvs_handle, "xs", xs, &xs_size);
    err_mass[1] = nvs_get_blob(nvs_handle, "ys", ys, &ys_size);
    err_mass[2] = nvs_get_blob(nvs_handle, "pcount", pcount, &pcount_size);
    nvs_close(nvs_handle);
    for (int i = 0; i < 3; i++)
        if (err_mass[i] == ESP_ERR_NVS_NOT_FOUND)
        {
            restoreNVS();
            esp_restart();
        }
        else
            ESP_ERROR_CHECK(err);

    ESP_LOGI("RD", "points:");
    for (int i = 0; i < DIST_CALIB_SECTORS; ++i){
        ESP_LOGI("RD", "SECTOR %d (%d points)", i, pcount[i]);
        for (int j = 0; j < pcount[i]; ++j){
            ESP_LOGI("RD", "%d;%d", xs[i][j], ys[i][j]);
        }
    }
}

// int dist_cm[DIST_CALIB_PLACES] - реальное расстояние для каждого места на поле
// int angles[DIST_CALIB_PLACES][DIST_CALIB_ROTATE_STEPS] - угол каждого замера
// int pixel_dist[DIST_CALIB_PLACES][DIST_CALIB_ROTATE_STEPS] - расстояние каждого замера
void RealDist::updatePoints(int *dist_cm, int *angles, int *pixel_dist)
{
    for (int i = 0; i < DIST_CALIB_SECTORS; ++i)
        pcount[i] = 0;

    for (int i = 0; i < DIST_CALIB_SECTORS; ++i)
        for (int j = 0; j < DIST_CALIB_MAX_POINTS; ++j)
            xs[i][j] = ys[i][j] = 0;

    ESP_LOGI("RD", "clear storage");

    for (int i = 0; i < DIST_CALIB_PLACES; ++i)
    {
        for (int j = 0; j < DIST_CALIB_ROTATE_STEPS; ++j)
        {
            int sect = angle_to_sector(angles[i * DIST_CALIB_ROTATE_STEPS + j]);
            
            ESP_LOGI("RD", "%d;%d;  %d;%d => %d (%d)", i, j, angles[i * DIST_CALIB_ROTATE_STEPS + j], pixel_dist[i * DIST_CALIB_ROTATE_STEPS + j], sect, pcount[sect]);

            xs[sect][pcount[sect]] = pixel_dist[i * DIST_CALIB_ROTATE_STEPS + j];
            ys[sect][pcount[sect]] = dist_cm[i];

            if (pcount[sect] + 1 < DIST_CALIB_MAX_POINTS)
                ++pcount[sect];
        }
    }

    for (int i = 0; i < DIST_CALIB_SECTORS; ++i)
        sort_points(xs[i], ys[i], pcount[i]);

    ESP_LOGI("RD", "points:");
    for (int i = 0; i < DIST_CALIB_SECTORS; ++i){
        ESP_LOGI("RD", "SECTOR %d (%d points)", i, pcount[i]);
        for (int j = 0; j < pcount[i]; ++j){
            ESP_LOGI("RD", "%d;%d", xs[i][j], ys[i][j]);
        }
    }

    // сохраняем массивы xs, ys и pcount в NVS
    save2NVS();
}

int RealDist::convertDist(int pixels, int angle)
{
    int sect = angle_to_sector(angle);
    int dist = 0; //linear_interpolate(pixels, xs[sect], ys[sect], pcount[sect]);
    return dist;
}
