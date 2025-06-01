#include "RealDist.h"
#include <cmath>

RealDist real_dist;

void sort_points(int* xarr, int* yarr, int n){
    for (int i = 1; i < n; ++i){
        for (int j = i; j > 0; --j){
            if (xarr[j] < xarr[j - 1]){
                std::swap(xarr[j], xarr[j - 1]);
                std::swap(yarr[j], yarr[j - 1]);
            }
            else
                break;
        }
    }
}

int linear_interpolate(int x, int* xs, int* ys, int n){
    if (x < xs[0])
        return 0;
    if (x > xs[n - 1])
        return n;

    int l = 0, r = n - 2;
    while (l < r){
        int val = (l + r) / 2;

        if (x < xs[val])
            r = val - 1;
        else if (x > xs[val + 1])
            l = val + 1;
        else{
            l = r = val;
            break;
        }
    }

    int y;
    if (xs[r + 1] == xs[r])
        y = (ys[r + 1] + ys[r]) / 2;
    else
        y = ys[r] + (x - xs[r]) * (ys[r + 1] - ys[r]) / (xs[r + 1] - xs[r]);

    return y;
}

int RealDist::angle_to_sector(int angle){
    if (angle < 0)
        angle += 360;
    return angle / (360 / DIST_CALIB_SECTORS);
}

void RealDist::init(){
    // читаем массивы xs, ys и pcount из NVS

    // пока заглушка для безопасности:
    for (int i = 0; i < DIST_CALIB_SECTORS; ++i)
        pcount[i] = 0;
    for (int i = 0; i < DIST_CALIB_SECTORS; ++i)
        for (int j = 0; j < DIST_CALIB_MAX_POINTS; ++j)
            xs[i][j] = ys[i][j] = 0;
}

// int dist_cm[DIST_CALIB_PLACES] - реальное расстояние для каждого места на поле
// int angles[DIST_CALIB_PLACES][DIST_CALIB_ROTATE_STEPS] - угол каждого замера
// int pixel_dist[DIST_CALIB_PLACES][DIST_CALIB_ROTATE_STEPS] - расстояние каждого замера
void RealDist::updatePoints(int* dist_cm, int* angles, int* pixel_dist)
{
    for (int i = 0; i < DIST_CALIB_SECTORS; ++i)
        pcount[i] = 0;
    
    for (int i = 0; i < DIST_CALIB_SECTORS; ++i)
        for (int j = 0; j < DIST_CALIB_MAX_POINTS; ++j)
            xs[i][j] = ys[i][j] = 0;

    for (int i = 0; i < DIST_CALIB_PLACES; ++i){
        for (int j = 0; j < DIST_CALIB_ROTATE_STEPS; ++i){

            int sect = angle_to_sector(angles[i * DIST_CALIB_ROTATE_STEPS + j]);
            xs[sect][pcount[sect]] = pixel_dist[i * DIST_CALIB_ROTATE_STEPS + j];
            ys[sect][pcount[sect]] = dist_cm[i];

            if (pcount[sect] + 1 < DIST_CALIB_MAX_POINTS)
                ++pcount[sect];
        }
    }

    for (int i = 0; i < DIST_CALIB_SECTORS; ++i)
        sort_points(xs[i], ys[i], pcount[i]);
    
    // сохраняем массивы xs, ys и pcount в NVS
}

int RealDist::convertDist(int pixels, int angle){
    int sect = angle_to_sector(angle);
    int dist = linear_interpolate(pixels, xs[sect], ys[sect], pcount[sect]);
    return dist;
}
