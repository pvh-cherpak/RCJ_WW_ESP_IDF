#ifndef _REAL_DIST_H_
#define _REAL_DIST_H_

const int DIST_CALIB_SECTORS = 8;
const int DIST_CALIB_MAX_POINTS = 40;
const int DIST_CALIB_MAX_DIST = 300;
const int DIST_CALIB_ROTATE_STEPS = 16;
const int DIST_CALIB_PLACES = 10;

void sort_points(int* xarr, int* yarr, int n);
int linear_interpolate(int x, int* xs, int* ys, int n);

class RealDist{
    int xs[DIST_CALIB_SECTORS][DIST_CALIB_MAX_POINTS];
    int ys[DIST_CALIB_SECTORS][DIST_CALIB_MAX_POINTS];
    int pcount[DIST_CALIB_SECTORS];

    int angle_to_sector(int angle);

    public:
        void init();
        void updatePoints(int* dist_cm, int* angles, int* pixel_dist);
        int convertDist(int pixels, int angle);
};

#endif