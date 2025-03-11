#ifndef _LOGICS_H_
#define _LOGICS_H_

#include "global.h"

const double DEG_TO_RAD = acos(-1) / 180;
const double RAD_TO_DEG = 180 / acos(-1);

int millis();

int goodAngle(int angle);

int constrain(int val, int minim, int maxim);

void make_pause(int ms);

void projectSpeedOnLineXY(int speedX, int speedY, float lineX, float lineY, float &resSpeedX, float &resSpeedY);

void projectSpeedOnLine(float speed, float moveAngle, float lineX, float lineY, float &resSpeedX, float &resSpeedY);

void killerFeature(int color);

void playGoalkeeperCamera(int color);

#endif
