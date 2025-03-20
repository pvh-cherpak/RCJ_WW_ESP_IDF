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

int getGlobalPosition_2gates(double &x, double &y, int color);

void killerFeature(int color);

bool isBall();

void playGoalkeeperCamera(int color);

void playForwardGoyda(int color);

void playForwardDribble2(int color);

#endif
