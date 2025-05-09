#ifndef _LOGICS_H_
#define _LOGICS_H_

#include "global.h"

int millis();

int goodAngle(int angle);

int constrain(int val, int minim, int maxim);

void make_pause(int ms);

void projectSpeedOnLineXY(int speedX, int speedY, float lineX, float lineY, float &resSpeedX, float &resSpeedY);

void projectSpeedOnLine(float speed, float moveAngle, float lineX, float lineY, float &resSpeedX, float &resSpeedY);

bool getRayIntersection(float x1, float y1, float a1, float x2, float y2, float a2, float& out_x, float& out_y);

int getGlobalPosition_2gates(float &x, float &y, int color);

void killerFeature(int color);

bool isBall();

void playGoalkeeperCamera(int color);

void playForwardGoyda(int color);

void playForwardDribble2(int color);

#endif
