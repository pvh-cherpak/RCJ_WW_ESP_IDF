#ifndef _LOGICS_H_
#define _LOGICS_H_

#include "global.h"

int millis();

int goodAngle(int angle);

int constrain(int val, int minim, int maxim);

void make_pause(int ms);

void projectSpeedOnLineXY(int speedX, int speedY, float lineX, float lineY, float &resSpeedX, float &resSpeedY);

void projectSpeedOnLine(float speed, float moveAngle, float lineX, float lineY, float &resSpeedX, float &resSpeedY);

void calibrateDistOffset(int color);

bool getRayIntersection(float x1, float y1, float ang1, float x2, float y2, float ang2, float &out_x, float &out_y);

float pixel_dist_to_real(float dist);

int getGlobalPosition_2gates(float &x, float &y, int color);

int getGlobalPosition_dist(float &x, float &y, int color);

void killerFeature(int color);

bool isBall();

void petrovich_iter(int color, int offset, bool useLine);

void goOverObstacleOmni(float generalSpeed, float generalAngle, float rot_angle, float critDist, bool rotate);

void playGoalkeeperCamera(int color);

void playForwardGoyda(int color);

void goalRotate(int color);

void goalDriveBack(int color);

void goalPush(int color);

void playForwardDribble2(int color);

void MPU_zakrut(int color);

void vyravnivanije(int color);

void penaltyDribbler(int color);

void penaltyKicker(int color);

#endif
