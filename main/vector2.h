#ifndef _VECTOR2_H_
#define _VECTOR2_H_

#include <cmath>
#include "global.h"

class Vector2 {
public:
    float x = 0, y = 0;

    Vector2();
    Vector2(float _x, float _y);
    Vector2(float angle);

    Vector2 operator+(Vector2 other);
    Vector2 operator-(Vector2 other);
    Vector2 operator*(float k);
    friend Vector2 operator*(float k, Vector2 other);
    Vector2 operator/(float k);
    float operator*(Vector2 other);
    float operator^(Vector2 other);

    float length();
    float angle();
};

#endif