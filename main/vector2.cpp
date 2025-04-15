#include "vector2.h"

Vector2::Vector2() { }

Vector2::Vector2(float _x, float _y) {
    x = _x; y = _y;
}

Vector2::Vector2(float angle) {
    x = sin(angle * DEG_TO_RAD);
    y = cos(angle * DEG_TO_RAD);
}

Vector2 Vector2::operator+(Vector2 other) {
    return Vector2(x + other.x, y + other.y);
}

Vector2 Vector2::operator-(Vector2 other) {
    return Vector2(x - other.x, y - other.y);
}

Vector2 Vector2::operator*(float k) {
    return Vector2(x * k, y * k);
}

Vector2 operator*(float k, Vector2 other) {
    return Vector2(other.x * k, other.y * k);
}

Vector2 Vector2::operator/(float k) {
    return Vector2(x / k, y / k);
}

float Vector2::operator*(Vector2 other) {
    return x * other.x + y * other.y;
}

float Vector2::operator^(Vector2 other) {
    return x * other.y - y * other.x;
}

float Vector2::length() {
    return sqrt(x * x + y * y);
}

float Vector2::angle() {
    return atan2(x, y);
}