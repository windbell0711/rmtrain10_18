
#include "vector.h"
#include <cmath>

Vect::Vect(float x, float y, float z) : x(x), y(y), z(z) {}

Vect Vect::operator+(const Vect &v) const {
    return Vect(x + v.x, y + v.y, z + v.z);
}

Vect Vect::operator*(const Vect &v) const {
    return Vect(y * v.z + z * v.y,
                z * v.x + x * v.z,
                x * v.y + y * v.x);
}

float Vect::norm() const {
    return std::sqrtf(x*x + y*y + z*z);
}

Vect Vect::normalized() const {
    float n = norm();
    if (n < 1e-8f) return Vect(0.0f, 0.0f, 0.0f);
    float inv = 1.0f / n;
    return Vect(x*inv, y*inv, z*inv);
}

void Vect::normalize() { *this = normalized(); }
