
// partly written by deepseek chatbot

#include "quat.h"
#include <cmath>

Quat::Quat() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {}

Quat::Quat(float w_, float x_, float y_, float z_)
    : w(w_), x(x_), y(y_), z(z_) {}

Quat Quat::operator+(const Quat& b) const {
    return Quat(w + b.w, x + b.x, y + b.y, z + b.z);
}

Quat Quat::operator*(float s) const {
    return Quat(w*s, x*s, y*s, z*s);
}

Quat Quat::operator*(const Quat& b) const {
    return Quat(
        w*b.w - x*b.x - y*b.y - z*b.z,
        w*b.x + x*b.w + y*b.z - z*b.y,
        w*b.y - x*b.z + y*b.w + z*b.x,
        w*b.z + x*b.y - y*b.x + z*b.w
    );
}

float Quat::norm() const {
    return std::sqrt(w*w + x*x + y*y + z*z);
}

Quat Quat::normalized() const {
    float n = norm();
    if (n < 1e-8f) return Quat(1.0f, 0.0f, 0.0f, 0.0f);
    float inv = 1.0f / n;
    return Quat(w*inv, x*inv, y*inv, z*inv);
}

void Quat::normalize() { *this = normalized(); }

void Quat::toEuler(float& roll, float& pitch, float& yaw) const {
    roll = std::atan2(2.0f*(w*x + y*z), 1.0f - 2.0f*(x*x + y*y));

    float sinp = 2.0f*(w*y - z*x);
    if (sinp >  1.0f) sinp =  1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    pitch = std::asin(sinp);

    yaw = std::atan2(2.0f*(w*z + x*y), 1.0f - 2.0f*(y*y + z*z));
}

void Quat::toEulerDeg(float& roll, float& pitch, float& yaw) const {
    const float R2D = 57.29577951308232f;
    toEuler(roll, pitch, yaw);
    roll  *= R2D;
    pitch *= R2D;
    yaw   *= R2D;
}

Quat Quat::fromAxisAngle(float ux, float uy, float uz, float angle) {
    float half = 0.5f * angle;
    float s = std::sin(half);
    return Quat(std::cos(half), ux*s, uy*s, uz*s);
}

Quat Quat::fromEuler(float roll, float pitch, float yaw) {
    float cr = std::cos(0.5f*roll),  sr = std::sin(0.5f*roll);
    float cp = std::cos(0.5f*pitch), sp = std::sin(0.5f*pitch);
    float cy = std::cos(0.5f*yaw),   sy = std::sin(0.5f*yaw);
    Quat qz(cy, 0, 0, sy);
    Quat qy(cp, 0, sp, 0);
    Quat qx(cr, sr, 0, 0);
    return qz * qy * qx;
}
