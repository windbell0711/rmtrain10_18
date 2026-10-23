
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
    if (n < 1e-12f) return Quat(1.0f, 0.0f, 0.0f, 0.0f);
    float inv = 1.0f / n;
    return Quat(w*inv, x*inv, y*inv, z*inv);
}

void Quat::normalize() { *this = normalized(); }

bool Quat::toEuler(float& roll, float& pitch, float& yaw) const {
    if (std::isnan(w) || std::isnan(x) || std::isnan(y) || std::isnan(z)) {
        // 保持上一帧的值
        return false;
    }

    roll = std::atan2(2.0f*(w*x + y*z), 1.0f - 2.0f*(x*x + y*y));

    float sinp = 2.0f*(w*y - z*x);
    if (sinp >  1.0f) sinp =  1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    pitch = std::asin(sinp);

    yaw = std::atan2(2.0f*(w*z + x*y), 1.0f - 2.0f*(y*y + z*z));
    return true;
}

bool Quat::toEulerDeg(float& roll, float& pitch, float& yaw) const {
    constexpr float R2D = 57.29577951308232f;
    float ar, ap, ay;
    if (!toEuler(ar, ap, ay)) {
        return false;
    }
    roll  = ar * R2D;
    pitch = ap * R2D;
    yaw   = ay * R2D;
    return true;
}
