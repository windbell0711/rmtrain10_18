
#ifndef RM10_18_QUAT_H
#define RM10_18_QUAT_H

class Quat {
public:
    float w, x, y, z;

    Quat();
    Quat(float w_, float x_, float y_, float z_);

    Quat operator+(const Quat& b) const;
    Quat operator*(float b) const;
    Quat operator*(const Quat& b) const;

    float norm() const;
    Quat  normalized() const;
    void  normalize();

    bool toEuler(float& roll, float& pitch, float& yaw) const;
    bool toEulerDeg(float& roll, float& pitch, float& yaw) const;

    static Quat fromAxisAngle(float ux, float uy, float uz, float angle);
    static Quat fromEuler(float roll, float pitch, float yaw);
};

#endif //RM10_18_QUAT_H
