
#ifndef RM10_18_QUAT_H
#define RM10_18_QUAT_H

class Quat {
public:
    float w, x, y, z;

    Quat();
    Quat(float w_, float x_, float y_, float z_);

    static Quat identity();

    Quat operator+(const Quat& b) const;
    Quat operator*(float b) const;
    Quat operator*(const Quat& b) const;

    float norm() const;
    Quat  normalized() const;
    void  normalize();
    Quat  conjugate() const;
    Quat  inverse() const;

    void toEuler(float& roll, float& pitch, float& yaw) const;
    void toEulerDeg(float& roll, float& pitch, float& yaw) const;

    static Quat fromAxisAngle(float ux, float uy, float uz, float angle);
    static Quat fromEuler(float roll, float pitch, float yaw);

    void rotateVector(float vx, float vy, float vz,
                      float& ox, float& oy, float& oz) const;
};

#endif //RM10_18_QUAT_H
