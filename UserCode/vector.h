
#ifndef RM10_18_VECTOR_H
#define RM10_18_VECTOR_H

class Vect {
public:
    float x, y, z;

    Vect(float x, float y, float z);

    Vect operator+(const Vect &v) const;
    Vect operator*(const Vect &v) const;

    float norm() const;
    Vect normalized() const;
    void normalize();
};

#endif //RM10_18_VECTOR_H
