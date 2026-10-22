#ifndef RM2610_11_PID_H
#define RM2610_11_PID_H

inline float abs(float v) {
    return (v > 0) ? v : -v;
}

inline float max(const float a, const float b) {
    return a > b ? a : b;
}

inline float min(const float a, const float b) {
    return a < b ? a : b;
}

inline float limit_between(float v, float l, float h) {
    return max(min(v, h), l);
}

inline float goto_nearest(float v, float a, float b) {
    return (abs(v-a) < abs(v-b)) ? a : b;
}

inline float minus_if_near(float v, float m, float tol = 1.0f) {
    return (abs(v-m) < tol) ? v-m : v;
}


class PID {

public:
    PID(float kp, float ki, float kd,
        float i_max, float out_max, float d_filter_k) :
        kp(kp), ki(ki), kd(kd), i_max(i_max), out_max(out_max), d_filter_k(d_filter_k) {};

    void reset();

    float calc(float ref, float fdb);

private:
    float kp;
    float ki;
    float kd;

    float i_max;
    float out_max;
    float d_filter_k;

    float last_err = 0;
    float err_sum = 0;
    float last_d_filter = 0;

    float outp = 0;
    float outd = 0;
    float outi = 0;
    float out = 0;
};


#endif //RM2610_11_PID_H
