#ifndef RM2610_11_PID_H
#define RM2610_11_PID_H

inline float max(const float a, const float b) {
    return a > b ? a : b;
}

inline float min(const float a, const float b) {
    return a < b ? a : b;
}

inline float limit_between(float v, float l, float h) {
    return max(min(v, h), l);
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
