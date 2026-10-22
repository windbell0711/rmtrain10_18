
#include "pid.hpp"


void PID::reset() {
    // 清理历史状态
    last_err = 0;
    err_sum = 0;
    last_d_filter = 0;
}

float PID::calc(float ref, float fdb) {
    const float err = ref - fdb;

    outp = kp * err;

    float derr = err - last_err;
    last_d_filter = (1-d_filter_k)*(last_d_filter) + (d_filter_k)*(derr);
    outd = kd * last_d_filter;

    err_sum += err;
    outi = limit_between(ki * err_sum, -i_max, i_max);

    last_err = err;
    out = limit_between(outp + outd + outi, -out_max, out_max);
    return out;
}
