
#pragma once

typedef struct pid_gains_t {
    double kp; // proportional
    double ki; // integral
    double kd; // deriavtive
    double kf; // feedforward
    double min_out;
    double max_out;
} PIDGains;