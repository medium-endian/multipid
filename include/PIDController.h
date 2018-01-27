#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

/* TODO implement explicit cascaded pid controller? */

#include <stdint.h>

#include "PIDParams.h"
#include "util.h"

class PIDController {
    private:
        bool enabled = true;

        float p_gain;
        float i_gain;
        float d_gain;

        float integral;
        float integral_limit;

        float derivative;

        /* For derivative-on-error */
        float last_error;

        /* For derivative-on-setpoint */
        float last_setpoint;

        float output_limit;

        uint64_t last_time;

    public:
        PIDController(PIDParams params);

        /* En/Disable Passthrough of setpoint */
        void set_enabled(bool enable);

        float compute(const uint64_t now, const float measured, const float setpoint);

        PIDController* set_p(const float _p_gain);
        PIDController* set_i(const float _i_gain);
        PIDController* set_d(const float _d_gain);

        PIDController* integral_reset();
};

#endif // PID_CONTROLLER_H

