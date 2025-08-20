/* =============================================================
 * Project:   treadmill_sideshift
 * File:      Pid.h
 * Author:    Martin Kriz
 * Company:   Ullmanna s.r.o.
 * Created:   2025-08-20
 * -------------------------------------------------------------
 * Description:
 *   This header defines the Pid class, a classical PID controller
 *   with integral windup protection. It calculates control output
 *   based on proportional, integral, and derivative terms.
 *
 * Notes:
 * ============================================================= */

#ifndef PID_H
#define PID_H

class Pid {
public:
    Pid(float kp, float ki, float kd, float max_output = 100.0f)
        : kp_(kp), ki_(ki), kd_(kd), prev_error_(0), integral_(0),
          max_output_(max_output) {}

    float step(float error, float dt) {
        integral_ += error * dt;

        if (integral_ * ki_ > max_output_) {
            integral_ = max_output_ / ki_;
        } else if (integral_ * ki_ < -max_output_) {
            integral_ = -max_output_ / ki_;
        }

        float derivative = (error - prev_error_) / dt;
        float output = kp_ * error + ki_ * integral_ + kd_ * derivative;

        if (output > max_output_) {
            output = max_output_;
        } else if (output < -max_output_) {
            output = -max_output_;
        }

        prev_error_ = error;
        return output;
    }

    void reset() {
        prev_error_ = 0;
        integral_ = 0;
    }

    void setMaxOutput(float max_output) {
        max_output_ = max_output;
    }

private:
    float kp_;
    float ki_;
    float kd_;
    float prev_error_;
    float integral_;
    float max_output_;
};

#endif
