#include <iostream>
#include <vector>

class PID{
    public:
        PID(double kp = 0.0f, double ki = 0.0f, double kd = 0.0f, double setpoint = 0.0f):kp_(kp), ki_(ki), kd_(kd), setpoint_(setpoint), prev_error_(0.0), integral_(0.0){};
        double kp_, ki_, kd_;
        double prev_error_, integral_;
        double setpoint_;

        double compute(double current_point, double dt){
            // Calculate Error
            double error = setpoint_ - current_point;

            // Proportional Term
            double p_out = kp_ * error;

            // Integral Term
            integral_ += error * dt;
            double i_out = ki_ * integral_;

            // Derivative Term
            double derivative = (error - prev_error_) / dt;
            double d_out = kd_ * derivative;

            // Compute total output
            double output = p_out + i_out + d_out;

            // Update previous error
            prev_error_ = error;

            return output;
        }

    
};