#ifndef PID_HPP
#define PID_HPP

namespace control
{
    class PID
    {
    public:
        double kp, ki, kd;
        double prev_error;
        double integral;

        PID(double p, double i, double d) : kp(p), ki(i), kd(d), prev_error(0), integral(0) {}

        double compute(double setpoint, double measured, double dt)
        {
            double error = setpoint - measured;
            integral += error * dt;
            double derivative = (error - prev_error) / dt;
            prev_error = error;
            return kp * error + ki * integral + kd * derivative;
        }

        void reset()
        {
            prev_error = 0;
            integral = 0;
        }
    };

#endif
}