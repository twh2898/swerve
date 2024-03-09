#include "util/PID.hpp"

#include <exception>

namespace util {
    using std::runtime_error;

    PID::PID(double min, double max, double Kp, double Ki, double Kd)
        : min(min),
          max(max),
          useRange(true),
          Kp(Kp),
          Ki(Ki),
          Kd(Kd),
          lastError(0),
          integral(0) {}

    PID::PID(double Kp, double Ki, double Kd)
        : min(0),
          max(0),
          useRange(false),
          Kp(Kp),
          Ki(Ki),
          Kd(Kd),
          lastError(0),
          integral(0) {}

    void PID::reset() {
        lastError = 0;
        integral = 0;
    }

    double PID::calculate(double dt, double setPoint, double processValue) {
        if (dt <= 0.0)
            throw runtime_error("dt must be greater than 0");

        double error = setPoint - processValue;
        integral += error * dt;

        double P = Kp * error;
        double I = Ki * integral;
        double D = Kd * (error - lastError) / dt;

        lastError = error;
        double output = P + I + D;

        if (useRange) {
            if (output > max)
                output = max;
            else if (output < min)
                output = min;
        }

        return output;
    }

    json PID::getTelemetry() const {
        return json {
            {"integral", integral},
            {"lastError", lastError},
        };
    }
}
