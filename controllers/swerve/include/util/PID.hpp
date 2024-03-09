#pragma once

// https://gist.github.com/bradley219/5373998

#include "Telemetry.hpp"

namespace util {
    class PID : public TelemetrySender {
        double max;
        double min;
        bool useRange;
        double Kp;
        double Ki;
        double Kd;
        double lastError;
        double integral;

    public:
        /**
         * @brief PID loop with bounds check.
         *
         * @param min minimum value of manipulated variable
         * @param max maximum value of manipulated variable
         * @param Kp proportional gain
         * @param Ki Integral gain
         * @param Kd derivative gain
         */
        PID(double min, double max, double Kp, double Ki, double Kd);

        /**
         * @brief PID loop with no bounds check.
         *
         * @param Kp proportional gain
         * @param Ki Integral gain
         * @param Kd derivative gain
         */
        PID(double Kp, double Ki, double Kd);

        /**
         * Reset the previousError and integral to 0.
         */
        void reset();

        /**
         * @brief Returns the manipulated variable given a setPoint and current
         * process value.
         *
         * @param dt loop interval time
         * @param setPoint target value
         * @param processValue current value
         *
         * @throw std::runtime_error if dt <= 0.0
         */
        double calculate(double dt, double setPoint, double processValue);

        json getTelemetry() const override;
    };
}
