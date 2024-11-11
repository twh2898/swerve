#pragma once

#include <stdexcept>

namespace util {
    class Ramp {
        double startValue = 0.0;
        double targetValue = 0.0;
        double startTime = 0.0;
        double slope;
        mutable bool atTarget;

    public:
        Ramp()
            : slope(0.0), atTarget(false) {}

        Ramp(double slope, double time)
            : slope(0.0), atTarget(false) {
            setSlope(slope, time);
        }

        void start(double startValue, double targetValue, double startTime) {
            startValue = startValue;
            targetValue = targetValue;
            startTime = startTime;
            atTarget = false;
        }

        double getStart() const {
            return startValue;
        }

        double getStartTime() const {
            return startTime;
        }

        void setSlope(double slope, double time) {
            if (slope < 0.0) {
                slope = 0.0;
            }
            startValue = getValue(time);
            startTime = time;
            atTarget = false;
            this->slope = slope;
        }

        double getSlope() const {
            return slope;
        }

        void setTarget(double target, double time) {
            startValue = getValue(time);
            startTime = time;
            atTarget = false;
            targetValue = target;
        }

        double getTarget() const {
            return targetValue;
        }

        double getValue(double time) const {
            if (atTarget) {
                return targetValue;
            }

            if (time < startTime) {
                throw std::invalid_argument("time is before start");
            }

            bool isNeg = targetValue < startValue;

            double dt = time - startTime;
            double delta = slope * dt;
            if (targetValue < startValue) {
                delta = -delta;
            }

            double value = startValue + delta;

            if (targetValue >= startValue) {
                if (value > targetValue) {
                    atTarget = true;
                    return targetValue;
                }
            }
            else {
                if (value < targetValue) {
                    atTarget = true;
                    return targetValue;
                }
            }

            return value;
        }
    };
}
