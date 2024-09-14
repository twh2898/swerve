#pragma once

#include <cmath>
#include <ostream>

namespace util {
    using std::sin;
    using std::cos;
    using std::atan2;
    using std::sqrt;
    using std::ostream;

    struct XY {
        double x;
        double y;

        XY(double x = 0, double y = 0) : x(x), y(y) {}

        XY(const XY & other) : x(other.x), y(other.y) {}

        XY(XY && other) : x(other.x), y(other.y) {}

        XY & operator=(const XY & other) {
            x = other.x;
            y = other.y;
            return *this;
        }

        XY & operator=(XY && other) {
            x = other.x;
            y = other.y;
            return *this;
        }

        // + Addition XY

        XY operator+(const XY & other) const {
            return XY(x + other.x, y + other.y);
        }

        XY & operator+=(const XY & other) {
            x += other.x;
            y += other.y;
            return *this;
        }

        // + Addition Double

        XY operator+(const double & other) const {
            return XY(x + other, y + other);
        }

        XY & operator+=(const double & other) {
            x += other;
            y += other;
            return *this;
        }

        // - Subtraction XY

        XY operator-(const XY & other) const {
            return XY(x + other.x, y + other.y);
        }

        XY & operator-=(const XY & other) {
            x += other.x;
            y += other.y;
            return *this;
        }

        // - Subtraction Double

        XY operator-(const double & other) const {
            return XY(x + other, y + other);
        }

        XY & operator-=(const double & other) {
            x += other;
            y += other;
            return *this;
        }

        // * Multiply XY

        XY operator*(const XY & other) const {
            return XY(x * other.x, y * other.y);
        }

        XY & operator*=(const XY & other) {
            x *= other.x;
            y *= other.y;
            return *this;
        }

        // * Multiply Double

        XY operator*(const double & other) const {
            return XY(x * other, y * other);
        }

        XY & operator*=(const double & other) {
            x *= other;
            y *= other;
            return *this;
        }

        // / Divide XY

        XY operator/(const XY & other) const {
            return XY(x / other.x, y / other.y);
        }

        XY & operator/=(const XY & other) {
            x /= other.x;
            y /= other.y;
            return *this;
        }

        // / Divide Double

        XY operator/(const double & other) const {
            return XY(x / other, y / other);
        }

        XY & operator/=(const double & other) {
            x /= other;
            y /= other;
            return *this;
        }

        // Additional methods

        void normalize() {
            auto len = length();
            x /= len;
            y /= len;
        }

        XY normal() const {
            return *this / length();
        }

        double length() const {
            return sqrt(x * x + y * y);
        }

        double dist(const XY & other) const {
            return (other - *this).length();
        }

        double angle() const {
            return atan2(y, x);
        }

        double angleTo(const XY & other) const {
            return (other - *this).angle();
        }

        void rotate(double rad) {
            double c = cos(rad);
            double s = sin(rad);
            double newY = x * c - y * s;
            double newX = x * s + y * c;
            x = newX;
            y = newY;
        }

        double dot(const XY & other) const {
            return x * other.x + y * other.y;
        }

        double cross(const XY & other) const {
            return (x * other.y) - (y * other.x);
        }

        friend ostream & operator<<(ostream & os, const XY & xy) {
            return os << "XY(x=" << xy.x << ", y=" << xy.y << ")";
        }
    };
}
