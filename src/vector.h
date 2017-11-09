#pragma

#include <cmath>

class Vector {
    public:
        double x;
        double y;

        Vector(double x=0, double y=0) {
            this-> x = x;
            this-> y = y;
        };

        Vector(const Vector& other) {
            x = other.x;
            y = other.y;
        };

        double magnitude() {
            return sqrt(pow(x, 2) + pow(y, 2));
        };

        Vector normalize() {
            auto mag = magnitude();
            if (mag != 0) {
                return Vector(x, y)/mag;
            } else {
                return Vector(*this);
            }
        };

        double dot(const Vector& other) {
            return x*other.x + y*other.y;
        }

        Vector operator+(const Vector& other) {
            return Vector(
                x + other.x,
                y + other.y
            );
        };

        Vector operator-() {
            return Vector(
                -x,
                -y
            );
        }

        Vector operator-(const Vector& other) {
            return Vector(
                x - other.x,
                y - other.y
            );
        };

        Vector operator*(const double& value) {
            return Vector(
                x*value,
                y*value
            );
        }

        Vector operator/(const double& value) {
            return Vector(
                x/value,
                y/value
            );
        }

        Vector operator+=(const Vector& other) {
            x += other.x;
            y += other.y;
        }

};