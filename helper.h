#ifndef HELPER_H
#define HELPER_H

#include <random>

/**
 * Helper namespace, provides a collection of useful maths for the
 * raytracer.
 */
namespace helper {
    double PI = 3.14159256;
    inline double random_double() 
    {
        static std::uniform_real_distribution<double> distribution(0.0, 1.0);
        static std::mt19937 generator;
        return distribution(generator);
    }
    inline double random_double(double min, double max) { return min + (max-min)*random_double();}

    inline int random_int(int min, int max) { return static_cast<int>(random_double(min, max+1));}

    inline double radians(const double degrees){return (degrees*PI)/180;}

    inline double degrees(const double radians){return (radians * 180)/PI;}

    inline double clamp(double x, double min, double max)
    {
        if (x < min) return min;
        if (x > max) return max;
        return x;
    }
}

#endif