#ifndef VEC3_H
#define VEC3_H

#include <cmath>
#include <ostream>

#include "helper.h"

class Vec3d {
    public:
        double e[3];
    public:
        Vec3d() : e{0,0,0} {}
        Vec3d(double e0, double e1, double e2) : e{e0, e1, e2} {}

        double x() const {return e[0];}
        double y() const {return e[1];}
        double z() const {return e[2];}

        inline static Vec3d random()
        {
            return Vec3d(helper::random_double(), helper::random_double(), helper::random_double());
        }

        inline static Vec3d random(double min, double max)
        {
            return Vec3d(helper::random_double(min,max), helper::random_double(min,max), helper::random_double(min,max));
        }

        bool near_zero() const 
        {
            const auto s = 1e-8;
            return (fabs(e[0]) < s) && (fabs(e[1]) < s) && (fabs(e[2]) < s);
        }


        Vec3d operator-() const { return Vec3d(-e[0], -e[1], -e[2]);}
        double operator[](int i) const {return e[i];}
        double& operator[](int i) {return e[i];}

        Vec3d& operator+=(const Vec3d &v) 
        {
            e[0] += v.e[0];
            e[1] += v.e[1];
            e[2] += v.e[2];
            return *this;
        }

        Vec3d& operator*=(const double t) 
        {
            e[0] *= t;
            e[1] *= t;
            e[2] *= t;
            return *this;
        }

        Vec3d& operator/=(const double t) 
        {
            return *this *= 1/t;
        }

        double length() const 
        {
            return std::sqrt(length_squared());
        }

        double length_squared() const 
        {
            return e[0]*e[0] + e[1]*e[1] + e[2]*e[2];
        }
};

using Point3d = Vec3d;
using Color3d = Vec3d;

// Utility Functions

inline std::ostream& operator<<(std::ostream &out, const Vec3d &v) 
{
    return out << v.e[0] << ' ' << v.e[1] << ' ' << v.e[2];
}

inline Vec3d operator+(const Vec3d &u, const Vec3d &v) 
{
    return Vec3d(u.e[0] + v.e[0], u.e[1] + v.e[1], u.e[2] + v.e[2]);
}

inline Vec3d operator-(const Vec3d &u, const Vec3d &v) 
{
    return Vec3d(u.e[0] - v.e[0], u.e[1] - v.e[1], u.e[2] - v.e[2]);
}

inline Vec3d operator*(const Vec3d &u, const Vec3d &v) 
{
    return Vec3d(u.e[0] * v.e[0], u.e[1] * v.e[1], u.e[2] * v.e[2]);
}

inline Vec3d operator*(double t, const Vec3d &v) 
{
    return Vec3d(t*v.e[0], t*v.e[1], t*v.e[2]);
}

inline Vec3d operator*(const Vec3d &v, double t) 
{
    return t * v;
}

inline Vec3d operator/(Vec3d v, double t) 
{
    return (1/t) * v;
}

inline double dot(const Vec3d &u, const Vec3d &v) 
{
    return u.e[0] * v.e[0]
         + u.e[1] * v.e[1]
         + u.e[2] * v.e[2];
}

inline Vec3d cross(const Vec3d &u, const Vec3d &v) 
{
    return Vec3d(u.e[1] * v.e[2] - u.e[2] * v.e[1],
                u.e[2] * v.e[0] - u.e[0] * v.e[2],
                u.e[0] * v.e[1] - u.e[1] * v.e[0]);
}

inline Vec3d unit_vector(Vec3d v) 
{
    return v / v.length();
}

Vec3d reflect(const Vec3d& v, const Vec3d& n)
{
    return v - 2*dot(v,n)*n;
}

Vec3d refract(const Vec3d& uv, const Vec3d n, double snells_const)
{
    auto cos_theta = fmin(dot(-uv, n), 1.0);
    Vec3d r_out_perp = snells_const * (uv + cos_theta*n);
    Vec3d r_out_parallel = -sqrt(fabs(1.0 - r_out_perp.length_squared())) * n;
    return r_out_perp + r_out_parallel;
}

Vec3d random_in_unit_sphere()
{
    while (true)
    {
        Vec3d p = Vec3d::random(-1, 1);
        if (p.length_squared() >= 1) continue;
        return p;
    }
}

Vec3d normalize(const Vec3d& v)
{
    return v / v.length();
}

Vec3d random_unit_vector()
{
    return unit_vector(random_in_unit_sphere());
}

Vec3d random_in_hemisphere(const Vec3d& normal)
{
    Vec3d in_unit_sphere = random_in_unit_sphere();
    if(dot(in_unit_sphere, normal) > 0.0)
    {
        return in_unit_sphere;
    }
    else
    {
        return -in_unit_sphere;
    }
}

#endif
