#ifndef RAY_H
#define RAY_H

#include "vec3d.h"

// Ray class, obvious purposes.
class Ray 
{
    public:
        Ray() {}
        Ray(const Point3d& origin, const Vec3d& direction) : orig(origin), dir(direction) {}

        Point3d origin() const { return orig; }
        Vec3d direction() const { return dir; }

        Point3d at(double t) const { return orig + t*dir; }

    public:
        Point3d orig;
        Vec3d dir;
};


#endif