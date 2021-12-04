#ifndef AABB_H
#define AABB_H

#include "helper.h"
#include "vec3d.h"

class AABB
{
public:
    Point3d minimum;
    Point3d maximum;
public:
    AABB() {}
    AABB(const Point3d& a, const Point3d& b): minimum(a), maximum(b) {}

    Point3d min() const {return minimum;}
    Point3d max() const {return maximum;}

    bool hit(const Ray& r, double t_min, double t_max) const 
    {
        for (int a = 0; a < 3; a++)
        {
            auto t0 = fmin((minimum[a] - r.origin()[a]) / r.direction()[a],
                            (maximum[a] - r.origin()[a]) / r.direction()[a]);
            auto t1 = fmax((minimum[a] - r.origin()[a]) / r.direction()[a],
                            (maximum[a] - r.origin()[a]) / r.direction()[a]);
            t_min = fmax(t0, t_min);
            t_max = fmin(t1, t_max);
            if (t_max <= t_min)
                return false;
        }
        return true;
    }
};

AABB surrounding_box(AABB box0, AABB box1)
{
    Point3d small(fmin(box0.min().x(), box1.min().x()),
                 fmin(box0.min().y(), box1.min().y()),
                 fmin(box0.min().z(), box1.min().z()));

    Point3d big(fmax(box0.max().x(), box1.max().x()),
               fmax(box0.max().y(), box1.max().y()),
               fmax(box0.max().z(), box1.max().z()));

    return AABB(small,big);
}

#endif