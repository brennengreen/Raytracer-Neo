#ifndef HITTABLE_H
#define HITTABLE_H

#include <memory>
#include <vector>
#include <iostream>
#include <algorithm>

#include "ray.h"
#include "vec3d.h"
#include "AABB.h"

class Material;


/**
 * HitRecord struct
 *  A HitRecord proves the raytracer with useful information
 *  such as uv transformations, the material, and the normal of intersection
 */
struct HitRecord {
    Point3d p;
    Vec3d normal;
    double t;
    double u;
    double v;
    std::shared_ptr<Material> mat_ptr;
    bool front_face;
    inline void set_face_normal(const Ray& r, const Vec3d& outward_normal)
    {
        front_face = dot(r.direction(), outward_normal) < 0;
        normal = front_face ? outward_normal : -outward_normal;
    }
};

/**
 * Hittable class
 *  A virtual class which all hittables derive from
 *  provides a hit() and bounding_box() function to determine intersections
 */
class Hittable {
public:
    virtual bool hit (const Ray& r, double t_min, double t_max, HitRecord& rec) const = 0;
    virtual bool bounding_box(double time0, double time1, AABB& out_box) const = 0;  
};

/**
 * HittableList class
 *  A scenegraph implementation of a hittable,
 *  provides the ability to group several hittables into one instance
 */
class HittableList : public Hittable
{
public:
    std::vector<std::shared_ptr<Hittable>> objects;

    HittableList() = default;
    explicit HittableList(std::shared_ptr<Hittable> object)
    {add(object);}

    void clear()
    {objects.clear();}

    void add(std::shared_ptr<Hittable> object)
    {objects.push_back(object);}

    virtual bool hit(const Ray& r, double t_min, double t_max, HitRecord& rec) const override 
    {
        HitRecord temp_rec;
        bool hit_anything = false;
        double closest_so_far = t_max;

        for (const auto& object : objects)
        {
            if (object->hit(r, t_min, closest_so_far, temp_rec))
            {
                hit_anything = true;
                closest_so_far = temp_rec.t;
                rec = temp_rec;
            }
        }

        return hit_anything;
    }

    virtual bool bounding_box(double time0, double time1, AABB& out_box) const override
    {
        if (objects.empty()) return false;

        AABB temp_box;
        bool first_box = true;

        for (const auto& object : objects)
        {
            if (!object->bounding_box(time0, time1, temp_box)) return false;
            out_box = first_box ? temp_box : surrounding_box(out_box, temp_box);
            first_box = false;
        }

        return true;
    }
};

class Sphere : public Hittable {
public:
    Point3d center;
    double radius;
    std::shared_ptr<Material> mat_ptr;

    Sphere()=default;
    Sphere(Point3d c, double r, std::shared_ptr<Material> m) : center(c), radius(r), mat_ptr(m) {};
    
    virtual bool hit(const Ray& r, double t_min, double t_max, HitRecord& rec) const override 
    {
        Vec3d oc = r.origin() - center;
        double a = r.direction().length_squared();
        double half_b = dot(oc, r.direction());
        double c = oc.length_squared() - radius*radius;

        double disc = half_b*half_b - a*c;
        if (disc < 0) return false;
        double sqrt_disc = sqrt(disc);

        double root = (-half_b - sqrt_disc) / a;
        if (root < t_min || t_max < root)
        {
            root = (-half_b + sqrt_disc) / a;
            if (root < t_min || t_max < root)
            {
                return false;
            }
        } 

        rec.t = root;
        rec.p = r.at(rec.t);
        rec.set_face_normal(r, (rec.p - center) / radius);
        rec.mat_ptr = mat_ptr;
        rec.u = atan2(-rec.p.z(), rec.p.x()) + helper::PI;
        rec.v = acos(-rec.p.y()) / helper::PI;

        return true;
    }

    virtual bool bounding_box(double time0, double time1, AABB& out_box) const override
    {
        out_box = AABB(center - Vec3d(radius, radius, radius), center + Vec3d(radius, radius, radius));
        return true;
    }
};

class Triangle : public Hittable
{
public:
    std::shared_ptr<Material> mat_ptr;
    Vec3d v0;
    Vec3d v1;
    Vec3d v2;
public:
    Triangle(Vec3d v_0, Vec3d v_1, Vec3d v_2, std::shared_ptr<Material> m) : v0(v_0), v1(v_1), v2(v_2), mat_ptr(m)
    {
    }

    virtual bool hit(const Ray& r, double t_min, double t_max, HitRecord& rec) const override
    {
        Vec3d v0v1 = v1-v0;
        Vec3d v0v2 = v2-v0;
        Vec3d pvec = cross(r.direction(), v0v2);
        float det = dot(v0v1, pvec);

        if (det < 0) return false;

        float invDet = 1 / det;

        Vec3d tvec = r.origin() - v0;
        float u = dot(tvec, pvec) * invDet;
        if (u < 0 || u > 1) return false;

        Vec3d qvec = cross(tvec, v0v1);
        float v = dot(r.direction(), qvec) * invDet;
        if (v < 0 || u+v > 1) return false;

        float t = dot(v0v2, qvec) * invDet;

        rec.t = t;
        rec.p = r.at(rec.t);
        rec.set_face_normal(r, normalize(cross(v0v1, v0v2)));
        rec.mat_ptr = mat_ptr;
        rec.u = u;
        rec.v = v;
        return true;
    }

    virtual bool bounding_box(double time0, double time1, AABB& out_box) const override
    {   
        double xmax = std::max(v0.x(), std::max(v1.x(), v2.x()));
        double ymax = std::max(v0.y(), std::max(v1.y(), v2.y()));
        double zmax = std::max(v0.z(), std::max(v1.z(), v2.z()));
        double xmin = std::min(v0.x(), std::min(v1.x(), v2.x()));
        double ymin = std::min(v0.y(), std::min(v1.y(), v2.y()));
        double zmin = std::min(v0.z(), std::min(v1.z(), v2.z()));

        Vec3d vmax(xmax, ymax, zmax);
        Vec3d vmin(xmin, ymin, zmin);
        out_box = AABB(vmin, vmax);
        return true;
    }
};


inline bool box_compare(const std::shared_ptr<Hittable> a, const std::shared_ptr<Hittable> b, int axis);
bool box_x_compare (const std::shared_ptr<Hittable> a, const std::shared_ptr<Hittable> b);
bool box_y_compare (const std::shared_ptr<Hittable> a, const std::shared_ptr<Hittable> b);
bool box_z_compare (const std::shared_ptr<Hittable> a, const std::shared_ptr<Hittable> b);

/**
 * BVHNode class
 *  Provides a node which can be used to define the bounding volume hierarchy
 */
class BVHNode : public Hittable
{
public:
    std::shared_ptr<Hittable> left;
    std::shared_ptr<Hittable> right;
    AABB box;
public:
    BVHNode() {}
    BVHNode(const HittableList& list, double time0, double time1) : BVHNode(list.objects, 0, list.objects.size(), time0, time1) {}
    BVHNode(const std::vector<std::shared_ptr<Hittable>>& src_objects, size_t start, size_t end, double time0, double time1)
    {
        auto objects = src_objects; // Create a modifiable array of the source scene objects

        int axis = helper::random_int(0,2);
        auto comparator = (axis == 0) ? box_x_compare
                        : (axis == 1) ? box_y_compare
                                      : box_z_compare;

        size_t object_span = end - start;

        if (object_span == 1)
        {
            left = right = objects[start];
        }
        else if (object_span == 2) 
        {
            if (comparator(objects[start], objects[start+1]))
            {
                left = objects[start];
                right = objects[start+1];
            }
            else
            {
                left = objects[start+1];
                right = objects[start];
            }
        }
        else
        {
            std::sort(objects.begin() + start, objects.begin() + end, comparator);

            auto mid = start + object_span/2;
            left = std::make_shared<BVHNode>(objects, start, mid, time0, time1);
            right = std::make_shared<BVHNode>(objects, mid, end, time0, time1);
        }

        AABB box_left, box_right;

        if (!left->bounding_box(time0, time1, box_left) || !right->bounding_box(time0, time1, box_right))
            std::cerr << "No bounding box in BVHNode constructor.\n";

        box = surrounding_box(box_left, box_right);       
    }

    virtual bool hit(const Ray& r, double t_min, double t_max, HitRecord& rec) const override
    {
        if (!box.hit(r, t_min, t_max)) return false;

        bool hit_left = left->hit(r, t_min, t_max, rec);
        bool hit_right = right->hit(r, t_min, hit_left ? rec.t : t_max, rec);

        return hit_left || hit_right;
    }
    virtual bool bounding_box(double time0, double time1, AABB& out_box) const override
    {
        out_box = box;
        return true;
    }

};

inline bool box_compare(const std::shared_ptr<Hittable> a, const std::shared_ptr<Hittable> b, int axis)
{
    AABB box_a;
    AABB box_b;

    if(!a->bounding_box(0, 0, box_a) || !b->bounding_box(0, 0, box_b))
    {
        std::cerr << "No bounding box in BVHNode constructor.\n";
    }

    return box_a.min().e[axis] < box_b.min().e[axis];
}

bool box_x_compare (const std::shared_ptr<Hittable> a, const std::shared_ptr<Hittable> b) {
    return box_compare(a, b, 0);
}

bool box_y_compare (const std::shared_ptr<Hittable> a, const std::shared_ptr<Hittable> b) {
    return box_compare(a, b, 1);
}

bool box_z_compare (const std::shared_ptr<Hittable> a, const std::shared_ptr<Hittable> b) {
    return box_compare(a, b, 2);
}

// Axis Aligned Classes

class XYRect : public Hittable {
    public:
        XYRect() {}

        XYRect(double _x0, double _x1, double _y0, double _y1, double _k, std::shared_ptr<Material> mat) : x0(_x0), x1(_x1), y0(_y0), y1(_y1), k(_k), mp(mat) {};

        virtual bool hit(const Ray& r, double t_min, double t_max, HitRecord& rec) const override
        {
            auto t = (k-r.origin().z()) / r.direction().z();
            if (t < t_min || t > t_max) return false;
            auto x = r.origin().x() + t*r.direction().x();
            auto y = r.origin().y() + t*r.direction().y();
            if (x < x0 || x > x1 || y < y0 || y > y1) return false;
            rec.u = (x-x0)/(x1-x0);
            rec.v = (y-y0)/(y1-y0);
            rec.t = t;
            auto outward_normal = Vec3d(0, 0, 1);
            rec.set_face_normal(r, outward_normal);
            rec.mat_ptr = mp;
            rec.p = r.at(t);
            return true;
        }

        virtual bool bounding_box(double time0, double time1, AABB& out_box) const override 
        {
            out_box = AABB(Point3d(x0,y0, k-0.0001), Point3d(x1, y1, k+0.0001));
            return true;
        }

    public:
        std::shared_ptr<Material> mp;
        double x0, x1, y0, y1, k;
};

class XZRect : public Hittable {
    public:
        XZRect() {}

        XZRect(double _x0, double _x1, double _z0, double _z1, double _k,
            std::shared_ptr<Material> mat)
            : x0(_x0), x1(_x1), z0(_z0), z1(_z1), k(_k), mp(mat) {};

        virtual bool hit(const Ray& r, double t_min, double t_max, HitRecord& rec) const override
        {
            auto t = (k-r.origin().y()) / r.direction().y();
            if (t < t_min || t > t_max)
                return false;
            auto x = r.origin().x() + t*r.direction().x();
            auto z = r.origin().z() + t*r.direction().z();
            if (x < x0 || x > x1 || z < z0 || z > z1)
                return false;
            rec.u = (x-x0)/(x1-x0);
            rec.v = (z-z0)/(z1-z0);
            rec.t = t;
            auto outward_normal = Vec3d(0, 1, 0);
            rec.set_face_normal(r, outward_normal);
            rec.mat_ptr = mp;
            rec.p = r.at(t);
            return true;
        }

        virtual bool bounding_box(double time0, double time1, AABB& out_box) const override 
        {
            out_box = AABB(Point3d(x0,k-0.0001,z0), Point3d(x1, k+0.0001, z1));
            return true;
        }

    public:
        std::shared_ptr<Material> mp;
        double x0, x1, z0, z1, k;
};

class YZRect : public Hittable {
    public:
        YZRect() {}

        YZRect(double _y0, double _y1, double _z0, double _z1, double _k,
            std::shared_ptr<Material> mat)
            : y0(_y0), y1(_y1), z0(_z0), z1(_z1), k(_k), mp(mat) {};

        virtual bool hit(const Ray& r, double t_min, double t_max, HitRecord& rec) const override
        {
            auto t = (k-r.origin().x()) / r.direction().x();
            if (t < t_min || t > t_max)
                return false;
            auto y = r.origin().y() + t*r.direction().y();
            auto z = r.origin().z() + t*r.direction().z();
            if (y < y0 || y > y1 || z < z0 || z > z1)
                return false;
            rec.u = (y-y0)/(y1-y0);
            rec.v = (z-z0)/(z1-z0);
            rec.t = t;
            auto outward_normal = Vec3d(1, 0, 0);
            rec.set_face_normal(r, outward_normal);
            rec.mat_ptr = mp;
            rec.p = r.at(t);
            return true;
        }

        virtual bool bounding_box(double time0, double time1, AABB& out_box) const override 
        {
            out_box = AABB(Point3d(k-0.0001, y0, z0), Point3d(k+0.0001, y1, z1));
            return true;
        }

    public:
        std::shared_ptr<Material> mp;
        double y0, y1, z0, z1, k;
};

class Box : public Hittable
{
public:
    Point3d box_min;
    Point3d box_max;
    HittableList sides;
public:
    Box() {}
    Box(const Point3d& p0, const Point3d& p1, std::shared_ptr<Material> ptr)
    {
        box_min = p0;
        box_max = p1;

        sides.add(std::make_shared<XYRect>(p0.x(), p1.x(), p0.y(), p1.y(), p1.z(), ptr));
        sides.add(std::make_shared<XYRect>(p0.x(), p1.x(), p0.y(), p1.y(), p0.z(), ptr));

        sides.add(std::make_shared<XZRect>(p0.x(), p1.x(), p0.z(), p1.z(), p0.y(), ptr));
        sides.add(std::make_shared<XZRect>(p0.x(), p1.x(), p0.z(), p1.z(), p1.y(), ptr));

        sides.add(std::make_shared<YZRect>(p0.y(), p1.y(), p0.z(), p1.z(), p0.x(), ptr));
        sides.add(std::make_shared<YZRect>(p0.y(), p1.y(), p0.z(), p1.z(), p1.x(), ptr));
    }
    
    virtual bool hit(const Ray& r, double t_min, double t_max, HitRecord& rec) const override
    {
        return sides.hit(r, t_min, t_max, rec);
    }

    virtual bool bounding_box(double time0, double time1, AABB& out_box) const override 
    {
        out_box = AABB(box_min, box_max);
        return true;
    }
};


// Transformation Classes

class Translate : public Hittable {
public:
    std::shared_ptr<Hittable> ptr;
    Vec3d offset;
public:
    Translate(std::shared_ptr<Hittable> p, const Vec3d& displacement) : ptr(p), offset(displacement) {}
    virtual bool hit(const Ray& r, double t_min, double t_max, HitRecord& rec) const override
    {
        Ray moved_r(r.origin() - offset, r.direction());
        if (!ptr->hit(moved_r, t_min, t_max, rec)) return false;

        rec.p += offset;
        rec.set_face_normal(moved_r, rec.normal);
        
        return true;
    }
    virtual bool bounding_box(double time0, double time1, AABB& out_box) const override 
    {
        if(!ptr->bounding_box(time0, time1, out_box)) return false;
        out_box = AABB(out_box.min() + offset, out_box.max() + offset);
        return true;
    }
};

class FlipFace : public Hittable {
public:
    std::shared_ptr<Hittable> ptr;
public:
    FlipFace(std::shared_ptr<Hittable> p) : ptr(p) {}
    virtual bool hit(const Ray& r, double t_min, double t_max, HitRecord& rec) const override
    {
        if (!ptr->hit(r, t_min, t_max, rec)) return false;

        rec.front_face = !rec.front_face;
        return true;
    }
    virtual bool bounding_box(double time0, double time1, AABB& out_box) const override 
    {
        return ptr->bounding_box(time0, time1, out_box);
    }
};

#endif