#ifndef CAMERA_H
#define CAMERA_H

#include "helper.h"
#include "ray.h"
#include "vec3d.h"
#include "sceneconfig.h"

/**
 * Camera class
 *  This class serves an obvious purpose, it is used to define
 *  the point of view from when the renderer draws at.
 */
class Camera {
public:
    Camera(
        Point3d lookfrom,
        Point3d lookat,
        Vec3d up,
        double vfov,
        double aspect_ratio
    ) 
    {
        double theta = helper::radians(vfov);
        double h = std::tan(theta/2);
        double vp_height = 2.0 * h;
        double vp_width = aspect_ratio * vp_height;

        auto w = unit_vector(lookfrom - lookat); auto u = unit_vector(cross(up, w)); auto v = cross(w,u);

        mOrigin = lookfrom;
        mHorizontal = vp_width * u;
        mVertical = vp_height * v;
        mLowerLeftCorner = mOrigin - mHorizontal/2 - mVertical/2 - w;
    }
    
    explicit Camera(const SceneConfig& config) {
        double theta = helper::radians(config.fov);
        double h = std::tan(theta/2);
        double vp_height = 2.0 * h;
        double vp_width = config.aspect_ratio * vp_height;

        auto w = unit_vector(config.lookfrom - config.lookat); auto u = unit_vector(cross(config.up, w)); auto v = cross(w,u);

        mOrigin = config.lookfrom;
        mHorizontal = vp_width * u;
        mVertical = vp_height * v;
        mLowerLeftCorner = mOrigin - mHorizontal/2 - mVertical/2 - w;
    }

    Ray GetRay(double u, double v) const 
    {
        return Ray(mOrigin, mLowerLeftCorner + u*mHorizontal + v*mVertical - mOrigin);
    }
private:
    Point3d mOrigin;
    Point3d mLowerLeftCorner;
    Vec3d mHorizontal;
    Vec3d mVertical;
};


#endif