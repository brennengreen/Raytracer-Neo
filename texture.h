#ifndef TEXTURE_H
#define TEXTURE_H

#include "helper.h"
#include "vec3d.h"

// Texture classes
// Provide the ability to map materials to a texture
class Texture 
{
public:
    virtual Color3d value(double u, double v, const Point3d& p) const = 0;
};

class SolidColor : public Texture
{
public:
    Color3d color_value;
public:
    SolidColor() {}
    SolidColor(Color3d c) : color_value(c) {}
    SolidColor(double r, double g, double b) : SolidColor(Color3d(r, g, b)) {}

    virtual Color3d value(double u, double v, const Vec3d& p) const override
    {
        return color_value;
    }

};

#endif