#ifndef MATERIAL_H
#define MATERIAL_H

#include <memory>

#include "helper.h"
#include "vec3d.h"
#include "texture.h"

struct HitRecord;

class Material
{
public:
    virtual bool scatter(const Ray& r_in, const HitRecord& rec, Color3d& attenuation, Ray& scattered) const = 0;
    virtual Color3d emitted(double u, double v, const Point3d& p) const {return Color3d(0,0,0);}
};

class Lambertian : public Material
{
public:
    std::shared_ptr<Texture> albedo;

public:
    Lambertian(const Color3d& a) : albedo(std::make_shared<SolidColor>(a)) {}
    Lambertian(std::shared_ptr<Texture> a) : albedo(a) {}

    virtual bool scatter(const Ray& r_in, const HitRecord& rec, Color3d& attenuation, Ray& scattered) const override
    {
        auto scatter_direction = rec.normal + random_unit_vector();

        if (scatter_direction.near_zero()) scatter_direction = rec.normal;

        scattered = Ray(rec.p, scatter_direction);
        attenuation = albedo->value(rec.u, rec.v, rec.p);
        return true;
    }
};


class Metal : public Material
{
public:
    Color3d albedo;
    double fuzz;
public:
    Metal(const Color3d& a, double f) : albedo(a), fuzz(f) {}
    virtual bool scatter(const Ray& r_in, const HitRecord& rec, Color3d& attenuation, Ray& scattered) const override
    {
        Vec3d reflected = reflect(unit_vector(r_in.direction()), rec.normal);
        scattered = Ray(rec.p, reflected + fuzz*random_in_hemisphere(rec.normal));
        attenuation = albedo;
        return (dot(scattered.direction(), rec.normal) > 0);
    }
};

class Dielectric : public Material
{
public:
    double ir;
public:
    Dielectric(double refraction_index) : ir(refraction_index) {}
    virtual bool scatter(const Ray& r_in, const HitRecord& rec, Color3d& attenuation, Ray& scattered) const override
    {
        attenuation = Color3d(1.0, 1.0, 1.0);
        double refraction_ratio = rec.front_face ? (1.0/ir) : ir;
        Vec3d unit_direction = unit_vector(r_in.direction());

        double cos_theta = fmin(dot(-unit_direction, rec.normal), 1.0);
        double sin_theta = std::sqrt(1.0 - cos_theta*cos_theta);

        bool cannot_refract = refraction_ratio * sin_theta > 1.0;
        Vec3d direction = (cannot_refract||reflectance(cos_theta, refraction_ratio) > helper::random_double()) ? reflect(unit_direction, rec.normal) : refract(unit_direction, rec.normal, refraction_ratio);

        scattered = Ray(rec.p, direction);
        return true;
    }
private:
    // Shlick Approximation
    static double reflectance(double cosine, double ref_idx)
    {
        double r0 = (1-ref_idx) / (1+ref_idx);
        r0 = r0*r0;
        return r0 + (1-r0)*pow((1-cosine), 5);
    }
};

class DiffuseLight : public Material
{
public:
    std::shared_ptr<Texture> emit;
public:
    DiffuseLight(std::shared_ptr<Texture> a) : emit(a) {}
    DiffuseLight(Color3d c) : emit(std::make_shared<SolidColor>(c)) {}

    virtual bool scatter(const Ray& r_in, const HitRecord& rec, Color3d& attenuation, Ray& scattered) const override
    {
        return false;
    }

    virtual Color3d emitted(double u, double v, const Point3d& p) const override
    {
        return emit->value(u, v, p);
    }
};


#endif