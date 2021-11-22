#include <iostream>
#include <cmath>
#include <limits>
#include <memory>
#include <cstdlib>

#include "color.h"
#include "camera.h"
#include "vec3d.h"
#include "ray.h"
#include "aabb.h"
#include "hittable.h"
#include "helper.h"
#include "material.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#define STBI_MSC_SECURE_CRT
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

Color3d ray_color(const Ray& r, const Color3d& bg, const Hittable& world, int depth)
{
    HitRecord rec;

    if (depth <= 0)
    {
        return Color3d(0,0,0);
    }

    if (!world.hit(r, 0.001, __builtin_inf(), rec))
    {
        return bg;
    } 
    else
    {
        Ray scattered;
        Color3d attenuation;
        Color3d emitted = rec.mat_ptr->emitted(rec.u, rec.v, rec.p);
        if(!rec.mat_ptr->scatter(r, rec, attenuation, scattered))
        {
            return emitted;
        }
        else
        {
            return emitted + attenuation * ray_color(scattered, bg, world, depth-1);
        }
    }  
}

HittableList default_scene()
{
    auto grey = std::make_shared<Lambertian>(Color3d(0.5, 0.5, 0.5));
    auto grey_metal = std::make_shared<Metal>(Color3d(0.5, 0.5, 0.5), 0.1);
    auto red = std::make_shared<Metal>(Color3d(1.0, 0.0, 0.0), 0.3);
    auto green = std::make_shared<Lambertian>(Color3d(0.0, 1.0, 0.0));
    auto blue = std::make_shared<Lambertian>(Color3d(0.0, 0.0, 1.0));
    auto glass = std::make_shared<Dielectric>(1.5);
    auto grass = std::make_shared<Metal>(Color3d(0.8, 0.8, 0.0), .55);
    auto light_r = std::make_shared<DiffuseLight>(Color3d(4,4,4));
    auto light_g = std::make_shared<DiffuseLight>(Color3d(1,1,1));
    auto light_b = std::make_shared<DiffuseLight>(Color3d(4,4,4));


    HittableList scene;
    scene.add(std::make_shared<Sphere>(Point3d(1.5,0,-1), 0.75, grey));
    scene.add(std::make_shared<Sphere>(Point3d(0,0,-1), 0.75, glass));
    scene.add(std::make_shared<Sphere>(Point3d(0,0,-1), -0.7, glass));
    scene.add(std::make_shared<Sphere>(Point3d(-1.5,0,-1), 0.75, red));
    scene.add(std::make_shared<XZRect>(-200, 200, -200, 200, -.75, grass));
    scene.add(std::make_shared<YZRect>(0, 20, -10, 10, 20, light_r));
    scene.add(std::make_shared<XZRect>(-25, 25, -25, 25, 150, light_b));

    for (int a = -11; a < 11; a++) {
        for (int b = -11; b < 11; b++) {
            auto choose_mat = helper::random_double();
            Point3d center(a + 0.9*helper::random_double(), -0.55, b + 0.9*helper::random_double());

            if ((center - Point3d(4, 0.2, 0)).length() > 0.9) {
                std::shared_ptr<Material> sphere_material;

                if (choose_mat < 0.8) {
                    // diffuse
                    auto albedo = Color3d::random() * Color3d::random();
                    sphere_material = std::make_shared<Lambertian>(albedo);
                    scene.add(std::make_shared<Sphere>(center, 0.2, sphere_material));
                } else if (choose_mat < 0.95) {
                    // metal
                    auto albedo = Color3d::random(0.5, 1);
                    auto fuzz = helper::random_double(0, 0.5);
                    sphere_material = std::make_shared<Metal>(albedo, fuzz);
                    scene.add(std::make_shared<Sphere>(center, 0.2, sphere_material));
                } else {
                    // glass
                    sphere_material = std::make_shared<Dielectric>(1.5);
                    scene.add(std::make_shared<Sphere>(center, 0.2, sphere_material));
                }
            }
        }
    }

    Point3d v0(-2,0.5,-2); Point3d v1(2, 0.5, -2); Point3d v2(0, 2.5, -1);
    scene.add(std::make_shared<Triangle>(v0, v1, v2, grey_metal));

    HittableList bvh_scene;
    bvh_scene.add(std::make_shared<BVHNode>(scene, 0, 1.0));
    return bvh_scene;
}

HittableList box_scene()
{
    auto grey = std::make_shared<Lambertian>(Color3d(0.5, 0.5, 0.5));
    auto grey_metal = std::make_shared<Metal>(Color3d(0.5, 0.5, 0.5), 0.1);
    auto orange_metal = std::make_shared<Metal>(Color3d(1.0, 0.51, 0.0), 0.0);
    auto red = std::make_shared<Metal>(Color3d(1.0, 0.0, 0.0), 0.3);
    auto green = std::make_shared<Lambertian>(Color3d(0.0, 1.0, 0.0));
    auto blue = std::make_shared<Lambertian>(Color3d(0.0, 0.0, 1.0));
    auto glass = std::make_shared<Dielectric>(1.5);
    auto grass = std::make_shared<Lambertian>(Color3d(0.8, 0.8, 0.0));
    auto light_r = std::make_shared<DiffuseLight>(Color3d(4,4,4));
    auto light_g = std::make_shared<DiffuseLight>(Color3d(1,1,1));
    auto light_b = std::make_shared<DiffuseLight>(Color3d(4,4,4));


    HittableList scene;
    scene.add(std::make_shared<Sphere>(Point3d(1.5,0,-1), 0.75, glass));
    scene.add(std::make_shared<Box>(Point3d(-1.5, -.5, -1), Point3d(-.75, 1, 0), glass));
    scene.add(std::make_shared<XZRect>(-200, 200, -200, 200, -.75, grass));
    scene.add(std::make_shared<YZRect>(0, 20, -10, 10, 20, light_r));
    scene.add(std::make_shared<XZRect>(-25, 25, -25, 25, 150, light_b));

    Point3d v0(-2,0.5,-2); Point3d v1(2, 0.5, -2); Point3d v2(0, 2.5, -1);
    scene.add(std::make_shared<Triangle>(v0, v1, v2, glass));

    HittableList bvh_scene;
    bvh_scene.add(std::make_shared<BVHNode>(scene, 0, 1.0));
    return bvh_scene;
}

HittableList box_test()
{
    auto red = std::make_shared<Metal>(Color3d(1.0, 0.0, 0.0), 0.3);
    auto green = std::make_shared<Lambertian>(Color3d(0.0, 1.0, 0.0));
    auto blue = std::make_shared<Lambertian>(Color3d(0.0, 0.0, 1.0));
    auto glass = std::make_shared<Dielectric>(1.5);
    auto grass = std::make_shared<Lambertian>(Color3d(0.8, 0.8, 0.0));
    auto light_r = std::make_shared<DiffuseLight>(Color3d(4,4,4));
    auto light_g = std::make_shared<DiffuseLight>(Color3d(1,1,1));
    auto light_b = std::make_shared<DiffuseLight>(Color3d(4,4,4));


    HittableList scene;
    scene.add(std::make_shared<Sphere>(Point3d(0,1,-2), 0.5, green));
    scene.add(std::make_shared<Sphere>(Point3d(0,1,2), 0.5, blue));
    scene.add(std::make_shared<Box>(Point3d(-1, 0.5, -0.5), Point3d(1, 1.5, 0.5), glass));
    scene.add(std::make_shared<XZRect>(-200, 200, -200, 200, -.75, grass));
    scene.add(std::make_shared<YZRect>(0, 20, -10, 10, 20, light_r));
    scene.add(std::make_shared<XZRect>(-25, 25, -25, 25, 150, light_b));

    HittableList bvh_scene;
    bvh_scene.add(std::make_shared<BVHNode>(scene, 0, 1.0));
    return bvh_scene;
}

HittableList get_scene(int scene_id)
{
    switch(scene_id){
    case 1:
        return box_scene();
        break;
    case 2:
        return box_test();
        break;
    default:
        return default_scene();
    }
}

int main(int argc, char** argv)
{
    int scene = atoi(argv[1]);
    std::cout << scene << std::endl;
    // Image
    const double ASPECT_RATIO = 22.0/9.0;
    const int WIDTH = 600;
    const int HEIGHT = static_cast<int>(WIDTH / ASPECT_RATIO);
    const int spp = 500;
    const int max_depth = 60;
    const int NUM_CHANNELS = 3;

    // Camera
    Point3d lookfrom(0, 1, 1.5);
    Point3d lookat(0, 1, -1);
    Point3d up(0, 1, 0);
    double fov = 65.0; 
    Camera cam(lookfrom, lookat, up, fov, ASPECT_RATIO);
    Color3d background(0.0, 0.0, 0.0);

    // World
    HittableList world;
    world = get_scene(scene);

    // Rendering!
    uint8_t* pixels = new uint8_t[WIDTH*HEIGHT*NUM_CHANNELS];
    int index = 0;

    for (int j = HEIGHT-1; j >= 0; --j)
    {
        int percent_left = static_cast<int>(100*((float)j/(float)HEIGHT));
        std::cerr << "\rScanlines Remaining: " << j << " | " << percent_left << "\% left" << " | "<< std::flush;
        for (int i = 0; i < WIDTH; ++i)
        {
            Color3d pixel_color(0,0,0);
            for (int s = 0; s < spp; ++s)
            {
                double u = (i + helper::random_double()) / (WIDTH-1);
                double v = (j + helper::random_double()) / (HEIGHT-1);
                Ray r = cam.GetRay(u, v);
                pixel_color += ray_color(r, background, world, max_depth);
            }
            write_color(pixels, pixel_color, index, spp);
        }
    }

    stbi_write_jpg("out.jpg", WIDTH, HEIGHT, NUM_CHANNELS, pixels, 100);
    std::cerr << "\nDone.\n";

    delete[] pixels;
    return 0;
}