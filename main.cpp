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

    if (!world.hit(r, 0.001, std::numeric_limits<double>::infinity(), rec))
    {
        return bg;
    } 
    
    Ray scattered;
    Color3d attenuation;
    Color3d emitted = rec.mat_ptr->emitted(rec.u, rec.v, rec.p);
    if(!rec.mat_ptr->scatter(r, rec, attenuation, scattered))
    {
        return emitted;
    }
    
    //auto transform_normal = (rec.normal*0.5) + Vec3d(0.5,0.5,0.5);
    //transform_normal.print();
    //return transform_normal;
    return attenuation * ray_color(scattered, bg, world, depth-1);
    //return emitted + attenuation * ray_color(scattered, bg, world, depth-1);
}

HittableList default_scene()
{
    HittableList scene;

    auto red   = std::make_shared<Metal>(Color3d(.65, .05, .05), 0.2);
    auto white = std::make_shared<Lambertian>(Color3d(.73, .73, .73));
    auto green = std::make_shared<Metal>(Color3d(.12, .45, .15), 0.2);
    auto light = std::make_shared<DiffuseLight>(Color3d(15, 15, 15));

    scene.add(std::make_shared<YZRect>(0, 555, 0, 555, 555, green));
    scene.add(std::make_shared<YZRect>(0, 555, 0, 555, 0, red));
    scene.add(std::make_shared<XZRect>(213, 343, 227, 332, 554, light));
    scene.add(std::make_shared<XZRect>(0, 555, 0, 555, 0, white));
    scene.add(std::make_shared<XZRect>(0, 555, 0, 555, 555, white));
    scene.add(std::make_shared<XYRect>(0, 555, 0, 555, 555, white));

    HittableList bvh_scene;
    bvh_scene.add(std::make_shared<BVHNode>(scene, 0, 1.0));
    return scene;
}


HittableList proj3_scene()
{
    HittableList objects;

    // Create colors
    std::shared_ptr<Lambertian> olive = std::make_shared<Lambertian>(Color3d(.71, .7, .36));
    std::shared_ptr<Lambertian> beige = std::make_shared<Lambertian>(Color3d(.56, .55, .36));
    std::shared_ptr<Lambertian> red = std::make_shared<Lambertian>(Color3d(.57, .0, .04));
    std::shared_ptr<Lambertian> blue = std::make_shared<Lambertian>(Color3d(.015, .015, .7));
    std::shared_ptr<Lambertian> grey = std::make_shared<Lambertian>(Color3d(.15, .15, .15));
    std::shared_ptr<Lambertian> black = std::make_shared<Lambertian>(Color3d(.05, .05, .05));
    std::shared_ptr<Lambertian> white = std::make_shared<Lambertian>(Color3d(.95, .95, .95));
    std::shared_ptr<DiffuseLight> light = std::make_shared<DiffuseLight>(Color3d(50, 50, 50));
    // Add objects

    // Walls
    objects.add(std::make_shared<YZRect>(0, 555, 0, 555, 555, olive));
    objects.add(std::make_shared<FlipFace>(std::make_shared<XZRect>(-1000, 1000, -1000, 1000, 8000, light)));
    objects.add(std::make_shared<XZRect>(0, 555, 0, 555, 0, red));
    objects.add(std::make_shared<XYRect>(0, 555, 0, 555, 555, olive));

    // Bed
    std::shared_ptr<Hittable> bed_bottom_box = std::make_shared<Box>(Point3d(0,0,0), Point3d(160,25,300), beige);
    bed_bottom_box = std::make_shared<Translate>(bed_bottom_box, Vec3d(555-170+10,0,555-310+10));
    objects.add(bed_bottom_box);

    std::shared_ptr<Hittable> bed_top_box = std::make_shared<Box>(Point3d(0,0,0), Point3d(170,10,310), beige);
    bed_top_box = std::make_shared<Translate>(bed_top_box, Vec3d(555-170,25,555-310));
    objects.add(bed_top_box);

    std::shared_ptr<Hittable> bed_sheet_box = std::make_shared<Box>(Point3d(0,0,0), Point3d(150,5,220), blue);
    bed_sheet_box = std::make_shared<Translate>(bed_sheet_box, Vec3d(555-170+10,35,555-310+10));
    objects.add(bed_sheet_box);

    std::shared_ptr<Hittable> pillow_box_one = std::make_shared<Box>(Point3d(0,0,0), Point3d(70,5,80), white);
    pillow_box_one = std::make_shared<Translate>(pillow_box_one, Vec3d(555-170+10,35,555-310+220));
    objects.add(pillow_box_one);

    std::shared_ptr<Hittable> pillow_box_two = std::make_shared<Box>(Point3d(0,0,0), Point3d(70,5,80), white);
    pillow_box_two = std::make_shared<Translate>(pillow_box_two, Vec3d(555-170+10+75,35,555-310+220));
    objects.add(pillow_box_two);

    // Table
    std::shared_ptr<Hittable> table_shelf = std::make_shared<Box>(Point3d(0, 0, 0), Point3d(80,5,80), beige);
    auto table_top_shelf = std::make_shared<Translate>(table_shelf, Vec3d(0,80,0));
    auto table_bot_shelf = std::make_shared<Translate>(table_shelf, Vec3d(0,40,0));
    objects.add(table_top_shelf);
    objects.add(table_bot_shelf);

    std::shared_ptr<Hittable> table_leg = std::make_shared<Box>(Point3d(0, 0, 0), Point3d(15,75,15), beige);
    auto leg_1 = std::make_shared<Translate>(table_leg, Vec3d(0,0,0));
    auto leg_2 = std::make_shared<Translate>(table_leg, Vec3d(65,0,0));
    auto leg_3 = std::make_shared<Translate>(table_leg, Vec3d(0,0,65));
    auto leg_4 = std::make_shared<Translate>(table_leg, Vec3d(65,0,65));
    objects.add(leg_1);
    objects.add(leg_2);
    objects.add(leg_3);
    objects.add(leg_4);


    // Dresser

    // Lamp

    // Fan

    HittableList bvh_scene;
    bvh_scene.add(std::make_shared<BVHNode>(objects, 0, 1.0));
    return objects;
}

HittableList get_scene(int scene_id)
{
    switch(scene_id){
    case 1:
        return proj3_scene();
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
    const double ASPECT_RATIO = 1.0;
    const int WIDTH = 600;
    const int HEIGHT = static_cast<int>(WIDTH / ASPECT_RATIO);
    const int spp = 50;
    const int max_depth = 50;
    const int NUM_CHANNELS = 3;

    // Camera
    Point3d lookfrom(-600, 578, -500);
    Point3d lookat(555, 000, 555);
    //Point3d lookfrom(278, 278, -800);
    //Point3d lookat(278, 278, 0);
    Point3d up(0, 1, 0);
    double fov = 45.0; 
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