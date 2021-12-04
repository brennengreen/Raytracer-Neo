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
#include "sceneconfig.h"

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
    return emitted + attenuation * ray_color(scattered, bg, world, depth-1);
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
    std::shared_ptr<Lambertian> olive = std::make_shared<Lambertian>(Color3d(.61, .6, .26));
    std::shared_ptr<Lambertian> beige = std::make_shared<Lambertian>(Color3d(.56, .55, .36));
    std::shared_ptr<Lambertian> red = std::make_shared<Lambertian>(Color3d(.57, .0, .04));
    std::shared_ptr<Lambertian> blue = std::make_shared<Lambertian>(Color3d(.015, .015, .7));
    std::shared_ptr<Lambertian> grey = std::make_shared<Lambertian>(Color3d(.15, .15, .15));
    std::shared_ptr<Lambertian> black = std::make_shared<Lambertian>(Color3d(.05, .05, .05));
    std::shared_ptr<Lambertian> white = std::make_shared<Lambertian>(Color3d(.95, .95, .95));
    std::shared_ptr<Dielectric> glass = std::make_shared<Dielectric>(0.90);
    std::shared_ptr<Dielectric> glass_refr = std::make_shared<Dielectric>(0.45);
    std::shared_ptr<DiffuseLight> light = std::make_shared<DiffuseLight>(Color3d(15, 15, 15));
    // Add objects

    // Walls
    objects.add(std::make_shared<YZRect>(0, 555, 0, 555, 555, olive));
    objects.add(std::make_shared<FlipFace>(std::make_shared<XZRect>(0, 555, 0, 555, 1000, light)));
    objects.add(std::make_shared<FlipFace>(std::make_shared<YZRect>(0, 555, 0, 555, -1000, light)));
    objects.add(std::make_shared<FlipFace>(std::make_shared<XYRect>(0, 555, 0, 555, -1000, light)));
    objects.add(std::make_shared<XZRect>(0, 555, 0, 555, 0, red));
    objects.add(std::make_shared<XYRect>(0, 555, 0, 555, 555, olive));

    // Bed
    std::shared_ptr<Hittable> bed_bottom_box = std::make_shared<Box>(Point3d(0,0,0), Point3d(160,25,300), grey);
    bed_bottom_box = std::make_shared<Translate>(bed_bottom_box, Vec3d(555-170+10,0,555-310+10));
    objects.add(bed_bottom_box);

    std::shared_ptr<Hittable> bed_top_box = std::make_shared<Box>(Point3d(0,0,0), Point3d(170,10,310), grey);
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
    std::shared_ptr<Hittable> table_shelf = std::make_shared<Box>(Point3d(0, 0, 0), Point3d(80,5,80), grey);
    auto table_top_shelf = std::make_shared<Translate>(table_shelf, Vec3d(0,80,0));
    auto table_bot_shelf = std::make_shared<Translate>(table_shelf, Vec3d(0,40,0));

    std::shared_ptr<HittableList> table = std::make_shared<HittableList>(table_top_shelf);
    table->add(table_bot_shelf);

    std::shared_ptr<Hittable> table_leg = std::make_shared<Box>(Point3d(0, 0, 0), Point3d(10,80,10), grey);
    auto leg_1 = std::make_shared<Translate>(table_leg, Vec3d(0,0,0));
    auto leg_2 = std::make_shared<Translate>(table_leg, Vec3d(70,0,0));
    auto leg_3 = std::make_shared<Translate>(table_leg, Vec3d(0,0,70));
    auto leg_4 = std::make_shared<Translate>(table_leg, Vec3d(70,0,70));
    table->add(leg_1);
    table->add(leg_2);
    table->add(leg_3);
    table->add(leg_4);

    auto table_translated = std::make_shared<Translate>(table, Vec3d(555-250, 0, 555-75));

    objects.add(table_translated);

    // Dresser
    std::shared_ptr<Hittable> dresser_block = std::make_shared<Box>(Point3d(0,0,0), Point3d(130, 90, 80), grey);
    std::shared_ptr<Hittable> dresser_drawer = std::make_shared<Box>(Point3d(0,0,0), Point3d(120, 35, 40), black);
    auto drawer_1 = std::make_shared<Translate>(dresser_drawer, Vec3d(5, 50, -5));
    auto drawer_2 = std::make_shared<Translate>(dresser_drawer, Vec3d(5, 5, -5));

    std::shared_ptr<HittableList> dresser = std::make_shared<HittableList>(dresser_block);
    dresser->add(drawer_1);
    dresser->add(drawer_2);

    auto dresser_translated = std::make_shared<Translate>(dresser, Vec3d(555-250-85-50, 0, 555-75));
    objects.add(dresser_translated);

    // Lamp

    std::shared_ptr<Hittable> lamp_shade = std::make_shared<Box>(Point3d(0,0,0), Point3d(35, 30, 35), blue);
    std::shared_ptr<Hittable> lamp_pole = std::make_shared<Box>(Point3d(0,0,0), Point3d(5,35,5), grey);
    std::shared_ptr<Hittable> lamp_base = std::make_shared<Box>(Point3d(0,0,0), Point3d(30,5,30), black);
    lamp_base = std::make_shared<Translate>(lamp_base, Vec3d(-10, -5, -10));
    lamp_shade = std::make_shared<Translate>(lamp_shade, Vec3d(-5, 35, -5));
    lamp_pole = std::make_shared<Translate>(lamp_pole, Vec3d(0, 5, 0));

    std::shared_ptr<HittableList> lamp = std::make_shared<HittableList>(lamp_base);
    lamp->add(lamp_shade);
    lamp->add(lamp_pole);

    auto lamp_translated = std::make_shared<Translate>(lamp, Vec3d(555-250-85-50+25, 95, 555-75+20));

    objects.add(lamp_translated);

    // Fan
    std::shared_ptr<Hittable> fan_pole = std::make_shared<Box>(Point3d(0,0,0), Point3d(10,120,10), black);
    std::shared_ptr<Hittable> fan_base = std::make_shared<Box>(Point3d(0,0,0), Point3d(50,5,50), black);
    std::shared_ptr<Hittable> fan_blade1 = std::make_shared<Box>(Point3d(0,0,0), Point3d(5,15,90), grey);
    std::shared_ptr<Hittable> fan_blade2 = std::make_shared<Box>(Point3d(0,0,0), Point3d(5,90,15), grey);
    fan_pole = std::make_shared<Translate>(fan_pole, Vec3d(-25, 25, -10));
    fan_blade1 = std::make_shared<Translate>(fan_blade1, Vec3d(-25, 120, -45));
    fan_blade2 = std::make_shared<Translate>(fan_blade2, Vec3d(-25, 85, -10));


    std::shared_ptr<HittableList> fan = std::make_shared<HittableList>(fan_base);
    fan->add(fan_pole);
    fan->add(fan_blade1);
    fan->add(fan_blade2);

    auto fan_translated = std::make_shared<Translate>(fan, Vec3d(555-75, 0, 50));

    objects.add(fan_translated);

    std::shared_ptr<Hittable> glass_sphere = std::make_shared<Sphere>(Point3d(250, 250, 250), 100, glass);
    objects.add(glass_sphere);

    std::shared_ptr<Hittable> glass_box = std::make_shared<Box>(Point3d(555-250-85, 0, 555-150), Point3d(555-250-85-150, 180, 555-100), glass_refr);
    objects.add(glass_box);

    HittableList bvh_scene;
    bvh_scene.add(std::make_shared<BVHNode>(objects, 0, 1.0));
    return objects;
}

HittableList get_scene(int scene_id, SceneConfig &config)
{
    switch(scene_id){
    case 1:
        config.aspect_ratio = 1.0;
        config.width = 600;
        config.height = static_cast<int>(config.width / config.aspect_ratio);
        config.spp = 3000;
        config.max_depth = 50;
        config.lookfrom = Point3d(-600, 578, -500);
        config.lookat = Point3d(555, 000, 555);
        config.up = Point3d(0, 1, 0);
        config.fov = 45.0;
        config.background = Color3d(0.0, 0.0, 0.0);
        return proj3_scene();
        break;
    default:
        config.aspect_ratio = 1.0;
        config.width = 600;
        config.height = static_cast<int>(config.width / config.aspect_ratio);
        config.spp = 50;
        config.max_depth = 50;
        config.lookfrom = Point3d(278, 278, -800);
        config.lookat = Point3d(278, 278, 0);
        config.up = Point3d(0, 1, 0);
        config.fov = 45.0;
        config.background = Color3d(0.0, 0.0, 0.0);
        return default_scene();
    }
}

int main(int argc, char** argv)
{
    SceneConfig config;
    int scene = atoi(argv[1]);
    const int NUM_CHANNELS = 3;

    // World
    HittableList world;
    world = get_scene(scene, config);

    // Camera
    Camera cam(config);

    // Rendering!
    uint8_t* pixels = new uint8_t[config.width*config.height*NUM_CHANNELS];
    int index = 0;

    for (int j = config.height-1; j >= 0; --j)
    {
        int percent_left = static_cast<int>(100*((float)j/(float)config.height));
        std::cerr << "\rScanlines Remaining: " << j << " | " << percent_left << "\% left" << " | "<< std::flush;
        for (int i = 0; i < config.width; ++i)
        {
            Color3d pixel_color(0,0,0);
            for (int s = 0; s < config.spp; ++s)
            {
                double u = (i + helper::random_double()) / (config.width-1);
                double v = (j + helper::random_double()) / (config.height-1);
                Ray r = cam.GetRay(u, v);
                pixel_color += ray_color(r, config.background, world, config.max_depth);
            }
            write_color(pixels, pixel_color, index, config.spp);
        }
    }

    stbi_write_jpg("out.jpg", config.width, config.height, NUM_CHANNELS, pixels, 100);
    std::cerr << "\nDone.\n";

    delete[] pixels;
    return 0;
}