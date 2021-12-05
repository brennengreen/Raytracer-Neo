#ifndef CONFIG_H
#define CONFIG_H
// SceneConfig struct used to pass information
// to the Camera object
struct SceneConfig  {
    double aspect_ratio;
    int width;
    int height;
    int spp;
    int max_depth;
    Point3d lookfrom;
    Point3d lookat;
    Point3d up;
    double fov;
    Color3d background;
};
#endif