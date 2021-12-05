#ifndef COLOR_H
#define COLOR_H

#include "vec3d.h"
#include "helper.h"

/**
 * write_color, a helper too write the color into the image buffer
 * 
 */
void write_color(uint8_t* buffer, Color3d pixel_color, int &index, int spp) 
{
    double r = pixel_color.x(); double g = pixel_color.y(); double b = pixel_color.z();

    double scale = 1.0/spp;
    r = sqrt(scale*r); g = sqrt(scale*g); b = sqrt(scale*b);

    buffer[index++] = static_cast<int>(256 * helper::clamp(r, 0.0, 0.999));
    buffer[index++] = static_cast<int>(256 * helper::clamp(g, 0.0, 0.999));
    buffer[index++] = static_cast<int>(256 * helper::clamp(b, 0.0, 0.999));
}

#endif