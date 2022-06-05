#include "geometry.h"
#include <math.h>

void convertSphericalToCartesian(float * pointSpherical, float * pointCartesian) {
    // pointSpherical = [radius r, inclination phi, azimuth theta]
    // pointCartesian = [x, y, z]
    pointCartesian[0] = pointSpherical[0] * cos(pointSpherical[2] * sin(pointSpherical[1]));
    pointCartesian[1] = pointSpherical[0] * sin(pointSpherical[2] * sin(pointSpherical[1]));
    pointCartesian[2] = pointSpherical[0] * cos(pointSpherical[1]);
}