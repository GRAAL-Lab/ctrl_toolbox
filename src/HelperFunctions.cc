#include <iostream>

#include "ctrl_toolbox/HelperFunctions.h"

namespace ctb {

double clamp(double n, double lower, double upper)
{
    return std::max(lower, std::min(n, upper));
}

double HeadingErrorRad(double from, double to)
{
    double oppositeHeading;
    double diffHeading;

    if (to < M_PI) {
        oppositeHeading = to + M_PI;

        if (from > oppositeHeading) {
            diffHeading = to + 2.0 * M_PI - from;
        } else {
            diffHeading = to - from;
        }
    } else {
        oppositeHeading = to - M_PI;

        if (from < oppositeHeading) {
            diffHeading = to - 2.0 * M_PI - from;
        } else {
            diffHeading = to - from;
        }
    }

    return diffHeading;
}

void DistanceAndAzimuthRad(const LatLong& from, const LatLong& to, double& distance, double& azimuthrad)
{
    // From some really weird reason not understable from the GeographicLib documentation, if the "finalHeading"
    // result variable is not passed to the "Inverse()" function, the result is almost always zero.
    double azimuthdeg(0.0), finalHeading(0.0);
    auto geod = GeographicLib::Geodesic::WGS84();
    geod.Inverse(from.latitude, from.longitude, to.latitude, to.longitude, distance, azimuthdeg, finalHeading);

    while (azimuthdeg < 0.0)
        azimuthdeg += 360.0;

    azimuthrad = azimuthdeg * M_PI / 180.0;
}
}
