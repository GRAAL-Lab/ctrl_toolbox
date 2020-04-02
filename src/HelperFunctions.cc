#include <iostream>

#include "ctrl_toolbox/HelperFunctions.h"

namespace ctb {

double clamp(double n, double lower, double upper)
{
    return std::max(lower, std::min(n, upper));
}

double HeadingErrorRad(double to, double from)
{
    //std::cout << " HeadingErrorRad(double from, double to)" << std::endl;
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

double Deg2Rad(double deg)
{
    return (M_PI / 180.0) * deg;
}

double Rad2Deg(double rad)
{
    return (180.0 / M_PI) * rad;
}

Eigen::Vector2d LatLong2mCoeff(LatLong LatLong)
{
    Eigen::Vector2d LatLongMCoeff;
    LatLong.latitude = Deg2Rad(LatLong.latitude);
    LatLong.longitude = Deg2Rad(LatLong.longitude);

    LatLongMCoeff[1] = 111132.92 - 559.82 * cos(2 * LatLong.latitude) + 1.175 * cos(4 * LatLong.latitude) - 0.0023 * cos(6 * LatLong.latitude);
    LatLongMCoeff[0] = 111412.84 * cos(LatLong.longitude) - 93.5 * cos(3 * LatLong.longitude) + 0.118 * cos(5 * LatLong.longitude);

    return LatLongMCoeff;
}
}
