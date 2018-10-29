#include "ctrl_toolbox/HelperFunctions.h"

namespace ctb {

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

void DistanceAndAzimuthRad(LatLong from, LatLong to, double &distance, double &azimuthrad)
{
    GeographicLib::Geodesic geod(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
    geod.Inverse(from.latitude, from.longitude, to.latitude, to.longitude, distance, azimuthrad);

    azimuthrad = azimuthrad * M_PI / 180.0;
}

}
