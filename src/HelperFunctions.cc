#include <iostream>

#include "ctrl_toolbox/HelperFunctions.h"

namespace ctb {

double clamp(double n, double lower, double upper)
{
    return std::max(lower, std::min(n, upper));
}


Eigen::VectorXd FilterAngularJump(const Eigen::VectorXd primaryHeading, const Eigen::VectorXd otherHeading)
{

    Eigen::VectorXd out;
    out.resize(primaryHeading.size());
    for (int i = 0; i < primaryHeading.size(); i++) {
        double diff = primaryHeading(i) - otherHeading(i);

        out(i) = otherHeading(i);

        if (diff > M_PI)
            out(i) += 2.0 * M_PI;
        else {
            if (diff < -M_PI) {
                out(i) -= 2.0 * M_PI;
            }
        }
    }

    return out;
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
}
