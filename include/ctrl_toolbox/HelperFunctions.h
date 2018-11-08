#ifndef CTRL_TOOLBOX_HELPERFUNCTIONS_H
#define CTRL_TOOLBOX_HELPERFUNCTIONS_H

#include "GeographicLib/Geodesic.hpp"
#include "ctrl_toolbox/DataStructs.h"
#include <cmath>

namespace ctb {

/**
 * @brief Helper clip function (available in std:: from c++17)
 */
double clamp(double n, double lower, double upper);

/**
 * @brief HeadingErrorRad
 * @param from
 * @param to
 * @return
 */
double HeadingErrorRad(double from, double to);

/**
 * @brief DistanceAndAzimuthRad
 * @param from
 * @param to
 * @param distance
 * @param azimuthrad
 */
void DistanceAndAzimuthRad(const LatLong &from, const LatLong &to, double &distance, double &azimuthrad);

/**
 * @brief Difference functor
 *
 * An utility templated functor to compute the difference between two objects
 * of the same type.
 */
template <typename T>
struct DifferenceFunctor {
    T operator()(T a, T b) const
    {
        return (a - b);
    }
};

struct HeadingErrorRadFunctor {
    double operator()(double from, double to) const
    {
        return HeadingErrorRad(from, to);
    }
};

}

#endif // CTRL_TOOLBOX_HELPERFUNCTIONS_H
