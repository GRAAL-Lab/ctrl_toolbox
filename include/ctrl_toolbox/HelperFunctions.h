#ifndef CTRL_TOOLBOX_HELPERFUNCTIONS_H
#define CTRL_TOOLBOX_HELPERFUNCTIONS_H

#include "GeographicLib/Geodesic.hpp"
#include "ctrl_toolbox/DataStructs.h"
#include <cmath>
#include <libconfig.h++>

namespace ctb {

/**
 * @brief SetParam functor
 *
 * An utility templated functor to set a pram from ConfigFIle
 */
template <class A>
void SetParam(libconfig::Config& confObj, A& param, std::string name)
{
    try {
        param = confObj.lookup(name);
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No " << name << " setting in configuration file." << std::endl;
    }
}

/**
 * @brief SetParam functor
 *
 * An utility templated functor to set a vector pram from ConfigFIle
 */
template <class A>
void SetParamVector(libconfig::Config& confObj, A& param, std::string name)
{
    try {
        const libconfig::Setting& filter_settings = confObj.lookup(name);

        for (int n = 0; n < filter_settings.getLength(); n++) {

            param.at(n) = filter_settings[n];
        }
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No " << name << " setting in configuration file." << std::endl;
    }
}

/**
 * @brief Helper clip function (available in std:: from c++17)
 */
double clamp(double n, double lower, double upper);

double FilterAngularJump(const double primaryHeading, const double otherHeading);

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
void DistanceAndAzimuthRad(const LatLong& from, const LatLong& to, double& distance, double& azimuthrad);

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

/**
 * @brief Degree to radiant conversion function
 *
 */
double Deg2Rad(double deg);

/**
 * @brief Radiant to degree conversion function
 *
 */
double Rad2Deg(double rad);

LatLong LatLong2mCoeff(LatLong LatLong);

template <class A>
void Euclidian2MapPoint(A*& euclidianPoint, ctb::LatLong centroid, ctb::LatLong& mapPoint)
{
    LatLong LatLonM = LatLong2mCoeff(centroid);

    mapPoint.latitude = centroid.latitude - euclidianPoint[1] / LatLonM.latitude;
    mapPoint.longitude = centroid.longitude - euclidianPoint[0] / LatLonM.longitude;
}

template <class A>
void Map2EuclidianPoint(LatLong mapPoint, ctb::LatLong centroid, A*& euclidianPoint)
{
    LatLong LatLonM = ctb::LatLong2mCoeff(centroid);

    euclidianPoint[0] = (centroid.longitude - mapPoint.longitude) * LatLonM.longitude;
    euclidianPoint[1] = (centroid.latitude - mapPoint.latitude) * LatLonM.latitude;
    euclidianPoint[2] = 0;
}
}

#endif // CTRL_TOOLBOX_HELPERFUNCTIONS_H
