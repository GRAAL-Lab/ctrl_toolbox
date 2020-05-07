#ifndef CTRL_TOOLBOX_HELPERFUNCTIONS_H
#define CTRL_TOOLBOX_HELPERFUNCTIONS_H

#include "GeographicLib/Geodesic.hpp"
#include "ctrl_toolbox/DataStructs.h"
#include <GeographicLib/LocalCartesian.hpp>
#include <cmath>
#include <exception>
#include <libconfig.h++>

namespace ctb {

/**
 * @brief SetParam functor
 *
 * An utility templated functor to set a pram from ConfigFIle
 */
template <typename A>
void SetParam(const libconfig::Setting& confObj, A& param, const std::string& name)
{
    try {
        confObj.lookupValue(name, param);
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No " << name << " setting in configuration file." << std::endl;
    }
}

template <typename A>
void SetParam(const libconfig::Config& confObj, A& param, const std::string& name)
{
    try {
        confObj.lookupValue(name, param);
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No " << name << " setting in configuration file." << std::endl;
    }
}

/**
 * @brief SetParam functor
 *
 * An utility templated functor to set a vector pram from ConfigFIle
 */
template <typename A>
void SetParamVector(const libconfig::Setting& confObj, A& param, const std::string& name)
{
    try {
        const libconfig::Setting& settings = confObj.lookup(name);
        param.resize(settings.getLength());
        for (int n = 0; n < settings.getLength(); n++) {

            param(n) = settings[n];
        }
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No " << name << " setting in configuration file." << std::endl;
    }
}

template <typename A>
void SetParamVector(const libconfig::Config& confObj, A& param, const std::string& name)
{
    try {
        const libconfig::Setting& settings = confObj.lookup(name);
        param.resize(settings.getLength());
        for (int n = 0; n < settings.getLength(); n++) {

            param(n) = settings[n];
        }
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No " << name << " setting in configuration file." << std::endl;
    }
}

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

Eigen::Vector2d LatLong2mCoeff(LatLong LatLong);

template <class A>
void Cartesian2MapPoint(const A& cartesianPoint, const LatLong& centroid, LatLong& mapPoint)
{
    try {
        const GeographicLib::Geocentric& earth = GeographicLib::Geocentric::WGS84();

        GeographicLib::LocalCartesian proj(centroid.latitude, centroid.longitude, 0, earth);
        {
            double h;
            proj.Reverse(cartesianPoint[0], cartesianPoint[1], 0.0, mapPoint.latitude, mapPoint.longitude, h);
        }
    } catch (const std::exception& e) {
        std::cerr << "Caught exception: " << e.what() << "\n";
    }
}

template <class A>
void Map2CartesianPoint(const LatLong& mapPoint, const LatLong& centroid, A& cartesianPoint)
{
    try {
        const GeographicLib::Geocentric& earth = GeographicLib::Geocentric::WGS84();

        GeographicLib::LocalCartesian proj(centroid.latitude, centroid.longitude, 0, earth);
        {
            proj.Forward(mapPoint.latitude, mapPoint.longitude, 0, cartesianPoint[0], cartesianPoint[1], cartesianPoint[2]);
        }
    } catch (const std::exception& e) {
        std::cerr << "Caught exception: " << e.what() << "\n";
    }
}
}

#endif // CTRL_TOOLBOX_HELPERFUNCTIONS_H
