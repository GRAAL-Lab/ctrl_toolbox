#ifndef CTRL_TOOLBOX_DATASTRUCTS_H
#define CTRL_TOOLBOX_DATASTRUCTS_H

#include <rml/RML.h>

namespace ctb {

struct PIDGains {
    double Kp = { 0.0 }; //!< Proportional gain
    double Ki = { 0.0 }; //!< Integral gain
    double Kd = { 0.0 }; //!< Derivative gain
    double Kff = { 0.0 }; //!< Feed-forward gain
    double N = { 0.0 }; //!< Maximum gain for derivative part
    double Tr = { 0.0 }; //!< Tracking time constant for anti-windup

    PIDGains() = default;
};

struct LatLong {
    double latitude, longitude;
    LatLong()
        : latitude(0.0)
        , longitude(0.0)
    {
    }
};

struct ADPGains {
    Eigen::Vector6d Kl = {}; //!< Pose error Proportional gain
    Eigen::Vector9d Kg = {}; //!< Dynamic parameters update law gain
    Eigen::Vector6d Ks = {}; //!< Velocity error Proportional gain
    Eigen::Vector9d gamma = {};
    Eigen::Vector3d upGB_m = {};
    Eigen::Vector3d lowGB_m = {};
    Eigen::Vector3d up_f = {};
    Eigen::Vector3d low_f = {};
    Eigen::Vector3d up_m = {};
    Eigen::Vector3d low_m = {};
    std::vector<bool> en_integration = {};


    //double Kff = { 0.0 }; //!< Feed-forward gain
    //double N = { 0.0 }; //!< Maximum gain for derivative part
    //double Tr = { 0.0 }; //!< Tracking time constant for anti-windup

    ADPGains() = default;
};

}

#endif // CTRL_TOOLBOX_DATASTRUCTS_H
