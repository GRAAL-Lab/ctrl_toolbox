#include "ctrl_toolbox/LineOfSight.h"

namespace ctb {
LineOfSight::LineOfSight(ComputeTrajectoryProjector computeTrajectoryProjector)
    : ComputeTrajectory(computeTrajectoryProjector)
{
}
LineOfSight::~LineOfSight() {}

void LineOfSight::Compute(const Eigen::TransfMatrix& wTt, const Eigen::TransfMatrix& wTg, Eigen::TransfMatrix& wTv)
{
    // Computing trajectory and norm to the trajectory
    trajectory_ = wTg.GetTransl() - wTvInitial_.GetTransl();
    Eigen::Vector3d wTt_traslation = wTt.GetTransl();
    Eigen::Vector3d wTt_initial_traslation = wTvInitial_.GetTransl();
    if (computeTrajectoryProjector_ == OnPlane) {
        Eigen::Vector3d projectorVector_worldFrame = wRp_.col(2);
        Eigen::Matrix3d P_OnPlane
            = (Eigen::Matrix3d::Identity() - projectorVector_worldFrame * projectorVector_worldFrame.transpose());
        trajectory_ = P_OnPlane * trajectory_;
        wTt_traslation = P_OnPlane * wTt_traslation;
        wTt_initial_traslation = P_OnPlane * wTt_initial_traslation;
    }
    Eigen::Vector3d normTrajectory = trajectory_.normalized();

    // Computing closest point to trajectory
    Eigen::MatrixXd P = normTrajectory * normTrajectory.transpose();
    Eigen::Vector3d initialT_t = wTt_traslation- wTt_initial_traslation;
    Eigen::Vector3d closestPoint = P * initialT_t + wTt_initial_traslation;


    // Incrementing closest point
    Eigen::Vector3d goalTraslation = wTg.GetTransl();
    if(computeTrajectoryProjector_ == OnPlane){
        Eigen::Vector3d projectorVector_worldFrame = wRp_.col(2);
        Eigen::Matrix3d P_OnPlane
            = (Eigen::Matrix3d::Identity() - projectorVector_worldFrame * projectorVector_worldFrame.transpose());
        goalTraslation =P_OnPlane*goalTraslation;
    }
    double DistanceCurrentPointGoal = (goalTraslation - closestPoint).norm();
    Eigen::Vector3d increment;
    Eigen::Vector3d newPosition;

    if (DistanceCurrentPointGoal > delta_) {
        increment = normTrajectory * delta_;
        newPosition = closestPoint + increment;
    } else {
        newPosition = wTg.GetTransl();
    }

    // Definition of new goal
    wTv.SetTransl(newPosition);
    wTgCurrent_ = wTv;
    Eigen::Vector3d errorLinear = wTv.GetTransl()-wTt.GetTransl();
    if (computeTrajectoryProjector_ == OnPlane) {
        Eigen::Vector3d projectorVector_worldFrame = wRp_.col(2);
        Eigen::Matrix3d P_OnPlane
            = (Eigen::Matrix3d::Identity() - projectorVector_worldFrame * projectorVector_worldFrame.transpose());
        errorLinear = P_OnPlane*errorLinear;
    }
    errorTrack_ = (normTrajectory * normTrajectory.transpose()) * (errorLinear);
    errorCross_ = (Eigen::Matrix3d::Identity() - normTrajectory * normTrajectory.transpose()) * errorLinear;
}

void LineOfSight::ResetState(const Eigen::TransfMatrix& wTv) { wTvInitial_ = wTv; }
}
