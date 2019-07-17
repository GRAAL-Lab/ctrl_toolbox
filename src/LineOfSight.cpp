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
    if (computeTrajectoryProjector_ == OnPlane) {
        Eigen::Vector3d projectorVector_worldFrame = wRp_.col(2);
        Eigen::Matrix3d P_OnPlane
            = (Eigen::Matrix3d::Identity() - projectorVector_worldFrame * projectorVector_worldFrame.transpose());
        trajectory_ = P_OnPlane * trajectory_;
    }
    Eigen::Vector3d normTrajectory = trajectory_.normalized();

    // Computing closest point to trajectory
    Eigen::MatrixXd P = normTrajectory * normTrajectory.transpose();
    Eigen::Vector3d closestPoint = P * wTt.GetTransl();

    // Incrementing closest point
    Eigen::Vector3d increment = normTrajectory * delta_;
    Eigen::Vector3d newPosition = closestPoint + increment;

    // Definition of new goal
    wTv.SetTransl(newPosition);
}

void LineOfSight::ResetState(const Eigen::TransfMatrix& wTv) { wTvInitial_ = wTv; }
}
