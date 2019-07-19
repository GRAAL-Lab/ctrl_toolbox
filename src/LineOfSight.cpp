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
    std::cout << "wTg = " << wTg.GetTransl().transpose() << std::endl;
    std::cout << "wTv_initial = " << wTvInitial_.GetTransl().transpose() << std::endl;
    std::cout << "wTt_current = " << wTt.GetTransl().transpose() << std::endl;
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
    Eigen::Vector3d closestPoint = P * wTt_traslation + wTt_initial_traslation;
    std::cout << "closestPoint = " << closestPoint.transpose() << std::endl;

    // Incrementing closest point
    double currentDelta;
    double DistanceCurrentPointGoal = (wTg.GetTransl()-closestPoint).norm();
    if (DistanceCurrentPointGoal>delta_){
        currentDelta = delta_;
    }
    else{
        currentDelta = DistanceCurrentPointGoal;
    }
    Eigen::Vector3d increment = normTrajectory * currentDelta;
    Eigen::Vector3d newPosition = closestPoint + increment;

    // problem if negative not working! TODO
    //for (int i = 0; i < 3; i++) {
    //    if (wTg.GetTransl()(i) > 0.0) {
    //        rml::SaturateScalar(wTg.GetTransl()(i), newPosition(i));
    //    }
    //    else {
    //        if(newPosition(i)<0.0){
    //            double newPositionAbsoluteValue = std::fabs(newPosition(i));
    //            rml::SaturateScalar(std::fabs(wTg.GetTransl()(i)),newPositionAbsoluteValue);
    //            newPosition(i) = - newPositionAbsoluteValue;
    //        }
    //    }
    //}
    std::cout << "increment = " << increment.transpose() << std::endl;
    std::cout << "newPosition = " << newPosition.transpose() << std::endl;

    // Definition of new goal
    wTv.SetTransl(newPosition);
}

void LineOfSight::ResetState(const Eigen::TransfMatrix& wTv) { wTvInitial_ = wTv; }
}
