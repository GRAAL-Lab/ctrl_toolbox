/*
 * ctrl_cartesian.cc
 *
 *  Created on: Jun 14, 2012
 *      Author: enrico
 */

#include "VirtualFrame.h"

namespace ctb {

VirtualFrame::VirtualFrame(VFType vft, ComputeTrajectoryProjector vfprojector)
    : ComputeTrajectory(vfprojector)
    , vftype_{ vft }
    , useErrorNorm_{ false }
{
    virtualFrameGain_ = 0;
    sampleTime_ = 0;
    onTrackAllowedDistance_.resize(2);
    crossTrackAllowedDistance_.resize(2);
    onTrackAllowedDistance_ = { 4.0, 4.5 };
    crossTrackAllowedDistance_ = { 1.0, 1.5 };
}

void VirtualFrame::ResetState() { startFrameToVirtualFrameError_.setZero(); }

void VirtualFrame::ResetState(const Eigen::TransfMatrix worldF_T_virtualF)
{
    worldF_T_virtualF_ = std::move(worldF_T_virtualF);
    worldF_T_virtualFInit_ = std::move(worldF_T_virtualF);
    startFrameToVirtualFrameError_.setZero();
}

void VirtualFrame::Compute(const Eigen::TransfMatrix& worldF_T_startF, const Eigen::TransfMatrix worldF_T_goalF, Eigen::TransfMatrix& worldF_T_virtualF)
{
    worldF_T_goalF_ = std::move(worldF_T_goalF);
    if (virtualFrameGain_ == 0.0 || sampleTime_ == 0.0) {
        std::cerr << "WARNING: No virtualFrameGain and/or sampleTime set!!" << std::endl;
    }

    virtualFrameToGoalError_ = rml::CartesianError(worldF_T_virtualF, worldF_T_goalF_);
    if (computeTrajectoryProjector_ == OnPlane) {
        Eigen::Vector3d projectorVector_worldFrame = worldF_R_projectionF_.col(2);
        Eigen::Matrix3d P = (Eigen::Matrix3d::Identity() - projectorVector_worldFrame * projectorVector_worldFrame.transpose());
        virtualFrameToGoalError_.SetSecondVect3(P * virtualFrameToGoalError_.GetSecondVect3());
        virtualFrameToGoalError_.SetFirstVect3(P * virtualFrameToGoalError_.GetFirstVect3());
    }

    if (useErrorNorm_ == true) {
        Eigen::Vector6d normalizedVirtualFrameToGoalError = virtualFrameToGoalError_;
        double linErrNorm = virtualFrameToGoalError_.GetSecondVect3().norm();
        if (linErrNorm != 0.0) {
            Eigen::Vector3d linErrNormalized = virtualFrameToGoalError_.GetSecondVect3() / virtualFrameToGoalError_.GetSecondVect3().norm();
            /// TODO: the linear error multiplier should be the linearErrorThreshold of the "CTRL::SingleArm" class
            ///       multiplied by 2, to be formalized better
            normalizedVirtualFrameToGoalError.SetSecondVect3(linErrNormalized * 0.01);
            virtualFrameVelocity_ = normalizedVirtualFrameToGoalError * virtualFrameGain_;
        }
    } else {
        virtualFrameVelocity_ = virtualFrameToGoalError_ * virtualFrameGain_;
    }

    Compute(worldF_T_startF, virtualFrameVelocity_, worldF_T_virtualF);
}

void VirtualFrame::Compute(const Eigen::TransfMatrix& frameID_T_e, const Eigen::Vector6d& xdotbar, Eigen::TransfMatrix& world_T_virtual)
{
    startFrameToVirtualFrameError_ = rml::CartesianError(frameID_T_e, world_T_virtual);
    if (computeTrajectoryProjector_ == OnPlane) {
        Eigen::Vector3d projectorVector_worldFrame = worldF_R_projectionF_.col(2);
        Eigen::Matrix3d P = (Eigen::Matrix3d::Identity() - projectorVector_worldFrame * projectorVector_worldFrame.transpose());
        startFrameToVirtualFrameError_.SetSecondVect3(P * startFrameToVirtualFrameError_.GetSecondVect3());
        startFrameToVirtualFrameError_.SetFirstVect3(P * startFrameToVirtualFrameError_.GetFirstVect3());
    }

    bool outOfReach = false;
    Eigen::Vector3d errorLinear, xdotbarLinear;
    if (vftype_ == Linear || vftype_ == FullPose) {
        errorLinear = startFrameToVirtualFrameError_.GetSecondVect3();
        xdotbarLinear = xdotbar.GetSecondVect3();
        Eigen::Vector3d n_vg = (worldF_T_goalF_.GetTransl() - worldF_T_virtualF_.GetTransl()).normalized();
        errorTrack_ = (n_vg * n_vg.transpose()) * errorLinear;
        errorCross_ = (Eigen::Matrix3d::Identity() - n_vg * n_vg.transpose()) * errorLinear;
        if ((errorLinear + xdotbarLinear * sampleTime_).norm() > errorLinear.norm()) {
            outOfReach = true;

            double sigma;
            double sigmaTrack = rml::DecreasingBellShapedFunction(
                onTrackAllowedDistance_(0), onTrackAllowedDistance_(1), 0, 1, errorTrack_.norm());
            double sigmaCross = rml::DecreasingBellShapedFunction(
                crossTrackAllowedDistance_(0), crossTrackAllowedDistance_(1), 0, 1, errorCross_.norm());
            sigma = std::min(sigmaTrack, sigmaCross);
            virtualFrameVelocity_.SetSecondVect3(xdotbarLinear * sigma);
        }
    }
    if (!outOfReach) {
        virtualFrameVelocity_ = xdotbar;
    }
    // move the virtual frame
    worldF_T_virtualF_ = worldF_T_virtualF_.Integral(virtualFrameVelocity_, sampleTime_);
    worldF_T_goalFCurrent_ = worldF_T_virtualF_;
}
}
