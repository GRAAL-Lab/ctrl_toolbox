/*
 * ctrl_cartesian.cc
 *
 *  Created on: Jun 14, 2012
 *      Author: enrico
 */

#include "VirtualFrame.h"

namespace ctb {

VirtualFrame::VirtualFrame(VFType vft)
    : vftype_(vft)
    , useErrorNorm(false)
{

    virtualFrameGain_ = 0;
    sampleTime_ = 0;
    onTrackAllowedDistance_(0) = 4.0;
    onTrackAllowedDistance_(1) = 4.5;
    crossTrackAllowedDistance_(0) = 1.0;
    crossTrackAllowedDistance_(1) = 1.5;

}

void VirtualFrame::SetSampleTime(double sampleTime)
{
    sampleTime_ = sampleTime;
}

void VirtualFrame::SetGain(double gain)
{
    virtualFrameGain_ = gain;
}

void VirtualFrame::ResetState()
{
    toolToVirtualFrameError_.setZero();
}

void VirtualFrame::SetOnTrackAllowedDistance(Eigen::Vector2d onTrackAllowedDistance)
{
    onTrackAllowedDistance_ = onTrackAllowedDistance;
}

void VirtualFrame::SetCrossTrackAllowedDistance(Eigen::Vector2d crossTrackAllowedDistance)
{
    crossTrackAllowedDistance_ = crossTrackAllowedDistance;
}

void VirtualFrame::ResetState(const Eigen::TransfMatrix& wTv)
{
    wTv_ = wTv;
    toolToVirtualFrameError_.setZero();
}

void VirtualFrame::Compute(const Eigen::TransfMatrix& wTt, const Eigen::TransfMatrix& wTg, Eigen::TransfMatrix& wTv)
{

    wTg_ = wTg;
    if (virtualFrameGain_ == 0.0 || sampleTime_ == 0.0) {
        std::cerr << "WARNING: No virtualFrameGain and/or sampleTime set!!" << std::endl;
    }

    virtualFrameToGoalError_ = rml::CartesianError(wTv_, wTg);


    if (useErrorNorm == true) {
        normalizedVirtualFrameToGoalError_ = virtualFrameToGoalError_;
        // std::cerr << "linErrNorm: " << virtualFrameToGoalError_.GetSecondVect3().norm() << std::endl;
        double linErrNorm = virtualFrameToGoalError_.GetSecondVect3().norm();
        if (linErrNorm != 0.0) {
            Eigen::Vector3d linErrNormalized = virtualFrameToGoalError_.GetSecondVect3() / virtualFrameToGoalError_.GetSecondVect3().norm();
            /// TODO: the linear error multiplier should be the linearErrorThreshold of the "CTRL::SingleArm" class
            ///       multiplied by 2, to be formalized better
            normalizedVirtualFrameToGoalError_.SetSecondVect3(linErrNormalized * 0.01);
            virtualFrameVelocity_ = normalizedVirtualFrameToGoalError_ * virtualFrameGain_;
        }
    } else {
        virtualFrameVelocity_ = virtualFrameToGoalError_ * virtualFrameGain_;
    }

    Compute(wTt, virtualFrameVelocity_, wTv);
}

void VirtualFrame::Compute(const Eigen::TransfMatrix& wTt, const Eigen::Vector6d& xdotbar, Eigen::TransfMatrix& wTv)
{
    toolToVirtualFrameError_ = rml::CartesianError(wTt, wTv);

    bool outOfReach = false;
    Eigen::Vector3d errorLinear, xdotbarLinear;
    if (vftype_ == Linear || vftype_ == FullPose) {
        errorLinear = toolToVirtualFrameError_.GetSecondVect3();
        xdotbarLinear = xdotbar.GetSecondVect3();
        if ((errorLinear + xdotbarLinear * sampleTime_).norm() > errorLinear.norm()) {
            outOfReach = true;
            Eigen::Vector3d n_vg = (wTg_.GetTransl() - wTv.GetTransl()).normalized();
            Eigen::Vector3d error_track = (n_vg * n_vg.transpose()) * errorLinear;
            Eigen::Vector3d error_cross = (Eigen::Matrix3d::Identity() - n_vg * n_vg.transpose()) * errorLinear;;
            double sigma;
            double sigmaTrack = rml::DecreasingBellShapedFunction(onTrackAllowedDistance_(0), onTrackAllowedDistance_(1), 0, 1, error_track.norm());
            double sigmaCross = rml::DecreasingBellShapedFunction(crossTrackAllowedDistance_(0), onTrackAllowedDistance_(1), 0, 1, error_cross.norm());
            sigma = std::min(sigmaTrack, sigmaCross);
            virtualFrameVelocity_.SetSecondVect3(xdotbarLinear * sigma);
        }
    }
   if (!outOfReach) {
        virtualFrameVelocity_ = xdotbar;
    }
    // move the virtual frame
    wTv_ = wTv_.Integral(virtualFrameVelocity_, sampleTime_);
    wTv = wTv_;
}
}
