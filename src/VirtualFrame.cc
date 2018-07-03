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
    maximumAllowedDistance_ = 0.2;
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

void VirtualFrame::ResetState(const Eigen::TransfMatrix& wTv)
{
    wTv_ = wTv;
    toolToVirtualFrameError_.setZero();
}

void VirtualFrame::Compute(const Eigen::TransfMatrix& wTt, const Eigen::TransfMatrix& wTg, Eigen::TransfMatrix& wTv)
{
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

    //std::cout.precision(3);
    //std::cout << "toolToVfError: " << toolToVirtualFrameError_.GetSecondVect3().norm() << "\t";
    //std::cout << "VfToGoalError: " << virtualFrameToGoalError_.GetSecondVect3().norm() << std::endl;

    /// Virtual frame is getting away from tool frame

    bool outOfReach = false;
    Eigen::Vector3d error3d, xdotbar3d;

    if (vftype_ == FullPose) {
        if ((toolToVirtualFrameError_ + xdotbar * sampleTime_).norm() > toolToVirtualFrameError_.norm()) {
            outOfReach = true;
            virtualFrameVelocity_ = xdotbar * rml::DecreasingBellShapedFunction(0.00, maximumAllowedDistance_, 0, 1, toolToVirtualFrameError_.norm());
        }
    } else if (vftype_ == Angular) {
        error3d = toolToVirtualFrameError_.GetFirstVect3();
        xdotbar3d = xdotbar.GetFirstVect3();
        if ((error3d + xdotbar3d * sampleTime_).norm() > error3d.norm()) {
            outOfReach = true;
            //virtualFrameVelocity_.SetSecondVect3(Eigen::Vector3d(0, 0, 0));
            virtualFrameVelocity_.SetFirstVect3(xdotbar3d * rml::DecreasingBellShapedFunction(0.00, maximumAllowedDistance_, 0, 1, error3d.norm()));
        }
    } else if (vftype_ == Linear) {
        error3d = toolToVirtualFrameError_.GetSecondVect3();
        xdotbar3d = xdotbar.GetSecondVect3();
        if ((error3d + xdotbar3d * sampleTime_).norm() > error3d.norm()) {
            outOfReach = true;
            //virtualFrameVelocity_.SetFirstVect3(Eigen::Vector3d(0, 0, 0));
            virtualFrameVelocity_.SetSecondVect3(xdotbar3d * rml::DecreasingBellShapedFunction(0.00, maximumAllowedDistance_, 0, 1, error3d.norm()));
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
