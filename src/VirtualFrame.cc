/*
 * ctrl_cartesian.cc
 *
 *  Created on: Jun 14, 2012
 *      Author: enrico
 */

#include "VirtualFrame.h"

namespace ctb {

VirtualFrame::VirtualFrame()
    : projectedOnPlane{ false }
{
    virtualFrameParams.gain.setZero();
    virtualFrameParams.sampleTime = 0.0;
    virtualFrameParams.onTrackAllowedDistance = { 4.0, 4.5 };
    virtualFrameParams.crossTrackAllowedDistance = { 1.0, 1.5 };
    virtualFrameParams.vfType = VFType::FullPose;
}

VirtualFrame::~VirtualFrame() {}

void VirtualFrame::ResetState()
{
    worldF_T_virtualF_.setIdentity();
    worldF_T_virtualFInit_.setIdentity();
    controlFrameToVirtualFrameError_.setZero();
}

void VirtualFrame::ResetState(const Eigen::TransfMatrix worldF_T_virtualF)
{
    worldF_T_virtualF_ = std::move(worldF_T_virtualF);
    worldF_T_virtualFInit_ = std::move(worldF_T_virtualF);
    controlFrameToVirtualFrameError_.setZero();
}

void VirtualFrame::Compute(const Eigen::TransfMatrix& worldF_T_startF, const Eigen::TransfMatrix worldF_T_goalF, Eigen::TransfMatrix& worldF_T_virtualF)
{
    worldF_T_goalF_ = std::move(worldF_T_goalF);

    if (virtualFrameParams.gain.norm() == 0.0 || virtualFrameParams.sampleTime == 0.0) {
        std::cerr << "WARNING: No virtualFrameGain and/or sampleTime set!!" << std::endl;
    }

    virtualFrameToGoalFrameError_ = rml::CartesianError(worldF_T_virtualF_, worldF_T_goalF_);

    if (projectedOnPlane) {
        Eigen::Vector3d worldF_planeDir = worldF_R_projectionF_.col(2);
        Eigen::Matrix3d P = (Eigen::Matrix3d::Identity() - worldF_planeDir * worldF_planeDir.transpose());
        virtualFrameToGoalFrameError_.SetSecondVect3(P * virtualFrameToGoalFrameError_.GetSecondVect3());
        virtualFrameToGoalFrameError_.SetFirstVect3(P * virtualFrameToGoalFrameError_.GetFirstVect3());
    }

    virtualFrameVelocity_ = { virtualFrameToGoalFrameError_.GetFirstVect3() * virtualFrameParams.gain[0], virtualFrameToGoalFrameError_.GetSecondVect3() * virtualFrameParams.gain[1] };

    Compute(worldF_T_startF, virtualFrameVelocity_, worldF_T_virtualF);
}

void VirtualFrame::Compute(const Eigen::TransfMatrix& worldF_T_controlF, const Eigen::Vector6d& xdotbar, Eigen::TransfMatrix& world_T_virtual)
{
    controlFrameToVirtualFrameError_ = rml::CartesianError(worldF_T_controlF, world_T_virtual);

    if (projectedOnPlane) {
        Eigen::Vector3d projectorVector_worldFrame = worldF_R_projectionF_.col(2);
        Eigen::Matrix3d P = (Eigen::Matrix3d::Identity() - projectorVector_worldFrame * projectorVector_worldFrame.transpose());
        controlFrameToVirtualFrameError_.SetSecondVect3(P * controlFrameToVirtualFrameError_.GetSecondVect3());
        controlFrameToVirtualFrameError_.SetFirstVect3(P * controlFrameToVirtualFrameError_.GetFirstVect3());
    }

    bool outOfReach = false;
    Eigen::Vector3d errorLinear, xdotbarLinear;

    if (virtualFrameParams.vfType != FullPose) {
        errorLinear = controlFrameToVirtualFrameError_.GetSecondVect3();
        xdotbarLinear = xdotbar.GetSecondVect3();
        Eigen::Vector3d n_vg = (worldF_T_goalF_.GetTransl() - worldF_T_virtualF_.GetTransl()).normalized();
        errorTrack_ = (n_vg * n_vg.transpose()) * errorLinear;
        errorCrossTrack_ = (Eigen::Matrix3d::Identity() - n_vg * n_vg.transpose()) * errorLinear;
        if ((errorLinear + xdotbarLinear * virtualFrameParams.sampleTime).norm() > errorLinear.norm()) {
            outOfReach = true;

            double sigma;
            double sigmaTrack = rml::DecreasingBellShapedFunction(virtualFrameParams.onTrackAllowedDistance(0), virtualFrameParams.onTrackAllowedDistance(1), 0, 1, errorTrack_.norm());
            double sigmaCross = rml::DecreasingBellShapedFunction(virtualFrameParams.crossTrackAllowedDistance(0), virtualFrameParams.crossTrackAllowedDistance(1), 0, 1, errorCrossTrack_.norm());
            sigma = std::min(sigmaTrack, sigmaCross);
            virtualFrameVelocity_.SetSecondVect3(xdotbarLinear * sigma);
        }
    }
    if (!outOfReach) {
        virtualFrameVelocity_ = xdotbar;
    }
    // move the virtual frame
    worldF_T_virtualF_ = worldF_T_virtualF_.Integral(virtualFrameVelocity_, virtualFrameParams.sampleTime);
    worldF_T_goalFCurrent_ = worldF_T_virtualF_;
}
}
