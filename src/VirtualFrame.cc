/*
 * ctrl_cartesian.cc
 *
 *  Created on: Jun 14, 2012
 *      Author: enrico
 */

#include "VirtualFrame.h"

namespace ctb
{

VirtualFrame::VirtualFrame() :
		useErrorNorm(false)
{
	virtualFrameGain_ = 0;
	sampleTime_ = 0;


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
	virtualFrameToGoalError_ = rml::CartesianError(wTg, wTv_);


	if (useErrorNorm == true) {
		normalizedVirtualFrameToGoalError_ = virtualFrameToGoalError_;
		//std::cerr << "linErrNorm: " << virtualFrameToGoalError_.GetSecondVect3().norm() << std::endl;
		double linErrNorm = virtualFrameToGoalError_.GetSecondVect3().norm();
		if (linErrNorm != 0.0) {
			Eigen::Vector3d linErrNormalized = virtualFrameToGoalError_.GetSecondVect3() / virtualFrameToGoalError_.GetSecondVect3().norm();
			/// TODO: the linear error multiplier should be the linearErrorThreshold of the "CTRL::SingleArm" class
			///       multiplied by 2, to be formalized better
			normalizedVirtualFrameToGoalError_.SetSecondVect3(linErrNormalized * 0.01);
			virtualFrameVelocity_ = normalizedVirtualFrameToGoalError_ * virtualFrameGain_;
		}
	}else{
		virtualFrameVelocity_ = virtualFrameToGoalError_ * virtualFrameGain_;
	}

	Compute(wTt, virtualFrameVelocity_, wTv);
}

void VirtualFrame::Compute(const Eigen::TransfMatrix& wTt, const Eigen::Vector6d& xdotbar, Eigen::TransfMatrix& wTv)
{
	toolToVirtualFrameError_ = rml::CartesianError(wTv, wTt);

	//std::cout.precision(3);
	//std::cout << "toolToVfError: " << toolToVirtualFrameError_.GetSecondVect3().norm() << "\t";
	//std::cout << "VfToGoalError: " << virtualFrameToGoalError_.GetSecondVect3().norm() << std::endl;

	/// virtual frame is getting away from tool frame
	if ((toolToVirtualFrameError_ + xdotbar * sampleTime_).norm() > toolToVirtualFrameError_.norm()) {
		virtualFrameVelocity_ = xdotbar * rml::DecreasingBellShapedFunction(0.00, 0.02, 0, 1, toolToVirtualFrameError_.norm());
	} else {
		virtualFrameVelocity_ = xdotbar;
	}

	// move the virtual frame
	wTv_ = wTv_.Integral(virtualFrameVelocity_, sampleTime_);

	wTv = wTv_;
}


}
