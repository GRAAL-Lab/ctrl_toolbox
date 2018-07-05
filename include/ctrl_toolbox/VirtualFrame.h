/*
 * VirtualFrame.h
 *
 *  Created on: Apr 26, 2018
 *      Author: wanderfra
 */

#ifndef VIRTUAL_FRAME_H_
#define VIRTUAL_FRAME_H_

#include <rml/RML.h>

namespace ctb {

/**
 * @brief A simple virtual frame for smooth control
 *
 * The virtual frame is a simple concept: instead of feeding a new goal position <g> directly to the tool frame <t> of the robot
 * a virtual frame is used to avoid discontinuities. The virtual frame is set initially coincident with the tool frame <t>, then
 * an integration process brings the virtual frame towards the goal frame. This virtual frame is then fed as "goal" to the tool frame
 * which does not see any jumps in the goal position because they are filtered by the virtual frame.
 *
 * A mechanism is implemented to avoid that the virtual frame "runs away" too much from the tool frame.
 */
class VirtualFrame {

public:
  enum VFType { FullPose, Angular, Linear };
  /**
	 * @brief Default constructor
	 */
    VirtualFrame(VFType vft = FullPose);

  /**
	 * @brief Set the integration sample time
	 * Sets the integration sample time which is used by the Compute calls. This time should be the interval between two
	 * consecutive calls to the Compute method, i.e. typically the sample time of the task itself, assuming the virtual frame
	 * updated every run
	 * @param[in] sampleTime integration sample time [s]
	 */
    void SetSampleTime(double sampleTime);

    /**
	 * @brief Set the virtual frame gain
	 * The virtual frame will evolve from its initial position towards the goal frame
	 * This gain sets the velocity of the convergence
	 * @param[in] gain the gain value
	 */
    void SetGain(double gain);

    /**
	 * @brief Reset the virtual frame
	 * Simply sets the virtual frame to the identity matrix and the error to zero
	 */
    void ResetState();

    /**
	 * @brief Reset the virtual frame to the given value
	 * Sets the virtual frame to be the same as the given value
	 * @param[in] wTv the transformation matrix of the virtual frame w.r.t. the world frame
	 */
    void ResetState(const Eigen::TransfMatrix& wTv);

    /**
	 * @brief Compute the new virtual frame position
	 *
	 * The method updates the position of the virtual frame \<v\> on the basis of the goal frame <g>
	 * The current tool position <t> is used to prevent the virtual frame <v> from getting too far away from <t>
	 *
	 * @param[in] wTt the current tool frame position
	 * @param[in] wTg the current goal frame position
	 * @param[out] wTv the new virtual frame position
	 */
    void Compute(const Eigen::TransfMatrix& wTt, const Eigen::TransfMatrix& wTg, Eigen::TransfMatrix& wTv);

    /**
	 * @brief Compute the new virtual frame position
	 * The method updates the position of the virtual frame <v> on the basis of the requested Cartesian velocity xdotbar
	 * The current tool position <t> is used to prevent the virtual frame <v> from getting too far away from <t>
	 * @param[in] wTt the current tool frame position
	 * @param[in] xdotbar the requested Cartesian velocity
	 * @param[out] wTv the new virtual frame position
	 */
    void Compute(const Eigen::TransfMatrix& wTt, const Eigen::Vector6d& xdotbar, Eigen::TransfMatrix& wTv);

    void SetUseErrorNorm(bool useErrorNorm)
    {
        this->useErrorNorm = useErrorNorm;
    }

    void SetMaximumAllowedDistance(double dist)
    {
        this->maximumAllowedDistance_ = dist;
    }

    const Eigen::Vector6d& getVirtualFrameToGoalError() const
    {
        return virtualFrameToGoalError_;
    }

    const Eigen::Vector6d& getToolToVirtualFrameError() const
    {
        return toolToVirtualFrameError_;
    }

    const Eigen::TransfMatrix& getwTvirt() const
    {
        return wTv_;
    }
private:
    double sampleTime_;
    double virtualFrameGain_;
    double maximumAllowedDistance_;
    Eigen::Vector6d toolToVirtualFrameError_;
    Eigen::Vector6d virtualFrameToGoalError_;
    Eigen::Vector6d normalizedVirtualFrameToGoalError_;
    Eigen::Vector6d virtualFrameVelocity_;
    Eigen::TransfMatrix wTv_;
    VFType vftype_;
    bool useErrorNorm;
};
}

#endif /* VIRTUAL_FRAME_H_ */
