/*
 * VirtualFrame.h
 *
 *  Created on: Apr 26, 2018
 *      Author: wanderfra
 */

#ifndef VIRTUAL_FRAME_H_
#define VIRTUAL_FRAME_H_

#include "ctrl_toolbox/ComputeTrajectory.h"
#include <rml/RML.h>
namespace ctb {

/**
 * @brief A simple virtual frame for smooth control
 *
 * The virtual frame is a simple concept: instead of feeding a new goal position <g> directly to the  frame <s> of
 * the robot
 * a virtual frame is used to avoid discontinuities. The virtual frame is set initially coincident with the  frame
 * <s>, then
 * an integration process brings the virtual frame towards the goal frame. This virtual frame is then fed as "goal" to
 * the  frame
 * which does not see any jumps in the goal position because they are filtered by the virtual frame.
 *
 * A mechanism is implemented to avoid that the virtual frame "runs away" too much from the  frame.
 */
class VirtualFrame : public ComputeTrajectory {

public:
    enum VFType {
        FullPose,
        Angular,
        Linear
    };
    /*
    * @brief Default constructor
    */
    VirtualFrame(VFType vft = FullPose, ComputeTrajectoryProjector vfprojector = Default);
    /*
     * @brief Default desconstructor
     */
    ~VirtualFrame() override {}
    /*
     * @brief Reset the virtual frame
     * Simply sets the virtual frame to the identity matrix and the error to zero
     */
    void ResetState();
    /*
     * @brief Reset the virtual frame to the given value
     * Sets the virtual frame to be the same as the given value
     * @param[in] worldF_T_virtualF the transformation matrix of the virtual frame w.r.t. the world frame
     */
    void ResetState(const Eigen::TransfMatrix worldF_T_virtualF) override;
    /*
     * @brief Compute the new virtual frame position
     * The method updates the position of the virtual frame \<v\> on the basis of the goal frame <g>
     * The current worldF_T_startF is used to prevent the virtual frame <v> from getting too far away from <e>
     * @param[in] worldF_T_startF the frame of what we want to reach the goal
     * @param[in] worldF_T_goalF the current goal frame position
     * @param[out] worldF_T_virtualF the new virtual frame position
     */
    void Compute(const Eigen::TransfMatrix& worldF_T_startF, const Eigen::TransfMatrix worldF_T_goalF, Eigen::TransfMatrix& worldF_T_virtualF) override;
    /*
     * @brief Compute the new virtual frame position
     * The method updates the position of the virtual frame <v> on the basis of the requested Cartesian velocity xdotbar
     * The current  position worldF_T_startF is used to prevent the virtual frame <v> from getting too far away from <s>
     * @param[in] worldF_T_startF the current  frame position
     * @param[in] xdotbar the requested Cartesian velocity
     * @param[out] world_T_virtual the new virtual frame position
     */
    void Compute(const Eigen::TransfMatrix& worldF_T_startF, const Eigen::Vector6d& xdotbar, Eigen::TransfMatrix& world_T_virtual);
    /*
     * @brief Set ErrorNorm mode
     */
    auto UseErrorNorm() -> bool { return useErrorNorm_; }
    /*
     * @brief Method getting the error between the goal and the virtual frame
     */
    auto VirtualFrameToGoalError() const -> const Eigen::Vector6d& { return virtualFrameToGoalError_; }
    /*
     * @brief Method getting the error between the virtual frame and the <e>
     */
    auto StartFrameToVirtualFrameError() const -> const Eigen::Vector6d& { return startFrameToVirtualFrameError_; }

private:
    Eigen::Vector6d startFrameToVirtualFrameError_;
    Eigen::Vector6d virtualFrameToGoalError_;
    Eigen::Vector6d virtualFrameVelocity_;
    VFType vftype_;
    bool useErrorNorm_;
};
}

#endif /* VIRTUAL_FRAME_H_ */
