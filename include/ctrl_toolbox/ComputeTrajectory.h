#ifndef COMPUTE_TRAJECTORY_H
#define COMPUTE_TRAJECTORY_H
#include <eigen3/Eigen/Dense>
#include <rml/RML.h>

namespace ctb {

enum ComputeTrajectoryProjector {
    Default,
    OnPlane,
};
class ComputeTrajectory {

public:
    /*
     * @brief Default constructor
     */
    ComputeTrajectory(ComputeTrajectoryProjector computeTrajectoryProjector = Default);
    /*
     * @brief  Default desconstructor
     */
    virtual ~ComputeTrajectory();
    /*
     * @brief Compute the new virtual frame position
     * The method updates the position of the virtual frame \<v\> on the basis of the goal frame <g>
     * The current tool position <t> is used to prevent the virtual frame <v> from getting too far away from <t>
     * @param[in] world_T_startF the frame of what we want to reach the goal
     * @param[in] worldF_T_goalF the current goal frame position
     * @param[out] worldF_T_virtualF the new virtual frame position
     */
    virtual void Compute(const Eigen::TransfMatrix& frameID_T_startF, const Eigen::TransfMatrix worldF_T_goalF, Eigen::TransfMatrix& worldF_T_virtualF) = 0;
    /*
     * @brief Method to reset the state
     */
    virtual void ResetState(const Eigen::TransfMatrix frameID_T_virtual) = 0;
    /*
     * @brief Set the integration sample time
     * Sets the integration sample time which is used by the Compute calls. This time should be the interval between two
     * consecutive calls to the Compute method, i.e. typically the sample time of the task itself, assuming the
     * virtual frame updated every run
     */
    auto SampleTime() -> double { return sampleTime_; }
    /*
    * @brief Set the virtual frame gain
    * The virtual frame will evolve from its initial position towards the goal frame. This gain sets the velocity of the convergence
    */
    auto Gain() -> double { return virtualFrameGain_; }
    /*
     * @brief Method setting the on track thresholds
     */
    auto OnTrackAllowedDistance() -> Eigen::Vector2d& { return onTrackAllowedDistance_; }
    /*
     * @brief Method setting the cross track thresholds
     */
    auto CrossTrackAllowedDistance() -> Eigen::Vector2d& { return crossTrackAllowedDistance_; }
    /*
     * @brief Method getting the track error
     */
    auto TrackError() -> Eigen::Vector3d& { return errorTrack_; }
    /*
     * @brief Method getting the cross track error
     */
    auto CrossTrackError() -> Eigen::Vector3d& { return errorCross_; }
    /*
     * @brief Method getting the current goal
     */
    auto GoalFrame() -> Eigen::TransfMatrix& { return worldF_T_goalFCurrent_; }
    /*
     * @brief Method getting the virtual goal
     */
    auto VirtualFrame() -> Eigen::TransfMatrix& { return worldF_T_virtualF_; }
    /**
     * @brief Method setting the projector rotation matrix, the normal to the plane must coincide with the z axis, the
     * transformation
     * matrix must be expressed wrt to the inertial frame.
     * @param wRp rotation matrix in between the world and the projector frame
     */
    auto Projector() -> Eigen::RotMatrix { return worldF_R_projectionF_; }

protected:
    double sampleTime_;
    double virtualFrameGain_;
    Eigen::Vector2d onTrackAllowedDistance_;
    Eigen::Vector2d crossTrackAllowedDistance_;
    Eigen::TransfMatrix worldF_T_virtualF_;
    Eigen::TransfMatrix worldF_T_goalF_;
    Eigen::Vector3d errorTrack_;
    Eigen::Vector3d errorCross_;
    Eigen::TransfMatrix worldF_T_virtualFInit_;
    Eigen::RotMatrix worldF_R_projectionF_;
    ComputeTrajectoryProjector computeTrajectoryProjector_;
    Eigen::TransfMatrix worldF_T_goalFCurrent_;
};
}

#endif
