#ifndef COMPUTE_TRAJECTORY_H
#define COMPUTE_TRAJECTORY_H
#include <eigen3/Eigen/Dense>
#include <rml/RML.h>

namespace ctb {
class ComputeTrajectory {

public:
    ComputeTrajectory();
    virtual ~ComputeTrajectory();
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
    virtual void Compute(const Eigen::TransfMatrix& wTt, const Eigen::TransfMatrix& wTg, Eigen::TransfMatrix& wTv) = 0;

    virtual void ResetState(const Eigen::TransfMatrix& wTv) = 0;

    void SetSampleTime(double sampleTime);

    void SetOnTrackAllowedDistance(Eigen::VectorXd onTrackAllowedDistance);

    /**
     * @brief Method setting the cross track thresholds
     * @param crossTrackAllowedDistance cross track thresholds
     */
    void SetCrossTrackAllowedDistance(Eigen::VectorXd crossTrackAllowedDistance);

    Eigen::Vector3d GetOnTrackError() { return errorTrack_; }
    Eigen::Vector3d GetCrossTrackError() { return errorCross_; }

protected:
    double sampleTime_;
    double virtualFrameGain_;
    Eigen::VectorXd onTrackAllowedDistance_;
    Eigen::VectorXd crossTrackAllowedDistance_;
    Eigen::TransfMatrix wTv_;
    Eigen::TransfMatrix wTg_;
    Eigen::Vector3d errorTrack_;
    Eigen::Vector3d errorCross_;
    Eigen::TransfMatrix wTvInitial_;
};
}

#endif
