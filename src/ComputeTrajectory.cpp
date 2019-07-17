#include "ctrl_toolbox/ComputeTrajectory.h"
namespace ctb {

ComputeTrajectory::ComputeTrajectory(ComputeTrajectoryProjector computeTrajectoryProjector)
{
    computeTrajectoryProjector_ = computeTrajectoryProjector;
}

ComputeTrajectory::~ComputeTrajectory() {}

void ComputeTrajectory::SetSampleTime(double sampleTime) { sampleTime_ = sampleTime; }

void ComputeTrajectory::SetOnTrackAllowedDistance(Eigen::VectorXd onTrackAllowedDistance)
{
    if (onTrackAllowedDistance.size() != 2) {
        std::cerr << "[WARNING] On track allowed distance should be of size =2, using default values" << std::endl;
    } else {
        onTrackAllowedDistance_ = onTrackAllowedDistance;
    }
}

void ComputeTrajectory::SetCrossTrackAllowedDistance(Eigen::VectorXd crossTrackAllowedDistance)
{
    if (crossTrackAllowedDistance.size() != 2) {
        std::cerr << "[WARNING] Cross track allowed distance should be of size = 2, using defalut value" << std::endl;
    } else {
        crossTrackAllowedDistance_ = crossTrackAllowedDistance;
    }
}

void ComputeTrajectory::SetProjectorRotMatrix(const Eigen::RotMatrix wRp) { wRp_ = wRp; }
}
