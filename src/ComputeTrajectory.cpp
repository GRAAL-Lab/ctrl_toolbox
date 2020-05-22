#include "ctrl_toolbox/ComputeTrajectory.h"
namespace ctb {

ComputeTrajectory::ComputeTrajectory(ComputeTrajectoryProjector computeTrajectoryProjector)
    : computeTrajectoryProjector_{ computeTrajectoryProjector }
{
}

ComputeTrajectory::~ComputeTrajectory() {}
}
