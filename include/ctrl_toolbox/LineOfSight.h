#ifndef LINE_OF_SIGHT_H_
#define LINE_OF_SIGHT_H_
#include "ctrl_toolbox/ComputeTrajectory.h"
namespace ctb {
class LineOfSight : public ComputeTrajectory {
public:
    LineOfSight(ComputeTrajectoryProjector computeTrajectoryProjector = Default);
    ~LineOfSight();
    void Compute(const Eigen::TransfMatrix& wTt, const Eigen::TransfMatrix& wTg, Eigen::TransfMatrix& wTv) override;
    void ResetState(const Eigen::TransfMatrix& wTv) override;
    void SetDelta(double delta) { delta_ = delta; }
private:
    double delta_;
    Eigen::Vector3d trajectory_;
};
}

#endif
