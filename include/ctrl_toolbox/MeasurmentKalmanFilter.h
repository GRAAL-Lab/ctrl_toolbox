#ifndef MEASURMENTKALMANFILTER_H
#define MEASURMENTKALMANFILTER_H

#include <eigen3/Eigen/Dense>

namespace ctb {
class MeasurmentKalmanFilter {
public:
    MeasurmentKalmanFilter();
    virtual ~MeasurmentKalmanFilter();
    virtual Eigen::MatrixXd ComputeG(const Eigen::VectorXd state, const Eigen::VectorXd input) = 0;
    Eigen::VectorXd GetMeasure();
    void SetMeasure(const Eigen::VectorXd measure);
    virtual Eigen::MatrixXd GetCovarianceMesure() = 0;

private:
    Eigen::VectorXd measure_;
};
}
#endif // MEASURMENTKALMANFILTER_H
