#ifndef MEASURMENTKALMANFILTER_H
#define MEASURMENTKALMANFILTER_H

#include <eigen3/Eigen/Dense>

namespace ctb {
class MeasurementKalmanFilter {
public:
    MeasurementKalmanFilter(bool isAngleMeasure);

    virtual ~MeasurementKalmanFilter();

    virtual Eigen::MatrixXd ComputeJacobian(const Eigen::VectorXd state, const Eigen::VectorXd input) = 0; // H = der(h)/der(x)

    auto MeasureVector() const -> const Eigen::VectorXd& { return measure_; }

    virtual Eigen::VectorXd ComputeObservationModel(const Eigen::VectorXd state) = 0;

    auto Covariance() const -> const Eigen::MatrixXd& { return covariance_; }
    auto Covariance() -> Eigen::MatrixXd& { return covariance_; }

    auto IsAngleMeasure() const -> bool { return isAngleMeasure_; }

private:
    Eigen::VectorXd measure_;
    Eigen::MatrixXd covariance_;
    bool isAngleMeasure_;
};
}
#endif // MEASURMENTKALMANFILTER_H
