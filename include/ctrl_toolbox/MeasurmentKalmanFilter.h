#ifndef MEASURMENTKALMANFILTER_H
#define MEASURMENTKALMANFILTER_H

#include <eigen3/Eigen/Dense>

namespace ctb {
class MeasurmentKalmanFilter {
public:
    MeasurmentKalmanFilter(bool isAngleMeasure);
    virtual ~MeasurmentKalmanFilter();
    virtual Eigen::MatrixXd ComputeG(const Eigen::VectorXd state, const Eigen::VectorXd input) = 0;
    Eigen::VectorXd GetMeasure();
    virtual Eigen::VectorXd GetPredictedMeasure(const Eigen::VectorXd state) = 0 ;
    void SetMeasure(const Eigen::VectorXd measure);
    void SetCovariance(const Eigen::MatrixXd);
    Eigen::MatrixXd GetCovarianceMesure() ;
    bool IsAngleMeasure();

private:
    Eigen::VectorXd measure_;
    Eigen::MatrixXd covariance_;
    bool isAngleMeasure_;
};
}
#endif // MEASURMENTKALMANFILTER_H
