#include "MeasurmentKalmanFilter.h"

namespace ctb {

MeasurmentKalmanFilter::MeasurmentKalmanFilter(bool isAngleMeasure) {
    isAngleMeasure_ = isAngleMeasure;
}
MeasurmentKalmanFilter::~MeasurmentKalmanFilter() {}
Eigen::VectorXd MeasurmentKalmanFilter::GetMeasure() { return measure_; }
void MeasurmentKalmanFilter::SetMeasure(const Eigen::VectorXd measure) { measure_ = measure; }
Eigen::MatrixXd MeasurmentKalmanFilter::GetCovarianceMesure() { return covariance_; }
void MeasurmentKalmanFilter::SetCovariance(const Eigen::MatrixXd covariance) { covariance_ = covariance; }
bool MeasurmentKalmanFilter::IsAngleMeasure(){
    return isAngleMeasure_;
}

}
