#include "ModelKalmanFilter.h"

namespace ctb {

    ModelKalmanFilter::ModelKalmanFilter() {}
    ModelKalmanFilter::~ModelKalmanFilter() {}
    Eigen::MatrixXd ModelKalmanFilter::GetCovariance()
    {
        return covariance_;
    }
    void ModelKalmanFilter::SetCovariance(const Eigen::MatrixXd covariance )
    {
        covariance_ = covariance;
    }
}
