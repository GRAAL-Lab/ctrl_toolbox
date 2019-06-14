#ifndef MODEL_KALMAN_FILTER_H_
#define MODEL_KALMAN_FILTER__H_

#include <eigen3/Eigen/Dense>

namespace ctb {
class ModelKalmanFilter {
public:
    ModelKalmanFilter();
    virtual ~ModelKalmanFilter();
    virtual Eigen::MatrixXd ComputeF(const Eigen::VectorXd state, const Eigen::VectorXd input) = 0;
    virtual Eigen::MatrixXd ComputeW(const Eigen::VectorXd state, const Eigen::VectorXd input) = 0;
    virtual Eigen::MatrixXd GetCovariance() = 0 ;

};
}

#endif
