#ifndef MODEL_KALMAN_FILTER_H_
#define MODEL_KALMAN_FILTER_H_

#include <eigen3/Eigen/Dense>
#include <chrono>
namespace ctb {
class ModelKalmanFilter {
public:
    ModelKalmanFilter();
    virtual ~ModelKalmanFilter();

    virtual Eigen::VectorXd ComputeState(const Eigen::VectorXd state, const Eigen::VectorXd input) = 0;
    virtual Eigen::MatrixXd ComputeF(const Eigen::VectorXd state, const Eigen::VectorXd input) = 0;
    void SetCovariance(const Eigen::MatrixXd covariance);
    Eigen::MatrixXd GetCovariance();

protected:
    std::chrono::system_clock::time_point last_comp_time_;
    Eigen::MatrixXd covariance_;
};
}

#endif
