#include "ExtendedKalmanFilter.h"
#include <rml/RML.h>

namespace ctb {

// F, note that f must depend on the state and the input and the dimension of the state, the indexAngles indicates
// which
// entries of the state are angles to be wrapped;
ExtendedKalmanFilter::ExtendedKalmanFilter(
    int stateDimension, std::vector<int> indexAngles, std::shared_ptr<ModelKalmanFilter> kalmanFilterModel)
{
    kalmanFilterModel_ = kalmanFilterModel;
    stateDimension_ = stateDimension;
    indexAngles_ = indexAngles;
    x_.setZero(stateDimension_);
    Sigma_.setZero(stateDimension_, stateDimension_);
    regularizationParameter_.params.lambda = 0.000001;
    regularizationParameter_.params.threshold = 0.000001;
    isFirst_ = true;
}

// Method that postpone to H the matrix output of the function, note that h is function of the stateitself
void ExtendedKalmanFilter::AddMeasurment(std::shared_ptr<MeasurmentKalmanFilter> h)
{
    if (isFirst_) {
        G_ = h->ComputeG(x_, u_);
        y_ = h->GetMeasure();
        ypredict_ = h->GetPredictedMeasure(x_);

        if (h->IsAngleMeasure()) {
            ypredict_  =FilterAngularJump(y_, ypredict_);
        }
        R_ = h->GetCovarianceMesure();
        isFirst_ = false;
    } else {

        int size_new_measure = h->GetMeasure().size();
        Eigen::MatrixXd zero_previous_covariance;
        zero_previous_covariance.setZero(size_new_measure, y_.size());
        Eigen::MatrixXd zero_new_covariance;
        zero_new_covariance.setZero(y_.size(), size_new_measure);
        G_ = rml::UnderJuxtapose(G_, h->ComputeG(x_, u_));
        Eigen::VectorXd yTemp = h->GetMeasure();
        Eigen::VectorXd ypredictTemp = h->GetPredictedMeasure(x_);
        if(h->IsAngleMeasure()){
            ypredictTemp = FilterAngularJump(yTemp, ypredictTemp);
        }
        y_ = rml::UnderJuxtapose(y_, yTemp);
        ypredict_ = rml::UnderJuxtapose(ypredict_, ypredictTemp);
        R_ = rml::UnderJuxtapose(R_, zero_previous_covariance);
        Eigen::MatrixXd R_temp = rml::UnderJuxtapose(zero_new_covariance, h->GetCovarianceMesure());
        R_ = rml::RightJuxtapose(R_, R_temp);
    }
}

void ExtendedKalmanFilter::Predict(Eigen::VectorXd u)
{

    u_ = u;
    F_ = kalmanFilterModel_->ComputeF(x_, u_);
    Q_ = kalmanFilterModel_->GetCovariance();

    // compute x(k|k-1)
    // x_ = F_ * x_ + W_ * u_; // compute via model
    x_ = kalmanFilterModel_->ComputeState(x_, u_);

    for (const auto i : indexAngles_) {

        NormalizeAngle(x_(i));
    }

    // compute Sigma(k|k-1)
    Sigma_ = F_ * Sigma_ * F_.transpose() + Q_;
}

void ExtendedKalmanFilter::ApplyMeasurements()
{

    if (!isFirst_) {

        // update of the covariance of the measures
        S_ = G_ * Sigma_ * G_.transpose() + R_;

        // update of the kalman filter gain
        K_ = Sigma_ * G_.transpose() * rml::RegularizedPseudoInverse(S_, regularizationParameter_);

        // compute x(k|k) = x(k|k-1)+K(k)*[y(k)-y(k|k-1)]
        x_ = x_ + K_ * (y_ - ypredict_);

        // compute Sigma(k|k)
        Sigma_ = Sigma_ - K_ * S_ * K_.transpose();

        isFirst_ = true;
    }
}

void ExtendedKalmanFilter::Reset() {}

void ExtendedKalmanFilter::Init(const Eigen::VectorXd initialState) {
    x_ = initialState;
    Sigma_.setZero(stateDimension_, stateDimension_);
    isFirst_ = true;
}

Eigen::VectorXd ExtendedKalmanFilter::GetState() { return x_; }

// private:

void ExtendedKalmanFilter::NormalizeAngle(double& angle)
{

    while (angle > M_PI) {
        angle -= 2 * M_PI;
    }

    while (angle < -M_PI) {
        angle += 2 * M_PI;
    }
}

Eigen::VectorXd ExtendedKalmanFilter::FilterAngularJump(const Eigen::VectorXd primaryHeading, const Eigen::VectorXd otherHeading)
{

    Eigen::VectorXd out;
    out.resize(primaryHeading.size());
    for (int i = 0; i < primaryHeading.size(); i++) {
        double diff = primaryHeading(i) - otherHeading(i);

        out(i) = otherHeading(i);

        if (diff > M_PI)
            out(i) += 2.0 * M_PI;
        else {
            if (diff < -M_PI) {
                out(i) -= 2.0 * M_PI;
            }
        }
    }

    return out;
}
}
