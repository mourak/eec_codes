#ifndef EXTENDED_KALMAN_FILTER_H
#define EXTENDED_KALMAN_FILTER_H

#include "kalman_model.hpp"

#include <eigen3/Eigen/Geometry>
#include <vector>

namespace tools
{
/**
 * @brief      Class implementing the Extended Kalman Filer.
 */
class ExtendedKalmanFilter
{
public:

    ExtendedKalmanFilter( );

    ExtendedKalmanFilter( KalmanModel *_prediction_model, KalmanModel *_measurement_model );

    bool setMeasurementModel( KalmanModel *_measurement_model );

    bool setPredictionModel( KalmanModel *_prediction_model );

    bool update( Eigen::MatrixXd &_state, Eigen::MatrixXd &_cov, const Eigen::MatrixXd &_control, const Eigen::MatrixXd &_measurement );

private:

    KalmanModel *prediction_model_;
    KalmanModel *measurement_model_;

    bool prediction_model_set_;
    bool measurement_model_set_;
};
} // End of namespace tools

#endif // EXTENDED_KALMAN_FILTER_H
