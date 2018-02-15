#include "extended_kalman_filter.hpp"

#include <iostream>

namespace tools
{
/**
  * @brief      Constructs the Extended Kalman Filter with the prediction model
  *             and the measurement model unset.
  *
  *             Before the constructed obect can be used, the prediction model and
  *             measurement model need to be set using the setPredictionModel(model) and
  *             setMeasurementModel(model) functions.
  */
ExtendedKalmanFilter::ExtendedKalmanFilter( ) : prediction_model_set_( false ),
    measurement_model_set_( false )
{} // ExtendedKalmanFilter

/**
  * @brief      Construct the Extended Kalman Filter with a provided prediction model
  *             and measurement model.
  *
  * @param      _prediction_model   The prediction model
  * @param      _measurement_model  The measurement model
  */
ExtendedKalmanFilter::ExtendedKalmanFilter( KalmanModel *_prediction_model, KalmanModel *_measurement_model ) : prediction_model_( _prediction_model ),
    measurement_model_( _measurement_model ),
    prediction_model_set_( true ),
    measurement_model_set_( true )
{} // ExtendedKalmanFilter

/**
  * @brief      Sets the prediction model.
  *
  * @param      _prediction_model  The prediction model.
  *
  * @return     Model successfully set?
  */
bool ExtendedKalmanFilter::setPredictionModel( KalmanModel *_prediction_model )
{
    prediction_model_ = _prediction_model;
    prediction_model_set_ = true;
    return true;
} // setPredictionModel

/**
  * @brief      Sets the measurement model.
  *
  * @param      _measurement_model  The measurement model
  *
  * @return     Model successfully set?
  */
bool ExtendedKalmanFilter::setMeasurementModel( KalmanModel *_measurement_model )
{
    measurement_model_ = _measurement_model;
    measurement_model_set_ = true;
    return true;
} // setMeasurementModel

/**
  * @brief      Run the Extended Kalman Filter to obtain a new state estimate.
  *
  * @param      _state        The state that will be updated.
  * @param      _cov          The covariance that will be updated.
  * @param[in]  _control      The control commands.
  * @param[in]  _measurement  The measurement.
  *
  * @return     Extended Kalman Filter updated sucessfully?
  */
bool ExtendedKalmanFilter::update( Eigen::MatrixXd &_state, Eigen::MatrixXd &_cov, const Eigen::MatrixXd &_control, const Eigen::MatrixXd &_measurement )
{
    // check if the models for the kalman filter are set
    if( !( prediction_model_set_ && measurement_model_set_ ) )
    {
        std::cout << "[ERROR] Kalman models not initialized." << std::endl;
        return false;
    }

    // update the prediction model based on the new input state
    if( !prediction_model_->update( _state, _control ) )
        return false;

    // do the prediction step of the kalman filter for the state and the covariance
    Eigen::MatrixXd pred_state = prediction_model_->evaluate( );
    Eigen::MatrixXd pred_cov = prediction_model_->jacobian( ) * _cov * prediction_model_->jacobian( ).transpose( ) + prediction_model_->covariance( );

    // update the measurement model based on the outcome of the prediction model
    if( !measurement_model_->update( pred_state, _control ) )
        return false;

    // calculate the kalman gain
    Eigen::MatrixXd K = pred_cov * measurement_model_->jacobian( ).transpose( ) *
                        ( measurement_model_->jacobian( ) * pred_cov * measurement_model_->jacobian( ).transpose( ) + measurement_model_->covariance( ) ).inverse( );

    // get the resulting, updated state and covariance
    _state = pred_state + K * ( _measurement - measurement_model_->evaluate( ) );
    _cov = ( Eigen::MatrixXd::Identity( _state.rows( ), _state.rows( ) ) - K * measurement_model_->jacobian( ) ) * pred_cov;

    return true;
} // update
} // End of namespace tools
