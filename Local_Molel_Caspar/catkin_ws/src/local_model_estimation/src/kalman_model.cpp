#include "kalman_model.hpp"

namespace tools
{
/**
 * @brief      Construct a model object for the Extended Kalman Filter.
 *
 * @param[in]  _n_functions  The number of functions in the model.
 * @param[in]  _n_states     The number of states in the model.
 */
KalmanModel::KalmanModel( const int _n_functions, const int _n_states ) : evaluation_( Eigen::MatrixXd( _n_functions, 1 ) ),
    jacobian_( Eigen::MatrixXd( _n_functions, _n_states ) ),
    covariance_( Eigen::MatrixXd( _n_functions, _n_functions ) ),
    n_functions_( _n_functions ),
    n_states_( _n_states )
{} // KalmanModel

/**
 * @brief      Evaluate the model.
 *
 * @return     The result of the model evaluation.
 */
const Eigen::MatrixXd &KalmanModel::evaluate( )
{
    return evaluation_;
} // evaluate

/**
 * @brief      Get the jacobian of the model.
 *
 * @return     The jacobian of the model.
 */
const Eigen::MatrixXd &KalmanModel::jacobian( )
{
    return jacobian_;
} // jacobian

/**
 * @brief      Get the covariance of the model.
 *
 * @return     The covariance of the model.
 */
const Eigen::MatrixXd &KalmanModel::covariance( )
{
    return covariance_;
} // covariance
} // End of namespace tools
