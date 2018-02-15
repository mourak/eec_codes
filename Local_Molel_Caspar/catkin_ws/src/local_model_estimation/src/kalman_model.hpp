#ifndef KALMAN_MODEL_H
#define KALMAN_MODEL_H

#include <eigen3/Eigen/Geometry>

namespace tools
{
/**
 * @brief      The base class for a prediction model or measurement model in an
 *             Extended Kalman Filter.
 */
class KalmanModel
{
public:

    KalmanModel( const int _n_functions, const int _n_states );

    // this function calculates the function evaluation, the jacobian and the covarance
    virtual bool update( const Eigen::MatrixXd &_state, const Eigen::MatrixXd &_control ) = 0;

    const Eigen::MatrixXd &evaluate( );

    const Eigen::MatrixXd &jacobian( );

    const Eigen::MatrixXd &covariance( );

protected:

    Eigen::MatrixXd evaluation_;
    Eigen::MatrixXd jacobian_;
    Eigen::MatrixXd covariance_;

    int n_functions_;
    int n_states_;
};
} // End of namespace tools

#endif // KALMAN_MODEL_H
