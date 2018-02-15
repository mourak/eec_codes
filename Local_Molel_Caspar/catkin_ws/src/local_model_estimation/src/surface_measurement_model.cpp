#include "surface_measurement_model.h"

SurfaceMeasurementModel::SurfaceMeasurementModel( const int _functions, const int _states ) : KalmanModel( _functions, _states ),
    configured_( false ),
    var_a_( 0 ),
    var_da_( 0 ),
    max_azimuth_var_scaling_( 0 )
{} // SurfaceMeasurementModel

SurfaceMeasurementModel::~SurfaceMeasurementModel( )
{} // ~SurfaceMeasurementModel

bool SurfaceMeasurementModel::configure( SurfaceIdentificationConfig &_config )
{
    // get measurement model properties
    SurfaceIdentificationConfig::MeasurementProperties &mp( _config.meas_prop_ );

    var_a_ = mp.var_a;
    var_da_ = mp.var_da;

    // set modifiers
    max_azimuth_var_scaling_ = _config.modifiers_.max_azimuth_var_scaling;

    // initialize model matrices
    jacobian_ <<
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

    Eigen::MatrixXd diag( n_functions_, 1 );
    diag << mp.var_dx, mp.var_dy, mp.var_dz, mp.var_a, mp.var_da, mp.var_a, mp.var_da, mp.var_d;
    covariance_ = diag.asDiagonal( );

    configured_ = true;
    return configured_;
} // configure

bool SurfaceMeasurementModel::update( const Eigen::MatrixXd &_state, const Eigen::MatrixXd &_control )
{
    if( !configured_ )
    {
        std::cout << "[ERROR] SurfaceMeasurementModel not configured." << std::endl;
        return false;
    }

    // evaluate the measurement model
    evaluation_ << _state( 0 ), _state( 1 ), _state( 2 ), _state( 3 ), _state( 4 ), _state( 5 ), _state( 6 ), _state( 9 );

    // determine the jacobian of the measurement model:
    // jacobian = unchanged

    // determine the covariance of the measurement model
    // covariance = unchanged, except for covariance(5,5) and covariance(6,6)
    covariance_( 5, 5 ) = var_a_ * scaleAzimuthVariance( _state( 3 ) );
    covariance_( 6, 6 ) = var_da_ * scaleAzimuthVariance( _state( 3 ) );

    return true;
} // update

double SurfaceMeasurementModel::scaleAzimuthVariance( const double _altitude )
{
    double cos_altitude = std::cos( _altitude );

    if( cos_altitude == 0 )
        return max_azimuth_var_scaling_;

    double var_scaling = 1. / ( cos_altitude * cos_altitude );

    if( var_scaling > max_azimuth_var_scaling_ )
        var_scaling = max_azimuth_var_scaling_;

    return var_scaling;
} // scaleAzimuthVariance
