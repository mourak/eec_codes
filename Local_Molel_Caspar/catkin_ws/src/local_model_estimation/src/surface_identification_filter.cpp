#include "surface_identification_filter.h"

#include <iostream>
#include <math.h>

// the state defined as:
//  1) dx: x-velocity of the tip
//  2) dy: y-velocity of the tip
//  3) dz: z-velocity of the tip
//  4) a : altitude of the tip viewing direction (alpha)
//  5) da: velocity of the altitude of the tip viewing direction
//  6) b : azimuth of the tip viewing direction (beta)
//  7) db: velocity of the azimuth of the tip viewing direction
//  8) th: altitude of the placenta normal (theta)
//  9) ep: azimuth of the placenta normal (epsilon)
// 10) d : distance from instrument tip to the placenta (positive in the tip viewing direction)

// there is prediction model noise on:
// 1) dx
// 2) dy
// 3) dz
// 4) da -> that results in noise on a
// 5) db -> that results in noise on b
// 6) th
// 7) ep
// 8) d

// the measured states are:
// 1) dx: x-velocity of the tip
// 2) dy: y-velocity of the tip
// 3) dz: z-velocity of the tip
// 4) a : altitude of the tip viewing direction (alpha)
// 5) da: velocity of the altitude of the tip viewing direction
// 6) b : azimuth of the tip viewing direction (beta)
// 7) db: velocity of the azimuth of the tip viewing direction
// 8) d : distance from instrument tip to the placenta (positive in the tip viewing direction)

#define PREDICTION_FUNCTIONS 10
#define MEASUREMENT_FUNCTIONS 8
#define STATES 10
#define PREDICTION_NOISE_SOURCES 8

SurfaceIdentificationFilter::SurfaceIdentificationFilter( ) : initialized_( false ),
    prediction_model_( PREDICTION_FUNCTIONS, STATES, PREDICTION_NOISE_SOURCES ),
    measurement_model_( MEASUREMENT_FUNCTIONS, STATES ),
    config_file_( "surface_identification_filter.xml" )
{
    kalman_ = tools::ExtendedKalmanFilter( &prediction_model_, &measurement_model_ );
} // SurfaceIdentificationFilter

SurfaceIdentificationFilter::~SurfaceIdentificationFilter( )
{} // ~SurfaceIdentificationFilter

void SurfaceIdentificationFilter::reset( )
{
    initialized_ = false;
} // reset

void SurfaceIdentificationFilter::setConfigFile( const std::string &_config_file )
{
    config_file_ = _config_file;
} // setConfigFile

bool SurfaceIdentificationFilter::init( const Eigen::MatrixXd &_meas )
{
    if( !config_.read( config_file_ ) )
        return false;

    prediction_model_.configure( config_ );
    measurement_model_.configure( config_ );

    // initialize the state vector
    SurfaceIdentificationConfig::InitProperties &ip( config_.init_prop_ );

    state_.resize( STATES, 1 );
    state_( 0 ) = _meas( 0 ); // dx
    state_( 1 ) = _meas( 1 ); // dy
    state_( 2 ) = _meas( 2 ); // dz
    state_( 3 ) = _meas( 3 ); // a
    state_( 4 ) = _meas( 4 ); // da
    state_( 5 ) = _meas( 5 ); // b
    state_( 6 ) = _meas( 6 ); // db
    state_( 7 ) = ip.th;      // th
    state_( 8 ) = ip.ep;      // ep
    state_( 9 ) = _meas( 7 ); // d

    // initialize the covariance matrix
    Eigen::Matrix < double, STATES, 1 > cov_diag;
    cov_diag << ip.var_dx, ip.var_dy, ip.var_dz, ip.var_a, ip.var_da, ip.var_b, ip.var_db, ip.var_th, ip.var_ep, ip.var_d;
    cov_ = cov_diag.asDiagonal( );

    // debug
    last_b_ = _meas( 5 );

    return true;
} // init

bool SurfaceIdentificationFilter::update( const Eigen::Affine3d &_w_T_tip, const Eigen::Matrix < double, 6, 1 > &_w_t_tip,
                                          const double _distance_measurement, const double _time_step, Eigen::Affine3d &_w_T_placenta )
{
    if( !initialized_ )
    {
        // initialize the state and the covariance, ao based on the first measurement
        initialized_ = init( extractMeasurement( _w_T_tip, _w_t_tip, _distance_measurement ) );
        return false;
    }

    // set the most recent time step
    prediction_model_.setTimeStep( _time_step );

    // update the state and covariance, using the extended kalman filter
    Eigen::MatrixXd meas = extractMeasurement( _w_T_tip, _w_t_tip, _distance_measurement ); // call this function only onces per update!!

    if( !kalman_.update( state_, cov_, Eigen::MatrixXd( 10, 1 ), meas ) )
        return false;

    // set the translational part of w_T_placenta, based on the distance
    Eigen::Vector3d tip_P_placenta( 0, 0, state_( 9 ) );
    _w_T_placenta.translation( ) = _w_T_tip * tip_P_placenta;

    // set the rotational part of w_T_placenta, based on the altitude theta and azimuth epsilon of the placenta
    Eigen::Vector3d e3; // this is the z-part of the rotational part of w_T_placenta
    e3 << std::cos( state_( 7 ) ) * std::cos( state_( 8 ) ), std::cos( state_( 7 ) ) * std::sin( state_( 8 ) ), std::sin( state_( 7 ) );

    // check if e3 is directed opposite of the tip direction. if not: change it,
    // such that the placenta normal points opposite of te viewing direction of the tip
    double dot = e3.dot( _w_T_tip.rotation( ).row( 2 ) );

    if( dot > 0 )
        e3 = -e3;

    // now find an arbitrary vector orthogonal to the normal: take the vectors [1,0,0] and [0,1,0] and retain the most orthogonal one
    Eigen::Vector3d e2;

    if( std::abs( e3.dot( Eigen::Vector3d::UnitX( ) ) ) < std::abs( e3.dot( Eigen::Vector3d::UnitY( ) ) ) )
        e2 = Eigen::Vector3d::UnitX( );
    else
        e2 = Eigen::Vector3d::UnitY( );

    // make e2 fully orthonogal
    e2 = e3.cross( e2 );
    e2.normalize( );

    // find the vector e1, to obtain an orthonomal frame {e1,e2,e3}
    Eigen::Vector3d e1( e2.cross( e3 ) );

    // write the frame to w_T_placenta
    _w_T_placenta.linear( ).col( 0 ) = e1;
    _w_T_placenta.linear( ).col( 1 ) = e2;
    _w_T_placenta.linear( ).col( 2 ) = e3;

    // debug
    debug_par_.th = state_( 7 );
    debug_par_.ep = state_( 8 );
    debug_par_.d = state_( 9 );
    debug_par_.meas_dx = meas( 0 );
    debug_par_.meas_dy = meas( 1 );
    debug_par_.meas_dz = meas( 2 );
    debug_par_.meas_a = meas( 3 );
    debug_par_.meas_da = meas( 4 );
    debug_par_.meas_b = meas( 5 );
    debug_par_.meas_db = meas( 6 );
    debug_par_.meas_d = meas( 7 );

    return true;
} // update

Eigen::MatrixXd SurfaceIdentificationFilter::extractMeasurement( const Eigen::Affine3d &_w_T_tip, const Eigen::Matrix < double, 6, 1 > &_w_t_tip,
                                                                 const double _distance_measurement )
{
    // get the tip rotation in the tip frame
    Eigen::Vector3d tip_w_tip( _w_T_tip.inverse( Eigen::Isometry ).linear( ) * _w_t_tip.tail( 3 ) );

    // subtract the rotation around the tip axis, as that rotation doesn't affect alpha or beta
    tip_w_tip( 2 ) = 0;

    // express again in the world frame
    Eigen::Vector3d w_w_tip( _w_T_tip.linear( ) * tip_w_tip );

    // extract the state parameters from the measurement
    Eigen::MatrixXd meas( 8, 1 );
    double b = std::atan2( _w_T_tip( 1, 2 ), _w_T_tip( 0, 2 ) );

    // unwrap b to avoid jumps
    // instead of where you would use np.mod( x,y ) in python, use here: x - floor(x/y)*y,
    // because fmod(x,y) in c++ doesn't give the desired behaviour
    // if mod would have given the right behaviour, the function would have been:
    // b = last_b_ + mod( b - last_b_ + M_PI, 2. * M_PI ) - M_PI;
    b = b - floor( ( b - last_b_ + M_PI ) / ( 2. * M_PI ) ) * ( 2. * M_PI );
    last_b_ = b;

    meas( 0 ) = _w_t_tip[ 0 ]; // dx
    meas( 1 ) = _w_t_tip[ 1 ]; // dy
    meas( 2 ) = _w_t_tip[ 2 ]; // dz
    meas( 3 ) = std::atan2( _w_T_tip( 2, 2 ), std::sqrt( _w_T_tip( 0, 2 ) * _w_T_tip( 0, 2 ) + _w_T_tip( 1, 2 ) * _w_T_tip( 1, 2 ) ) ); // a
    meas( 4 ) = w_w_tip[ 0 ] * std::sin( b ) - w_w_tip[ 1 ] * std::cos( b ); // da
    meas( 5 ) = b; // b
    meas( 6 ) = w_w_tip[ 2 ]; // db
    meas( 7 ) = _distance_measurement; // d

    return meas;
} // extractMeasurement

SurfaceDebugParameters &SurfaceIdentificationFilter::getDebugParameters( )
{
    return debug_par_;
} // getDebugParameters
