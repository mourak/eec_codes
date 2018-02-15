#include "surface_prediction_model.h"

SurfacePredictionModel::SurfacePredictionModel( const int _functions, const int _states, const int _noise_sources ) : KalmanModel( _functions, _states ),
    dt_( 0 ),
    N_( _states, _noise_sources ),
    M_( _noise_sources ),
    configured_( false )
{} // SurfacePredictionModel

SurfacePredictionModel::~SurfacePredictionModel( )
{} // ~SurfacePredictionModel

bool SurfacePredictionModel::configure( SurfaceIdentificationConfig &_config )
{
    // get measurement model properties
    SurfaceIdentificationConfig::PredictionProperties &pp( _config.pred_prop_ );

    var_da_ = pp.var_da;
    var_th_ = pp.var_th;

    // set modifiers
    max_azimuth_var_scaling_ = _config.modifiers_.max_azimuth_var_scaling;
    min_aux_ = _config.modifiers_.min_aux;
    min_den_ = _config.modifiers_.min_den;

    // initialize model matrices
    jacobian_.setIdentity( );

    N_ <<
    1, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, dt_, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, dt_, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 1;

    M_.setIdentity( );
    M_.diagonal( ) << pp.var_dx, pp.var_dy, pp.var_dz, pp.var_da, pp.var_da, pp.var_th, pp.var_th, pp.var_d;

    configured_ = true;
    return configured_;
} // configure

bool SurfacePredictionModel::update( const Eigen::MatrixXd &_state, const Eigen::MatrixXd &_control )
{
    if( !configured_ )
    {
        std::cout << "[ERROR] SurfacePredictionModel not configured." << std::endl;
        return false;
    }

    // extract the individual states
    double dx = _state( 0 );
    double dy = _state( 1 );
    double dz = _state( 2 );
    double a = _state( 3 );
    double da = _state( 4 );
    double b = _state( 5 );
    double db = _state( 6 );
    double th = _state( 7 );
    double ep = _state( 8 );
    double d = _state( 9 );

    // calculate auxilary angles
    double at = a + da * dt_;
    double bt = b + db * dt_;

    double w1 = at - bt + ep - th;
    double w2 = at - bt + ep + th;
    double w3 = at + bt - ep - th;
    double w4 = at + bt - ep + th;
    double w5 = at + th;
    double w6 = at - th;
    double w7 = b - ep;
    double w8 = bt - ep;

    // precompute sin/cos of all occurring angles
    double sa = std::sin( a );
    double ca = std::cos( a );
    double sat = std::sin( at );
    double cat = std::cos( at );
    double sbt = std::sin( bt );
    double cbt = std::cos( bt );
    double sep = std::sin( ep );
    double cep = std::cos( ep );
    double sth = std::sin( th );
    double cth = std::cos( th );

    double sw1 = std::sin( w1 );
    double cw1 = std::cos( w1 );
    double sw2 = std::sin( w2 );
    double cw2 = std::cos( w2 );
    double sw3 = std::sin( w3 );
    double cw3 = std::cos( w3 );
    double sw4 = std::sin( w4 );
    double cw4 = std::cos( w4 );
    double sw5 = std::sin( w5 );
    double cw5 = std::cos( w5 );
    double sw6 = std::sin( w6 );
    double cw6 = std::cos( w6 );
    double sw7 = std::sin( w7 );
    double cw7 = std::cos( w7 );
    double sw8 = std::sin( w8 );
    double cw8 = std::cos( w8 );

    // calculate some auxilary variables
    double num = -d * sa * sth - d * ca * cth * cw7 + dx * dt_ * cep * cth + dy * dt_ * sep * cth + dz * dt_ * sth;
    double den = sep * sbt * cth * cat + sth * sat + cep * cth * cat * cbt;
    double aux = 2. * cw6 - 2. * cw5 + cw1 + cw2 + cw3 + cw4;

    // avoid problems with zero divisions
    if( std::abs( den ) < min_den_ )
    {
        if( den == 0 )
            den = min_den_;
        else
            den = den / std::abs( den ) * min_den_;
    }

    if( std::abs( aux ) < min_aux_ )
    {
        if( aux == 0 )
            aux = min_aux_;
        else
            aux = aux / std::abs( aux ) * min_aux_;
    }

    // evaluate the prediction model
    evaluation_ << dx, dy, dz, at, da, bt, db, th, ep, -num / den;

    // determine the jacobian of the prediction model:
    // jacobian = unchanged, except jacobian(3,4), jacobian(5,6), and the last row jacobian(9,:)
    jacobian_( 3, 4 ) = dt_;
    jacobian_( 5, 6 ) = dt_;
    jacobian_.row( 9 ) <<
    -4. * dt_ * cep * cth / aux,
    -4. * dt_ * sep * cth / aux,
    -4. * dt_ * sth / aux,
    -16. * ( -d * ( -sa * cth * cw7 + sth * ca ) * aux / 4. + ( -sth * cat + sat * cth * cw8 ) * num ) / ( aux * aux ),
    -16. * dt_ * ( -sth * cat + sat * cth * cw8 ) * num / ( aux * aux ),
    -16. * ( d * aux * sw7 * ca / 4. + num * sw8 * cat ) * cth / ( aux * aux ),
    -16. * dt_ * num * sw8 * cth * cat / ( aux * aux ),
    -16. * ( num * ( -2. * sw6 - 2. * sw5 - sw1 + sw2 - sw3 + sw4 ) / 4. + ( -d * sa * cth + d * sth * ca * cw7 - dx * dt_ * sth * cep - dy * dt_ * sep * sth + dz * dt_ * cth ) * aux / 4. ) / ( aux * aux ),
    -16. * ( ( -d * sw7 * ca - dx * dt_ * sep + dy * dt_ * cep ) * aux / 4. - num * sw8 * cat ) * cth / ( aux * aux ),
    4. * ( sa * sth + ca * cth * cw7 ) / aux;

    // determine the covariance of the prediction model
    // the variance is build from N and M
    // N contains the noise coefficients, that determine for every state how it is influence by the different noise sources
    // M contains the variances of the noise sources, as its diagonal
    // the covariance will finally look like: N*M*N^T

    // N: only N(3,3) and N(5,4) change
    N_( 3, 3 ) = dt_;
    N_( 5, 4 ) = dt_;

    // M: only M(4,4) and M(6,6) change
    M_.diagonal( )[ 4 ] = var_da_ * scaleAzimuthVariance( a );
    M_.diagonal( )[ 6 ] = var_th_ * scaleAzimuthVariance( th );

    covariance_ = N_ * M_ * N_.transpose( );

    return true;
} // update

double SurfacePredictionModel::scaleAzimuthVariance( const double _altitude )
{
    double cos_altitude = std::cos( _altitude );

    if( cos_altitude == 0 )
        return max_azimuth_var_scaling_;

    double var_scaling = 1. / ( cos_altitude * cos_altitude );

    if( var_scaling > max_azimuth_var_scaling_ )
        var_scaling = max_azimuth_var_scaling_;

    return var_scaling;
} // scaleAzimuthVariance

void SurfacePredictionModel::setTimeStep( const double _dt )
{
    dt_ = _dt;
} // setTimeStep
