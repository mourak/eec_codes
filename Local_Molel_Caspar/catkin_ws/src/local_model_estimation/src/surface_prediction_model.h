#ifndef SURFACE_PREDICTION_MODEL_H
#define SURFACE_PREDICTION_MODEL_H

#include "kalman_model.hpp"
#include "surface_identification_config.h"

class SurfacePredictionModel : public tools::KalmanModel
{
public:

    SurfacePredictionModel( const int _functions, const int _states, const int _noise_sources );

    ~SurfacePredictionModel( );

    bool configure( SurfaceIdentificationConfig &_config );

    bool update( const Eigen::MatrixXd &_state, const Eigen::MatrixXd &_control );

    void setTimeStep( const double _dt );

private:

    double scaleAzimuthVariance( const double _altitude );

    double dt_; // the time step between two model calls

    Eigen::MatrixXd N_;
    Eigen::DiagonalMatrix < double, Eigen::Dynamic > M_;

    bool configured_;

    double var_da_;
    double var_th_;

    double max_azimuth_var_scaling_;
    double min_aux_;
    double min_den_;
};

#endif // SURFACE_PREDICTION_MODEL_H
