#ifndef SURFACE_MEASUREMENT_MODEL_H
#define SURFACE_MEASUREMENT_MODEL_H

#include "kalman_model.hpp"
#include "surface_identification_config.h"

class SurfaceMeasurementModel : public tools::KalmanModel
{
public:

    SurfaceMeasurementModel( const int _functions, const int _states );

    ~SurfaceMeasurementModel( );

    bool configure( SurfaceIdentificationConfig &_config );

    bool update( const Eigen::MatrixXd &_state, const Eigen::MatrixXd &_control );

private:

    double scaleAzimuthVariance( const double _altitude );

    bool configured_;

    double var_a_;
    double var_da_;

    double max_azimuth_var_scaling_;
};

#endif // SURFACE_MEASUREMENT_MODEL_H
