#ifndef SURFACE_IDENTIFICATION_FILTER_H
#define SURFACE_IDENTIFICATION_FILTER_H

#include "extended_kalman_filter.hpp"
#include "surface_prediction_model.h"
#include "surface_measurement_model.h"
#include "surface_identification_config.h"

#include <eigen3/Eigen/Geometry>
#include <vector>

struct SurfaceDebugParameters
{
    double th;
    double ep;
    double d;
    double meas_dx;
    double meas_dy;
    double meas_dz;
    double meas_a;
    double meas_da;
    double meas_b;
    double meas_db;
    double meas_d;
};

class SurfaceIdentificationFilter
{
public:

    SurfaceIdentificationFilter( );

    ~SurfaceIdentificationFilter( );

    void reset( );

    void setConfigFile( const std::string &_config_file );

    bool update( const Eigen::Affine3d &_w_T_tip, const Eigen::Matrix < double, 6, 1 > &_w_t_tip, const double _distance_measurement, const double _time_step, Eigen::Affine3d &_w_T_placenta );

private:

    bool init( const Eigen::MatrixXd &_meas );

    Eigen::MatrixXd extractMeasurement( const Eigen::Affine3d &_w_T_tip, const Eigen::Matrix < double, 6, 1 > &_w_t_tip, const double _distance_measurement );

    Eigen::MatrixXd state_;
    Eigen::MatrixXd cov_;

    bool initialized_;

    SurfaceMeasurementModel measurement_model_;
    SurfacePredictionModel prediction_model_;

    tools::ExtendedKalmanFilter kalman_;

    SurfaceIdentificationConfig config_;

    std::string config_file_;

    // debug

public:

    SurfaceDebugParameters &getDebugParameters( );
    SurfaceDebugParameters debug_par_;
    double last_a_;
    double last_b_;
};

#endif // SURFACE_IDENTIFICATION_FILTER_H
