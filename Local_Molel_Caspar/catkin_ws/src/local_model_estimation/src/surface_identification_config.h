#ifndef SURFACE_IDENTIFICATION_CONFIG_H
#define SURFACE_IDENTIFICATION_CONFIG_H

#include <tinyxml.h>

#include <string>
#include <sstream>
#include <iostream>

class SurfaceIdentificationConfig
{
public:

    struct InitProperties
    {
        double th; // in radians
        double ep; // in radians

        double var_dx;
        double var_dy;
        double var_dz;
        double var_a;
        double var_da;
        double var_b;
        double var_db;
        double var_th;
        double var_ep;
        double var_d;
    } init_prop_;

    struct PredictionProperties
    {
        double var_dx;
        double var_dy;
        double var_dz;
        double var_da;
        double var_th;
        double var_d;
    } pred_prop_;

    struct MeasurementProperties
    {
        double var_dx;
        double var_dy;
        double var_dz;
        double var_a;
        double var_da;
        double var_d;
    } meas_prop_;

    struct Modifiers
    {
        double max_azimuth_var_scaling;
        double min_aux;
        double min_den;
    } modifiers_;

    SurfaceIdentificationConfig( );

    ~SurfaceIdentificationConfig( );

    bool read( const std::string &_file );

private:

    double getProperty( const TiXmlHandle &base, const std::string &_category, const std::string &_type, bool &_success );

    // auxilary functions
    inline void vErrorMessage( const std::string &_msg )
    {
        std::cout << "[Error] " << _msg << std::endl;
        return;
    }

    inline bool bErrorMessage( const std::string &_msg, const bool _return_value = false )
    {
        std::cout << "[Error] " << _msg << std::endl;
        return _return_value;
    }

    inline double charToDouble( const char *_ch )
    {
        std::istringstream ss( _ch );
        double d;

        ss >> d;
        return d;
    }

    inline int charToInteger( const char *_ch )
    {
        std::istringstream ss( _ch );
        int i;

        ss >> i;
        return i;
    }
};

#endif // SURFACE_IDENTIFICATION_CONFIG_H
