#include "surface_identification_config.h"

#include <math.h>

#define INIT_TH M_PI / 3.
#define INIT_EP 0.

#define INIT_VAR_DX 1
#define INIT_VAR_DY 1
#define INIT_VAR_DZ 1
#define INIT_VAR_A 2
#define INIT_VAR_DA 5
#define INIT_VAR_B 10
#define INIT_VAR_DB 10
#define INIT_VAR_TH 0.01
#define INIT_VAR_EP 0.01
#define INIT_VAR_D 10

#define PRED_VAR_DX 0.0001
#define PRED_VAR_DY 0.0001
#define PRED_VAR_DZ 0.0001
#define PRED_VAR_DA 0.0001
#define PRED_VAR_TH 0.0000001
#define PRED_VAR_D 0.000001

#define MEAS_VAR_DX 0.00000004
#define MEAS_VAR_DY 0.00000004
#define MEAS_VAR_DZ 0.00000004
#define MEAS_VAR_A 0.00000004
#define MEAS_VAR_DA 0.000004
#define MEAS_VAR_D 100.

#define MIN_AUX 0.05
#define MIN_DEN 0.05
#define MAX_AZIMUTH_VAR_SCALING 200.

SurfaceIdentificationConfig::SurfaceIdentificationConfig( )
{
    //initialize everything
    init_prop_.th = INIT_TH;
    init_prop_.ep = INIT_EP;
    init_prop_.var_dx = INIT_VAR_DX;
    init_prop_.var_dy = INIT_VAR_DY;
    init_prop_.var_dz = INIT_VAR_DZ;
    init_prop_.var_a = INIT_VAR_A;
    init_prop_.var_da = INIT_VAR_DA;
    init_prop_.var_b = INIT_VAR_B;
    init_prop_.var_db = INIT_VAR_DB;
    init_prop_.var_th = INIT_VAR_TH;
    init_prop_.var_ep = INIT_VAR_EP;
    init_prop_.var_d = INIT_VAR_D;

    pred_prop_.var_dx = PRED_VAR_DX;
    pred_prop_.var_dy = PRED_VAR_DY;
    pred_prop_.var_dz = PRED_VAR_DZ;
    pred_prop_.var_da = PRED_VAR_DA;
    pred_prop_.var_th = PRED_VAR_TH;
    pred_prop_.var_d = PRED_VAR_D;

    meas_prop_.var_dx = MEAS_VAR_DX;
    meas_prop_.var_dy = MEAS_VAR_DY;
    meas_prop_.var_dz = MEAS_VAR_DZ;
    meas_prop_.var_a = MEAS_VAR_A;
    meas_prop_.var_da = MEAS_VAR_DA;
    meas_prop_.var_d = MEAS_VAR_D;

    modifiers_.max_azimuth_var_scaling = MAX_AZIMUTH_VAR_SCALING;
    modifiers_.min_aux = MIN_AUX;
    modifiers_.min_den = MIN_DEN;
} // SurfaceIdentificationConfig

SurfaceIdentificationConfig::~SurfaceIdentificationConfig( )
{} // ~SurfaceIdentificationConfig

bool SurfaceIdentificationConfig::read( const std::string &_file )
{
    // define some XML tools
    TiXmlDocument doc;

    // grab the settings for the surface identification filter from the xml file
    if( !doc.LoadFile( _file.c_str( ) ) )
        return bErrorMessage( "Loading the settings of the surface identification failed\nCheck file: " + _file );

    TiXmlHandle base( doc.FirstChildElement( ) );

    bool success = true;

    // get the initialization properties of the filter
    init_prop_.var_dx = getProperty( base, "init_prop", "var_dx", success );
    init_prop_.var_dy = getProperty( base, "init_prop", "var_dy", success );
    init_prop_.var_dz = getProperty( base, "init_prop", "var_dz", success );
    init_prop_.var_a = getProperty( base, "init_prop", "var_a", success );
    init_prop_.var_da = getProperty( base, "init_prop", "var_da", success );
    init_prop_.var_b = getProperty( base, "init_prop", "var_b", success );
    init_prop_.var_db = getProperty( base, "init_prop", "var_db", success );
    init_prop_.var_th = getProperty( base, "init_prop", "var_th", success );
    init_prop_.var_ep = getProperty( base, "init_prop", "var_ep", success );
    init_prop_.var_d = getProperty( base, "init_prop", "var_d", success );
    init_prop_.th = getProperty( base, "init_prop", "th", success ) * M_PI / 180.;
    init_prop_.ep = getProperty( base, "init_prop", "ep", success ) * M_PI / 180.;

    // get the prediction model properties of the filter
    pred_prop_.var_dx = getProperty( base, "pred_prop", "var_dx", success );
    pred_prop_.var_dy = getProperty( base, "pred_prop", "var_dy", success );
    pred_prop_.var_dz = getProperty( base, "pred_prop", "var_dz", success );
    pred_prop_.var_da = getProperty( base, "pred_prop", "var_da", success );
    pred_prop_.var_th = getProperty( base, "pred_prop", "var_th", success );
    pred_prop_.var_d = getProperty( base, "pred_prop", "var_d", success );

    // get the measurement model properties of the filter
    meas_prop_.var_dx = getProperty( base, "meas_prop", "var_dx", success );
    meas_prop_.var_dy = getProperty( base, "meas_prop", "var_dy", success );
    meas_prop_.var_dz = getProperty( base, "meas_prop", "var_dz", success );
    meas_prop_.var_a = getProperty( base, "meas_prop", "var_a", success );
    meas_prop_.var_da = getProperty( base, "meas_prop", "var_da", success );
    meas_prop_.var_d = getProperty( base, "meas_prop", "var_d", success );

    // get the modifiers for some parameters in the filter
    modifiers_.max_azimuth_var_scaling = getProperty( base, "modifier", "max_azimuth_var_scaling", success );
    modifiers_.min_aux = getProperty( base, "modifier", "min_aux", success );
    modifiers_.min_den = getProperty( base, "modifier", "min_den", success );

    // check if reading was successful
    if( !success )
        return bErrorMessage( "Not all surface filter properties could be read from the file: " + _file );

    return true;
} // read

double SurfaceIdentificationConfig::getProperty( const TiXmlHandle &base, const std::string &_category, const std::string &_type, bool &_success )
{
    // parse all properties of type in category
    for( TiXmlElement *elem = base.FirstChildElement( _category ).Element( ); elem; elem = elem->NextSiblingElement( _category ) )
    {
        if( elem->FirstAttribute( )->ValueStr( ) == _type )
            return charToDouble( elem->GetText( ) );
    }

    vErrorMessage( "No property '" + _type + "' of category '" + _category + "' found." );
    _success = false;

    return 0;
} // getProperty
