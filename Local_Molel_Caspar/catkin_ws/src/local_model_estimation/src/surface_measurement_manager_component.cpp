#include "surface_measurement_manager_component.hpp"
#include <rtt/Component.hpp>
using namespace RTT;

/**
  * @brief      Constructor of the OROCOS component
  *
  * @param      _name  The name of the OROCOS component
  */
SurfaceMeasurementManager::SurfaceMeasurementManager( std::string const &_name ) : TaskContext( _name, PreOperational )
    , msg_w_T_ee_( 16, 0)
    , msg_w_t_ee_( 6, 0 ) 
//    , msg_ee_T_tip_( 16, 0,0 )
//   , msg_ee_t_tip_( 6, 0 ,0)
    , msg_w_T_plane_( 16, 0 )
    , prop_max_no_kalman_duration_( 0 )
    , max_no_kalman_duration_( 0 )
    , prop_max_valid_distance_( 1000 )
    , max_valid_distance_( 1000 )
    , prop_error_( 0 )
    , duration_since_kalman_( 0 )
    , weighted_sum_w_t_tip_( Eigen::Matrix < double, 6, 1 >::Zero( ) )
    , valid_w_T_plane_( false )
{
    msg_distance_.data = 0;

    // add properties
    addProperty( "filter_config_path", prop_filter_config_path_ ).doc(
        "The path of the file that contains the configuration parameters for the kalman filter." );
    addProperty( "max_no_kalman_duration", prop_max_no_kalman_duration_ ).doc(
        "The maximum duration for not running the kalman filter due to lack of data. If surpassed, generate distance data from model." );
    addProperty( "max_valid_distance", prop_max_valid_distance_ ).doc(
        "The upper threshold for a measured distance to be considered invalid [m]." );
    addProperty( "error", prop_error_ ).doc( "The code of the current error state." );

    // add ports
    addPort( "in_w_T_ee", in_w_T_ee_ ).doc( "End effector pose, in column-major format." );
    addPort( "in_w_t_ee", in_w_t_ee_ ).doc( "End effector twist: vx, vy, vz, wx, wy, wz." );
    addPort( "in_ee_T_tip", in_ee_T_tip_ ).doc( "Tip pose, in column-major format." );
    addPort( "in_ee_t_tip", in_ee_t_tip_ ).doc( "Tip twist: vx, vy, vz, wx, wy, wz." );
    addPort( "in_distance", in_distance_ ).doc( "The measured distance between the tip and the plane [m]." );
    addPort( "out_w_T_plane", out_w_T_plane_ ).doc( "Pose of the plane, in column-major format." );
    addPort( "out_normal_arrow", out_normal_arrow_ ).doc( "Arrow indicating the direction of the plane normal." );
    addPort( "out_error", out_error_ ).doc( "Send error codes." );

    // add operations

    // show messages to the output ports to guarantee real-timeness
   // out_w_T_plane_.setDataSample( msg_w_T_plane_ );

    log( Info ) << "[" << getName( ) << "] Constructed" << endlog( );

    // debug
    msg_debug_.data.resize( NUMBER_DEBUG_PARAMETERS, 0 );
    addPort( "out_debug", out_debug_ );
    out_debug_.setDataSample( msg_debug_ );
    visualization_msgs::Marker sample;
    out_normal_arrow_.setDataSample( sample );
    d_truth_ = 0;
    last_a_ = 0;
    last_b_ = 0;
} // SurfaceMeasurementManager

/**
  * @brief      Configuration hook of the OROCOS component
  *
  * @return     Configuration success
  */
bool SurfaceMeasurementManager::configureHook( )
{
    log( Info ) << "[" << getName( ) << "] Configured" << endlog( );

    return true;
} // configureHook

/**
  * @brief      Start hook of the OROCOS component
  *
  * @return     Startup success
  */
bool SurfaceMeasurementManager::startHook( )
{
    // verify properties
    bool valid = true;

    if( prop_max_no_kalman_duration_ < 0 )
        valid = false;
    else
        max_no_kalman_duration_ = prop_max_no_kalman_duration_;

    if( prop_max_valid_distance_ > 0 )
        max_valid_distance_ = prop_max_valid_distance_;
    else
        valid = false;

    if( !valid )
    {
        prop_error_ = 102;
        out_error_.write( prop_error_ );
        log( Error ) << "[" << getName( ) << "] Error: " << prop_error_ << endlog( );
        return false;
    }

    // reset the filter for processing the synthetic distance measurements
    surface_filter_.reset( );
    surface_filter_.setConfigFile( prop_filter_config_path_ );

    // make sure the period isn't too small at startup
    last_period_ = 0.01;
    last_timestamp_ = RTT::os::TimeService::Instance( )->getTicks( ) - RTT::os::TimeService::nsecs2ticks( last_period_ * 1000000000 );
    duration_since_kalman_ = 0;

    // reset weighted sum of twists
    weighted_sum_w_t_tip_ = Eigen::Matrix < double, 6, 1 >::Zero( );

    // flags
    valid_w_T_plane_ = false;

    // reset input ports
    in_w_T_ee_.clear( );
    in_w_t_ee_.clear( );
    in_ee_T_tip_.clear( );
    in_ee_t_tip_.clear( );
    in_distance_.clear( );

    log( Info ) << "[" << getName( ) << "] Started" << endlog( );

    return true;
} // startHook

/**
  * @brief      Update hook of the OROCOS component
  */
void SurfaceMeasurementManager::updateHook( )
{


//    std::cout << "I am here "<< std::endl;

//    in_ee_T_tip_.read(msg_ee_T_tip_);
//    std::cout <<  msg_ee_T_tip_.x<< std::endl;
//    std::cout <<  msg_ee_T_tip_.y<< std::endl;
//    std::cout <<  msg_ee_T_tip_.z<< std::endl;
    // check the period between last two calls of update hook
    last_period_ = RTT::os::TimeService::Instance( )->secondsSince( last_timestamp_ );
    last_timestamp_ = RTT::os::TimeService::Instance( )->getTicks( );

    RTT::FlowStatus status_distance = in_distance_.read( msg_distance_ );

    //std::cout <<  "msg_distance_ :  "  << msg_distance_.data<< std::endl;

    if( in_ee_T_tip_.read( msg_ee_T_tip_ ) != NoData
     && in_ee_t_tip_.read( msg_ee_t_tip_ ) != NoData
     && status_distance != NoData )
    {
        // Read the RTP values of the robot
//        std::cout << "R : " <<msg_ee_T_tip_.x<< std::endl;
//        std::cout << "T : " <<msg_ee_T_tip_.y<< std::endl;
//        std::cout << "P : " <<msg_ee_T_tip_.z<< std::endl;

        double distance = msg_distance_.data;

        double lengthAir = 0.004; //for a distance of the reference arm 1617 no changed

        double radius = -msg_ee_T_tip_.x + lengthAir; // all distances are in mm
        double theta = msg_ee_T_tip_.y -1.677 ;
        double phi =  msg_ee_T_tip_.z ;
       // std::cout << "R : " <<radius<< std::endl;
        //std::cout << "T : " <<theta<< std::endl;
        //std::cout << "P : " <<phi<< std::endl;


        double radius_dot = -msg_ee_t_tip_.x;
        double theta_dot = msg_ee_t_tip_.y;
        double phi_dot = msg_ee_t_tip_.z;


        // Phd Andy
        //double x_0 = radius * sin(theta) * sin(phi);
        //double y_0 = radius * cos(phi);
        //double z_0 = radius * sin(theta) * cos(phi);

        double x_0 = radius * sin(theta) * cos(phi);
        double y_0 = -radius              * sin(phi);
        double z_0 = radius * cos(theta) * cos(phi);

        p_[0] = x_0;
        p_[1] = y_0;
        p_[2] = z_0;

        w_[0] = phi_dot;
        w_[1] = theta_dot;
        w_[2] = 0;


        // previous transformation based on the
        /*S. Widnall, J. Peraire
        16.07 Dynamics
        Fall 2008
        Version 2.0
        Lecture L5 - Other Coordinate Systems*/
        //double vx = - radius_dot*cos(phi)*sin(theta)    -radius*theta_dot*cos(phi) *cos(theta);
        //double vy = - radius_dot* radius *sin(phi)      -radius*phi_dot*cos(phi);
        //double vz = - radius_dot*cos(theta)*cos(phi)    +radius*theta_dot*cos(phi)*sin(theta) + radius*phi_dot*sin(phi);

        //New tranformation based on discussion with Gianni
        //double vx =  radius_dot* sin(phi)*sin(theta) +theta_dot*radius* cos(theta)*sin(phi) +phi_dot*radius*sin(theta)*cos(phi) ;
        //double vy =  radius_dot* cos(phi)            +0                                     -phi_dot*radius*sin(phi) ;
        //double vz =  radius_dot* sin(theta)*cos(phi) +theta_dot*radius* cos(theta)*cos(phi) -phi_dot*radius*sin(theta)*sin(phi);


        double vx = radius_dot* sin(theta)*cos(phi) +theta_dot* radius* cos(theta)*cos(phi) -phi_dot* radius*sin(theta)*sin(phi) ;
        double vy = -radius_dot* sin(phi)           -0                                      -phi_dot* radius*cos(phi) ;
        double vz = radius_dot* cos(theta)*cos(phi) -theta_dot* radius* sin(theta)*cos(phi) -phi_dot* radius*cos(theta)*sin(phi);


        v_[0] = vx;
        v_[1] = vy;
        v_[2] = vz;

        // Create the rotation matrix
        Eigen::Affine3d rx = Eigen::Affine3d(Eigen::AngleAxisd(theta, Eigen::Vector3d(1, 0, 0)));
        Eigen::Affine3d ry = Eigen::Affine3d(Eigen::AngleAxisd(phi, Eigen::Vector3d(0, 1, 0)));
        Eigen::Affine3d rz = Eigen::Affine3d(Eigen::AngleAxisd(0, Eigen::Vector3d(0, 0, 1)));
        Eigen::Affine3d rw = rz*ry*rx;

        // Create the translation vector
        Eigen::Affine3d t(Eigen::Translation3d(Eigen::Vector3d(x_0,y_0,z_0)));

        // Build the transformation matrix
        // get the tip pose in the world frame
        Eigen::Affine3d w_T_tip = rw * t;

        // compute the the tip twist in the world frame
        Eigen::Matrix < double, 6, 1 > w_t_tip;
        w_t_tip << vx, vy, vz, w_[0], w_[1], w_[2];

        // update duration relation variables
        duration_since_kalman_ += last_period_;
        weighted_sum_w_t_tip_ += w_t_tip * last_period_;

        // check if distance data is valid
        if( msg_distance_.data < 0 )
            status_distance = NoData;

        if( msg_distance_.data > max_valid_distance_ )
            status_distance = NoData;

        // check whether new distance data is available
        if( status_distance != NewData )
        {
            // unusable distance data
            // check if it is necessary to generate intermediate distance data
            // for kalman filter, based on the previous plane model
            if( duration_since_kalman_ > max_no_kalman_duration_
              && generateDistanceMeasurement( w_T_tip ) )
                status_distance = NewData;
        }

        if( status_distance == NewData )
        {
            // average the tip twist over the period since the last kalman run
            Eigen::Matrix < double, 6, 1 > average_w_t_tip = weighted_sum_w_t_tip_ / duration_since_kalman_;

            // apply kalman filter
            Eigen::Affine3d w_T_plane;

            if( surface_filter_.update( w_T_tip, average_w_t_tip, distance, duration_since_kalman_, w_T_plane ) )
            {
                // flag for valid plane estimation
                valid_w_T_plane_ = true;

                // write the plane pose to output port
                std::copy( w_T_plane.data( ), w_T_plane.data( ) + 16, msg_w_T_plane_.data( ) );
                out_w_T_plane_.write( msg_w_T_plane_ );

                // output an visualisation msg of type arrow to display the normal
                makeArrowMessage( w_T_plane );
            }

            // reset
            weighted_sum_w_t_tip_ = Eigen::Matrix < double, 6, 1 >::Zero( );
            duration_since_kalman_ = 0;
        }
    }

    // debug
    debugMode( );
} // updateHook

/**
  * @brief      Stop hook of the OROCOS component
  */
void SurfaceMeasurementManager::stopHook( )
{
    // set output port and msg to end value

    log( Info ) << "[" << getName( ) << "] Stopped" << endlog( );
} // stopHook

/**
  * @brief      Cleanup hook of the OROCOS component
  */
void SurfaceMeasurementManager::cleanupHook( )
{
    log( Info ) << "[" << getName( ) << "] Cleaned up" << endlog( );
} // cleanupHook

/**
  * @brief      Generate a distance measurement based on the previous plane model and
  * the current pose.
  *
  * @param[in]  _w_T_tip  The pose of the tip in the world frame.
  *
  * @return     Was a valid distance measurement generated?
  */
bool SurfaceMeasurementManager::generateDistanceMeasurement( const Eigen::Affine3d &_w_T_tip )
{
    // check if there is a plane model available to generate the measurement with
    if( !valid_w_T_plane_ )
        return false;

    Eigen::Affine3d w_T_plane;
    std::copy( msg_w_T_plane_.begin( ), msg_w_T_plane_.end( ), w_T_plane.data( ) );
    Eigen::Affine3d plane_T_tip = w_T_plane.inverse( Eigen::Isometry ) * _w_T_tip;

    // get parameters for axial distance calculation
    double h = plane_T_tip.translation( ) ( 2 );
    double cos_t = plane_T_tip.linear( ) ( 2, 2 );

    // prevent errors from numerical instability
    if( cos_t > 1 )
        cos_t = 1;

    if( cos_t < -1 )
        cos_t = -1;

    if( cos_t == 0 )
        return false;

    // get the axial distance between the tip and the plane
    msg_distance_.data = -h / cos_t;

    return true;
} // generateDistanceMeasurement

// debug

/**
  * @brief      Functions that collects and sends debugging information
  */
void SurfaceMeasurementManager::debugMode( )
{
    SurfaceDebugParameters &debug_filter = surface_filter_.getDebugParameters( );
    double th_truth = 80;
    double th_estim = debug_filter.th;
    double ep_truth = 30;
    double ep_estim = debug_filter.ep;

    double d_truth = 0;
    double d_estim = debug_filter.d;
    double d_meas = debug_filter.meas_d;

    simplifyAngle( th_truth, ep_truth );
    simplifyAngle( th_estim, ep_estim );

    std::vector < double > data( NUMBER_DEBUG_PARAMETERS, 0 );
    data[ 0 ] = th_truth * 180 / M_PI;
    data[ 1 ] = th_estim * 180 / M_PI;
    data[ 2 ] = ep_truth * 180 / M_PI;
    data[ 3 ] = ep_estim * 180 / M_PI;
    data[ 4 ] = d_truth * 1000;
    data[ 5 ] = d_estim * 1000;
    data[ 6 ] = d_meas * 1000;

    data[7] = p_[0] * 1000;
    data[8] = p_[1] * 1000;
    data[9] = p_[2] * 1000;
    data[10] = v_[0] * 1000;
    data[11] = v_[1] * 1000;
    data[12] = v_[2] * 1000;
    data[13] = w_[0] / 3.14 * 180;
    data[14] = w_[1] / 3.14 * 180;
    data[15] = w_[2] / 3.14 * 180;

    Eigen::Vector3d n( msg_w_T_plane_.data() + 8 );
    Eigen::Vector3d n_ref1( 0, 0, 1 );
    Eigen::Vector3d n_ref2( 0, 0, -1 );

    double error_angle1 = std::abs( std::acos( n.dot( n_ref1 ) ) );
    double error_angle2 = std::abs( std::acos( n.dot( n_ref2 ) ) );
    double error_angle = std::min( error_angle1, error_angle2 ) * 180. / M_PI;

    data[ 16 ] = error_angle;
    data[ 17 ] = d_meas * 1000 * 10;

//    data[ 7 ] = debug_filter.meas_dx;
//    data[ 8 ] = debug_filter.meas_dy;
//    data[ 9 ] = debug_filter.meas_dz;
//    data[ 10 ] = debug_filter.meas_a * 180. / M_PI;
//    data[ 11 ] = debug_filter.meas_da * 180. / M_PI;
//    data[ 12 ] = debug_filter.meas_b * 180. / M_PI;
//    data[ 13 ] = debug_filter.meas_db * 180. / M_PI;
//    data[ 14 ] = debug_filter.meas_d;
//    data[ 15 ] = last_period_;
//    data[ 16 ] = ( debug_filter.meas_a - last_a_ ) / last_period_ * 180. / M_PI;
//    data[ 17 ] = ( debug_filter.meas_b - last_b_ ) / last_period_ * 180. / M_PI;
//    last_a_ = debug_filter.meas_a;
//    last_b_ = debug_filter.meas_b;

    msg_debug_.data = data;
    out_debug_.write( msg_debug_ );
} // debugMode

void SurfaceMeasurementManager::makeArrowMessage( const Eigen::Affine3d &_w_T_plane )
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    std::vector < geometry_msgs::Point > pts;
    geometry_msgs::Point pt;

    pt.x = 0; pt.y = 0; pt.z = 0;
    pts.push_back( pt );

    pt.x = _w_T_plane( 0, 2 ); pt.y = _w_T_plane( 1, 2 ); pt.z = _w_T_plane( 2, 2 );
    pts.push_back( pt );

    marker.points = pts;

//    Eigen::Quaterniond quat( _w_T_plane.linear( ) );
//    marker.pose.orientation.x = quat.x( );
//    marker.pose.orientation.y = quat.y( );
//    marker.pose.orientation.z = quat.z( );
//    marker.pose.orientation.w = quat.w( );
//    marker.pose.position.x = 0;
//    marker.pose.position.y = 0;
//    marker.pose.position.z = 0;
    marker.scale.x = 0.025;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    out_normal_arrow_.write( marker );

} // makeArrowMessage

/**
  * @brief      Simplify azimuth and altitude angles to a range around 0
  *
  * @param      _th   Altitude angle
  * @param      _ep   Azimuth angle
  */
void SurfaceMeasurementManager::simplifyAngle( double &_th, double &_ep )
{
    _th = _th - boost::math::round( _th / ( M_PI * 2 ) ) * M_PI * 2;

    if( _th < 0 )
        _th += M_PI;

    if( _th > M_PI / 2. )
    {
        _th = M_PI - _th;
        _ep -= M_PI;
    }

    _ep = _ep - boost::math::round( _ep / ( M_PI * 2 ) ) * M_PI * 2;
} // simplifyAngle

ORO_CREATE_COMPONENT( SurfaceMeasurementManager )
