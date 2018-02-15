#ifndef SURFACE_MEASUREMENT_MANAGER_HPP
#define SURFACE_MEASUREMENT_MANAGER_HPP

#include "surface_identification_filter.h"

#include <rtt/RTT.hpp>
#include <Eigen/Geometry>

// debug
#define NUMBER_DEBUG_PARAMETERS 18
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include "geometry_msgs/Vector3.h"
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <boost/math/special_functions/round.hpp>

/**
  * @brief      Component that manages the data concerning
  *             a planar surface that is estimated from axial
  *             distance measurements.
  */
class SurfaceMeasurementManager : public RTT::TaskContext
{
public:

    SurfaceMeasurementManager( std::string const &_name );
    bool configureHook( );
    bool startHook( );
    void updateHook( );
    void stopHook( );
    void cleanupHook( );

protected:

    // operations

    // properties
    std::string prop_filter_config_path_;
    double prop_max_no_kalman_duration_;
    double prop_max_valid_distance_;
    int prop_error_;

    // ports
    RTT::InputPort < std::vector < double > > in_w_T_ee_;
    RTT::InputPort < std::vector < double > > in_w_t_ee_;

//    RTT::InputPort < geometry_msgs::Vector3 > in_ee_T_tip_;
//    RTT::InputPort < geometry_msgs::Vector3 > in_ee_t_tip_;
    RTT::InputPort < geometry_msgs::Vector3 > in_ee_T_tip_;
    RTT::InputPort < geometry_msgs::Vector3 > in_ee_t_tip_;
    RTT::InputPort < std_msgs::Float64 > in_distance_;
    RTT::OutputPort < visualization_msgs::Marker > out_normal_arrow_;
    RTT::OutputPort < std::vector < double > > out_w_T_plane_;
    RTT::OutputPort < int > out_error_;

private:

    // member functions
    bool generateDistanceMeasurement( const Eigen::Affine3d &_w_T_tip );
    void makeArrowMessage( const Eigen::Affine3d &_w_T_plane );

    // members
    SurfaceIdentificationFilter surface_filter_;
    Eigen::Matrix < double, 6, 1 > weighted_sum_w_t_tip_;
    double duration_since_kalman_;

    // time
    RTT::os::TimeService::ticks last_timestamp_;
    RTT::os::TimeService::Seconds last_period_;

    // verified properties
    double max_no_kalman_duration_;
    double max_valid_distance_;

    // messages
    std::vector < double > msg_w_T_ee_;
    std::vector < double > msg_w_t_ee_;
    geometry_msgs::Vector3  msg_ee_T_tip_;
    geometry_msgs::Vector3  msg_ee_t_tip_;
//    std::vector < float > msg_ee_T_tip_;
//    std::vector < float > msg_ee_t_tip_;
    std_msgs::Float64 msg_distance_;
    std::vector < double > msg_w_T_plane_;

    // flags
    bool valid_w_T_plane_;

    // debug
    void debugMode( );
    void simplifyAngle( double &_th, double &_ep );
    double d_truth_;
    std_msgs::Float64MultiArray msg_debug_;
    RTT::OutputPort < std_msgs::Float64MultiArray > out_debug_;
    double last_a_;
    double last_b_;

    double v_[3];
    double w_[3];
    double p_[3];

};

#endif // End of SURFACE_MEASUREMENT_MANAGER_HPP
