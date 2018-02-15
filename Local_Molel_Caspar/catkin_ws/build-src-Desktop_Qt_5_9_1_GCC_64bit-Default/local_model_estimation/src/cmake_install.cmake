# Install script for directory: /home/eec/Documents/Mouloud/Code_V2/catkin_ws/src/local_model_estimation/src

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/local_model_estimation/libsurface_measurement_manager-gnulinux.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/local_model_estimation/libsurface_measurement_manager-gnulinux.so")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/local_model_estimation/libsurface_measurement_manager-gnulinux.so"
         RPATH "/opt/ros/indigo/lib/orocos/gnulinux/rtt_std_msgs/plugins:/opt/ros/indigo/lib/orocos/gnulinux/rtt_std_msgs/types:/opt/ros/indigo/lib:/opt/ros/indigo/lib/orocos/gnulinux/rtt_roscomm/plugins:/opt/ros/indigo/lib/orocos/gnulinux:/opt/ros/indigo/lib/orocos/gnulinux/plugins:/opt/ros/indigo/lib/orocos/gnulinux/types:/usr/local/lib/orocos/gnulinux/local_model_estimation:/usr/local/lib/orocos/gnulinux/local_model_estimation/types:/usr/local/lib/orocos/gnulinux/local_model_estimation/plugins:/usr/local/lib")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/local_model_estimation" TYPE SHARED_LIBRARY FILES "/home/eec/Documents/Mouloud/Code_V2/catkin_ws/build-src-Desktop_Qt_5_9_1_GCC_64bit-Default/devel/lib/orocos/gnulinux/local_model_estimation/libsurface_measurement_manager-gnulinux.so")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/local_model_estimation/libsurface_measurement_manager-gnulinux.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/local_model_estimation/libsurface_measurement_manager-gnulinux.so")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/local_model_estimation/libsurface_measurement_manager-gnulinux.so"
         OLD_RPATH "/opt/ros/indigo/lib:/opt/ros/indigo/lib/orocos/gnulinux/rtt_std_msgs/plugins:/opt/ros/indigo/lib/orocos/gnulinux/rtt_std_msgs/types:/opt/ros/indigo/lib/orocos/gnulinux/rtt_roscomm/plugins:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::"
         NEW_RPATH "/opt/ros/indigo/lib/orocos/gnulinux/rtt_std_msgs/plugins:/opt/ros/indigo/lib/orocos/gnulinux/rtt_std_msgs/types:/opt/ros/indigo/lib:/opt/ros/indigo/lib/orocos/gnulinux/rtt_roscomm/plugins:/opt/ros/indigo/lib/orocos/gnulinux:/opt/ros/indigo/lib/orocos/gnulinux/plugins:/opt/ros/indigo/lib/orocos/gnulinux/types:/usr/local/lib/orocos/gnulinux/local_model_estimation:/usr/local/lib/orocos/gnulinux/local_model_estimation/types:/usr/local/lib/orocos/gnulinux/local_model_estimation/plugins:/usr/local/lib")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/local_model_estimation/libsurface_measurement_manager-gnulinux.so")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/orocos/local_model_estimation" TYPE FILE FILES
    "/home/eec/Documents/Mouloud/Code_V2/catkin_ws/src/local_model_estimation/src/surface_measurement_manager_component.hpp"
    "/home/eec/Documents/Mouloud/Code_V2/catkin_ws/src/local_model_estimation/src/surface_identification_filter.hpp"
    "/home/eec/Documents/Mouloud/Code_V2/catkin_ws/src/local_model_estimation/src/surface_identification_config.hpp"
    "/home/eec/Documents/Mouloud/Code_V2/catkin_ws/src/local_model_estimation/src/surface_prediction_model.hpp"
    "/home/eec/Documents/Mouloud/Code_V2/catkin_ws/src/local_model_estimation/src/surface_measurement_model.hpp"
    "/home/eec/Documents/Mouloud/Code_V2/catkin_ws/src/local_model_estimation/src/extended_kalman_filter.hpp"
    "/home/eec/Documents/Mouloud/Code_V2/catkin_ws/src/local_model_estimation/src/kalman_model.hpp"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

