
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library lib<package_name>-<target>.so
# and installs in the directory lib/orocos/<package_name>/
#
SET( SOURCES
surface_identification_filter.cpp
surface_identification_config.cpp
surface_prediction_model.cpp
surface_measurement_model.cpp
extended_kalman_filter.cpp
kalman_model.cpp
)

SET( HEADERS
surface_identification_filter.hpp
surface_identification_config.hpp
surface_prediction_model.hpp
surface_measurement_model.hpp
extended_kalman_filter.hpp
kalman_model.hpp
)

orocos_component(surface_measurement_manager surface_measurement_manager_component.cpp ${SOURCES} ) # ...you may add multiple source files
target_link_libraries(surface_measurement_manager ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})

# Additional headers:
#
# Installs in the include/orocos/<package_name>/ directory
#
orocos_install_headers(surface_measurement_manager_component.hpp ${HEADERS}) # ...you may add multiple header files
