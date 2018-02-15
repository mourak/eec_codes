require "rttlib"
require "rttros"
require "os"

-- ------------------------------------------ --
-- Get the deployer and the necessary imports --
-- ------------------------------------------ --

-- get the deployer
tc=rtt.getTC()
if tc:getName() == "lua" then
  dep=tc:getPeer("Deployer")     
elseif tc:getName() == "Deployer" then
  dep=tc
end

-- do the necessary imports
dep:import("ocl")
dep:import("rtt_ros")
dep:import("rtt_rosnode")
dep:import("rtt_rospack")
dep:import("rtt_std_msgs")
dep:import("rtt_geometry_msgs")
dep:import("rtt_visualization_msgs")
dep:import("rtt_rosclock")

-- ------------------ --
-- General properties --
-- ------------------ --

-- get project name
project_name = "local_model_estimation"
project_folder = rtt.provides("ros"):find(project_name)

-- period
-- choose the periode of the acquisition process of the distance
period = 1/100;

-- distance measurement
-- put the name of the topic you are using for the distance 
rostopic_distance = "/Ascan/distanceMF"

-- -------------- --
-- Load component --
-- -------------- --

component_data = 
{  
    name     = "SurfaceMeasurementManager",
    class    = "SurfaceMeasurementManager",
    location = project_name,
    activity = period,
}

rtt.provides("ros"):import(component_data["location"])
dep:loadComponent(component_data["name"], component_data["class"])
meas = dep:getPeer(component_data["name"])
meas:addPeer(dep)
dep:setActivity(component_data["name"],component_data["activity"],1,1)

-- ------------------- --
-- Configure component --
-- ------------------- --

meas:getProperty("filter_config_path"):set(project_folder.."/config/surface_identification_config.xml")
meas:getProperty("max_no_kalman_duration"):set(0)
meas:getProperty("max_valid_distance"):set(3.8 / 1000.)

meas:configure()

-- ----------------- --
-- Connect component --
-- ----------------- --

cp = rtt.Variable('ConnPolicy')
cp.type = 0 -- type DATA
cp_ros = rtt.Variable('ConnPolicy')
cp_ros.transport = 3   -- transport layer: ROS


-- connect result
--dep:connect( meas:getName()..".out_w_T_plane", TODO_COMPONENT:getName()..".TODO_PORT", cp )

-- connect debugging information on kalman filter
cp_ros.name_id = '/kalman' -- ROS topic name
dep:stream(meas:getName()..".out_debug",cp_ros)

cp_ros.name_id = rostopic_distance -- ROS topic name
dep:stream(meas:getName()..".in_distance",cp_ros)

--Get the data of the robot 
cp_ros.name_id = '/Robot/RTP_Pos' -- ROS topic name
dep:stream(meas:getName()..".in_ee_T_tip",cp_ros)

cp_ros.name_id = '/Robot/RTP_Vel' -- ROS topic name
dep:stream(meas:getName()..".in_ee_t_tip",cp_ros)

cp_ros.name_id = '/plane_normal' -- ROS topic name
dep:stream(meas:getName()..".out_normal_arrow",cp_ros)

-- ----------------- --
-- Start component --
-- ----------------- --

meas:start()
