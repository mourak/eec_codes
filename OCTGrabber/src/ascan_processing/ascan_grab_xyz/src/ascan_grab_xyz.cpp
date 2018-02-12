#include "ros/ros.h"
#include "x_eureyecase_messages_generation/OCTRaw.h"
#include "geometry_msgs/Vector3.h" 
#include "std_msgs/String.h"
#include <vector>
#include <math.h>
#include <ros/node_handle.h>
#include <fstream>

// PCL LIBRERIES

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/io/pcd_io.h>
#include <pcl/point_types.h>


typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;




/* creat a class to manage the different elements */

class AscanCscan{
private:
    float x,y,z;
    float prev_x, prev_y, prev_z;
    bool changePos;

    int num_pose;
    int num_ascan;
    bool Stop_motion;
    bool Start_Stop_acqui;

    std::string scannig_platform;

protected:
    ros::Subscriber sub_XYZ;
    ros::Subscriber sub_Ascan;
    pcl::PointCloud<pcl::PointXYZI>::Ptr Cscan;

public:
    AscanCscan();
    void Cmd_Ros_Callback(const std_msgs::String::ConstPtr& msg);
    void ascanCallback(const x_eureyecase_messages_generation::OCTRaw::ConstPtr& msg);
    void XYZCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void Stop_Acquitision();
    bool getStopFlag();
    pcl::PointCloud<pcl::PointXYZI>::Ptr getPointCloud();
    bool SavePCDfile();
    std::vector<float> medianfilter(std::vector<float> signal, int N, int size_mf);
};


AscanCscan Read;

// Member functions definitions including constructor
AscanCscan::AscanCscan(void) :  Cscan (new pcl::PointCloud<pcl::PointXYZI>) {
   //PointCloud::Ptr Cscan (new PointCloud);
   num_ascan = 0;
   num_pose = 0;
   std::cout << "Class is being created" << std::endl;
}

/**
 * This call back to get the data of the ascan from node /OCT_CAM/OCT_CAM_RAW.
 * Where the ros package is x_eureyecase_messages_generation
 * And the defined message is OCTRaw
 **/
void AscanCscan::ascanCallback(const x_eureyecase_messages_generation::OCTRaw::ConstPtr& msg)
{
  /*Check what is your scanning platforme
    Choose between the robot or the xyz stage
    */
  std::cout << "Please enter the name of the scanning platform "<< std::endl;
  std::cout << "Type : robot or xyz "<< std::endl;
  std::cin >>  scannig_platform ;

  std::cout << "the scanning platform is :  " << scannig_platform << std::endl;


  int w = msg->width;
  int h = msg->height;
  std::vector<float> ascan = msg->data;

  int N = ascan.size();
  int size_mf = 5;
  std::vector<float> ascan_filtred = medianfilter(ascan, N, size_mf);

  // define the max min values of the point cloud
  float min = 20;
  float max = 35;


  if (x!= prev_x  || y!=prev_y || z!=prev_z){
      //save the new Ascan vector in a new position
      //ROS_INFO("X = %f, Y = %f, Z = %f",prev_x , prev_y , prev_z);
      //std::cout << " position Changed ----------" <<  std::endl;
      prev_x = x;
      prev_y = y;
      prev_z = z;

      // Application on XYZ stage
      //-------------------------------------------------------------------
      //PointCloud::Ptr Cscan (new PointCloud);
            pcl::PointXYZI pointI;
            pointI.x = x;
            pointI.y = y;
            for (int i=0; i<ascan_filtred.size(); i++){
                pointI.z = z+((i*0.0039)/1024);
                if (ascan_filtred[i] < min)
                    pointI.intensity = 0.0;
                else if (ascan_filtred[i] > max)
                    pointI.intensity = 1.0;
                else
                    pointI.intensity = (ascan_filtred[i] - min)/(max - min);
                Cscan -> points.push_back(pointI);
                num_ascan++;
            }
            num_pose ++;
      //ROS_INFO("num pose = %d",num_pose);
      //-------------------------------------------------------------------


  }
  else{
      //compute the mean of the nez ascan with the existing one in the same position
      //ROS_INFO("X = %f, Y = %f, Z = %f",prev_x , prev_y , prev_z);
      //std::cout << " position Unchanged ----------" <<  std::endl;
      float tmp ;

      for (int i=0; i<ascan_filtred.size(); i++){
          if (ascan[i] < min)
              tmp = 0.0;
          else if (ascan[i] > max)
              tmp = 1.0;
          else
              tmp = (ascan_filtred[i] - min)/(max - min);
      Cscan->points[num_ascan-(ascan.size()-i)].intensity = (tmp + Cscan->points[num_ascan-(ascan_filtred.size()-i)].intensity)/2;
      }
  }
  //std::cout << " size of cscan  ::  " << Cscan->points.size () << std::endl;

}

/**
 * This call back to get the data of the ascan from node /OCT_CAM/OCT_CAM_RAW.
 * Where the ros package is x_eureyecase_messages_generation
 * And the defined message is OCTRaw
 */
void AscanCscan::XYZCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    x = msg->x;
    y = msg->y;
    z = msg->z;
  //ROS_INFO("X = %f, Y = %f, Z = %f",x , y , z);

    // in the robot case we extract the angle of each encodeur using the /joint topic

}


/**
 * This call back to get the flag on the acquisition process.
 * Send form orocos component as a string message
 *  value of the scan running = "start_oct_recording"
 *  value of the scan stoped = "stop_oct_recording"
 */
void AscanCscan::Cmd_Ros_Callback(const std_msgs::String::ConstPtr& msg)
{
//    std::cout << "acquisition mode :  " << msg->data << std::endl;
    ros::NodeHandle nh;

    if (msg->data == "start_oct_recording"){
        //std::cout << msg->data << std::endl;
        std::cout << " ##  - - - - -  recording - - - - - - " << std::endl;
        /** --------------------------------------------------------
        *   subscribe to the XYZ stage positions
        *   ---------------------------------------------------------- **/
        sub_XYZ = nh.subscribe("/XYZ_Position", 10, &AscanCscan::XYZCallback, &Read);

        /** --------------------------------------------------------
        *   subscribe to the ascan acquisition
        *   For the moment we choose to have a buffer of 10 ascans
        *   This value can be changed if necessery
        *   And the variables that we read are only OCT Ascan Raw data
        *   ---------------------------------------------------------- **/
        sub_Ascan = nh.subscribe("/OCT_CAM/OCT_CAM_RAW", 10, &AscanCscan::ascanCallback, &Read);

    }else if (msg->data == "stop_oct_recording"){
        //std::cout << msg->data << std::endl;
        std::cout << " ## ************************* - - - - - Stop recording - - - - - - *************************" << std::endl;
        sub_XYZ.shutdown();
        sub_Ascan.shutdown();
    }else{
        ROS_INFO("-- The flag is not correct as defined --");
    }

    ros::spin();
}
/**
 * Method to indicate that the motion is finish
 */
void AscanCscan::Stop_Acquitision()
{
    if (num_pose >= 30){
        ROS_INFO("-- The acquisition has reached the number of points required --");
        Stop_motion =  true;
    }
    else
        Stop_motion = false;
}

/**
 * Get the stop flag
 */
bool AscanCscan::getStopFlag(){

    return Stop_motion;
}
pcl::PointCloud<pcl::PointXYZI>::Ptr AscanCscan::getPointCloud(){

    return Cscan;
}

bool AscanCscan::SavePCDfile(){
    std::cout << "Cscan size value "<< Cscan->size() << std::endl;
    Cscan->width = 1;
    Cscan->height = Cscan->points.size();
   std::time_t t = std::time(NULL);
   std::tm *ptm = std::localtime(&t);
   std::stringstream ss;
   ss << ptm->tm_year+1900 << "_" << std::setw(2) << ptm->tm_mon+1<< "_" << ptm->tm_mday << "_"<< ptm->tm_hour << "_"<< ptm->tm_min+1<< "_" << ptm->tm_sec+1;
    pcl::io::savePCDFile(("/home/eec/Documents/Mouloud/Data/DataA2Ccorrection/Mouloud/A2Bscan_"+ss.str()+".pcd").c_str(), *Cscan );
    return true;
}



//   1D MEDIAN FILTER implementation
//     signal - input signal
//     result - output signal
//     N      - length of the signal
std::vector<float> AscanCscan::medianfilter(std::vector<float> signal, int N, int size_mf)
{
   std::vector<float> filtered_signal(N);
   //   half of the filter size
   int hs_mf_down = int (floor(size_mf/2));
   int hs_mf_up = hs_mf_down + 1;

   //   Move window through all elements of the signal
   for (int i = hs_mf_down; i < N - hs_mf_down; ++i)
   {
      //   Pick up window elements
      std::vector<float> window(size_mf);
      for (int j = 0; j < size_mf; ++j)
         window.push_back(signal.at(i - hs_mf_down + j));
      //   Order elements (only half of them)
      for (int j = 0; j < hs_mf_up; ++j)
      {
         //   Find position of minimum element
         int min = j;
         for (int k = j + 1; k < size_mf; ++k)
            if (window[k] < window[min])
               min = k;
         //   Put found minimum element in its place
         float temp = window[j];
         window[j] = window[min];
         window[min] = temp;
      }
      //   Get result - the middle element
      filtered_signal[i - hs_mf_down] = window[hs_mf_down];
   }
   return filtered_signal;
}



int main(int argc, char **argv)
{



    AscanCscan Ascan2Cscan;
   //Ascan2Cscan();

    ros::init(argc, argv, "ascan_grab");

    ros::Subscriber sub_XYZ;
    ros::Subscriber sub_Ascan;

    ros::NodeHandle n;
    ros::NodeHandle nh;
    while(ros::ok()){

    //ros::Subscriber sub_cmd_ros = n.subscribe("/command_to_ros", 10, &AscanCscan::Cmd_Ros_Callback, &Ascan2Cscan);
    //ros::Subscriber sub_cmd_ros = n.subscribe("/STATE_MACHINE/STATE_MACHINE_CTRL", 10, &AscanCscan::Cmd_Ros_Callback, &Ascan2Cscan);


      /** --------------------------------------------------------
      *   subscribe to the XYZ stage positions
      *   ---------------------------------------------------------- **/
      ros::Subscriber sub_XYZ = n.subscribe("/XYZ_Position", 10, &AscanCscan::XYZCallback, &Ascan2Cscan);

      /** --------------------------------------------------------
      *   subscribe to the ascan acquisition
      *   For the moment we choose to have a buffer of 10 ascans
      *   This value can be changed if necessery
      *   And the variables that we read are only OCT Ascan Raw data
      *   ---------------------------------------------------------- **/

      ros::Subscriber sub_Ascan = n.subscribe("/OCT_CAM/OCT_CAM_RAW", 10, &AscanCscan::ascanCallback, &Ascan2Cscan);

   ros::spin();
  }

    // save the point cloud as pcd file
    Read.SavePCDfile();
    std::cout << " Ros is stopped :: time to save the data as .PCD" << std::endl;



    // and also publish the PointCloud


  return 0;
}
