#include "ros/ros.h"
#include "x_eureyecase_messages_generation/OCTRaw.h"
#include "geometry_msgs/Vector3.h" 
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <iostream>
#include <vector>
#include <math.h>
#include <algorithm>
#include <iterator>
#include <ros/node_handle.h>
#include <fstream>


/* creat a class to manage the different elements */

class Ascan2Distance{
private:
    ros::Subscriber sub_Ascan;
    ros::Subscriber sub_Ascan2dist;
    ros::NodeHandle nd;
    ros::Publisher distance_pub;


public:
    Ascan2Distance();
    void Ascan2distCallback(const x_eureyecase_messages_generation::OCTRaw::ConstPtr& msg);
    std::vector<float> medianfilter(std::vector<float> signal, int N, int size_mf);
};


//Ascan2Distance Read;


// Member functions definitions including constructor
Ascan2Distance::Ascan2Distance() {

   //show that the class is create	
   std::cout << "Publishing the distance measured by the OCT sensor ..." << std::endl;
   std::cout << " ..." << std::endl;
   std::cout << "Press Ctrl+C to stop the distance publication" << std::endl;

   // initialize the publisher
   distance_pub = nd.advertise<std_msgs::Float64>("/Ascan/distanceMF", 1000);
   // Publish in a topic
   sub_Ascan2dist = nd.subscribe("/OCT_CAM/OCT_CAM_RAW", 10, &Ascan2Distance::Ascan2distCallback, this);
}



/**
 * This call back to get the data of the ascan from node /OCT_CAM/OCT_CAM_RAW.
 * Where the ros package is x_eureyecase_messages_generation
 * And the defined message is OCTRaw
 **/
void Ascan2Distance::Ascan2distCallback(const x_eureyecase_messages_generation::OCTRaw::ConstPtr& msg)
{
  int w = msg->width;
  int h = msg->height;
  std::vector<float> ascan = msg->data;

  int N = ascan.size();
  int size_mf = 3;
  std::vector<float> ascan_filtred = medianfilter(ascan, N, size_mf);

  // define the max min values of the point cloud
  float min = 25;
  float max = 35;

  // Threshold vector
  std::vector<float> ascan_threshold;

  for (int i=0; i<ascan.size(); i++){
      if (ascan[i] < min)
          ascan_threshold.push_back(0.0);
      else if (ascan[i] > max)
          ascan_threshold.push_back(1.0);
      else
          ascan_threshold.push_back((ascan[i] - min)/(max - min));
   }

  std::vector<float>::iterator maxElement;
  maxElement = max_element(ascan_threshold.begin(), ascan_threshold.end());

  int dist_pix = distance(ascan_threshold.begin(), maxElement);
  float dist_mm = dist_pix * 0.0039 / 1024;

  std_msgs::Float64 msg_pub;
  msg_pub.data = dist_mm;
  distance_pub.publish(msg_pub);

  //ROS_INFO("dist_mm = %f ", dist_mm);
}



//   1D MEDIAN FILTER implementation
//     signal - input signal
//     result - output signal
//     N      - length of the signal
std::vector<float> Ascan2Distance::medianfilter(std::vector<float> signal, int N, int size_mf)
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
    ros::init(argc, argv, "ascan_distance");
    Ascan2Distance ascan2distance;
    ros::spin();
    return 0;
}
