#include "ros/ros.h"
#include <grasp_pose_gpd/grasp_detection.h>

int main(int argc, char **argv) {
  std::cout << "\033]0;" << "grasp_detection" << "\007";
  ros::init(argc, argv, "grasp_detection");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  GraspDetection graspdetection(nh);
  
  //std::vector<double> values;
  //ros::param::get("~trans_base_cam", values);
  //convertDoublesToEigen(values, trans_base_cam_);
  //std::cout <<  transform_opt_grasp.matrix()  << "\n";
  //ROS_INFO_STREAM(grasp_pose);
  ros::waitForShutdown();
  return 0;
}