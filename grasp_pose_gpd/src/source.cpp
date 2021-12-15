#include "ros/ros.h"
#include <gpd/grasp_detector.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/server/simple_action_server.h>
#include <grasp_pose_gpd/GraspPoseAction.h>
#include <grasp_pose_gpd/grasp_detection.h>



 void convertDoublesToEigen(std::vector<double> values, Eigen::Isometry3d &transform)
 {
   if (values.size() != 6)
   {
     std::cout << "Invalid number of doubles provided for transform, size= " <<values.size()  << "\n";
   }
 
   Eigen::AngleAxisd roll_angle(values[3], Eigen::Vector3d::UnitX());
   Eigen::AngleAxisd pitch_angle(values[4], Eigen::Vector3d::UnitY());
   Eigen::AngleAxisd yaw_angle(values[5], Eigen::Vector3d::UnitZ());
   Eigen::Quaternion<double> quaternion = roll_angle * pitch_angle * yaw_angle;
 
   transform = Eigen::Translation3d(values[0], values[1], values[2]) * quaternion;
 
 }


GraspDetection::GraspDetection(const ros::NodeHandle& nh) : nh_(nh)
{
    ros::param::get("~action_name", action_name_);
    // action server
    server_.reset(new actionlib::SimpleActionServer<grasp_pose_gpd::GraspPoseAction>(
      nh_, action_name_, false));
    server_->registerGoalCallback(std::bind(&GraspDetection::goalCallback, this));
    server_->registerPreemptCallback(std::bind(&GraspDetection::preemptCallback, this));
    server_->start();
    ros::param::get("~camera_position", camera_position);
    ros::param::get("~config_file", config_file);
    std::vector<double> trans_base_cam_values;
    std::vector<double> trans_grasp_to_approach_point_values;
    std::vector<double> trans_grasp_to_another_pose_values;
    ros::param::get("~trans_base_cam", trans_base_cam_values);
    ros::param::get("~trans_grasp_to_approach_point", trans_grasp_to_approach_point_values);
    ros::param::get("~trans_grasp_to_another_pose", trans_grasp_to_another_pose_values);
    convertDoublesToEigen(trans_base_cam_values, trans_base_cam_);
    convertDoublesToEigen(trans_grasp_to_approach_point_values, trans_grasp_to_approach_point_);
    convertDoublesToEigen(trans_grasp_to_another_pose_values, trans_grasp_to_another_pose_);
    // Grasp detector
    detector.reset(new gpd::GraspDetector(config_file));
}

void GraspDetection::goalCallback()
{
    goal_file_ = server_->acceptNewGoal()->action_name;
    ROS_INFO("New goal accepted: %s", goal_file_.c_str());
    // use GPD to find the grasp candidates
    sampleGrasps();
    
}

void GraspDetection::preemptCallback()
{
    ROS_INFO("Preempted %s", goal_file_.c_str());
    server_->setPreempted();
}

void GraspDetection::sampleGrasps()
{
    pcd_file = goal_file_;
    Eigen::Matrix3Xd view_points(3, 1);
    view_points << camera_position[0], camera_position[1], camera_position[2];
    cloud.reset(new gpd::util::Cloud(pcd_file, view_points));
    detector->preprocessPointCloud(*cloud);
    std::vector<std::unique_ptr<gpd::candidate::Hand>> grasps = detector->detectGrasps(*cloud);
    // Check if grasp score > 0
    if (grasps.at(0)->getScore() > 0.0)
    {
      Eigen::Isometry3d transform_cam_grasp = Eigen::Translation3d(grasps.at(0)->getPosition()) * Eigen::Quaterniond(grasps.at(0)->getOrientation());
      Eigen::Isometry3d transform_base_grasp_pose_one = trans_base_cam_*transform_cam_grasp;
      Eigen::Isometry3d transform_base_approach_point_pose_one = trans_base_cam_*transform_cam_grasp*trans_grasp_to_approach_point_;
      // A second pose for grasping, if robot arm can not reach the first pose because of the gripper direction, it can try with the second pose.
      Eigen::Isometry3d transform_base_grasp_pose_two = trans_base_cam_*transform_cam_grasp*trans_grasp_to_another_pose_;
      Eigen::Isometry3d transform_base_approach_point_pose_two = trans_base_cam_*transform_cam_grasp*trans_grasp_to_another_pose_*trans_grasp_to_approach_point_;

      Eigen::Vector3d trans_base_grasp_pose_one = transform_base_grasp_pose_one.translation();
      Eigen::Vector3d trans_camera_grasp = transform_cam_grasp.translation();
      Eigen::Vector3d trans_base_approach_point_pose_one = transform_base_approach_point_pose_one.translation();
      Eigen::Vector3d trans_base_grasp_pose_two = transform_base_grasp_pose_two.translation();
      Eigen::Vector3d trans_base_approach_point_pose_two = transform_base_approach_point_pose_two.translation();

      Eigen::Quaterniond rot_base_grasp_pose_one(transform_base_grasp_pose_one.rotation());
      Eigen::Quaterniond rot_camera_grasp(transform_cam_grasp.rotation());
      Eigen::Quaterniond rot_base_approach_point_pose_one(transform_base_approach_point_pose_one.rotation());
      Eigen::Quaterniond rot_base_grasp_pose_two(transform_base_grasp_pose_two.rotation());
      Eigen::Quaterniond rot_base_approach_point_pose_two(transform_base_approach_point_pose_two.rotation());

      geometry_msgs::PoseStamped grasp_pose_robot_link_pose_one;
      geometry_msgs::PoseStamped grasp_pose_camera_link;
      geometry_msgs::PoseStamped approach_pose_robot_link_pose_one;
      geometry_msgs::PoseStamped grasp_pose_robot_link_pose_two;
      geometry_msgs::PoseStamped approach_pose_robot_link_pose_two;
      //frame of the grasps in robot coordinate pose one
      grasp_pose_robot_link_pose_one.pose.position.x = trans_base_grasp_pose_one.x();
      grasp_pose_robot_link_pose_one.pose.position.y = trans_base_grasp_pose_one.y();
      grasp_pose_robot_link_pose_one.pose.position.z = trans_base_grasp_pose_one.z();

      grasp_pose_robot_link_pose_one.pose.orientation.w = rot_base_grasp_pose_one.w();
      grasp_pose_robot_link_pose_one.pose.orientation.x = rot_base_grasp_pose_one.x();
      grasp_pose_robot_link_pose_one.pose.orientation.y = rot_base_grasp_pose_one.y();
      grasp_pose_robot_link_pose_one.pose.orientation.z = rot_base_grasp_pose_one.z();

      //frame of the grasps in robot coordinate pose two
      grasp_pose_robot_link_pose_two.pose.position.x = trans_base_grasp_pose_two.x();
      grasp_pose_robot_link_pose_two.pose.position.y = trans_base_grasp_pose_two.y();
      grasp_pose_robot_link_pose_two.pose.position.z = trans_base_grasp_pose_two.z();

      grasp_pose_robot_link_pose_two.pose.orientation.w = rot_base_grasp_pose_two.w();
      grasp_pose_robot_link_pose_two.pose.orientation.x = rot_base_grasp_pose_two.x();
      grasp_pose_robot_link_pose_two.pose.orientation.y = rot_base_grasp_pose_two.y();
      grasp_pose_robot_link_pose_two.pose.orientation.z = rot_base_grasp_pose_two.z();

      //frame of the grasps in camera coordinate
      grasp_pose_camera_link.pose.position.x = trans_camera_grasp.x();
      grasp_pose_camera_link.pose.position.y = trans_camera_grasp.y();
      grasp_pose_camera_link.pose.position.z = trans_camera_grasp.z();

      grasp_pose_camera_link.pose.orientation.w = rot_camera_grasp.w();
      grasp_pose_camera_link.pose.orientation.x = rot_camera_grasp.x();
      grasp_pose_camera_link.pose.orientation.y = rot_camera_grasp.y();
      grasp_pose_camera_link.pose.orientation.z = rot_camera_grasp.z();

      //frame of the approach pose in robot coordinate pose one
      approach_pose_robot_link_pose_one.pose.position.x = trans_base_approach_point_pose_one.x();
      approach_pose_robot_link_pose_one.pose.position.y = trans_base_approach_point_pose_one.y();
      approach_pose_robot_link_pose_one.pose.position.z = trans_base_approach_point_pose_one.z();

      approach_pose_robot_link_pose_one.pose.orientation.w = rot_base_approach_point_pose_one.w();
      approach_pose_robot_link_pose_one.pose.orientation.x = rot_base_approach_point_pose_one.x();
      approach_pose_robot_link_pose_one.pose.orientation.y = rot_base_approach_point_pose_one.y();
      approach_pose_robot_link_pose_one.pose.orientation.z = rot_base_approach_point_pose_one.z();

      //frame of the approach pose in robot coordinate pose two
      approach_pose_robot_link_pose_two.pose.position.x = trans_base_approach_point_pose_two.x();
      approach_pose_robot_link_pose_two.pose.position.y = trans_base_approach_point_pose_two.y();
      approach_pose_robot_link_pose_two.pose.position.z = trans_base_approach_point_pose_two.z();

      approach_pose_robot_link_pose_two.pose.orientation.w = rot_base_approach_point_pose_two.w();
      approach_pose_robot_link_pose_two.pose.orientation.x = rot_base_approach_point_pose_two.x();
      approach_pose_robot_link_pose_two.pose.orientation.y = rot_base_approach_point_pose_two.y();
      approach_pose_robot_link_pose_two.pose.orientation.z = rot_base_approach_point_pose_two.z();

      result_.grasp_candidate_robot_link.emplace_back(grasp_pose_robot_link_pose_one);
      result_.grasp_candidate_robot_link.emplace_back(grasp_pose_robot_link_pose_two);
      result_.grasp_candidate_camera_link = grasp_pose_camera_link;
      result_.approach_pose_robot_link.emplace_back(approach_pose_robot_link_pose_one);
      result_.approach_pose_robot_link.emplace_back(approach_pose_robot_link_pose_two);

      server_->setSucceeded(result_);
    }
    else
    {
      feedback_.feedback = "failed";
      server_->setAborted(result_);
    }

}
