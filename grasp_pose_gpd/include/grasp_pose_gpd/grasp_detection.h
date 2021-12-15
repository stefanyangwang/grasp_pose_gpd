#include "ros/ros.h"
#include <gpd/grasp_detector.h>
#include <actionlib/server/simple_action_server.h>
#include <grasp_pose_gpd/GraspPoseAction.h>

class GraspDetection
{
public:
    GraspDetection(const ros::NodeHandle& nh);

private:
    void goalCallback();
    void preemptCallback();
    void sampleGrasps();

    ros::NodeHandle nh_;         // node handle

    std::unique_ptr<actionlib::SimpleActionServer<grasp_pose_gpd::GraspPoseAction>>
      server_;                                                       // action server
    std::unique_ptr<gpd::GraspDetector> detector;
    std::unique_ptr<gpd::util::Cloud> cloud;      // stores point cloud

    grasp_pose_gpd::GraspPoseResult result_;      // action result message
    grasp_pose_gpd::GraspPoseFeedback feedback_;      // action result message

    std::vector<double> camera_position;
    std::string config_file;
    std::string pcd_file;
    std::string frame_id;            // frame of grasps
    std::string goal_file_;           // path_to_pcd
    std::string action_name_;         // action namespace

    Eigen::Isometry3d trans_base_cam_;     // transformation from base link to camera link
    Eigen::Isometry3d trans_grasp_to_approach_point_;     // transformation from grasp link to approach point link above object
    Eigen::Isometry3d trans_grasp_to_another_pose_;     // transformation from grasp link to another pose to let robot reach this pose

};

void convertDoublesToEigen(std::vector<double> values, Eigen::Isometry3d &transform);