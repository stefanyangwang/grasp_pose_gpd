#!/usr/bin/python3
from logging import StringTemplateStyle
import rospy
import actionlib
from grasp_pose_gpd.msg import *
from scipy.spatial.transform import Rotation as R
import numpy as np
import open3d as o3d
import copy


def grasp_client():
    client = actionlib.SimpleActionClient('generate_grasps', GraspPoseAction)
    print("Waiting for the gpd server ...")
    client.wait_for_server()
    print("\n --- Server ready --- \n")
    goal = GraspPoseGoal()
    goal.action_name = '/home/wang/grasp_task_gpd/src/grasp_pose_gpd/data/2021-11-24_11-32-44_grasp_image.pcd'
    client.send_goal(goal)
    client_state = client.get_state()
    while client_state != 3: # not in [2,3,4,5,8]
            client_state = client.get_state()
            # ABORTED : 4
            if client_state == 4:
                return 'Action aborted'
    print('--- Grasp detection completed ---\n')
    return client.get_result()

if __name__ == '__main__':
     try:
         rospy.init_node('grasp_client')
         result = grasp_client()
         if result == 'Action aborted':
             print('--- No grasp candidates found with a positive score ---\n')
         else:

            r = R.from_quat([result.grasp_candidate_camera_link.pose.orientation.x, result.grasp_candidate_camera_link.pose.orientation.y, result.grasp_candidate_camera_link.pose.orientation.z, result.grasp_candidate_camera_link.pose.orientation.w])
            T_object_to_camera = np.eye(4)
            T_object_to_camera[:3, :3] = r.as_matrix()
            T_object_to_camera[0, 3] = result.grasp_candidate_camera_link.pose.position.x
            T_object_to_camera[1, 3] = result.grasp_candidate_camera_link.pose.position.y
            T_object_to_camera[2, 3] = result.grasp_candidate_camera_link.pose.position.z
            print('approach point pose one')
            print('px:', result.approach_pose_robot_link[0].pose.position.x)
            print('py:', result.approach_pose_robot_link[0].pose.position.y)
            print('pz:', result.approach_pose_robot_link[0].pose.position.z)
            print('ox:', result.approach_pose_robot_link[0].pose.orientation.x)
            print('oy:', result.approach_pose_robot_link[0].pose.orientation.y)
            print('oz:', result.approach_pose_robot_link[0].pose.orientation.z)
            print('ow:', result.approach_pose_robot_link[0].pose.orientation.w)
            print('grasp point pose one')
            print('px:', result.grasp_candidate_robot_link[0].pose.position.x)
            print('py:', result.grasp_candidate_robot_link[0].pose.position.y)
            print('pz:', result.grasp_candidate_robot_link[0].pose.position.z)
            print('ox:', result.grasp_candidate_robot_link[0].pose.orientation.x)
            print('oy:', result.grasp_candidate_robot_link[0].pose.orientation.y)
            print('oz:', result.grasp_candidate_robot_link[0].pose.orientation.z)
            print('ow:', result.grasp_candidate_robot_link[0].pose.orientation.w)
            print('approach point pose two')
            print('px:', result.approach_pose_robot_link[1].pose.position.x)
            print('py:', result.approach_pose_robot_link[1].pose.position.y)
            print('pz:', result.approach_pose_robot_link[1].pose.position.z)
            print('ox:', result.approach_pose_robot_link[1].pose.orientation.x)
            print('oy:', result.approach_pose_robot_link[1].pose.orientation.y)
            print('oz:', result.approach_pose_robot_link[1].pose.orientation.z)
            print('ow:', result.approach_pose_robot_link[1].pose.orientation.w)
            print('grasp point pose two')
            print('px:', result.grasp_candidate_robot_link[1].pose.position.x)
            print('py:', result.grasp_candidate_robot_link[1].pose.position.y)
            print('pz:', result.grasp_candidate_robot_link[1].pose.position.z)
            print('ox:', result.grasp_candidate_robot_link[1].pose.orientation.x)
            print('oy:', result.grasp_candidate_robot_link[1].pose.orientation.y)
            print('oz:', result.grasp_candidate_robot_link[1].pose.orientation.z)
            print('ow:', result.grasp_candidate_robot_link[1].pose.orientation.w)
            pcd = o3d.io.read_point_cloud("/home/wang/grasp_task_gpd/src/grasp_pose_gpd/data/2021-11-24_11-32-44_grasp_image.pcd")
            coord = o3d.create_mesh_coordinate_frame(0.1, [0, 0, 0])
            T_camera_to_robot = np.eye(4)
            T_camera_to_robot[0, 0] = 0.34202014
            T_camera_to_robot[0, 1] = 0.93969262
            T_camera_to_robot[0, 2] = 0
            T_camera_to_robot[0, 3] = -0.0876
            T_camera_to_robot[1, 0] = -0.93969262
            T_camera_to_robot[1, 1] = 0.34202014
            T_camera_to_robot[1, 2] = 0
            T_camera_to_robot[1, 3] = 0.3218
            T_camera_to_robot[2, 0] = 0
            T_camera_to_robot[2, 1] = 0
            T_camera_to_robot[2, 2] = 1
            T_camera_to_robot[2, 3] = 0
            robo_link = copy.deepcopy(coord).transform(T_camera_to_robot)
            mesh_t = copy.copy(coord).transform(T_object_to_camera)
            o3d.visualization.draw_geometries([pcd, coord, robo_link, mesh_t])
     except rospy.ROSInterruptException:
         print("program interrupted before completion", file=sys.stderr)