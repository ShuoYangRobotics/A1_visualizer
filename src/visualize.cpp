 
// stl
#include <iostream>

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/TransformStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>


// visualizer
#include "robot_markers/builder.h"

// unitree message 
#include <unitree_legged_msgs/LowState.h>


// for visualize robot in rviz robomarker
robot_markers::Builder* builder;
visualization_msgs::MarkerArray a1_viz_array;
ros::Publisher marker_arr_pub;

// move joint position as global variable 
std::map<std::string, double> joint_positions;

void a1_state_callback(const unitree_legged_msgs::LowStateConstPtr& a1_state) 
{
    // ROS_INFO("received");
    // FL 
    joint_positions["FL_hip_joint"]   = a1_state -> motorState[0].q;
    joint_positions["FL_thigh_joint"] = a1_state -> motorState[1].q;
    joint_positions["FL_calf_joint"]  = a1_state -> motorState[2].q;
    // FR 
    joint_positions["FR_hip_joint"]   = a1_state -> motorState[3].q;
    joint_positions["FR_thigh_joint"] = a1_state -> motorState[4].q;
    joint_positions["FR_calf_joint"]  = a1_state -> motorState[5].q;
    // RL 
    joint_positions["RL_hip_joint"]   = a1_state -> motorState[6].q;
    joint_positions["RL_thigh_joint"] = a1_state -> motorState[7].q;
    joint_positions["RL_calf_joint"]  = a1_state -> motorState[8].q;
    // RR 
    joint_positions["RR_hip_joint"]   = a1_state -> motorState[9].q;
    joint_positions["RR_thigh_joint"] = a1_state -> motorState[10].q;
    joint_positions["RR_calf_joint"]  = a1_state -> motorState[11].q;

    // ROS_INFO("build");
    builder->SetNamespace("a1_robot");
    builder->SetFrameId("a1_world");
    // ROS_INFO("build a1_viz_array");
    builder->Build(&a1_viz_array,1);

    // ROS_INFO("vis_body_pose");
    geometry_msgs::Pose vis_body_pose;
    vis_body_pose.orientation.x = a1_state -> imu.quaternion[1];
    vis_body_pose.orientation.y = a1_state -> imu.quaternion[2];
    vis_body_pose.orientation.z = a1_state -> imu.quaternion[3];
    vis_body_pose.orientation.w = a1_state -> imu.quaternion[0];  // notice this order   0:w, 1:x, 2:y, 3:z
    vis_body_pose.position.x    = a1_state -> position.x; 
    vis_body_pose.position.y    = a1_state -> position.y; 
    vis_body_pose.position.z    = a1_state -> position.z;

    // ROS_INFO("set");
    builder->SetPose(vis_body_pose);
    builder->SetJointPositions(joint_positions);
    marker_arr_pub.publish(a1_viz_array);
    // clear array for next time use
    a1_viz_array.markers.clear();
}


int main(int argc, char** argv) {

    ros::init(argc, argv, "a1_visualizer");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);

    // the robot must publish a message with type unitree_legged_msgs::LowState
    ros::Subscriber sub_a1_msg = nh.subscribe("/a1/debug_state", 1000, a1_state_callback);


    // robomarker visualizer
    marker_arr_pub = nh.advertise<visualization_msgs::MarkerArray>("a1_robot_visualizer", 100);
    urdf::Model model;
    model.initFile(ros::package::getPath("unitree_legged_real") + "/../robots/a1_description/urdf/a1.urdf");
    builder = new robot_markers::Builder(model);
    builder->Init();


    // ros spinner
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }


  return 0;
}