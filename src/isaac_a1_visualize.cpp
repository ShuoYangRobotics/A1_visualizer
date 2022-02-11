 
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
// #include <unitree_legged_msgs/LowStateStamped.h>


// for visualize robot in rviz robomarker
robot_markers::Builder* builder;
visualization_msgs::MarkerArray a1_viz_array;
ros::Publisher marker_arr_pub;

// move joint position as global variable 
std::map<std::string, double> joint_positions;
tf::TransformListener* listener;
// body pose is global variable
geometry_msgs::Pose vis_body_pose;


void a1_joint_state_callback(const sensor_msgs::JointStateConstPtr& a1_state) 
{
    // ROS_INFO("received");
    // FL 
    joint_positions["FL_hip_joint"]   = a1_state -> position[0];
    joint_positions["FL_thigh_joint"] = a1_state -> position[1];
    joint_positions["FL_calf_joint"]  = a1_state -> position[2];
    // FR 
    joint_positions["FR_hip_joint"]   = a1_state -> position[3];
    joint_positions["FR_thigh_joint"] = a1_state -> position[4];
    joint_positions["FR_calf_joint"]  = a1_state -> position[5];
    // RL 
    joint_positions["RL_hip_joint"]   = a1_state -> position[6];
    joint_positions["RL_thigh_joint"] = a1_state -> position[7];
    joint_positions["RL_calf_joint"]  = a1_state -> position[8];
    // RR 
    joint_positions["RR_hip_joint"]   = a1_state -> position[9];
    joint_positions["RR_thigh_joint"] = a1_state -> position[10];
    joint_positions["RR_calf_joint"]  = a1_state -> position[11];

    // ROS_INFO("build");
    builder->SetNamespace("a1_robot");
    builder->SetFrameId("world");
    // ROS_INFO("build a1_viz_array");
    builder->Build(&a1_viz_array,1);

    // ROS_INFO("set");
    builder->SetPose(vis_body_pose);
    builder->SetJointPositions(joint_positions);
    marker_arr_pub.publish(a1_viz_array);
    // clear array for next time use
    a1_viz_array.markers.clear();
}

// if a ground truth pose exist
void gt_pose_callback(const nav_msgs::Odometry::ConstPtr &odom) {
    // update
    vis_body_pose.orientation.x = odom->pose.pose.orientation.x;
    vis_body_pose.orientation.y = odom->pose.pose.orientation.y;
    vis_body_pose.orientation.z = odom->pose.pose.orientation.z;
    vis_body_pose.orientation.w = odom->pose.pose.orientation.w;  // notice this order   0:w, 1:x, 2:y, 3:z
    vis_body_pose.position.x    = odom->pose.pose.position.x;
    vis_body_pose.position.y    = odom->pose.pose.position.y;
    vis_body_pose.position.z    = odom->pose.pose.position.z;

}


int main(int argc, char** argv) {

    ros::init(argc, argv, "isaac_a1_visualizer");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);
    listener = new tf::TransformListener();

    // the robot can also publish a message with type sensor_msgs::JointState
    ros::Subscriber sub_a1_joint_foot_msg = nh.subscribe("/isaac_a1/joint_foot", 1000, a1_joint_state_callback);

    // if a ground truth pose exist
    ros::Subscriber sub_gt_pose_msg = nh.subscribe("/isaac_a1/gt_body_pose", 100, gt_pose_callback);

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