/**
 * @file   state_estimator_node.cpp
 * @author George Andrew Brindeiro (georgebrindeiro@lara.unb.br)
 * @date   Apr 24, 2014
 *
 * @attention Copyright (C) 2014
 * @attention Laboratório de Automação e Robótica (LARA)
 * @attention Universidade de Brasília (UnB)
 *
 * @brief  ROS node used to p
 *
 * This ROS node receives
 */

#include <ros/ros.h>

#include <state_estimator/state_estimator_node.h>

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "lara_rgbd_state_estimator");
    ros::NodeHandle nh;

    // Create a ROS subscriber for sensor data
    ros::Subscriber sub_cloud = nh.subscribe("feature_cloud", 1, cloud_cb);
    ros::Subscriber sub_odom  = nh.subscribe("odometry", 1, odom_cb);

    // Create a ROS publisher for estimated pose
    pub_pose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("estimated_pose", 1);

    // Create SensorProcessor
    state_estimator = new StateEstimator();

    // Spin
    ros::spin ();
}
