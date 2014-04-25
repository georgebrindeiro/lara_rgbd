/**
 * @file   state_estimator_node.cpp
 * @author George Andrew Brindeiro (georgebrindeiro@lara.unb.br)
 * @date   Apr 24, 2014
 *
 * @attention Copyright (C) 2014
 * @attention Laboratório de Automação e Robótica (LARA)
 * @attention Universidade de Brasília (UnB)
 *
 * @brief  ROS node used to provide the current state estimate obtained by LARA RGB-D SLAM
 *
 * This ROS node receives processed sensor data and executes the EKF algorithm proposed for LARA RGB-D SLAM using the
 * StateEstimator class. Whenever necessary, additional processing such as cloud matching is also performed here.
 */

#include <state_estimator/state_estimator_node.h>

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "lara_rgbd_state_estimator");
    ros::NodeHandle nh;

    // Create a ROS subscriber for sensor data
    ros::Subscriber sub_cloud = nh.subscribe("feature_cloud", 1, cloud_cb);
    ros::Subscriber sub_odom  = nh.subscribe("/rgbd_dataset/pioneer/odometry", 1, odom_cb);

    // Create a ROS publisher for estimated pose
    pub_pose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("estimated_pose", 1);

    // Create SensorProcessor
    state_estimator = new StateEstimator();

    ros::Rate loop_rate(10);

    // Spin
    while(ros::ok())
    {
        /*geometry_msgs::PoseWithCovarianceStamped current_pose;

        current_pose.header.stamp = ros::Time::now();
        current_pose.header.frame_id = "world";
        state_estimator->estimated_pose(current_pose);

        pub_pose.publish(current_pose);*/

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
    ROS_WARN("Cloud matching measurement model still not implemented!");
}

void odom_cb(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    ROS_WARN("Odometry motion model still not implemented! Publishing fake pose estimate");

    // Fake pose estimate from odometry
    geometry_msgs::PoseWithCovarianceStamped::Ptr estimated_pose_msg(new geometry_msgs::PoseWithCovarianceStamped);
    estimated_pose_msg->header = odom_msg->header;
    estimated_pose_msg->pose = odom_msg->pose;

    pub_pose.publish(estimated_pose_msg);
}
