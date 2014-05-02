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

    // Create a ROS publisher for keyframes
    pub_keyframes = nh.advertise<lara_rgbd_msgs::PoseWithCovarianceStampedArray>("keyframes", 1);

    // Create SensorProcessor
    state_estimator = new StateEstimator();

    ros::Rate loop_rate(10);

    // Spin
    while(ros::ok())
    {
        state_estimator->quasiconstant_motion_model();

        pub_pose_estimate();
        pub_keyframes_estimate();

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
    state_estimator->cloud_measurement_model(cloud_msg);
}

void odom_cb(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    state_estimator->odom_motion_model(odom_msg);

    pub_fake_pose_estimate(odom_msg);
}

void pub_fake_pose_estimate(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    // Fake pose estimate from odometry
    geometry_msgs::PoseWithCovarianceStamped::Ptr estimated_pose_msg(new geometry_msgs::PoseWithCovarianceStamped);
    estimated_pose_msg->header = odom_msg->header;
    estimated_pose_msg->header.frame_id = "world";
    estimated_pose_msg->pose = odom_msg->pose;

    estimated_pose_msg->pose.covariance[0] = 1;
    estimated_pose_msg->pose.covariance[6+1] = 1;
    estimated_pose_msg->pose.covariance[12+2] = 0.1;
    estimated_pose_msg->pose.covariance[18+3] = 0.01;
    estimated_pose_msg->pose.covariance[24+4] = 0.01;
    estimated_pose_msg->pose.covariance[30+5] = 0.01;

    pub_pose.publish(estimated_pose_msg);
}

void pub_pose_estimate()
{
    // Real pose estimate from state_estimator
    geometry_msgs::PoseWithCovarianceStamped current_pose;

    current_pose.header.stamp = ros::Time::now();
    current_pose.header.frame_id = "world";
    state_estimator->estimated_pose(current_pose);

    pub_pose.publish(current_pose);
}

void pub_keyframes_estimate()
{
    lara_rgbd_msgs::PoseWithCovarianceStampedArray keyframes;

    keyframes.header.stamp = ros::Time::now();
    keyframes.header.frame_id = "world";
    state_estimator->keyframes(keyframes);

    pub_keyframes.publish(keyframes);
}
