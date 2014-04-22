/**
 * @file   sensor_processor_node.h
 * @author George Andrew Brindeiro (georgebrindeiro@lara.unb.br)
 * @date   Apr 22, 2014
 *
 * @attention Copyright (C) 2014
 * @attention Laboratório de Automação e Robótica (LARA)
 * @attention Universidade de Brasília (UnB)
 */

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>

#include <cv_bridge/cv_bridge.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>

#include <pcl_conversions/pcl_conversions.h>

#include <iostream>

void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
void print_cloud_msg(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
void extract_rgb_image(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg, cv::Mat& img);
void extract_keypoints(cv::Mat& img, std::vector<cv::KeyPoint>& keypoints);
void build_feature_cloud(const std::vector<cv::KeyPoint>& keypoints, const sensor_msgs::PointCloud2::ConstPtr& cloud_msg, sensor_msgs::PointCloud2::Ptr& feature_cloud_msg);

// TODO: remove these when filter is running
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

void odom_cb(const nav_msgs::Odometry::ConstPtr& odom_msg);
