/**
 * @file   sensor_processor_node.h
 * @author George Andrew Brindeiro (georgebrindeiro@lara.unb.br)
 * @date   Apr 22, 2014
 *
 * @attention Copyright (C) 2014
 * @attention Laboratório de Automação e Robótica (LARA)
 * @attention Universidade de Brasília (UnB)
 */

#ifndef LARA_RGBD_SENSOR_PROCESSOR_NODE_H_
#define LARA_RGBD_SENSOR_PROCESSOR_NODE_H_

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/common/time.h>
#include <pcl/features/pfhrgb.h>

// ROS Subscriber Callbacks
void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);

// ROS Publisher Functions
void pub_feature_cloud(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);

// Auxiliary Functions
void print_cloud_msg(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);

ros::Publisher pub_cloud;

void initKeypointDescriptor();
void detectKeypoints (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints);
void extractDescriptors (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints, pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr features);

boost::shared_ptr< pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> > keypoint_detector_;
pcl::Feature<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::Ptr feature_extractor_;

#endif  // LARA_RGBD_SENSOR_PROCESSOR_NODE_H_
