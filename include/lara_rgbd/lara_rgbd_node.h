/**
 * @file   sensor_processor_node.h
 * @author George Andrew Brindeiro (georgebrindeiro@lara.unb.br)
 * @date   Apr 22, 2014
 *
 * @attention Copyright (C) 2014
 * @attention Laboratório de Automação e Robótica (LARA)
 * @attention Universidade de Brasília (UnB)
 */

#ifndef LARA_RGBD_SENSOR_PROCER_NODE_H_
#define LARA_RGBD_SENSOR_PROCER_NODE_H_

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>

#include <pcl_conversions/pcl_conversions.h>

#include <lara_rgbd/sensor_processor/sensor_processor.h>

// ROS Subscriber Callbacks
void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);

ros::Publisher pub_feature_cloud;


#endif  // LARA_RGBD_SENSOR_PROCER_NODE_H_
