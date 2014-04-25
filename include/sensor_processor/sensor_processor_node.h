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

#include <sensor_processor/sensor_processor.h>

void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
void print_cloud_msg(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);

ros::Publisher pub_cloud;

SensorProcessor* sensor_processor;

#endif  // LARA_RGBD_SENSOR_PROCESSOR_NODE_H_
