/**
 * @file   sensor_processor_node.cpp
 * @author George Andrew Brindeiro (georgebrindeiro@lara.unb.br)
 * @date   Apr 22, 2014
 *
 * @attention Copyright (C) 2014
 * @attention Laboratório de Automação e Robótica (LARA)
 * @attention Universidade de Brasília (UnB)
 *
 * @brief  ROS node used to process raw sensor data
 *
 * This ROS node receives both the RGB-D point cloud from the Kinect and the odometry info from the Pioneer and does
 * some processing using the SensorProcessor class to transform it to the format expected by LARA RGB-D SLAM.
 */

#include <sensor_processor/sensor_processor_node.h>

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "lara_rgbd_sensor_processor");
    ros::NodeHandle nh;

    // Create a ROS subscriber for sensor data
    ros::Subscriber sub_cloud = nh.subscribe("/rgbd_dataset/kinect/depth_registered/points", 1, cloud_cb);

    // Create a ROS publisher for processed sensor data
    pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("feature_cloud", 1);

    // Create SensorProcessor
    sensor_processor = new SensorProcessor();

    // Spin
    ros::spin ();
}

void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
    // Print cloud message
    print_cloud_msg(cloud_msg);

    // Extract RGB image from cloud
    cv::Mat img;
    sensor_processor->extract_rgb_image(cloud_msg, img);

    // Detect keypoints from RGB image
    std::vector<cv::KeyPoint> keypoints;
    sensor_processor->extract_keypoints(img, keypoints);

    // Build feature cloud from keypoints
    sensor_msgs::PointCloud2::Ptr feature_cloud_msg(new sensor_msgs::PointCloud2);
    sensor_processor->build_feature_cloud(keypoints, cloud_msg, feature_cloud_msg);

    // Publish the data
    pub_feature_cloud(feature_cloud_msg);
}

void pub_feature_cloud(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
    pub_cloud.publish(cloud_msg);
}

 void print_cloud_msg(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
    // Parse cloud message
    int seq = cloud_msg->header.seq;
    int time_sec = cloud_msg->header.stamp.sec;
    int time_nsec = cloud_msg->header.stamp.nsec;
    std::string frame_id = cloud_msg->header.frame_id;
    int width = cloud_msg->width;
    int height = cloud_msg->height;
    bool is_bigendian = cloud_msg->is_bigendian;
    int point_step = cloud_msg->point_step;
    int row_step = cloud_msg->row_step;
    bool is_dense = cloud_msg->is_dense;

    // Print cloud message for debugging
    ROS_DEBUG("Point cloud\n");
    ROS_DEBUG("seq: %d", seq);
    ROS_DEBUG("timestamp: %d.%d", time_sec, time_nsec);
    ROS_DEBUG("frame_id: %s", frame_id.c_str());
    ROS_DEBUG("dimensions: %dx%d", width, height);
    ROS_DEBUG("is_bigendian: %d", is_bigendian);
    ROS_DEBUG("point_step: %d\n", point_step);
    ROS_DEBUG("row_step: %d\n", row_step);
    ROS_DEBUG("is_dense: %d\n", is_dense);
    ROS_DEBUG("------------------------------------------------------");
}
