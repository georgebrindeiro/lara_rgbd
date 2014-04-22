/**
 * @file   sensor_processor_node.cpp
 * @author George Andrew Brindeiro (georgebrindeiro@lara.unb.br)
 * @date   Apr 22, 2014
 *
 * @attention Copyright (C) 2014
 * @attention Laboratório de Automação e Robótica (LARA)
 * @attention Universidade de Brasília (UnB)
 *
 * @brief  <small description of the file purpose>
 *
 * <detailed description which may contain examples and test cases>
 */

#include <sensor_processor/sensor_processor_node.h>

bool print_msgs = false;
bool display_features = true;

ros::Publisher pub_cloud;
ros::Publisher pub_pose; // TODO: remove this when filter is running

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "lara_rgbd_sensor_processor");
    ros::NodeHandle nh;

    // Create ROS subscribers for sensor data
    ros::Subscriber sub_cloud = nh.subscribe ("kinect_cloud", 1, cloud_cb);

    // Create a ROS publisher for processed sensor data
    pub_cloud = nh.advertise<sensor_msgs::PointCloud2> ("feature_cloud", 1);

    // Create a CV window for debugging
    if(display_features)
        cv::namedWindow("Display window", CV_WINDOW_AUTOSIZE);

    // TODO: remove this when filter is running
    ros::Subscriber sub_odom  = nh.subscribe ("odometry", 1, odom_cb);
    pub_pose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped> ("estimated_pose", 1);

    // Spin
    ros::spin ();
}

void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
    // Print cloud message
    if(print_msgs)
        print_cloud_msg(cloud_msg);

    // Extract RGB image from cloud
    cv::Mat img;
    extract_rgb_image(cloud_msg, img);

    // Detect keypoints from RGB image
    std::vector<cv::KeyPoint> keypoints;
    extract_keypoints(img, keypoints);

    // Build feature cloud from keypoints
    sensor_msgs::PointCloud2::Ptr feature_cloud_msg(new sensor_msgs::PointCloud2);
    build_feature_cloud(keypoints, cloud_msg, feature_cloud_msg);

    // Publish the data
    pub_cloud.publish(feature_cloud_msg);
}

void odom_cb(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    // Fake pose estimate from odometry
    geometry_msgs::PoseWithCovarianceStamped::Ptr estimated_pose_msg(new geometry_msgs::PoseWithCovarianceStamped);
    estimated_pose_msg->header = odom_msg->header;
    estimated_pose_msg->pose = odom_msg->pose;
    
    pub_pose.publish(estimated_pose_msg);
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
    ROS_INFO("Point cloud\n");
    ROS_INFO("seq: %d", seq);
    ROS_INFO("timestamp: %d.%d", time_sec, time_nsec);
    ROS_INFO("frame_id: %s", frame_id.c_str());
    ROS_INFO("dimensions: %dx%d", width, height);
    ROS_INFO("is_bigendian: %d", is_bigendian);
    ROS_INFO("point_step: %d\n", point_step);
    ROS_INFO("row_step: %d\n", row_step);
    ROS_INFO("is_dense: %d\n", is_dense);
    ROS_INFO("------------------------------------------------------");
}

void extract_rgb_image(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg, cv::Mat& img)
{
    // Copy the RGB fields of a sensor_msgs::PointCloud2 into a sensor_msgs::Image
    sensor_msgs::Image image_msg;
    pcl::toROSMsg(*cloud_msg, image_msg);

    // Convert a sensor_msgs::Image to an OpenCV-compatible CvImage, copying the image data.
    cv_bridge::CvImagePtr cv_image_ptr;
    cv_image_ptr = cv_bridge::toCvCopy(image_msg);

    // Assign cv::Mat portion of CvImage to output
    img = cv_image_ptr->image;
}

void extract_keypoints(cv::Mat& img, std::vector<cv::KeyPoint>& keypoints)
{
    // Extract SURF keypoints
    int minHessian = 400;

    cv::SurfFeatureDetector detector(minHessian);
    detector.detect(img, keypoints);

    // Display detected keypoints
    if(display_features)
    {
        // Print out number of features
        //ROS_INFO("# features: %ld", keypoints.size());

        // Overlay keypoints on image
        cv::drawKeypoints(img, keypoints, img, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
      
        // Show results in display window
        cv::imshow("Display window", img);
        cv::waitKey(10);
    }
}

void build_feature_cloud(const std::vector<cv::KeyPoint>& keypoints, const sensor_msgs::PointCloud2::ConstPtr& cloud_msg, sensor_msgs::PointCloud2::Ptr& feature_cloud_msg)
{
    // Work with PCL compatible variables
    pcl::PointIndices::Ptr keypoint_indices(new pcl::PointIndices());
    pcl::PCLPointCloud2::Ptr pcl_input(new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2::Ptr pcl_output(new pcl::PCLPointCloud2);

    // Convert keypoints to PCL format
    for(int i=0; i < keypoints.size(); i++)
    {
        int x = keypoints[i].pt.x;//1-640
        int y = keypoints[i].pt.y;//1-480

        keypoint_indices->indices.push_back(cloud_msg->width*y+x);
    }

    // Convert cloud_msg to PCL format
    pcl_conversions::toPCL(*cloud_msg, *pcl_input);

    // Use ExtractIndices filter to build feature cloud
    pcl::ExtractIndices<pcl::PCLPointCloud2> extract;   

    extract.setInputCloud(pcl_input);
    extract.setIndices(keypoint_indices);
    extract.setNegative(false);
    extract.filter(*pcl_output);

    // Convert pcl_output to ROS format
    pcl_conversions::fromPCL(*pcl_output, *feature_cloud_msg);

    // Print out number of features for debugging
    if(display_features)
        ROS_INFO("# feature cloud points: %d", pcl_output->width * pcl_output->height);
}
