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

#include <sensor_processor/new_sensor_processor_node.h>

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "lara_rgbd_new_sensor_processor");
    ros::NodeHandle nh;

    // Initialize features
    initKeypointDescriptor();

    // Create a ROS subscriber for sensor data
    ros::Subscriber sub_cloud = nh.subscribe("/rgbd_dataset/kinect/depth_registered/points", 1, cloud_cb);

    // Create a ROS publisher for processed sensor data
    pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("feature_cloud", 1);

    ROS_INFO("new_sensor_processor");

    // Spin
    ros::spin ();
}

void initKeypointDescriptor()
{
    /*keypoint_detector_.reset(new pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI> (pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI>::HARRIS));
    keypoint_detector_->setNonMaxSupression (true);
    keypoint_detector_->setRadius (0.01f);
    keypoint_detector_->setRadiusSearch (0.01f);
    keypoint_detector_->setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI>::HARRIS);*/

    // Parameters for sift computation
    const float min_scale = 0.1f;
    const int n_octaves = 6;
    const int n_scales_per_octave = 10;
    const float min_contrast = 0.5f;

    keypoint_detector_.reset(new pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> ());
    keypoint_detector_->setScales(min_scale, n_octaves, n_scales_per_octave);
    keypoint_detector_->setMinimumContrast(min_contrast);

    feature_extractor_.reset(new pcl::PFHRGBEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHRGBSignature250>);
    feature_extractor_->setKSearch(50);
}

void detectKeypoints (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints)
{
    // Estimate the sift interest points using Intensity values from RGB values
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB> ());
    keypoint_detector_->setSearchMethod(tree);
    keypoint_detector_->setInputCloud(input);
    pcl::StopWatch watch;
    keypoint_detector_->compute(*keypoints);
    ROS_INFO("Detected %zd points in %lfs", keypoints->size (), watch.getTimeSeconds ());

    /*ROS_INFO("keypoint detection...");

    pcl::PointCloud<int> indices;
    pcl::UniformSampling<pcl::PointXYZRGB> uniform_sampling;
    uniform_sampling.setInputCloud (input);
    uniform_sampling.setRadiusSearch (0.05f); //the 3D grid leaf size
    uniform_sampling.compute (indices);

    ROS_INFO("uniformly sampled %ld points (out of 307200)", indices.size());

    std::vector<int> data;
    unsigned long int size = indices.size();

    for(int ii = 0; ii < size; ii++)
        data.push_back(indices[ii]);

    boost::shared_ptr< std::vector<int> > indicesptr (new std::vector<int> (data));

    ROS_INFO_STREAM("indicesptr size: " << indicesptr->size());

    keypoint_detector_->setIndices(indicesptr);
    keypoint_detector_->setInputCloud(input);
    pcl::StopWatch watch;
    keypoint_detector_->compute(*keypoints);
    pcl::console::print_highlight ("Detected %zd points in %dms\n", keypoints->size (), watch.getTimeSeconds ());*/
}

void extractDescriptors (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints, pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr features)
{
    /*pcl::PointCloud<pcl::PointXYZRGB>::Ptr kpts(new pcl::PointCloud<pcl::PointXYZRGB>);
    kpts->points.resize(keypoints->points.size());

    pcl::copyPointCloud(*keypoints, *kpts);

    pcl::FeatureFromNormals<pcl::PointXYZRGB, pcl::Normal, pcl::PFHRGBSignature250>::Ptr feature_from_normals = boost::dynamic_pointer_cast<pcl::FeatureFromNormals<pcl::PointXYZRGB, pcl::Normal, pcl::PFHRGBSignature250> > (feature_extractor_);

    feature_extractor_->setSearchSurface(input);
    feature_extractor_->setInputCloud(kpts);

    if (feature_from_normals)
    //if (boost::dynamic_pointer_cast<typename pcl::FeatureFromNormals<pcl::PointXYZRGB, pcl::Normal, pcl::PFHRGBSignature250> > (feature_extractor_))
    {
        std::cout << "normal estimation..." << std::flush;
        pcl::PointCloud<pcl::Normal>::Ptr normals (new  pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimation;
        normal_estimation.setSearchMethod (pcl::search::Search<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));
        normal_estimation.setRadiusSearch (0.01);
        normal_estimation.setInputCloud (input);
        normal_estimation.compute (*normals);
        feature_from_normals->setInputNormals(normals);
        std::cout << "OK" << std::endl;
    }

    std::cout << "descriptor extraction..." << std::flush;
    feature_extractor_->compute (*features);
    std::cout << "OK" << std::endl;*/
}

void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
    ROS_INFO("cloud_cb start");

    // Print cloud message
    print_cloud_msg(cloud_msg);

    // Convert to PCL format
    pcl::PointCloud<pcl::PointXYZRGB>* cloud = new pcl::PointCloud<pcl::PointXYZRGB>();

    pcl::fromROSMsg(*cloud_msg, *cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input(cloud);
    pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints(new pcl::PointCloud<pcl::PointWithScale>());

    // Detect SIFT keypoints
    detectKeypoints(input, keypoints);

    // Extract PFHRGB descriptors
    //extractDescriptors (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints, pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr features);

    sensor_msgs::PointCloud2::Ptr feature_cloud_msg(new sensor_msgs::PointCloud2);

    pcl::toROSMsg(*keypoints, *feature_cloud_msg);
    feature_cloud_msg->header = cloud_msg->header;

    // Publish the data
    pub_feature_cloud(feature_cloud_msg);

    ROS_INFO("cloud_cb end");
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
