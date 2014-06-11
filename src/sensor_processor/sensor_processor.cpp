/**
 * @file   sensor_processor.cpp
 * @author George Andrew Brindeiro (georgebrindeiro@lara.unb.br)
 * @date   Apr 22, 2014
 *
 * @attention Copyright (C) 2014
 * @attention Laboratório de Automação e Robótica (LARA)
 * @attention Universidade de Brasília (UnB)
 *
 * @brief  Sensor processing routines for LARA RGB-D SLAM
 *
 * This class is responsible for processing the RGB-D point cloud and odometry data for LARA RGB-D SLAM
 */

#include <sensor_processor/sensor_processor.h>

void SensorProcessor::extract_rgb_image(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg, cv::Mat& img)
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

void SensorProcessor::extract_keypoints(cv::Mat& img, std::vector<cv::KeyPoint>& keypoints)
{
    // Extract keypoints using the chosen feature detector
    switch(feature_detector_)
    {
        case SURF:
        {
            int minHessian = 400;

            cv::SurfFeatureDetector detector(minHessian);
            detector.detect(img, keypoints);

            break;
        }

        default:
        {
            ROS_WARN("Invalid option (feature_detector = %d)", feature_detector_);

            break;
        }
    }

    // Display detected keypoints
    if(display_features_)
    {
        // Overlay keypoints on image
        cv::drawKeypoints(img, keypoints, img, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);

        // Show results in display window
        cv::imshow("Display window", img);
        cv::waitKey(10);
    }

    // Print out number of features detected for debugging
    ROS_DEBUG("# features: %ld", keypoints.size());
}

void SensorProcessor::build_feature_cloud(const std::vector<cv::KeyPoint>& keypoints, const sensor_msgs::PointCloud2::ConstPtr& cloud_msg, sensor_msgs::PointCloud2::Ptr& feature_cloud_msg)
{
    // Work with PCL compatible variables
    pcl::PointIndices::Ptr keypoint_indices(new pcl::PointIndices());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_input(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_output(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Convert keypoints to PCL format
    for(int i=0; i < keypoints.size(); i++)
    {
        int x = keypoints[i].pt.x;//1-640
        int y = keypoints[i].pt.y;//1-480

        keypoint_indices->indices.push_back(cloud_msg->width*y+x);
    }

    // Convert cloud_msg to PCL format
    pcl::fromROSMsg(*cloud_msg, *pcl_input);

    // Use ExtractIndices filter to build feature cloud
    pcl::ExtractIndices< pcl::PointXYZRGB > extract;

    extract.setInputCloud(pcl_input);
    extract.setIndices(keypoint_indices);
    extract.setNegative(false);
    extract.filter(*pcl_output);

    // Print out number of features in cloud for debugging
    ROS_DEBUG("# feature cloud points (prior to NaN removal): %d", pcl_output->width * pcl_output->height);

    // Remove NaNs from output?
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*pcl_output, *pcl_output, indices);

    // Print out number of features in cloud for debugging
    ROS_DEBUG("# feature cloud points: %d", pcl_output->width * pcl_output->height);

    // Convert pcl_output to ROS format
    pcl::toROSMsg(*pcl_output, *feature_cloud_msg);
}


void SensorProcessor::initKeypointDescriptor()
{
    pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI>* harris3D = new pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI> (pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI>::HARRIS);
    harris3D->setNonMaxSupression (true);
    harris3D->setRadius (0.01f);
    harris3D->setRadiusSearch (0.01f);
    harris3D->setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI>::HARRIS);
    keypoint_detector_.reset(harris3D);

    feature_extractor_.reset(new pcl::PFHRGBEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHRGBSignature250>);
    feature_extractor_->setKSearch(50);
}

void SensorProcessor::detectKeypoints (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints) const
{
  std::cout << "keypoint detection..." << std::flush;
  keypoint_detector_->setInputCloud(input);
  keypoint_detector_->compute(*keypoints);
  std::cout << "OK. keypoints found: " << keypoints->points.size() << std::endl;
}

void SensorProcessor::extractDescriptors (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints, pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr features)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr kpts(new pcl::PointCloud<pcl::PointXYZRGB>);
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
  std::cout << "OK" << std::endl;
}
