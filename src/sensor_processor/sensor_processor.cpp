/**
 * @file   sensor_processor.cpp
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
	switch(feature_detector_)
	{
		case SURF:
		{
			// Extract SURF keypoints
			int minHessian = 400;

			cv::SurfFeatureDetector detector(minHessian);
			detector.detect(img, keypoints);
			
			break;
		}
		
		default:
		{
			ROS_INFO("Invalid option: feature_detector = %d", feature_detector_);
			
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

    // Print out number of features in cloud for debugging
    ROS_DEBUG("# feature cloud points: %d", pcl_output->width * pcl_output->height);
}

