/**
 * @file   sensor_processor.h
 * @author George Andrew Brindeiro (georgebrindeiro@lara.unb.br)
 * @date   Apr 22, 2014
 *
 * @attention Copyright (C) 2014
 * @attention Laboratório de Automação e Robótica (LARA)
 * @attention Universidade de Brasília (UnB)
 */

#ifndef LARA_RGBD_SENSOR_PROCESSOR_H_
#define LARA_RGBD_SENSOR_PROCESSOR_H_

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

enum FeatureDetector {SURF};

class SensorProcessor
{
	public:
		SensorProcessor(FeatureDetector feature_detector = SURF, bool display_features = false)
		{
			feature_detector_ = feature_detector;
			display_features_ = display_features;
			
			// Create a CV window for debugging
			if(display_features)
				cv::namedWindow("Display window", CV_WINDOW_AUTOSIZE);
		}
		
		~SensorProcessor() {}

		void extract_rgb_image(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg, cv::Mat& img);
		
		void extract_keypoints(cv::Mat& img, std::vector<cv::KeyPoint>& keypoints);
		
		void build_feature_cloud(const std::vector<cv::KeyPoint>& keypoints, const sensor_msgs::PointCloud2::ConstPtr& cloud_msg, sensor_msgs::PointCloud2::Ptr& feature_cloud_msg);

	private:
		FeatureDetector feature_detector_;
		
		bool display_features_;
};

#endif  // LARA_RGBD_SENSOR_PROCESSOR_H_
