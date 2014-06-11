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
#include <pcl/filters/filter.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/keypoints/harris_3d.h>
#include <pcl/features/pfhrgb.h>

enum FeatureDetector {SURF};

class SensorProcessor
{
    public:
        SensorProcessor(FeatureDetector feature_detector = SURF, bool display_features = false)
        {
            feature_detector_ = feature_detector;
            display_features_ = display_features;

            initKeypointDescriptor();

            // Create a CV window for debugging
            if(display_features)
                cv::namedWindow("Display window", CV_WINDOW_AUTOSIZE);
        }

        ~SensorProcessor() {}

        /*!
        * @brief Extracts RGB image from an RGB-D point cloud
        *
        * Repackages RGB data found in point cloud so that conventional feature detectors can be used in OpenCV
        *
        * @param cloud_msg              Pointer to kinect cloud message (as received by the ROS callback function)
        * @param img                    RGB image receiving the result
        */
        void extract_rgb_image(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg, cv::Mat& img);

        /*!
        * @brief Extracts keypoints from RGB image using the selected feature detector
        *
        * Applies the feature detector selected at initialization on an RGB image and returns the detected keypoints
        *
        * @param img                    RGB image the feature detector will be applied on
        * @param keypoints              Keypoint vector receiving the result
        */
        void extract_keypoints(cv::Mat& img, std::vector<cv::KeyPoint>& keypoints);

        /*!
        * @brief Builds feature point cloud from keypoints
        *
        * Uses the detected keypoints to filter the original RGB-D point cloud
        *
        * @param keypoints              Keypoints that will be used to filter the RGB-D point cloud
        * @param cloud_msg              Pointer to kinect cloud message (as received by the ROS callback function)
        * @param feature_cloud_msg      Pointer to feature cloud message receiving the result
        */
        void build_feature_cloud(const std::vector<cv::KeyPoint>& keypoints,
                                 const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
                                 sensor_msgs::PointCloud2::Ptr& feature_cloud_msg);

        // New features from feature_matching example
        void initKeypointDescriptor();


        /**
         * @brief Detects key points in the input point cloud
         * @param input the input point cloud
         * @param keypoints the resulting key points. Note that they are not necessarily a subset of the input cloud
         */
        void detectKeypoints (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints) const;

        /**
         * @brief extract descriptors for given key points
         * @param input point cloud to be used for descriptor extraction
         * @param keypoints locations where descriptors are to be extracted
         * @param features resulting descriptors
         */
        void extractDescriptors (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints, pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr features);

    private:
        FeatureDetector feature_detector_;      /**< Indicates which feature detector we are using to process point clouds  */

        bool display_features_;                 /**< Indicates whether we want to the extracted keypoints in a CV window    */

        boost::shared_ptr<pcl::Keypoint<pcl::PointXYZRGB, pcl::PointXYZI> > keypoint_detector_;
        pcl::Feature<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::Ptr feature_extractor_;
};

#endif  // LARA_RGBD_SENSOR_PROCESSOR_H_
