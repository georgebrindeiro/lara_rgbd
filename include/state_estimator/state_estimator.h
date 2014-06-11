/**
 * @file   state_estimator.h
 * @author George Andrew Brindeiro (georgebrindeiro@lara.unb.br)
 * @date   Apr 24, 2014
 *
 * @attention Copyright (C) 2014
 * @attention Laboratório de Automação e Robótica (LARA)
 * @attention Universidade de Brasília (UnB)
 */

#ifndef LARA_RGBD_STATE_ESTIMATOR_H_
#define LARA_RGBD_STATE_ESTIMATOR_H_

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>

#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

#include <Eigen/Dense>

#include <lara_rgbd_msgs/PoseWithCovarianceStampedArray.h>

// PCL Registration headers
//#include <pcl/filters/extract_indices.h>
//#include <pcl/sample_consensus/ransac.h>
//#include <pcl/sample_consensus/sac_model_registration.h>

// kdtree search
//#include <pcl/kdtree/kdtree_flann.h>

// PCL features
#include <pcl/features/normal_3d.h>

// PCL visualizer
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

class StateEstimator
{
    public:
        StateEstimator()
        {
            num_keyframes_ = 0;

            init_state_estimate();
        }

        ~StateEstimator() {}

        /*!
        * @brief Initializes state estimate using the initial pose as the arbitrary origin
        *
        * Initializes the state estimator state vector according to the arbitrary origin method described in LARA RGB-D
        * SLAM, by assigning the initial pose as the map origin with a fixed covariance.
        */
        void init_state_estimate();

        /*!
        * @brief Updates the state estimate according to the Quasiconstant Motion Model
        *
        * Updates the state estimate according to the Quasiconstant Motion Model, increasing the estimate uncertainty
        * when no sensor data is available, as described in LARA RGB-D SLAM. In practice, this function is called at a
        * fixed rate, according to the rate at which the state_estimator_node loop is executed.
        */
        void quasiconstant_motion_model();

        /*!
        * @brief Updates the state estimate according to the Odometry Motion Model
        *
        * Updates the state estimate according to the Odometry Motion Model, whenever odometry information is available.
        * In practice, this function updates the state estimate to reflect the difference between two odometry messages,
        * increasing state uncertainty, as described in LARA RGB-D SLAM.
        */
        void odom_motion_model(const nav_msgs::Odometry::ConstPtr& odom_msg);

        /*!
        * @brief Updates the state estimate according to the Cloud Matching Measurement Model
        *
        * Updates the state estimate according to the Cloud Matching Measurement Model, whenever a new point cloud is
        * available. In practice, this function tries to match the current cloud to several past point clouds, using the
        * relative transform found and the state estimates associated to the past clouds to generate measurement vectors,
        * as described in LARA RGB-D SLAM.
        */
        void cloud_measurement_model(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);

        // From feature matching example
        /**
         * @brief find corresponding features based on some metric
         * @param source source feature descriptors
         * @param target target feature descriptors
         * @param correspondences indices out of the target descriptors that correspond (nearest neighbor) to the source descriptors
         */
        //void findCorrespondences (typename pcl::PointCloud<FeatureType>::Ptr source, typename pcl::PointCloud<FeatureType>::Ptr target, std::vector<int>& correspondences) const;

        /**
         * @brief  remove non-consistent correspondences
         */
        //void filterCorrespondences ();

        /*!
        * @brief Repackages estimated pose into a ROS message for publishing
        *
        * Repackages current pose estimate into a geometry_msgs::PoseWithCovarianceStamped ROS message for publishing
        */
        void estimated_pose(geometry_msgs::PoseWithCovarianceStamped& current_pose);

        /*!
        * @brief Repackages keyframes into a ROS message for publishing
        *
        * Repackages keyframes into a geometry_msgs::PoseWithCovarianceStampedArray ROS message for publishing
        */
        void keyframes(lara_rgbd_msgs::PoseWithCovarianceStampedArray& keyframes);

    private:
        /*!
        * @brief Performs state vector augmentation using current pose
        *
        * Performs state vector augmentation using current pose, as described in LARA RGB-D SLAM
        */
        void augment_state_vector_();

        Eigen::VectorXf state_estimate_;       /**< State vector for LARA RGB-D SLAM  */
        Eigen::MatrixXf cov_state_estimate_;   /**< State vector covariance matrix for LARA RGB-D SLAM  */

        int num_keyframes_;                                     /**< Number of keyframes added to history */
        std::vector< Eigen::VectorXf > keyframes_pose_;         /**< State vector for keyframes */
        std::vector< Eigen::MatrixXf > keyframes_cov_;          /**< Covariance matrix for keyframes */
        //std::vector< pcl::PointCloud<pcl::PointXYZRGB> > keyframes_cloud_;    /**< Feature cloud for keyframes */
        //std::vector< pcl::KdTreeFLANN<pcl::PointXYZRGB> > keyframes_kdtree_;   /**< Feature kdtree for keyframes */
};

#endif  // LARA_RGBD_STATE_ESTIMATOR_H_
