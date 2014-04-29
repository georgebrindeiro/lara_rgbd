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

class StateEstimator
{
    public:
        StateEstimator()
        {
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

        /*!
        * @brief Repackages estimated pose into a ROS message for publishing
        *
        * Repackages current pose estimate into a geometry_msgs::PoseWithCovarianceStamped ROS message for publishing
        */
        void estimated_pose(geometry_msgs::PoseWithCovarianceStamped& current_pose);

    private:
        /*!
        * @brief Calculates the size of the state vector
        *
        * Repackages current pose estimate into a geometry_msgs::PoseWithCovarianceStamped ROS message for publishing
        */
        inline int n_()                         {   return (7*(num_past_poses_+1));    }

        /*!
        * @brief Returns pose of index i inside the state vector
        *
        * Returns pose of index i inside the state vector
        */
        //inline Eigen::VectorXf pose_(int i)     {   return subvector(7*i,7*(i+1)-1);  }

        /*!
        * @brief Repackages a pose into a position vector + orientation quaternion
        *
        * Repackages a pose into a position vector + orientation quaternion
        */
        //inline Eigen::VectorXf pose_to_pq(pose,p,q)     {   return subvector(7*i,7*(i+1)-1)  };

        /*!
        * @brief Repackages a position vector + orientation quaternion into a pose
        *
        * Repackages a position vector + orientation quaternion into a pose
        */
        //inline Eigen::VectorXf pq_to_pose(p,q,pose)     {   return subvector(7*i,7*(i+1)-1)  };

        /*!
        * @brief Performs state vector augmentation using current pose
        *
        * Performs state vector augmentation using current pose, as described in LARA RGB-D SLAM
        */
        void augment_state_vector_();

        Eigen::VectorXf state_estimate_;       /**< State vector for LARA RGB-D SLAM  */
        Eigen::MatrixXf cov_state_estimate_;   /**< State vector covariance matrix for LARA RGB-D SLAM  */

        int num_past_poses_;            /**< Indicates how many past poses we have stored */
};

#endif  // LARA_RGBD_STATE_ESTIMATOR_H_
