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

class StateEstimator
{
    public:
        StateEstimator()
        {

        }

        ~StateEstimator() {}

        /*!
        * @brief Repackages estimated pose into a ROS message for publishing
        *
        * Repackages current pose estimate into a geometry_msgs::PoseWithCovarianceStamped ROS message for publishing
        */
        void estimated_pose(geometry_msgs::PoseWithCovarianceStamped& current_pose);

    private:
        int var_;      /**< Indicates ...  */
};

#endif  // LARA_RGBD_STATE_ESTIMATOR_H_
