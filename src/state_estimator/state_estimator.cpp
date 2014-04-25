/**
 * @file   state_estimator.cpp
 * @author George Andrew Brindeiro (georgebrindeiro@lara.unb.br)
 * @date   Apr 24, 2014
 *
 * @attention Copyright (C) 2014
 * @attention Laboratório de Automação e Robótica (LARA)
 * @attention Universidade de Brasília (UnB)
 *
 * @brief State estimation module responsible for the core functionality of LARA RGB-D SLAM
 *
 * This class is responsible for implementing the EKF algorithm proposed in LARA RGB-D SLAM, as well as any auxiliary
 * routines that depend on the estimated state vector or past sensor data, such as cloud matching.
 */

#include <state_estimator/state_estimator.h>

#include <tf/transform_datatypes.h>

void StateEstimator::estimated_pose(geometry_msgs::PoseWithCovarianceStamped& current_pose)
{
    ROS_WARN("StateEstimator::estimated_pose not implemented yet: returning a fixed pose value!");

    current_pose.pose.pose.position.x = 0;
    current_pose.pose.pose.position.y = 0;
    current_pose.pose.pose.position.z = 0;
    current_pose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
}
