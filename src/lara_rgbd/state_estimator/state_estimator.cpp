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

#include <lara_rgbd/state_estimator/state_estimator.h>

#include <tf/transform_datatypes.h>

void StateEstimator::init_state_estimate()
{
    // Init pose to p = (0,0,0) and q=(1,0,0,0)
    // beter if pq_to_pose
    state_estimate_ = Eigen::VectorXf::Zero(7);
    state_estimate_(3) = 1;

    ROS_DEBUG_STREAM("init state" << std::endl << state_estimate_);

    // Init covariance to diag(1,1,0.01...)... not sure if too much for quaternion
    // better if cov_pq_to_pose
    cov_state_estimate_ = Eigen::MatrixXf::Identity(7,7);

    for(int i = 2; i < 7; i++)
        cov_state_estimate_(i,i) = 0.01;

    ROS_DEBUG_STREAM("init cov_state" << std::endl << cov_state_estimate_);
}

void StateEstimator::quasiconstant_motion_model()
{
    Eigen::MatrixXf Q = Eigen::MatrixXf::Zero(7,7);

    Q(0,0) = 0.01;
    Q(1,1) = 0.01;
    Q(2,2) = 0.001;
    Q(3,3) = 0.001;
    Q(4,4) = 0.001;
    Q(5,5) = 0.001;
    Q(6,6) = 0.001;

    cov_state_estimate_.block(0,0,7,7) += Q;
}

void StateEstimator::odom_motion_model(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    ROS_DEBUG("Odometry motion model still not implemented!");
}

void StateEstimator::cloud_measurement_model(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
    ROS_DEBUG("Cloud measurement model still not implemented!");
}

void StateEstimator::estimated_pose(geometry_msgs::PoseWithCovarianceStamped& current_pose)
{
    current_pose.pose.pose.position.x = state_estimate_(0);
    current_pose.pose.pose.position.y = state_estimate_(1);
    current_pose.pose.pose.position.z = state_estimate_(2);
    current_pose.pose.pose.orientation.w = state_estimate_(3);
    current_pose.pose.pose.orientation.x = state_estimate_(4);
    current_pose.pose.pose.orientation.y = state_estimate_(5);
    current_pose.pose.pose.orientation.z = state_estimate_(6);

    ROS_DEBUG("estimated_pose not fully implemented. returning fixed covariance for orientation!");
    current_pose.pose.covariance[0] = cov_state_estimate_(0,0);
    current_pose.pose.covariance[6+1] = cov_state_estimate_(1,1);
    current_pose.pose.covariance[12+2] = cov_state_estimate_(2,2);
    current_pose.pose.covariance[18+3] = 0.01;
    current_pose.pose.covariance[24+4] = 0.01;
    current_pose.pose.covariance[30+5] = 0.01;
}

void StateEstimator::augment_state_vector()
{
    int n = state_estimate_.size();

    state_estimate_.conservativeResize(n+7);
    state_estimate_.block(7,0,7,1) = state_estimate_.block(0,0,7,1);

    cov_state_estimate_.conservativeResize(n+7,n+7);

    int i = n/7;

    for(int ii = 0; ii < i; ii++)
    {
        Eigen::MatrixXf cov_block = cov_state_estimate_.block(7*ii,0,7,7);

        cov_state_estimate_.block(7*ii,n,7,7) = cov_block;
        cov_state_estimate_.block(n,7*ii,7,7) = cov_block.transpose();

        if(ii == 0)
            cov_state_estimate_.block(n,n,7,7) = cov_state_estimate_.block(7*ii,0,7,7);
    }
}


