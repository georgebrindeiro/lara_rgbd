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
    ROS_DEBUG("Cloud matching measurement model still not implemented!");

    // Grab current pose and covariance
    Eigen::VectorXf current_pose = state_estimate_.head(7);
    Eigen::MatrixXf current_cov = cov_state_estimate_.block(0,0,7,7);

    // Separate pose for transform calculation
    Eigen::Vector3f p(current_pose[0], current_pose[1], current_pose[2]);
    Eigen::Quaternionf q(current_pose[3],current_pose[4],current_pose[5],current_pose[6]);

    // Convert cloud_msg to PCL format
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::fromROSMsg(*cloud_msg, *current_cloud_ptr);

    // Turn into ConstPtr (probably in the dumbest way possible)
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr current_cloud(current_cloud_ptr);

    // Look for matches in pose history
    bool matches_found = false;

    for(int i = 0; i < num_keyframes_; i++)
    {
        // Grab past pose
        Eigen::VectorXf past_pose = keyframes_pose_[i];

        // Get past cloud kdtree
        pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree = keyframes_kdtree_[i];

        // Coordinate transform current cloud to past cloud coordinate frame
        Eigen::Vector3f p_star(past_pose[0], past_pose[1], past_pose[2]);
        Eigen::Quaternionf q_star(past_pose[3], past_pose[4], past_pose[5], past_pose[6]);

        Eigen::Matrix3f R = (q_star*q.inverse()).toRotationMatrix();
        Eigen::Vector3f T = p_star-p;

        ROS_INFO_STREAM("R" << std::endl << R);
        ROS_INFO_STREAM("T" << std::endl << T);

        // Need to find correspondences between current_cloud and past_cloud before attempting to align...
        // Use kdtree radius search to do that
        std::vector<int> indices_src;
        std::vector<int> indices_tgt;

        float radius = 0.1;

        Eigen::Vector3f c_j;
        pcl::PointXYZRGB search_point;

        for(int j = 0; j < current_cloud->size(); j++)
        {
            // Check if NaN
            //if(!pcl_isfinite((*current_cloud)[j].x) || !pcl_isfinite((*current_cloud)[j].y) || !pcl_isfinite((*current_cloud)[j].z))
            //    continue;

            // Get one point in current cloud
            c_j[0] = (*current_cloud)[j].x;
            c_j[1] = (*current_cloud)[j].y;
            c_j[2] = (*current_cloud)[j].z;

            ROS_INFO_STREAM("c_" << j << std::endl << c_j);

            // Transform to past cloud coordinate frame
            c_j = R*c_j+T;

            ROS_INFO_STREAM("c_" << j << "*" << std::endl << c_j);

            // Change to pcl format and add rgb info
            search_point.x = c_j[0];
            search_point.y = c_j[1];
            search_point.z = c_j[2];
            search_point.r = (*current_cloud)[j].r;
            search_point.g = (*current_cloud)[j].g;
            search_point.b = (*current_cloud)[j].b;

            std::vector<int> point_idx_vector;
            std::vector<float> point_sq_dist_vector;

            int point_idx = -1;
            float point_sq_dist = -1;

            ROS_INFO("still alive!");

            if(kdtree.radiusSearch(search_point, radius, point_idx_vector, point_sq_dist_vector) > 0)
            {
                // Get first nearest neighbor in radius search results
                point_idx = point_idx_vector[0];
                point_sq_dist = point_sq_dist_vector[0];

                ROS_DEBUG("Found correspondence! old_idx: %d new_idx: %d (sq_dist=%f)", j, point_idx, point_sq_dist);
                indices_src.push_back(j);
                indices_tgt.push_back(point_idx);
            }
            else
            {
                ROS_DEBUG("Found no correspondences for old_idx: %d with radius %f", j, radius);
            }
        }

        // Attempt RANSAC alignment
        ROS_INFO_STREAM("Attempting to align current cloud with keyframe #" << i);

        // RANSAC Registration Model
        pcl::SampleConsensusModelRegistration<pcl::PointXYZRGB>::Ptr model_r(new pcl::SampleConsensusModelRegistration<pcl::PointXYZRGB>(current_cloud));

        // Set target cloud
        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr past_cloud(&keyframes_cloud_[i]);
        model_r->setInputTarget(past_cloud, indices_tgt);

        // Use model in RANSAC framework and set parameters
        pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac(model_r);
        ransac.setDistanceThreshold (0.1);
        ransac.computeModel(1);

        Eigen::VectorXf coeffs;
        ransac.getModelCoefficients(coeffs);
        assert(coeffs.size() == 16);

        // Resulting transform
        Eigen::Matrix4f transform = Eigen::Map<Eigen::Matrix4f>(coeffs.data(),4,4);

        std::vector<int> inliers;
        ransac.getInliers(inliers);

        ROS_INFO_STREAM("Found " << inliers.size()   << " inliers!");

        ROS_INFO_STREAM("transform: " << std::endl << transform);

        // Check if good match
        // Separate good matches and go through cloud matching model equations
    }

    if((num_keyframes_ == 0) || matches_found)
    {
        // Augment state vector with initial pose
        augment_state_vector_();

        ROS_DEBUG_STREAM("aug init state" << std::endl << state_estimate_);
        ROS_DEBUG_STREAM("aug init cov_state" << std::endl << cov_state_estimate_);

        // Generate kdtree for feature cloud
        pcl::KdTreeFLANN<pcl::PointXYZRGB> current_kdtree;
        current_kdtree.setInputCloud(current_cloud);

        // Store keyframe
        keyframes_pose_.push_back(current_pose);
        keyframes_cov_.push_back(current_cov);
        keyframes_cloud_.push_back(*current_cloud);
        keyframes_kdtree_.push_back(current_kdtree);

        num_keyframes_++;
    }
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

void StateEstimator::keyframes(lara_rgbd_msgs::PoseWithCovarianceStampedArray& keyframes)
{
    char frame_id[20];

    for(int i = 0; i < num_keyframes_; i++)
    {
        geometry_msgs::PoseWithCovarianceStamped keyframe;

        sprintf(frame_id, "/base_link_%d", i);

        keyframe.header.frame_id = frame_id;
        keyframe.header.stamp = keyframes.header.stamp;

        keyframe.pose.pose.position.x = keyframes_pose_[i](0);
        keyframe.pose.pose.position.y = keyframes_pose_[i](1);
        keyframe.pose.pose.position.z = keyframes_pose_[i](2);
        keyframe.pose.pose.orientation.w = keyframes_pose_[i](3);
        keyframe.pose.pose.orientation.x = keyframes_pose_[i](4);
        keyframe.pose.pose.orientation.y = keyframes_pose_[i](5);
        keyframe.pose.pose.orientation.z = keyframes_pose_[i](6);

        ROS_DEBUG("keyframes not fully implemented. returning fixed covariance for orientation!");
        for(int ii = 0; ii < 3; ii++)
        {
            for(int jj = 0; jj < 3; jj++)
            {
                keyframe.pose.covariance[6*ii+jj] = keyframes_cov_[i](ii,jj);
            }
        }

        keyframe.pose.covariance[18+3] = 0.01;
        keyframe.pose.covariance[24+4] = 0.01;
        keyframe.pose.covariance[30+5] = 0.01;

        //ROS_INFO_STREAM_THROTTLE(1, "keyframe #" << i << ":\n\n" << keyframe << "\n");

        keyframes.poses.push_back(keyframe);
    }
}

void StateEstimator::augment_state_vector_()
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


