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

#include <lara_rgbd/map_manager/map_manager.h>

bool MapManager::match(Eigen::VectorXf& pose_vector, Eigen::MatrixXf& pose_covariance, pcl::PointCloud<pcl::PointWithScale>& keypoint_cloud, pcl::PointCloud<pcl::PFHRGBSignature250>& feature_cloud, Eigen::VectorXf& measurement_vector)
{
    // This stores the biggest number of point matches with past clouds
    int max_matches = 0;

    pcl::PointCloud<pcl::PFHRGBSignature250>::ConstPtr current_feature(new pcl::PointCloud<pcl::PFHRGBSignature250>(feature_cloud));
    pcl::PointCloud<pcl::PointWithScale>::ConstPtr current_keypoint(new pcl::PointCloud<pcl::PointWithScale>(keypoint_cloud));

    pcl::registration::CorrespondenceEstimation<pcl::PFHRGBSignature250, pcl::PFHRGBSignature250> est;
    est.setInputSource(current_feature);

    pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointWithScale> rej;
    rej.setInputSource(current_keypoint);
    rej.setInlierThreshold(0.05);
    rej.setMaximumIterations(300*300);

    for(int i = 0; i < num_keyframes_; i++)
    {
        pcl::PointCloud<pcl::PFHRGBSignature250>::ConstPtr past_feature(new pcl::PointCloud<pcl::PFHRGBSignature250>(keyframes_features_[i]));
        pcl::PointCloud<pcl::PointWithScale>::ConstPtr past_keypoint(new pcl::PointCloud<pcl::PointWithScale>(keyframes_keypoints_[i]));

        // Correspondence Estimation. if: not enough, next keyframe
        pcl::Correspondences all_correspondences;
        est.setInputTarget(past_feature);
        est.determineReciprocalCorrespondences(all_correspondences);

        // Correspondence Rejection.
        // This should be done using RANSAC
        pcl::Correspondences remaining_correspondences;
        rej.setInputTarget(past_keypoint);
        rej.getRemainingCorrespondences(all_correspondences, remaining_correspondences);

        // Check if max matches
        int est_matches = all_correspondences.size();
        int rej_matches = remaining_correspondences.size();

        std::cout << "Estimated matches: " << est_matches << " at keyframe #" << i << std::endl;
        std::cout << "Remaining matches: " << rej_matches << " at keyframe #" << i << std::endl;

        if(rej_matches > max_matches)
        {
            max_matches = rej_matches;
            std::cout << "Max matches: " << max_matches << " at keyframe #" << i << std::endl;
        }

    }

    if(max_matches < 100 || num_keyframes_ == 0)
    {
        // Generate kdtree for feature cloud
        pcl::search::KdTree<pcl::PFHRGBSignature250> feature_kdtree;
        feature_kdtree.setInputCloud(current_feature);

        // Store keyframe
        keyframes_pose_.push_back(pose_vector);
        keyframes_cov_.push_back(pose_covariance);
        keyframes_keypoints_.push_back(keypoint_cloud);
        keyframes_features_.push_back(feature_cloud);
        keyframes_kdtree_.push_back(feature_kdtree);

        num_keyframes_++;

        std::cout << "Num keyframes: " << num_keyframes_ << std::endl;

        // Return true to signal that state vector must be augmented
        return true;
    }
    else
        return false;
}

void MapManager::update(Eigen::VectorXf state_vector)
{
}

void MapManager::keyframes(lara_rgbd_msgs::PoseWithCovarianceStampedArray& keyframes)
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

        //ROS_DEBUG("keyframes not fully implemented. returning fixed covariance for orientation!");
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
