/**
 * @file   state_estimator.h
 * @author George Andrew Brindeiro (georgebrindeiro@lara.unb.br)
 * @date   Apr 24, 2014
 *
 * @attention Copyright (C) 2014
 * @attention Laboratório de Automação e Robótica (LARA)
 * @attention Universidade de Brasília (UnB)
 */

#ifndef LARA_RGBD_MAP_MANAGER_H_
#define LARA_RGBD_MAP_MANAGER_H_

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>

// PCL Registration headers
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_registration.h>

// kdtree search
#include <pcl/search/kdtree.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

#include <lara_rgbd_msgs/PoseWithCovarianceStampedArray.h>

class MapManager
{
    public:
        MapManager()
        {
            num_keyframes_ = 0;
        }

        ~MapManager() {}

        bool match(Eigen::VectorXf& pose_vector, Eigen::MatrixXf& pose_covariance, pcl::PointCloud<pcl::PointWithScale> & keypoint_cloud, pcl::PointCloud<pcl::PFHRGBSignature250> & feature_cloud, Eigen::VectorXf& measurement_vector);
        void update(Eigen::VectorXf state_vector);

       /*!
        * @brief Repackages keyframes into a ROS message for publishing
        *
        * Repackages keyframes into a geometry_msgs::PoseWithCovarianceStampedArray ROS message for publishing
        */
        void keyframes(lara_rgbd_msgs::PoseWithCovarianceStampedArray& keyframes);

    private:
        int num_keyframes_;                                     /**< Number of keyframes added to history */
        std::vector< Eigen::VectorXf > keyframes_pose_;         /**< State vector for keyframes */
        std::vector< Eigen::MatrixXf > keyframes_cov_;          /**< Covariance matrix for keyframes */
        std::vector< pcl::PointCloud<pcl::PointWithScale> > keyframes_keypoints_;    /**< Keypoints cloud for keyframes */
        std::vector< pcl::PointCloud<pcl::PFHRGBSignature250> > keyframes_features_;    /**< Feature cloud for keyframes */
        std::vector< pcl::search::KdTree<pcl::PFHRGBSignature250> > keyframes_kdtree_;   /**< Feature kdtree for keyframes */

};

/*MapManager::match(pose_estimate, feature_cloud)
{
    // for every cloud in db, estimate and reject correspondences
        // add surviving correspondences to measurement difference vector: current(transformed)-past
        // covariance for each match: comes from ransac rejection?
}

MapManager::update(state_estimate, <cloud,feature_cloud>)
{
    // for every pose in state_estimate, copy to nearest-neighbor kdtree
    // if state_estimate size changed, add pose+cloud+feature_cloud to kdtree
}*/

#endif  // LARA_RGBD_MAP_MANAGER_H_
