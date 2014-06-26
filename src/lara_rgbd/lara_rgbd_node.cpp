#include <lara_rgbd/lara_rgbd_node.h>

// LARA RGBD classes
StateEstimator state_estimator;     // Init state vector+covariance, model params
SensorProcessor sensor_processor;   // Init keypoint detector and feature descriptors
MapManager map_manager;             // Init cloud storage: empty kdtree with poses, empty std::vectors of xyzrgb and feature clouds

pcl::PointCloud<pcl::PointXYZRGB> xyzrgb_cloud;
pcl::PointCloud<pcl::PointWithScale> keypoint_cloud;
pcl::PointCloud<pcl::PFHRGBSignature250> feature_cloud;

Eigen::VectorXf pose_vector;
Eigen::MatrixXf pose_covariance;

Eigen::VectorXf measurement_vector;

int main (int argc, char** argv)
{
    // ROS Init
    ros::init(argc, argv, "lara_rgbd");
    ros::NodeHandle nh;

    // ROS Subscribers
    ros::Subscriber sub_cloud = nh.subscribe("/rgbd_dataset/kinect/depth_registered/points", 1, cloud_cb);

    // ROS Publishers
    // Add stuff I want published here, use functions inside classes: SensorProcessor, StateEstimator, MapManager...
    pub_feature_cloud = nh.advertise<sensor_msgs::PointCloud2>("feature_cloud", 1);
    pub_pose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("estimated_pose", 1);
    pub_keyframes = nh.advertise<lara_rgbd_msgs::PoseWithCovarianceStampedArray>("keyframes", 1);

    ros::Rate loop_rate(30);

    // Spin
    while(ros::ok())
    {
        // Estimation loop
        state_estimator.predict();                          // Apply motion model. For now, quasiconstant

        bool new_feature_cloud = sensor_processor.process(keypoint_cloud, feature_cloud);      // Extract keypoints and calculate descriptors from queued cloud

        if(new_feature_cloud)
        {
            // Start what MapManager will do

            // Compare to previous clouds and return measurement vector+covariance
            state_estimator.pose_and_covariance(pose_vector, pose_covariance);
            bool new_keyframe = map_manager.match(pose_vector, pose_covariance, keypoint_cloud, feature_cloud, measurement_vector);

            if(new_keyframe)
                state_estimator.augment_state_vector();

            // Use measurement vector+covariance to correct state estimate. If (measurement_vector.size() < thresh), add pose after correction
            //state_estimator.correct(measurement_vector);

            // Update poses used in kdtree search
            //<state_vector,state_covariance> = state_estimator.state();
            //map_manager.update(state_vector);
        }

        pub_feature_cloud_estimate();
        pub_pose_estimate();
        pub_keyframes_estimate();

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
    //ROS_INFO("cloud_cb in");

    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);

    if(!sensor_processor.enqueue(cloud))
        ROS_WARN("Sensor processing queue is full! Cloud message dropped.");
    else
        ROS_INFO("Cloud message enqueued.");

    //ROS_INFO("cloud_cb out");
}

void pub_fake_pose_estimate(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    // Fake pose estimate from odometry
    geometry_msgs::PoseWithCovarianceStamped::Ptr estimated_pose_msg(new geometry_msgs::PoseWithCovarianceStamped);
    estimated_pose_msg->header = odom_msg->header;
    estimated_pose_msg->header.frame_id = "world";
    estimated_pose_msg->pose = odom_msg->pose;

    estimated_pose_msg->pose.covariance[0] = 1;
    estimated_pose_msg->pose.covariance[6+1] = 1;
    estimated_pose_msg->pose.covariance[12+2] = 0.1;
    estimated_pose_msg->pose.covariance[18+3] = 0.01;
    estimated_pose_msg->pose.covariance[24+4] = 0.01;
    estimated_pose_msg->pose.covariance[30+5] = 0.01;

    pub_pose.publish(estimated_pose_msg);
}

void pub_pose_estimate()
{
    // Real pose estimate from state_estimator
    geometry_msgs::PoseWithCovarianceStamped current_pose;

    current_pose.header.stamp = ros::Time::now();
    current_pose.header.frame_id = "world";
    state_estimator.estimated_pose(current_pose);

    pub_pose.publish(current_pose);
}

void pub_keyframes_estimate()
{
    lara_rgbd_msgs::PoseWithCovarianceStampedArray keyframes;

    keyframes.header.stamp = ros::Time::now();
    keyframes.header.frame_id = "world";
    map_manager.keyframes(keyframes);

    pub_keyframes.publish(keyframes);
}

void pub_feature_cloud_estimate()
{
    sensor_msgs::PointCloud2 msg;

    pcl::toROSMsg(keypoint_cloud, msg);

    pub_feature_cloud.publish(msg);
}
