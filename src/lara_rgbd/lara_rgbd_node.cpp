#include <lara_rgbd/lara_rgbd_node.h>

// LARA RGBD classes
//StateEstimator state_estimator;     // Init state vector+covariance, model params
SensorProcessor sensor_processor;   // Init keypoint detector and feature descriptors
//MapManager map_manager;             // Init cloud storage: empty kdtree with poses, empty std::vectors of xyzrgb and feature clouds

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

    ros::Rate loop_rate(30);

    // Spin
    while(ros::ok())
    {
        // Estimation loop
        //state_estimator.predict();                                              // Apply motion model. For now, quasiconstant
        std::pair< pcl::PointCloud<pcl::PointXYZRGB>, pcl::PointCloud<pcl::PointWithScale> > processed_clouds;
        if(sensor_processor.process(processed_clouds)) // Extract keypoints and calculate descriptors
            ROS_INFO("Successful pair return");
        else
            ROS_INFO("Returned empty pair");
        //pcl::PointCloud<pcl::PointWithScale>::Ptr feature_cloud = new pcl::PointCloud<pcl::PointWithScale>();
        //<pose_vector,pose_covariance> = state_estimator.pose();
        //measurement_vector = map_manager.match(pose_vector, feature_cloud);     // Compare to previous clouds and return measurement vector+covariance
        //state_estimator.correct(measurement_vector);                            // Use measurement vector+covariance to correct state estimate. If (measurement_vector.size() < thresh), add pose after correction
        //<state_vector,state_covariance> = state_estimator.state();
        //map_manager.update(state_vector, <cloud,feature_cloud>);                // Update poses used in kdtree search.

        // Publish latest: posewithcovariance, posewithcovariancevector, featurecloud, aggregated_featurecloud(too much? need to go through all stored clouds and transform using kdtree pose)
        sensor_msgs::PointCloud2 msg;

        pcl::toROSMsg(processed_clouds.second, msg);

        pub_feature_cloud.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
    ROS_INFO("cloud_cb in");

    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);

    if(!sensor_processor.enqueue(cloud))
        ROS_WARN("Sensor processing queue is full! Cloud message dropped.");
    else
        ROS_INFO("Cloud message enqueued.");

    ROS_INFO("cloud_cb out");
}

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
