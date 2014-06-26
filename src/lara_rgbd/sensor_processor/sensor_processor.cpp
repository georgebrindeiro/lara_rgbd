#include <lara_rgbd/sensor_processor/sensor_processor.h>

void SensorProcessor::init()
{
    max_size_ = 20;

    const float min_scale = 0.1f;
    const int n_octaves = 6;
    const int n_scales_per_octave = 10;
    const float min_contrast = 0.5f;

    keypoint_detector_.reset(new pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> ());
    keypoint_detector_->setScales(min_scale, n_octaves, n_scales_per_octave);
    keypoint_detector_->setMinimumContrast(min_contrast);

    feature_extractor_.reset(new pcl::PFHRGBEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHRGBSignature250>);
    feature_extractor_->setKSearch(50);
}

bool SensorProcessor::enqueue(const pcl::PointCloud<pcl::PointXYZRGB>& cloud)
{
    if(processing_queue_.size() < max_size_)
    {
        std::cout << "queue: " << processing_queue_.size() << std::endl;
        processing_queue_.push(cloud);
        return true;
    }
    else
    {
        return false;
    }
}

pcl::PointCloud<pcl::PointXYZRGB> SensorProcessor::dequeue()
{
    pcl::PointCloud<pcl::PointXYZRGB> cloud = processing_queue_.front();
    processing_queue_.pop();

    return cloud;
}

bool SensorProcessor::process(pcl::PointCloud<pcl::PointWithScale>& keypoint_cloud,
                              pcl::PointCloud<pcl::PFHRGBSignature250>& feature_cloud)
{
    //std::cout << "process in" << std::endl;

    if(processing_queue_.size() > 0)
    {
        std::cout << "processing cloud..." << std::flush;
        pcl::PointCloud<pcl::PointXYZRGB>* cloud = new pcl::PointCloud<pcl::PointXYZRGB>(dequeue());

        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input(cloud);
        pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints(new pcl::PointCloud<pcl::PointWithScale>());

        // Extract keypoints, calculate descriptors, return cloud+feature_cloud

        // Estimate the sift interest points using Intensity values from RGB values
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB> ());
        keypoint_detector_->setSearchMethod(tree);
        keypoint_detector_->setInputCloud(input);
        pcl::StopWatch watch;
        keypoint_detector_->compute(*keypoints);
        std::cout << "OK." << std::endl;
        printf("Detected %zd points in %lfs\n", keypoints->size (), watch.getTimeSeconds ());

        // Calculate descriptors
        pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr features(new pcl::PointCloud<pcl::PFHRGBSignature250>);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr kpts(new pcl::PointCloud<pcl::PointXYZRGB>);
        kpts->points.resize(keypoints->points.size());

        pcl::copyPointCloud(*keypoints, *kpts);

        pcl::FeatureFromNormals<pcl::PointXYZRGB, pcl::Normal, pcl::PFHRGBSignature250>::Ptr feature_from_normals = boost::dynamic_pointer_cast<pcl::FeatureFromNormals<pcl::PointXYZRGB, pcl::Normal, pcl::PFHRGBSignature250> > (feature_extractor_);

        feature_extractor_->setSearchSurface(input);
        feature_extractor_->setInputCloud(kpts);

        if (feature_from_normals)
        //if (boost::dynamic_pointer_cast<typename pcl::FeatureFromNormals<pcl::PointXYZRGB, pcl::Normal, pcl::PFHRGBSignature250> > (feature_extractor_))
        {
            std::cout << "normal estimation..." << std::flush;
            pcl::PointCloud<pcl::Normal>::Ptr normals (new  pcl::PointCloud<pcl::Normal>);
            pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimation;
            normal_estimation.setSearchMethod (pcl::search::Search<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));
            normal_estimation.setRadiusSearch (0.01);
            normal_estimation.setInputCloud (input);
            normal_estimation.compute (*normals);
            feature_from_normals->setInputNormals(normals);
            std::cout << "OK" << std::endl;
        }

        std::cout << "descriptor extraction..." << std::flush;
        watch.reset();
        feature_extractor_->compute (*features);
        std::cout << "OK." << std::endl;
        printf("Calculated %zd descriptors in %lfs\n", features->size (), watch.getTimeSeconds ());

        keypoints->header = cloud->header;
        features->header = cloud->header;

        keypoint_cloud = *keypoints;
        feature_cloud = *features;
        std::cout << "process out true" << std::endl;
        return true;
    }

    //std::cout << "process out false" << std::endl;
    return false;
}
