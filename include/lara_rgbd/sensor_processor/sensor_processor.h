#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/common/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfhrgb.h>

#include <iostream>
#include <queue>
#include <utility>

class SensorProcessor
{
    public:
        SensorProcessor()
        {
            init();
        }

        bool enqueue(const pcl::PointCloud<pcl::PointXYZRGB>& cloud);
        bool process(std::pair< pcl::PointCloud<pcl::PointXYZRGB>, pcl::PointCloud<pcl::PointWithScale> >& processed_clouds);

    private:
        void init();
        pcl::PointCloud<pcl::PointXYZRGB> dequeue();

        std::queue< pcl::PointCloud<pcl::PointXYZRGB> > processing_queue_;
        int max_size_;

        boost::shared_ptr< pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> > keypoint_detector_;
        pcl::Feature<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::Ptr feature_extractor_;
};
