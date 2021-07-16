#ifndef POINT_CLOUD__CLUSTER_TRACKER_H
#define POINT_CLOUD_CLUSTER_TRACKER_H

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <iterator>
#include <vector>

namespace point_cloud
{
    class ClusterTracker : public nodelet::Nodelet
    {
    public:
        ClusterTracker();

    private:
        void onInit();
        void _init_extractor();

        void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
        std::vector<pcl::PointIndices> cluster_indices_;

        pcl::SACSegmentation<pcl::PointXYZ> segmenter_;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr search_tree_;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster_extr_;

        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        boost::mutex mutex_;

        std::vector<cv::KalmanFilter> k_filters_;
        std::vector<ros::Publisher> cluster_pubs_;
        ros::Publisher objID_pub_;
        ros::Subscriber sub_;

        unsigned int input_queue_size_;
        bool first_frame_ = true;
    };
}

#endif // POINT_CLOUD_CLUSTER_TRACKER_H