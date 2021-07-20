#ifndef POINT_CLOUD__CLUSTER_TRACKER_H
#define POINT_CLOUD__CLUSTER_TRACKER_H

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Point.h>
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
        class Cluster2PubSync;

    private:
        virtual void onInit();
        void _init_extractor();
        void _init_KFilters(size_t);

        double euclidian_dst(geometry_msgs::Point &, geometry_msgs::Point &);
        void publish_cloud(ros::Publisher &, pcl::PointCloud<pcl::PointXYZ>::Ptr &);
        void KFTrack(const std_msgs::Float32MultiArray &, std::vector<pcl::PointXYZ> &);

        void sync_cluster_publishers_size(size_t);

        void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
        std::vector<geometry_msgs::Point> prev_cluster_centres_;

        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        boost::mutex mutex_;
        boost::recursive_mutex filter_mutex_;
        boost::mutex obj_mutex_;

        std::vector<int> objID;
        std::vector<cv::KalmanFilter *> k_filters_;
        std::vector<ros::Publisher *> cluster_pubs_;
        ros::Publisher objID_pub_;
        ros::Publisher marker_pub_;
        message_filters::Subscriber<sensor_msgs::PointCloud2> sub_;

        unsigned int input_queue_size_;
        double tolerance_;
        int cluster_max_;
        int cluster_min_;
        bool first_frame_ = true;
        std::string scan_frame_;
    };
}

#endif // POINT_CLOUD__CLUSTER_TRACKER_H