#include <point_cloud/cluster_tracker.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/thread.hpp>

namespace point_cloud
{
    ClusterTracker::ClusterTracker() {}

    void ClusterTracker::onInit()
    {
        boost::mutex::scoped_lock lock(mutex_);
        private_nh_ = getPrivateNodeHandle();

        int concurrency_level = private_nh_.param("tracker_concurrency_level", concurrency_level);

        // Check if explicitly single threaded, otherwise, let nodelet manager dictate thread pool size
        if (concurrency_level == 1)
        {
            nh_ = getNodeHandle();
        }
        else
        {
            nh_ = getMTNodeHandle();
        }

        // Only queue one pointcloud per running thread
        if (concurrency_level > 0)
        {
            input_queue_size_ = static_cast<unsigned int>(concurrency_level);
        }
        else
        {
            input_queue_size_ = boost::thread::hardware_concurrency();
        }

        this->_init_extractor();

        /// TODO: Subscriber needs to be initialized somewhere
    }

    void ClusterTracker::_init_extractor()
    {
        // Query parameters
        double tolerance = private_nh_.param<double>("tolerance", 0.2);
        int cluster_max = private_nh_.param<int>("max_cluster_size", 300);
        int cluster_min = private_nh_.param<int>("min_cluster_size", 30);

        // Initializing cluster extractor and plane segmenter
        search_tree_ = pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>);
        cluster_extr_.setClusterTolerance(tolerance);
        cluster_extr_.setMaxClusterSize(cluster_max);
        cluster_extr_.setMinClusterSize(cluster_min);
        cluster_extr_.setSearchMethod(search_tree_);
        segmenter_.setOptimizeCoefficients(true);
        segmenter_.setModelType(pcl::SACMODEL_PLANE);
        segmenter_.setMethodType(pcl::SAC_RANSAC);
        segmenter_.setMaxIterations(100);
        segmenter_.setDistanceThreshold(tolerance);
    }
}