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
        private_nh_.param<std::string>("scan_frame", scan_frame_, "laser");

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

        // Initializing cluster extractor
        search_tree_ = pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>);
        cluster_extr_.setClusterTolerance(tolerance);
        cluster_extr_.setMaxClusterSize(cluster_max);
        cluster_extr_.setMinClusterSize(cluster_min);
        cluster_extr_.setSearchMethod(search_tree_);
        /* segmenter_.setOptimizeCoefficients(true);
        segmenter_.setModelType(pcl::SACMODEL_PLANE);
        segmenter_.setMethodType(pcl::SAC_RANSAC);
        segmenter_.setMaxIterations(100);
        segmenter_.setDistanceThreshold(tolerance); */
    }

    void ClusterTracker::_init_KFilters(size_t cnt)
    {
        int stateDim = 4; // [x, y, v_x, v_y]
        int measDim = 2;
        float dx = 1.0f, dy = dx, dvx = 0.01f, dvy = dvx;
        double sigmaP = 0.01, sigmaQ = 0.1;
        boost::unique_lock<boost::recursive_mutex> lock(filter_mutex_);
        for (size_t i = 0; i < cnt; i++)
        {
            cv::KalmanFilter *_filt = new cv::KalmanFilter(stateDim, measDim);
            _filt->transitionMatrix = (cv::Mat_<float>(4, 4) << dx, 0, 1, 0, 0, dy, 0, 1, 0, 0, dvx, 0, 0, 0, 0, dvy);
            cv::setIdentity(_filt->measurementMatrix);
            cv::setIdentity(_filt->processNoiseCov, cv::Scalar::all(sigmaP));
            cv::setIdentity(_filt->measurementNoiseCov, cv::Scalar(sigmaQ));
            k_filters_.push_back(_filt);
        }
    }

    double ClusterTracker::euclidian_dst(geometry_msgs::Point &p1, geometry_msgs::Point &p2)
    {
        return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
    }

    void ClusterTracker::publish_cloud(ros::Publisher &pub, pcl::PointCloud<pcl::PointXYZ>::Ptr cluster)
    {
        sensor_msgs::PointCloud2::Ptr clustermsg(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*cluster, *clustermsg);
        clustermsg->header.frame_id = scan_frame_;
        clustermsg->header.stamp = ros::Time::now();
        pub.publish(*clustermsg);
    }

    void ClusterTracker::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *input_cloud);
        search_tree_->setInputCloud(input_cloud);
        std::vector<pcl::PointIndices> cluster_indices;

        boost::mutex::scoped_lock lock(mutex_);
        cluster_extr_.setInputCloud(input_cloud);
        cluster_extr_.extract(cluster_indices);
        lock.release();

        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_vec;
        std::vector<pcl::PointXYZ> cluster_centres;

        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr _cluster(new pcl::PointCloud<pcl::PointXYZ>);
            float x = 0.0f, y = 0.0f;
            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
            {
                _cluster->push_back((*input_cloud)[*pit]);
                x += _cluster->back().x;
                y += _cluster->back().y;
            }
            _cluster->width = _cluster->size();
            _cluster->height = 1;
            _cluster->is_dense = true;

            pcl::PointXYZ centre;
            centre.x = x / _cluster->size();
            centre.y = y / _cluster->size();
            centre.z = 0.0;

            cluster_vec.push_back(_cluster);
            cluster_centres.push_back(centre);
        }

        if (first_frame_)
        {
            boost::unique_lock<boost::recursive_mutex> lock(filter_mutex_);
            _init_KFilters(cluster_vec.size());
            for (size_t i = 0; i < cluster_vec.size(); i++)
            {
                geometry_msgs::Point pt;
                pt.x = cluster_centres.at(i).x;
                pt.y = cluster_centres.at(i).y;

                k_filters_[i]->statePre.at<float>(0) = pt.x;
                k_filters_[i]->statePre.at<float>(1) = pt.y;
                k_filters_[i]->statePre.at<float>(2) = 0;
                k_filters_[i]->statePre.at<float>(3) = 0;

                prev_cluster_centres_.push_back(pt);
            }
            _sync_cluster_publishers();
            first_frame_ = false;
            lock.unlock();
        }
        else
        {
            std_msgs::Float32MultiArray cc;
            for (size_t i = 0; i < cluster_centres.size(); i++)
            {
                auto c = cluster_centres.at(i);
                cc.data.push_back(c.x);
                cc.data.push_back(c.y);
                cc.data.push_back(c.z);
            }

            KFTrack(cc);
            _sync_cluster_publishers();

            boost::mutex::scoped_lock lock(obj_mutex_);
            for (auto it = objID.begin(); it != objID.end(); it++)
                publish_cloud(cluster_pubs_[*it], cluster_vec[*it]);
        }
    }
}