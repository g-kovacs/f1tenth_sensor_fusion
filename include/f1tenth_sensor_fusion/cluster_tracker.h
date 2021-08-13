/* Copyright 2021 Kovács Gergely Attila
*
*   Licensed under the Apache License,
*   Version 2.0(the "License");
*  you may not use this file except in compliance with the License.
*  You may obtain a copy of the License at
*
*      http://www.apache.org/licenses/LICENSE-2.0
*
*  Unless required by applicable law or agreed to in writing, software
*  distributed under the License is distributed on an "AS IS" BASIS,
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*  See the License for the specific language governing permissions and
*  limitations under the License. 
*   
*/

#ifndef F1TENTH_SENSOR_FUSION__CLUSTER_TRACKER_H
#define F1TENTH_SENSOR_FUSION__CLUSTER_TRACKER_H

#include <f1tenth_sensor_fusion/tracker_config.hpp>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
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
#include <memory>

namespace f1tenth_sensor_fusion
{

#define prune_interval 50

    /// The ClusterTracker class to clusterize a point cloud (converted from a lidar scan) and track said clusters.
    class ClusterTracker : public nodelet::Nodelet
    {
    public:
        ClusterTracker();
        virtual ~ClusterTracker() = 0;

    protected:
        virtual void onInit();

        /// Initialize nodelet with necessary parameters.
        virtual int _load_params();

        /**
         * Initialize a given number of Kalman-filters. Parameters only, the initial states must be set after calling this function.
         * 
         * @param n the number of filters to initialize
         */
        void _init_KFilters(size_t n);

        template <class C>
        void _set_kfilter_state_pre(const C &pt, std::unique_ptr<cv::KalmanFilter> &filter);

        /// Calculate the Euclidian distance of two 3D points.
        float euclidian_dst(geometry_msgs::Point &, geometry_msgs::Point &);

        /**
         * Return the indices of the smallest element of a 2D matrix. Each row represents a KF predition, each column a detected cluster centroid, their 
         * intersections the distance between the two.
         * 
         * @param distMat the distance matrix
         * @return the indicies of the minimum element. <-1, -1> if no minimum was found (i.e. all elements equal)
         */
        std::pair<int, int> find_min_IDX(std::vector<std::vector<float>> &dist_mat);

        /**
         * Publish a PointCloud to a ROS topic.
         * 
         * @param pub the ROS publisher to said topic
         * @param cluster the cluster of points to be published
         */
        void publish_cloud(ros::Publisher &pub, pcl::PointCloud<pcl::PointXYZ>::Ptr &cluster);

        void publish_objects();

        /**
         * Track detected clusters using Kalman-filters. 
         * 
         * First, predictions of already working Kalman-filters are registered and matched against measured point data to find correspondence. Then, 
         * unmatched points are assigned to a new filter, or those filters are deleted that have not been used for the last few input messages. If 
         * visualization is enabled, markers are assigned to each tracked cluster. Finally, the Kalman-filters are updated based on measured data.
         * 
         * @param ccs data of detected cluster centres as a multi array
         */
        void KFTrack(const std_msgs::Float32MultiArray &ccs);

        std::vector<geometry_msgs::Point> generate_predictions();

        /**
         * Initialize a vector of negative ones for representing object IDs, then match incoming prediction data to measured cluster data. Keep track of 
         * successfully matched clusters.
         * 
         * @param[in] pred The predictions of KFilters in use
         * @param[in] cCentres The measured point cloud cluster data
         * @param[out] used Flags of which clusters were successfully matched
         * @return The list of cluster indices matched to each filter (-1 if no match for a certain filter)
         */
        std::vector<int> match_objID(const std::vector<geometry_msgs::Point> &pred, const std::vector<geometry_msgs::Point> &cCentres, bool *cluster_used);

        void create_kfilters_for_new_clusters(const std::vector<geometry_msgs::Point> &centres, const bool *cluster_used);
        void prune_unused_kfilters();

        /**
         * Create ROS Markers to later publish for enabling the visualization of detected objects.
         * 
         * @param[in] pts predicted cluster centroid points (using KFilter)
         * @param[in] IDs object IDs detected by filters
         * @param[out] markers markers adjusted to fit points
         */
        void fit_markers(const std::vector<geometry_msgs::Point> &pts, const std::vector<int> &IDs, visualization_msgs::MarkerArray &markers);

        void transform_centre(geometry_msgs::Point &centre);

        void correct_kfilter_matrices(const std::vector<geometry_msgs::Point> &cCentres);

        void sync_cluster_publishers_size(size_t num_clusters);

        /**
         * Process incoming point cloud data: perform clusterization and calculation of cluster centroids, publish output data using ROS 
         * publishers after performing the object detection.
         * 
         * @param cloud_msg the incoming point cloud message
         */
        void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);

        void extract_cluster_data(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, const std::vector<pcl::PointIndices> &cluster_indices, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cluster_vec, std::vector<pcl::PointXYZ> &cluster_centres);

        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster_extr_;
        boost::recursive_mutex filter_mutex_;
        boost::mutex mutex_;

        std::vector<int> objID;
        std::vector<std::unique_ptr<cv::KalmanFilter>> k_filters_;
        std::vector<ros::Publisher *> cluster_pubs_;
        ros::Publisher objID_pub_;
        ros::Publisher marker_pub_;
        message_filters::Subscriber<sensor_msgs::PointCloud2> sub_;

        TrackerConfig _config;

        size_t input_queue_size_;
        size_t kf_prune_ctr_ = 0;
        size_t publisher_prune_ctr_ = 0;
        bool transform_;
        bool first_frame_ = true;
    };
}

#endif // F1TENTH_SENSOR_FUSION__CLUSTER_TRACKER_H
