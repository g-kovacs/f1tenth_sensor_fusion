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
#include <f1tenth_sensor_fusion/KFTracker.hpp>
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <pcl/segmentation/extract_clusters.h>
#include <opencv2/video/tracking.hpp>

namespace f1tenth_sensor_fusion
{
    /// The ClusterTracker class to clusterize a point cloud (converted from a lidar scan) and track said clusters.
    class ClusterTracker
    {
    protected:
        virtual void initialize(int concurrency);
        virtual int _load_params();
        TrackerConfig _config;
        ros::NodeHandle handle_;
        ros::NodeHandle private_handle_;

    private:
        /**
         * Publish a PointCloud to a ROS topic.
         * 
         * @param pub the ROS publisher to said topic
         * @param cluster the cluster of points to be published
         */
        void publish_cloud(ros::Publisher &pub, pcl::PointCloud<pcl::PointXYZ>::Ptr &cluster);

        void publish_objects(const boost::container::vector<pcl::PointXYZ> &cCentres, const boost::container::vector<int> &objIDs);

        /**
         * Create ROS Markers to later publish for enabling the visualization of detected objects.
         * 
         * @param[in] pts predicted cluster centroid points (using KFilter)
         * @param[in] IDs object IDs detected by filters
         * @param[out] markers markers adjusted to fit points
         */
        void fit_markers(const boost::container::vector<pcl::PointXYZ> &pts,
                         const boost::container::vector<int> &IDs, visualization_msgs::MarkerArray &markers);

        void transform_centre(pcl::PointXYZ &centre);

        void sync_cluster_publishers_size(size_t num_clusters);

        /**
         * Process incoming point cloud data: perform clusterization and calculation of cluster centroids, publish output data using ROS 
         * publishers after performing the object detection.
         * 
         * @param cloud_msg the incoming point cloud message
         */
        void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);

        void extract_cluster_data(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                                  boost::container::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cluster_vec,
                                  boost::container::vector<pcl::PointXYZ> &cluster_centres);

        pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster_extr_;
        boost::mutex mutex_;

        KFTracker _KFTracker;
        boost::container::vector<ros::Publisher *> cluster_pubs_;
        ros::Publisher obj_pub_;
        ros::Publisher marker_pub_;
        message_filters::Subscriber<sensor_msgs::PointCloud2> sub_;

        size_t input_queue_size_;
        size_t publisher_prune_ctr_ = 0;
        bool transform_;
        bool first_frame_ = true;
    };
}

#endif // F1TENTH_SENSOR_FUSION__CLUSTER_TRACKER_H
