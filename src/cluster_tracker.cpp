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

#include <f1tenth_sensor_fusion/cluster_tracker.h>
#include <std_msgs/Int32MultiArray.h>
#include <boost/thread.hpp>
#include <random>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl/search/kdtree.h>
#include <pcl_ros/point_cloud.h>
#include <f1tenth_sensor_fusion/ObjectMessage.h>

namespace f1tenth_sensor_fusion
{
    void ClusterTracker::initialize(int concurrency_level)
    {
        srand(time(NULL));

#ifndef NDEBUG
        _config.info(concurrency_level);
#endif

        cluster_extr_.setClusterTolerance(_config.tolerance);
        cluster_extr_.setMaxClusterSize(_config.clust_max);
        cluster_extr_.setMinClusterSize(_config.clust_min);

        // Only queue one pointcloud per running thread
        if (concurrency_level > 0)
        {
            input_queue_size_ = static_cast<size_t>(concurrency_level);
        }
        else
        {
            input_queue_size_ = boost::thread::hardware_concurrency();
        }

        transform_ = _config.scan_frame.compare(_config.target_frame) == 0 ? false : true;

        // Subscribe to topic with input data
        sub_.subscribe(handle_, _config.scan_topic.c_str(), input_queue_size_);
        sub_.registerCallback(boost::bind(&ClusterTracker::cloudCallback, this, _1));
        // Init marker publisher if necessary
        if (_config.rviz)
            marker_pub_ = handle_.advertise<visualization_msgs::MarkerArray>(_config.tracker_name + std::string("/viz"), 100);
        obj_pub_ = handle_.advertise<ObjectMessage>(_config.tracker_name + std::string("/detections"), 100);
    }

    int ClusterTracker::_load_params()
    {
        _config.rviz = private_handle_.param<bool>("visualize_rviz", _config.rviz);
        _config.tolerance = private_handle_.param<double>("tolerance", _config.tolerance);
        _config.clust_max = private_handle_.param<int>("max_cluster_size", _config.clust_max);
        _config.clust_min = private_handle_.param<int>("min_cluster_size", _config.clust_min);
        _config.marker_size = private_handle_.param<int>("marker_size", _config.marker_size);
        private_handle_.param<std::string>("subscription_frame", _config.scan_frame, _config.scan_frame.c_str());
        private_handle_.param<std::string>("target_frame", _config.target_frame, _config.scan_frame.c_str());
        private_handle_.param<std::string>("subscription_topic", _config.scan_topic, _config.scan_topic.c_str());
        return private_handle_.param("concurrency_level", 0);
    }

    void ClusterTracker::publish_cloud(ros::Publisher &pub, pcl::PointCloud<pcl::PointXYZ>::Ptr &cluster)
    {
        sensor_msgs::PointCloud2::Ptr clustermsg(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*cluster, *clustermsg);
        clustermsg->header.frame_id = _config.target_frame;
        clustermsg->header.stamp = ros::Time::now();
        pub.publish(*clustermsg);
    }

    void ClusterTracker::publish_objects(const boost::container::vector<pcl::PointXYZ> &cCentres, const boost::container::vector<int> &objIDs)
    {
        ObjectMessage msg;
        for (auto it = objIDs.begin(); it != objIDs.end(); ++it)
        {
            ObjectData data;
            data.ID = *it;
            data.centre[0] = *it == -1 ? 0.f : cCentres[*it].x;
            data.centre[1] = *it == -1 ? 0.f : cCentres[*it].y;
            data.centre[2] = *it == -1 ? 0.f : cCentres[*it].z;

            msg.data.push_back(data);
        }
        msg.header.frame_id = _config.target_frame;
        msg.header.stamp = ros::Time::now();
        obj_pub_.publish(msg);
    }

    void ClusterTracker::fit_markers(const boost::container::vector<pcl::PointXYZ> &pts,
                                     const boost::container::vector<int> &IDs, visualization_msgs::MarkerArray &markers)
    {
        for (auto i = 0; i < IDs.size(); i++)
        {
            if (IDs[i] == -1)
                continue;

            visualization_msgs::Marker m;
            m.id = i;
            m.header.frame_id = _config.target_frame;
            m.type = _config.marker_type;
            m.scale.x = (double)_config.marker_size / 100;
            m.scale.y = (double)_config.marker_size / 100;
            m.scale.z = (double)_config.marker_size / 100;
            m.action = visualization_msgs::Marker::ADD;
            m.color.a = 1.0;
            m.color.r = i % 2 ? 1 : 0;
            m.color.g = i % 3 ? 1 : 0;
            m.color.b = i % 4 ? 1 : 0;
            m.lifetime = ros::Duration(0.1);

            pcl::PointXYZ clusterC(pts[IDs[i]]);

            if (transform_)
            {
                transform_centre(clusterC);
            }

            m.pose.position.x = clusterC.x;
            m.pose.position.y = clusterC.y;
            m.pose.position.z = clusterC.z;

            m.pose.orientation.w = 1;
            m.pose.orientation.x = m.pose.orientation.y = m.pose.orientation.z = 0;

            markers.markers.push_back(m);
        }
    }

    void ClusterTracker::transform_centre(pcl::PointXYZ &centres)
    {
        tf2_ros::Buffer buf_;
        tf2_ros::TransformListener tfListener(buf_);

        pcl::PointXYZ trans_cluster;

        geometry_msgs::TransformStamped trans_msg;
        tf2::Stamped<tf2::Transform> stamped_trans;
        try
        {
            trans_msg = buf_.lookupTransform(_config.target_frame, _config.scan_frame, ros::Time(0), ros::Duration(0.2));
            tf2::fromMsg(trans_msg, stamped_trans);
            trans_msg = tf2::toMsg(tf2::Stamped<tf2::Transform>(stamped_trans.inverse(), stamped_trans.stamp_, stamped_trans.frame_id_));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            return;
        }

        tf2::doTransform(centres, trans_cluster, trans_msg);
        centres = trans_cluster;
    }

    void ClusterTracker::sync_cluster_publishers_size(size_t num_clusters)
    {
        // Remove unnecessary publishers from time to time
        if (cluster_pubs_.size() > num_clusters && publisher_prune_ctr_++ > prune_interval)
        {
            while (cluster_pubs_.size() > num_clusters)
            {
                cluster_pubs_.back()->shutdown();
                cluster_pubs_.pop_back();
            }
            publisher_prune_ctr_ = 0;
        }

        while (num_clusters > cluster_pubs_.size())
        {
            try
            {
                std::stringstream ss;
                ss << _config.tracker_name << "/cluster_" << cluster_pubs_.size();
                ros::Publisher *pub = new ros::Publisher(handle_.advertise<sensor_msgs::PointCloud2>(ss.str(), 100));
                cluster_pubs_.push_back(pub);
            }
            catch (ros::Exception &ex)
            {
                ROS_ERROR(ex.what());
            }
        }
    }

    void ClusterTracker::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *input_cloud);

        boost::container::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_vec;
        boost::container::vector<pcl::PointXYZ> cluster_centres;
        boost::container::vector<int> objIDs;

        extract_cluster_data(input_cloud, cluster_vec, cluster_centres);

        if (first_frame_)
        {
            boost::mutex::scoped_lock lock(mutex_);
            _KFTracker.initialize(cluster_centres);
            first_frame_ = false;
        }
        else
        {
            objIDs = _KFTracker.track(cluster_centres);
        }

        if (_config.rviz)
        {
            visualization_msgs::MarkerArray markers;
            fit_markers(cluster_centres, objIDs, markers);

            marker_pub_.publish(markers);
        }

        boost::mutex::scoped_lock lock(mutex_);
        sync_cluster_publishers_size(cluster_vec.size());
        publish_objects(cluster_centres, objIDs);

        for (size_t i = 0; i < objIDs.size(); ++i)
            if (objIDs[i] != -1)
                publish_cloud(*(cluster_pubs_[i]), cluster_vec[objIDs[i]]);
    }

    void ClusterTracker::extract_cluster_data(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                                              boost::container::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cluster_vec,
                                              boost::container::vector<pcl::PointXYZ> &cluster_centres)
    {
        pcl::search::KdTree<pcl::PointXYZ>::Ptr search_tree(new pcl::search::KdTree<pcl::PointXYZ>);
        search_tree->setInputCloud(input_cloud);

        std::vector<pcl::PointIndices> cluster_indices;

        cluster_extr_.setSearchMethod(search_tree);
        cluster_extr_.setInputCloud(input_cloud);
        cluster_extr_.extract(cluster_indices);

        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr _cluster(new pcl::PointCloud<pcl::PointXYZ>);
            float x = 0.0f, y = 0.0f, z = 0.0f;
            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
            {
                _cluster->push_back((*input_cloud)[*pit]);
                x += _cluster->back().x;
                y += _cluster->back().y;
                z += _cluster->back().z;
            }
            _cluster->width = _cluster->size();
            _cluster->height = 1;
            _cluster->is_dense = true;

            pcl::PointXYZ centre;
            centre.x = x / _cluster->size();
            centre.y = y / _cluster->size();
            centre.z = z == 0 ? 0 : (z / _cluster->size());

            cluster_vec.push_back(_cluster);
            cluster_centres.push_back(centre);
        }
    }
}