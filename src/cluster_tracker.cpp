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
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int32MultiArray.h>
#include <boost/thread.hpp>
#include <random>
#include <opencv2/video/tracking.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace f1tenth_sensor_fusion
{
    ClusterTracker::ClusterTracker() {}

    void ClusterTracker::onInit()
    {
        boost::mutex::scoped_lock lock(mutex_);
        srand(time(NULL));
        private_nh_ = getPrivateNodeHandle();

        // Load parameters

        int concurrency_level = _load_params();

#ifndef NDEBUG
        _config.info(concurrency_level);
#endif

        cluster_extr_.setClusterTolerance(_config.tolerance);
        cluster_extr_.setMaxClusterSize(_config.clust_max);
        cluster_extr_.setMinClusterSize(_config.clust_min);

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
            input_queue_size_ = static_cast<size_t>(concurrency_level);
        }
        else
        {
            input_queue_size_ = boost::thread::hardware_concurrency();
        }

        transform_ = _config.scan_frame.compare(_config.target_frame) == 0 ? false : true;

        // Subscribe to topic with input data
        sub_.subscribe(nh_, _config.scan_topic, input_queue_size_);
        sub_.registerCallback(boost::bind(&ClusterTracker::cloudCallback, this, _1));
        // Init marker publisher if necessary
        if (_config.rviz)
            marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(_config.tracker_name + std::string("/viz"), 100);
        objID_pub_ = nh_.advertise<std_msgs::Int32MultiArray>(_config.tracker_name + std::string("obj_id"), 100);
        NODELET_INFO("Tracker nodelet initialized...");
    }

    int ClusterTracker::_load_params()
    {
        auto prefix = std::string("tracker/").append(_config.tracker_name);
        ROS_INFO(prefix.c_str());
        private_nh_.param<bool>(prefix + std::string("/visualize_rviz"), _config.rviz, _config.rviz);
        private_nh_.param<std::string>(prefix + std::string("/scan_frame"), _config.scan_frame, _config.scan_frame);
        private_nh_.param<std::string>(prefix + std::string("/target_frame"), _config.target_frame, _config.scan_frame.c_str());
        private_nh_.param<std::string>(prefix + std::string("/scan_topic"), _config.scan_topic, _config.scan_topic);
        _config.tolerance = private_nh_.param<double>(prefix + std::string("/tolerance"), 0.2);
        _config.clust_max = private_nh_.param<int>(prefix + std::string("/max_cluster_size"), 100);
        _config.clust_min = private_nh_.param<int>(prefix + std::string("/min_cluster_size"), 40);
        return private_nh_.param(prefix + std::string("/concurrency_level"), 0);
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
            auto _filt = std::make_unique<cv::KalmanFilter>(stateDim, measDim);
            _filt->transitionMatrix = (cv::Mat_<float>(4, 4) << dx, 0, 1, 0, 0, dy, 0, 1, 0, 0, dvx, 0, 0, 0, 0, dvy);
            cv::setIdentity(_filt->measurementMatrix);
            cv::setIdentity(_filt->processNoiseCov, cv::Scalar::all(sigmaP));
            cv::setIdentity(_filt->measurementNoiseCov, cv::Scalar(sigmaQ));
            k_filters_.push_back(std::move(_filt));
        }
    }

    template <class C>
    void ClusterTracker::_set_kfilter_state_pre(const C &pt, std::unique_ptr<cv::KalmanFilter> &filter)
    {
        filter->statePre.at<float>(0) = pt.x;
        filter->statePre.at<float>(1) = pt.y;
        filter->statePre.at<float>(2) = 0;
        filter->statePre.at<float>(3) = 0;
    }

    float ClusterTracker::euclidian_dst(geometry_msgs::Point &p1, geometry_msgs::Point &p2)
    {
        return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
    }

    std::pair<int, int> ClusterTracker::find_min_IDX(std::vector<std::vector<float>> &distMat)
    {
        std::pair<int, int> minIndex(-1, -1); // needed for preventing a cluster to be registered to multiple filters in case
                                              // there were more filters than detected clusters
        float minEl = std::numeric_limits<float>::max();
        for (int pp = 0; pp < distMat.size(); pp++)        // for each predicted point pp
            for (int c = 0; c < distMat.at(0).size(); c++) // for each centre c
            {
                if (distMat[pp][c] < minEl)
                {
                    minEl = distMat[pp][c];
                    minIndex = std::make_pair(pp, c);
                }
            }
        return minIndex;
    }

    void ClusterTracker::publish_cloud(ros::Publisher &pub, pcl::PointCloud<pcl::PointXYZ>::Ptr &cluster)
    {
        sensor_msgs::PointCloud2::Ptr clustermsg(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*cluster, *clustermsg);
        clustermsg->header.frame_id = _config.target_frame;
        clustermsg->header.stamp = ros::Time::now();
        pub.publish(*clustermsg);
    }

    void ClusterTracker::publish_objects()
    {
        std_msgs::Int32MultiArray obj_msg;
        for (auto it = objID.begin(); it != objID.end(); ++it)
            obj_msg.data.push_back(*it);
        objID_pub_.publish(obj_msg);
    }

    void ClusterTracker::KFTrack(const std_msgs::Float32MultiArray &ccs)
    {
        std::vector<geometry_msgs::Point> predicted_points = generate_predictions();

        // Convert multiarrray back to point data (regarding detected cluster centres)

        std::vector<geometry_msgs::Point> cCentres;
        for (auto it = ccs.data.begin(); it != ccs.data.end(); it += 3)
        {
            geometry_msgs::Point pt;
            pt.x = *it;
            pt.y = *(it + 1);
            pt.z = *(it + 2);
            cCentres.push_back(pt);
        }

        bool cluster_used[cCentres.size()];
        for (auto b : cluster_used)
            b = false;

        boost::mutex::scoped_lock obj_lock(mutex_);
        // Match predictions to clusters
        objID = match_objID(predicted_points, cCentres, cluster_used);

        boost::unique_lock<boost::recursive_mutex> flock(filter_mutex_);
        // if there are new clusters, initialize new kalman filters with data of unmatched clusters
        if (objID.size() < cCentres.size())
        {
            create_kfilters_for_new_clusters(cCentres, cluster_used);
        }
        // if there are unused filters for some time, delete them
        else if (cCentres.size() < objID.size() && kf_prune_ctr_++ > prune_interval)
        {
            prune_unused_kfilters();
        }

        if (_config.rviz)
        {
            visualization_msgs::MarkerArray markers;
            fit_markers(cCentres, objID, markers);

            marker_pub_.publish(markers);
        }

        publish_objects();

        if (k_filters_.size() > 0)
            correct_kfilter_matrices(cCentres);
    }

    void ClusterTracker::correct_kfilter_matrices(const std::vector<geometry_msgs::Point> &cCentres)
    {
        for (size_t i = 0; i < objID.size(); i++)
        {
            if (objID[i] == -1)
                continue;
            float meas[2] = {(float)cCentres[objID[i]].x, (float)cCentres[objID[i]].y};
            cv::Mat measMat = cv::Mat(2, 1, CV_32F, meas);
            if (!(meas[0] == 0.0f || meas[1] == 0.0f))
                k_filters_[i]->correct(measMat);
        }
    }

    std::vector<geometry_msgs::Point> ClusterTracker::generate_predictions()
    {
        std::vector<geometry_msgs::Point> pred;
        for (auto it = k_filters_.begin(); it != k_filters_.end(); it++)
        {
            auto p = (*it)->predict();
            geometry_msgs::Point pt;
            pt.x = p.at<float>(0);
            pt.y = p.at<float>(1);
            pt.z = p.at<float>(2);

            pred.push_back(pt);
        }
        return pred;
    }

    void ClusterTracker::create_kfilters_for_new_clusters(const std::vector<geometry_msgs::Point> &centres, const bool *cluster_used)
    {
        size_t diff = centres.size() - objID.size();
        _init_KFilters(diff);
        objID.resize(k_filters_.size(), -1);
        int f = 0, c = 0;
        while (f != k_filters_.size() && c != centres.size())
        {
            if (objID[f] != -1)
            {
                f++;
            }
            else if (cluster_used[c])
                c++;
            else
            {
                _set_kfilter_state_pre(centres[c], k_filters_[f]);
                f++;
                c++;
            }
        }
    }

    void ClusterTracker::prune_unused_kfilters()
    {
        size_t deleted = 0, i = 0;
        for (auto it = objID.begin(); it != objID.end(); it++)
        {
            if (*it == -1) // no matching cluster for this filter
            {
                objID.erase(it--); // remove '-1' from objID --> thus sizes of filters and objects remain the same
                k_filters_.erase(k_filters_.begin() + i - deleted++);
            }
            i++;
        }
        kf_prune_ctr_ = 0;
    }

    std::vector<int> ClusterTracker::match_objID(const std::vector<geometry_msgs::Point> &pred, const std::vector<geometry_msgs::Point> &cCentres, bool *cluster_used)
    {
        std::vector<int> vec(pred.size(), -1); // Initializing object ID vector with negative ones

        // Generating distance matrix to make cross-compliance between centres and KF-s easier
        // rows: predicted points (indirectly the KFilter)
        // columns: the detected centres

        std::vector<std::vector<float>> distMatrix;

        for (auto pp : pred)
        {
            std::vector<float> distVec;
            for (auto centre : cCentres)
                distVec.push_back(euclidian_dst(pp, centre));
            distMatrix.push_back(distVec);
        }

        // Matching objectID to KF
        for (size_t i = 0; i < k_filters_.size(); i++)
        {
            std::pair<int, int> minIdx(find_min_IDX(distMatrix)); // find closest match
            if (minIdx.first != -1)                               // if a match was found, then
            {
                vec[minIdx.first] = minIdx.second;  // save this match
                cluster_used[minIdx.second] = true; // record that this cluster was matched

                distMatrix[minIdx.first] = std::vector<float>(cCentres.size(), std::numeric_limits<float>::max()); // erase the row (filter)
                for (size_t r = 0; r < distMatrix.size(); r++)                                                     // erase the column (point cloud)
                    distMatrix[r][minIdx.second] = std::numeric_limits<float>::max();
            }
        }

        return vec;
    }

    void ClusterTracker::fit_markers(const std::vector<geometry_msgs::Point> &pts, const std::vector<int> &IDs, visualization_msgs::MarkerArray &markers)
    {
        for (auto i = 0; i < IDs.size(); i++)
        {
            if (IDs[i] == -1)
                continue;

            visualization_msgs::Marker m;
            m.id = i;
            m.header.frame_id = _config.target_frame;
            m.type = visualization_msgs::Marker::CUBE;
            m.scale.x = 0.08;
            m.scale.y = 0.08;
            m.scale.z = 0.08;
            m.action = visualization_msgs::Marker::ADD;
            m.color.a = 1.0;
            m.color.r = i % 2 ? 1 : 0;
            m.color.g = i % 3 ? 1 : 0;
            m.color.b = i % 4 ? 1 : 0;
            m.lifetime = ros::Duration(0.1);

            geometry_msgs::Point clusterC(pts[IDs[i]]);

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

    void ClusterTracker::transform_centre(geometry_msgs::Point &centres)
    {
        tf2_ros::Buffer buf_;
        tf2_ros::TransformListener tfListener(buf_);

        geometry_msgs::Point trans_cluster;

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
        boost::mutex::scoped_lock lock(mutex_);

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
                ros::Publisher *pub = new ros::Publisher(nh_.advertise<sensor_msgs::PointCloud2>(ss.str(), 100));
                cluster_pubs_.push_back(pub);
            }
            catch (ros::Exception &ex)
            {
                NODELET_ERROR(ex.what());
            }
        }
    }

    void ClusterTracker::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *input_cloud);

        pcl::search::KdTree<pcl::PointXYZ>::Ptr search_tree(new pcl::search::KdTree<pcl::PointXYZ>);
        search_tree->setInputCloud(input_cloud);

        std::vector<pcl::PointIndices> cluster_indices;

        cluster_extr_.setSearchMethod(search_tree);
        cluster_extr_.setInputCloud(input_cloud);
        cluster_extr_.extract(cluster_indices);

        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_vec;
        std::vector<pcl::PointXYZ> cluster_centres;

        extract_cluster_data(input_cloud, cluster_indices, cluster_vec, cluster_centres);

        sync_cluster_publishers_size(cluster_vec.size());

        boost::unique_lock<boost::recursive_mutex> flock(filter_mutex_);
        if (first_frame_)
        {
            _init_KFilters(cluster_centres.size());
            for (size_t i = 0; i < cluster_centres.size(); i++)
            {
                _set_kfilter_state_pre(cluster_centres[i], k_filters_[i]);
            }
            first_frame_ = false;
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
        }

        boost::mutex::scoped_lock lock(mutex_);
        for (size_t i = 0; i < objID.size(); ++i)
            if (objID[i] != -1)
                publish_cloud(*(cluster_pubs_[i]), cluster_vec[objID[i]]);
    }

    void ClusterTracker::extract_cluster_data(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, const std::vector<pcl::PointIndices> &cluster_indices, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cluster_vec, std::vector<pcl::PointXYZ> &cluster_centres)
    {
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