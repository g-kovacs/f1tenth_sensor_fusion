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

#include <f1tenth_sensor_fusion/pointcloud_filter.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <boost/thread.hpp>
#include <pluginlib/class_list_macros.h>

namespace f1tenth_sensor_fusion
{
    PointCloudFilter::PointCloudFilter() {}

    void PointCloudFilter::onInit()
    {
        boost::mutex::scoped_lock lock(mutex_);
        private_nh_ = getPrivateNodeHandle();
        private_nh_.param<std::string>("subscription_topic", sub_topic_, "cloud");
        private_nh_.param<std::string>("target_frame", target_frame_, "fusion_base");
        int concurrency = private_nh_.param<int>("concurrency_level", 0);

#ifndef NDEBUG
        info(concurrency);
#endif

        // Check if explicitly single threaded, otherwise, let nodelet manager dictate thread pool size
        if (concurrency == 1)
        {
            nh_ = getNodeHandle();
        }
        else
        {
            nh_ = getMTNodeHandle();
        }

        // Only queue one pointcloud per running thread
        if (concurrency > 0)
        {
            input_queue_size_ = static_cast<size_t>(concurrency);
        }
        else
        {
            input_queue_size_ = boost::thread::hardware_concurrency();
        }

        pub_ = nh_.advertise<sensor_msgs::PointCloud2>("filtered_camera_cloud", 30, boost::bind(&PointCloudFilter::connectCb, this),
                                                       boost::bind(&PointCloudFilter::disconnectCb, this));
        if (target_frame_.empty())
            sub_.registerCallback(boost::bind(&PointCloudFilter::callback, this, _1));
        else
        {
            tf2_.reset(new tf2_ros::Buffer());
            tf2_listener_.reset(new tf2_ros::TransformListener(*tf2_));
            message_filter_.reset(new PointCloud2MessageFilter(sub_, *tf2_, target_frame_, input_queue_size_, nh_));
            message_filter_->registerCallback(boost::bind(&PointCloudFilter::callback, this, _1));
            message_filter_->registerFailureCallback(boost::bind(&PointCloudFilter::failureCallback, this, _1, _2));
        }
    }

    void PointCloudFilter::connectCb()
    {
        boost::mutex::scoped_lock lock(mutex_);
        if (pub_.getNumSubscribers() > 0 && sub_.getSubscriber().getNumPublishers() == 0)
        {
            NODELET_INFO("Got a subscriber to filtered_camera_cloud, starting %s subscriber", sub_topic_.c_str());
            sub_.subscribe(nh_, sub_topic_, input_queue_size_);
        }
    }

    void PointCloudFilter::disconnectCb()
    {
        boost::mutex::scoped_lock lock(mutex_);
        if (pub_.getNumSubscribers() == 0)
        {
            NODELET_INFO("No subscibers to filtered_camera_cloud, shutting down subscriber to %s.", sub_topic_.c_str());
            sub_.unsubscribe();
        }
    }

    void PointCloudFilter::callback(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        filter(cloud);

        sensor_msgs::PointCloud2::Ptr clustermsg(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*cloud, *clustermsg);
        clustermsg->header = msg->header;
        clustermsg->header.stamp = ros::Time::now();

        if (!target_frame_.empty() && cloud->header.frame_id != target_frame_)
        {
            try
            {
                *clustermsg = tf2_->transform(*clustermsg, target_frame_);
            }
            catch (tf2::TransformException &ex)
            {
                NODELET_ERROR_STREAM("Transform failure: " << ex.what());
                return;
            }
        }
        pub_.publish(clustermsg);
    }

    void PointCloudFilter::filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
    {
        // Create the filtering object: downsample the dataset using a leaf size of 1cm
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        vg.setInputCloud(cloud);
        vg.setLeafSize(0.01f, 0.01f, 0.01f);
        vg.filter(*cloud_filtered);

        // Create the segmentation object for the planar model and set all the parameters
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>()), cloud_f(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PCDWriter writer;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(100);
        seg.setDistanceThreshold(0.02);

        int i = 0, nr_points = (int)cloud_filtered->size();
        while (cloud_filtered->size() > 0.6 * nr_points)
        {
            // Segment the largest planar component from the remaining cloud
            seg.setInputCloud(cloud_filtered);
            seg.segment(*inliers, *coefficients);
            if (inliers->indices.size() == 0)
            {
                NODELET_ERROR_STREAM("Could not estimate a planar model for the given dataset.");
                break;
            }

            // Extract the planar inliers from the input cloud
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(cloud_filtered);
            extract.setIndices(inliers);

            // Remove the planar inliers, extract the rest
            extract.setNegative(true);
            extract.filter(*cloud_f);
            *cloud_filtered = *cloud_f;
        }

        *cloud = *cloud_filtered;
    }

    void PointCloudFilter::failureCallback(const sensor_msgs::PointCloud2ConstPtr &scan_msg,
                                           tf2_ros::filter_failure_reasons::FilterFailureReason reason)
    {
        NODELET_WARN_STREAM_THROTTLE(1.0, "Can't transform laserscan from frame " << scan_msg->header.frame_id << " to "
                                                                                  << message_filter_->getTargetFramesString()
                                                                                  << " at time " << scan_msg->header.stamp
                                                                                  << ", reason: " << reason);
    }

    void PointCloudFilter::info(int concurrency)
    {
        std::stringstream ss;
        ss << "Point cloud filter info:" << std::endl;
        NODELET_INFO(ss.str().c_str());
        ss.str(std::string());
        ss << "\tconcurrency level:\t" << concurrency << std::endl;
        ss << "\tsubscription topic:\t" << sub_topic_ << std::endl;
        std::cout << ss.str() << std::endl;
    }

} // namespace f1tenth_sensor_fusion
PLUGINLIB_EXPORT_CLASS(f1tenth_sensor_fusion::PointCloudFilter, nodelet::Nodelet)