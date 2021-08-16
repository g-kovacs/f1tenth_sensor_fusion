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
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
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
        int concurrency = private_nh_.param<int>("concurrency_level", 0);

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
        sub_.registerCallback(boost::bind(&PointCloudFilter::callback, this, _1));
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

        // Create the filtering object: downsample the dataset using a leaf size of 1cm
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        vg.setInputCloud(cloud);
        vg.setLeafSize(0.01f, 0.01f, 0.01f);
        vg.filter(*cloud_filtered);

        sensor_msgs::PointCloud2::Ptr clustermsg(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*cloud_filtered, *clustermsg);
        pub_.publish(clustermsg);
    }

} // namespace f1tenth_sensor_fusion
PLUGINLIB_EXPORT_CLASS(f1tenth_sensor_fusion::PointCloudFilter, nodelet::Nodelet)