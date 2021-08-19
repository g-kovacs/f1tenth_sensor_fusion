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

#ifndef F1TENTH_SENSOR_FUSION__POINTCLOUD_FILTER_H
#define F1TENTH_SENSOR_FUSION__POINTCLOUD_FILTER_H

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <boost/thread/mutex.hpp>
#include <message_filters/subscriber.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

namespace f1tenth_sensor_fusion
{
    typedef tf2_ros::MessageFilter<sensor_msgs::PointCloud2> PointCloud2MessageFilter;

    class PointCloudFilter : public nodelet::Nodelet
    {
    public:
        PointCloudFilter();

    private:
        virtual void onInit();
        void callback(const sensor_msgs::PointCloud2ConstPtr &msg);
        void failureCallback(const sensor_msgs::PointCloud2ConstPtr &,
                             tf2_ros::filter_failure_reasons::FilterFailureReason);
        void connectCb();
        void disconnectCb();
        void filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
        void planar_segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
        void info(int);
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        boost::mutex mutex_;
        boost::shared_ptr<tf2_ros::Buffer> tf2_;
        boost::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
        boost::shared_ptr<PointCloud2MessageFilter> message_filter_;
        ros::Publisher pub_;
        message_filters::Subscriber<sensor_msgs::PointCloud2> sub_;
        std::string sub_topic_;
        std::string out_topic_;
        std::string target_frame_;
        unsigned int input_queue_size_;
        bool segmentation;
        float segmentation_factor;
    };
}

#endif // F1TENTH_SENSOR_FUSION__POINTCLOUD_FILTER_H