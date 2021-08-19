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

#include <f1tenth_sensor_fusion/colormap.h>
#include <boost/thread.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pluginlib/class_list_macros.h>

namespace f1tenth_sensor_fusion
{
    Colormap::Colormap() {}
    void Colormap::onInit()
    {
        boost::mutex::scoped_lock lock(mutex_);
        private_nh_ = getPrivateNodeHandle();
        private_nh_.param<std::string>("colormap_subscription_topic", sub_topic_, "grayscale_image");
        int concurrency = private_nh_.param<int>("converter_concurrency_level", 0);

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

        pub_ = nh_.advertise<sensor_msgs::Image>("colormap_image", 30, boost::bind(&Colormap::connectCb, this),
                                                 boost::bind(&Colormap::disconnectCb, this));
        sub_.registerCallback(boost::bind(&Colormap::callback, this, _1));
    }

    void Colormap::connectCb()
    {
        boost::mutex::scoped_lock lock(mutex_);
        if (pub_.getNumSubscribers() > 0 && sub_.getSubscriber().getNumPublishers() == 0)
        {
            NODELET_INFO("Got a subscriber to colormap_image, starting %s subscriber", sub_topic_.c_str());
            sub_.subscribe(nh_, sub_topic_, input_queue_size_);
        }
    }

    void Colormap::disconnectCb()
    {
        boost::mutex::scoped_lock lock(mutex_);
        if (pub_.getNumSubscribers() == 0)
        {
            NODELET_INFO("No subscibers to colormap_image, shutting down subscriber to %s.", sub_topic_.c_str());
            sub_.unsubscribe();
        }
    }

    void Colormap::callback(const sensor_msgs::ImageConstPtr &msg)
    {
        cv_bridge::CvImage cvImage;
        cvImage.header = msg->header;
        cvImage.encoding = sensor_msgs::image_encodings::BGR8;
        cv::Mat img;
        cv_bridge::CvImageConstPtr input = cv_bridge::toCvShare(msg);
        cv::applyColorMap(input->image, img, cv::COLORMAP_JET);
        cvImage.image = img;
        pub_.publish(*cvImage.toImageMsg());
    }
}

PLUGINLIB_EXPORT_CLASS(f1tenth_sensor_fusion::Colormap, nodelet::Nodelet)