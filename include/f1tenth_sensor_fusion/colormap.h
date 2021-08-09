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

#ifndef F1TENTH_SENSOR_FUSION__COLORMAP_H
#define F1TENTH_SENSOR_FUSION__COLORMAP_H

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc.hpp>
#include <boost/thread/mutex.hpp>
#include <message_filters/subscriber.h>

namespace f1tenth_sensor_fusion
{
    class Colormap : public nodelet::Nodelet
    {
    public:
        Colormap();

    private:
        virtual void onInit();
        void callback(const sensor_msgs::ImageConstPtr &msg);
        void connectCb();
        void disconnectCb();
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        boost::mutex mutex_;
        ros::Publisher pub_;
        message_filters::Subscriber<sensor_msgs::Image> sub_;
        std::string sub_topic_;
        unsigned int input_queue_size_;
    };
}

#endif // F1TENTH_SENSOR_FUSION__COLORMAP_H