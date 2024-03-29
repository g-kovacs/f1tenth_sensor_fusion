/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Eurotec, Netherlands
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

/*
 * Author: Rein Appeldoorn
 */

#ifndef F1TENTH_SENSOR_FUSION_LASERSCAN_TO_POINTCLOUD_NODELET_H
#define F1TENTH_SENSOR_FUSION_LASERSCAN_TO_POINTCLOUD_NODELET_H

#include <boost/thread/mutex.hpp>
#include <laser_geometry/laser_geometry.h>
#include <message_filters/subscriber.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

namespace f1tenth_sensor_fusion
{
  typedef tf2_ros::MessageFilter<sensor_msgs::LaserScan> MessageFilter;

  //! \brief The PointCloudToLaserScanNodelet class to process incoming laserscans into pointclouds.
  //!
  class LaserScanToPointCloudNodelet : public nodelet::Nodelet
  {
  public:
    LaserScanToPointCloudNodelet();

  private:
    virtual void onInit();

    void scanCallback(const sensor_msgs::LaserScanConstPtr &scan_msg);
    void failureCallback(const sensor_msgs::LaserScanConstPtr &scan_msg,
                         tf2_ros::filter_failure_reasons::FilterFailureReason reason);

    void connectCb();
    void disconnectCb();

    void info(int);

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher pub_;
    boost::mutex connect_mutex_;

    boost::shared_ptr<tf2_ros::Buffer> tf2_;
    boost::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
    message_filters::Subscriber<sensor_msgs::LaserScan> sub_;
    boost::shared_ptr<MessageFilter> message_filter_;

    laser_geometry::LaserProjection projector_;

    // ROS Parameters
    unsigned int input_queue_size_;
    std::string target_frame_;
    std::string subscription_topic_;
  };

} // namespace f1tenth_sensor_fusion

#endif // F1TENTH_SENSOR_FUSION__LASERSCAN_TO_POINTCLOUD_NODELET_H
