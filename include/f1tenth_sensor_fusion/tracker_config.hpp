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

#ifndef F1TENTH_SENSOR_FUSION__TRACKER_CONFIG_HPP
#define F1TENTH_SENSOR_FUSION__TRACKER_CONFIG_HPP

#include <iostream>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

namespace f1tenth_sensor_fusion
{

    using namespace std;

    struct TrackerConfig
    {
        TrackerConfig() {}
        TrackerConfig(string tracker_name, int clust_min, int clust_max, double tolerance,
                      string scan_frame, string scan_topic, int marker, int m_type,
                      bool rviz = true, string target_frame = "") : tracker_name(tracker_name),
                                                                    scan_frame(scan_frame), scan_topic(scan_topic), target_frame(target_frame)
        {
            rviz = rviz;
            clust_max = clust_max;
            clust_min = clust_min;
            tolerance = tolerance;
            marker_size = marker;
            marker_type = m_type;
        }
        inline void info(const int concurrency)
        {
            stringstream ss;
            ss << tracker_name << " information:" << endl;
            ROS_INFO(ss.str().c_str());
            ss.str(string());
            ss << "\tconcurrency level:\t" << concurrency << endl;
            ss << "\tclust_min:\t" << clust_min << endl;
            ss << "\tclust_max:\t" << clust_max << endl;
            ss << "\ttolerance:\t" << tolerance << endl;
            ss << "\tmarker size (cm):\t" << marker_size << endl;
            ss << "\tscan frame:\t" << scan_frame << endl;
            ss << "\tscan topic:\t" << scan_topic << endl;
            ss << "\ttarget frame:\t" << target_frame << endl;
            ss << "\tvisualize:\t" << rviz << endl;
            cout << ss.str() << endl;
        }
        bool rviz;
        int clust_min, clust_max, marker_size;
        double tolerance;
        string scan_frame, target_frame, scan_topic, tracker_name;
        int marker_type;
    };
}

#endif // F1TENTH_SENSOR_FUSION__TRACKER_CONFIG_HPP