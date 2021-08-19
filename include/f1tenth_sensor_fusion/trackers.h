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

#ifndef F1TENTH_SENSOR_FUSION__TRACKERS_H
#define F1TENTH_SENSOR_FUSION__TRACKERS_H

#include <f1tenth_sensor_fusion/cluster_tracker.h>
#include <nodelet/nodelet.h>

namespace f1tenth_sensor_fusion
{

    class LidarTracker : protected ClusterTracker, public nodelet::Nodelet
    {
    public:
        LidarTracker();

    private:
        virtual void onInit();
    };

    class CameraTracker : protected ClusterTracker, public nodelet::Nodelet
    {
    public:
        CameraTracker();

    protected:
        virtual void onInit();
    };
}

#endif // F1TENTH_SENSOR_FUSION__TRACKERS_H