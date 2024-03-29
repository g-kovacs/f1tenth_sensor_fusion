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

#ifndef F1TENTH_SENSOR_FUSION__KFTRACKER_HPP
#define F1TENTH_SENSOR_FUSION__KFTRACKER_HPP

#include <opencv2/video/tracking.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/container/vector.hpp>
#include <pcl/point_types.h>
#include <memory>

namespace f1tenth_sensor_fusion
{
    typedef boost::container::vector<pcl::PointXYZ> PointVector;

#define prune_interval 50

    class KFTracker
    {
    public:
        boost::container::vector<int> track(const PointVector &cCentres);
        void initialize(const PointVector &cCentres);

    private:
        boost::mutex mutex_;
        boost::container::vector<std::unique_ptr<cv::KalmanFilter>> k_filters_;
        size_t kf_prune_ctr_ = 0;

        inline float euclidian_dst(const pcl::PointXYZ &p, const pcl::PointXYZ &q)
        {
            return sqrt((p.x - q.x) * (p.x - q.x) + (p.y - q.y) * (p.y - q.y + (p.z - q.z) * (p.z - q.z)));
        }

        inline void _set_kfilter_state_pre(std::unique_ptr<cv::KalmanFilter> &filter, const pcl::PointXYZ &pt)
        {
            filter->statePre.at<float>(0) = pt.x;
            filter->statePre.at<float>(1) = pt.y;
            filter->statePre.at<float>(2) = 0;
            filter->statePre.at<float>(3) = 0;
        }

        void _init_KFilters(size_t n);
        PointVector generate_predictions();
        boost::container::vector<int> match_objID(const PointVector &pred, const PointVector &cCentres, bool *cluster_used);
        void create_kfilters_for_new_clusters(boost::container::vector<int> &objID, const PointVector &centres, const bool *cluster_used);
        void prune_unused_kfilters(boost::container::vector<int> &objID);
        void correct_kfilter_matrices(const PointVector &cCentres, const boost::container::vector<int> &objID);

        /**
         * Return the indices of the smallest element of a 2D matrix. Each row represents a KF predition, each column a detected cluster centroid, their 
         * intersections the distance between the two.
         * 
         * @param mat the distance matrix
         * @return the indicies of the minimum element. <-1, -1> if no minimum was found (i.e. all elements equal)
         */
        std::pair<int, int> find_min_IDX(const float *mat, const size_t row, const size_t col);
    };
}

#endif // F1TENTH_SENSOR_FUSION__KFTRACKER_HPP