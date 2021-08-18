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

#include <f1tenth_sensor_fusion/KFTracker.hpp>

namespace f1tenth_sensor_fusion
{
    void KFTracker::_init_KFilters(size_t n)
    {
        int stateDim = 4; // [x, y, v_x, v_y]
        int measDim = 2;
        float dx = 1.0f, dy = dx, dvx = 0.01f, dvy = dvx;
        double sigmaP = 0.01, sigmaQ = 0.1;
        boost::mutex::scoped_lock lock(mutex_);
        for (size_t i = 0; i < n; i++)
        {
            auto _filt = std::make_unique<cv::KalmanFilter>(stateDim, measDim);
            _filt->transitionMatrix = (cv::Mat_<float>(4, 4) << dx, 0, 1, 0, 0, dy, 0, 1, 0, 0, dvx, 0, 0, 0, 0, dvy);
            cv::setIdentity(_filt->measurementMatrix);
            cv::setIdentity(_filt->processNoiseCov, cv::Scalar::all(sigmaP));
            cv::setIdentity(_filt->measurementNoiseCov, cv::Scalar(sigmaQ));
            k_filters_.push_back(std::move(_filt));
        }
    }

    std::pair<int, int> KFTracker::find_min_IDX(const float *mat, const size_t row, const size_t col)
    {
        std::pair<int, int> minIndex(-1, -1); // needed for preventing a cluster to be registered to multiple filters in case
                                              // there were more filters than detected clusters
        float minEl = std::numeric_limits<float>::max();
        for (int pp = 0; pp < row; pp++)  // for each predicted point pp
            for (int c = 0; c < col; c++) // for each centre c
            {
                if (mat[pp * col + c] < minEl)
                {
                    minEl = mat[pp * col + c];
                    minIndex = std::make_pair(pp, c);
                }
            }
        return minIndex;
    }

    void KFTracker::correct_kfilter_matrices(const vector<pcl::PointXYZ> &cCentres, const vector<int> &objID)
    {
        for (size_t i = 0; i < objID.size(); i++)
        {
            if (objID[i] == -1)
                continue;
            float meas[2] = {cCentres[objID[i]].x, cCentres[objID[i]].y};
            cv::Mat measMat = cv::Mat(2, 1, CV_32F, meas);
            if (!(meas[0] == 0.0f || meas[1] == 0.0f))
                k_filters_[i]->correct(measMat);
        }
    }

    vector<pcl::PointXYZ> KFTracker::generate_predictions()
    {
        vector<pcl::PointXYZ> pred;
        for (auto it = k_filters_.begin(); it != k_filters_.end(); it++)
        {
            auto p = (*it)->predict();
            pcl::PointXYZ pt;
            pt.x = p.at<float>(0);
            pt.y = p.at<float>(1);
            pt.z = p.at<float>(2);

            pred.push_back(pt);
        }
        return pred;
    }

    void KFTracker::create_kfilters_for_new_clusters(vector<int> &objID, const vector<pcl::PointXYZ> &centres, const bool *cluster_used)
    {
        size_t diff = centres.size() - objID.size();
        _init_KFilters(diff);
        objID.resize(k_filters_.size(), -1);
        int f = 0, c = 0;
        while (f != k_filters_.size() && c != centres.size())
        {
            if (objID[f] != -1) // tracked object
            {
                f++;
            }
            else if (cluster_used[c])
                c++;
            else
            {
                _set_kfilter_state_pre(k_filters_[f], centres[c]);
                f++;
                c++;
            }
        }
    }

    void KFTracker::prune_unused_kfilters(vector<int> &objID)
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

    vector<int> KFTracker::match_objID(const vector<pcl::PointXYZ> &pred, const vector<pcl::PointXYZ> &cCentres, bool *cluster_used)
    {
        vector<int> vec(pred.size(), -1); // Initializing object ID vector with negative ones

        // Generating distance matrix to make cross-compliance between centres and KF-s easier
        // rows: predicted points (indirectly the KFilter)
        // columns: the detected centres
        float *distMatrix = new float[pred.size() * cCentres.size()];

        for (size_t i = 0; i < pred.size(); ++i)
            for (size_t k = 0; k < cCentres.size(); ++k)
                distMatrix[i * cCentres.size() + k] = euclidian_dst(pred[i], cCentres[k]);

        // Matching objectID to KF
        for (size_t i = 0; i < k_filters_.size(); i++)
        {
            std::pair<int, int> minIdx(find_min_IDX(distMatrix, pred.size(), cCentres.size())); // find closest match
            if (minIdx.first != -1)                                                             // if a match was found, then
            {
                vec[minIdx.first] = minIdx.second;  // save this match
                cluster_used[minIdx.second] = true; // record that this cluster was matched

                for (size_t c = 0; c < cCentres.size(); ++c)
                    distMatrix[minIdx.first * cCentres.size() + c] = std::numeric_limits<float>::max(); // erase the row (filter)
                for (size_t r = 0; r < pred.size(); r++)                                                // erase the column (point cloud)
                    distMatrix[r * cCentres.size() + minIdx.second] = std::numeric_limits<float>::max();
            }
        }

        delete distMatrix;

        return vec;
    }

    void KFTracker::initialize(const vector<pcl::PointXYZ> &cCentres)
    {
        _init_KFilters(cCentres.size());
        for (size_t i = 0; i < cCentres.size(); i++)
        {
            _set_kfilter_state_pre(k_filters_[i], cCentres[i]);
        }
    }

    vector<int> KFTracker::track(const vector<pcl::PointXYZ> &cCentres)
    {
        vector<pcl::PointXYZ> predicted_pts = generate_predictions();

        bool cluster_used[cCentres.size()];
        for (auto b : cluster_used)
            b = false;

        vector<int> objID = match_objID(predicted_pts, cCentres, cluster_used);

        // if there are new clusters, initialize new kalman filters with data of unmatched clusters
        if (objID.size() < cCentres.size())
        {
            create_kfilters_for_new_clusters(objID, cCentres, cluster_used);
        }
        // if there are unused filters for some time, delete them
        else if (cCentres.size() < objID.size() && kf_prune_ctr_++ > prune_interval)
        {
            prune_unused_kfilters(objID);
        }

        if (k_filters_.size() > 0)
            correct_kfilter_matrices(cCentres, objID);

        return objID;
    }

}