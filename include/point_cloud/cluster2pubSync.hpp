#pragma once

#include <point_cloud/cluster_tracker.h>

namespace point_cloud
{
    class ClusterTracker::Cluster2PubSync
    {
    public:
        static void sync_clusters_to_publishers();
        //static int asd() { return 1000; }

    private:
        Cluster2PubSync() {}
    };
}