#include <ros/ros.h>
#include <nodelet/loader.h>
#include <string>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "cluster_tracker_node");
    ros::NodeHandle private_nh("~");
    int concurrency_level = private_nh.param("concurrency_level", 4);

    nodelet::Loader nodelet;
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;
    nodelet.load("laserscan_to_pointcloud_nodelet", "point_cloud/laserscan_to_pointcloud_nodelet", remap, nargv);
    nodelet.load("cluster_tracker_nodelet", "point_cloud/cluster_tracker_nodelet", remap, nargv);

    boost::shared_ptr<ros::MultiThreadedSpinner> spinner;
    if (concurrency_level)
    {
        spinner.reset(new ros::MultiThreadedSpinner(static_cast<uint32_t>(concurrency_level)));
    }
    else
    {
        spinner.reset(new ros::MultiThreadedSpinner());
    }
    spinner->spin();
    return 0;
}