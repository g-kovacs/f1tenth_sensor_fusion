#include <ros/ros.h>
#include <nodelet/loader.h>
#include <string>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "cam_debug");
    ros::NodeHandle private_nh("~");

    nodelet::Loader nodelet;
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;
    nodelet.load("cam_debug", "f1tenth_sensor_fusion/cam_debug_nodelet", remap, nargv);

    boost::shared_ptr<ros::MultiThreadedSpinner> spinner;
    spinner.reset(new ros::MultiThreadedSpinner());
    spinner->spin();
    return 0;
}
