#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>

class Laser2PC
{
public:
    Laser2PC();
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan);

private:
    ros::NodeHandle node_;
    laser_geometry::LaserProjection projector_;
    tf::TransformListener tfListener_;
    ros::Publisher pcPub_;
    ros::Subscriber laserSub_;
};

Laser2PC::Laser2PC()
{
    laserSub_ = node_.subscribe<sensor_msgs::LaserScan>("/scan", 100, &Laser2PC::scanCallback, this);
    pcPub_ = node_.advertise<sensor_msgs::PointCloud2>("/cloud", 100, false);
}

void Laser2PC::scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    if (!tfListener_.waitForTransform(
            scan->header.frame_id,
            "/base_link",
            scan->header.stamp + ros::Duration().fromSec(scan->ranges.size() * scan->time_increment), ros::Duration(1.0)))
    {
        return;
    }

    sensor_msgs::PointCloud2 cloud;
    projector_.transformLaserScanToPointCloud("base_link", *scan, cloud, tfListener_);
    pcPub_.publish(cloud);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Laser2PC");
    Laser2PC converter;
    ros::spin();
    return 0;
}