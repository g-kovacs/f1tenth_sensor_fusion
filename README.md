# f1tenth_sensor_fusion

## Overview

This is a ROS package developed in hope of achieving sensor fusion using the LiDAR and camera sensors of an [F1TENTH][f1tenth] autonomous vehicle during my internship at [MTA SZTAKI][sztaki].

The project in its current state provides configurable classes to process, clusterize and dinamically track detected clusters within incoming point clouds. Data about the tracked clusters are continuously brodcast on their specific ROS topics to make further processing and computations possible, as well as to ease visual representation using RViz.

![Detecting clusters of LiDAR scan](/assets/images/lidar.png "Lidar clusterization")
![Detecting clusters of camera point cloud](/assets/images/camera.png "Camera clusterization")

The package was developed and tested under **ROS Noetic** running on an **Ubuntu 20.04** system. The project -- as is -- is designed specifically for the hardware on site, that is vehicles equipped with a [*Hokuyo UST-10LX*][hokuyo] 2D LiDAR and a [*MYNT EYE S1040*][mynteye] mono camera with IR as the two sources of input. Therefore, any fitness for other particular use cases is disclaimed.

**Author:** Gergely Attila Kovács, gkovacs1@edu.bme.hu

### Citing

The ROS nodelet used for the conversion of LiDAR scans was taken from the ROS package [**pointcloud_to_laserscan**][laser].
The basic idea of the algorithms used were published in [**Multiple-Object-Tracking-from-Point-Clouds_v1.0.2**][track].

If you plan on using this project for your own research, please add the following citation:

Gergely Attila Kovács, . "F1TENTH sensor data clustering in ROS." https://github.com/kovika98/f1tenth_sensor_fusion. (2021). 

    @misc{f1tenth_fusion,
      author = {Gergely Attila Kovács},
      title = {{F1TENTH} sensor data clustering in {ROS}},
      howpublished = {\url{https://github.com/kovika98/f1tenth_sensor_fusion}},
      year = {2021},
    }
    
## Installation

### Dependencies

In addtion to the [Robot Operating System][ros], the project depends on:
 - [**OpenCV 3.4.5**][opencv]  
 - [**boost**][boost]

Newer OpenCV versions were not tested and are not guranteed to work. Addittional ROS packages that are needed can be found in the *package.xml* file.

### Building

First, clone this repository into your workspace.

    cd your_catkin_workspace/src
    git clone git@github.com:kovika98/f1tenth_sensor_fusion.git
    # alternatively you can clone it via HTTPS
    # git clone https://github.com/kovika98/f1tenth_sensor_fusion.git
    cd ..
    
Building in *Debug* mode enables some logging for making it easier to check the parameters you have set. Building in *Release* mode is advised, since it results in best performance.

    # choose either one
    catkin_make -DCMAKE_BUILD_TYPE=Debug
    catkin_make -DCMAKE_BUILD_TYPE=Release

### Pre-recorded data

The repository contains some pre-recorded sensor data I had to use. These *.bag* files can be found compressed in the */rosbag* directory.
To use them, you must decompress the files using *rosbag*. You may want to change the output directory, since these files take up a lot of space.

    cd rosbag
    rosbag decompress *.bag
    cd ..
    
The file */launch/fusion.launch* initiates a node which plays back one selected rosbag file. To select the file to be loaded, change the path in the *args* parameter in line:

    # /launch/fusion.launch
    ...
        <node pkg="rosbag" type="play" name="player" output="log" args="--clock -l /data/f1tenth/fusion_data/take2.bag"/>
    </launch>

## Usage

The easy way is to use the launch files provided (you may modify them to your needs). The behaviour of the nodes can be manipulated by changing parameters in the parameter files specific to each nodelet.

### Launch files

Files *display_fusion.launch* and *fusion.launch* set up static tf transorms (coordinates measured for our setup) to transform the two frames of the sensors into a single base, and also starts the playback of pre-recorded data.

The nodelet manager in *tracker.launch* initiates the nodelets needed for preprocessing the sensor data and the tracking nodelets themselves.

Upon building, source the files and use *roslaunch* to start a launch file.

    # build project
    cd your_catkin_workspace
    catkin_make
    # source & run
    . devel/setup.bash
    roslaunch f1tenth_sensor_fusion <launch-file>

### Parameters

Quick changes to nodelets without needing to recompile the sources can be made by changing parameters in the respective parameter files. Some of these have default values, check the *.yaml* files or the source code to learn about them.

#### Common parameters

-**`concurrency_level`**: sets the number of threads the manager should give the nodelet  
-**`subscription_topic`**: the ROS topic providing the input  
-**`target_frame`**: the ROS frame of the output data  

## Nodelets & classes

### Nodelets

The package contains nodelets that are managed by a ROS nodelet manager.

#### laserscan_to_pointcloud_nodelet

This nodelet is responsible for converting the LiDAR scan data into *sensor_msgs::PointCLoud2* messages.

##### Related parameters & topics:

-**`${subscription_topic}`**: input topic of *sensor_msgs::LaserScan* messages  
-**`lidar_cloud`**: output topic to publish scan data as *sensor_msgs::PointCloud2* messages  

#### pointcloud_filter_nodelet

This nodelet is responsible for filtering raw point cloud data (currently used for preprocessing the point cloud published by the camera).

##### Related parameters & topics:

-**`${subscription_topic}`**: input topic of *sensor_msgs::PointCloud2* messages  
-**`${output_topic}`**: output topic to publish filtered cloud as *sensor_msgs::PointCloud2* messages  
-**`${segmentation}`** [boolean]: whether planar segments should be removed from the point cloud  
-**`${segmentation_factor}`**: what ratio of the cloud density before segmentetion should be preserved  

#### tracker nodelets

These nodelets are responsible for tracking clusters found in the input point cloud. If enabled, they produce cube visualization markers for each detected cluster.

##### Related parameters & topics:

-**`${subscription_topic}`**: input topic of *sensor_msgs::PointCloud2* messages  
-**`${subscription_frame}`**: frame of the incoming messages. Must be specified.  
-**`${target_frame}`**: frame of published clouds. Defaults to **`${subscription_frame}`** if not specified.  
-**`${max_cluster_size}`** and **`${min_cluster_size}`**: size boundaries of the clusters to be taken into consideration  
-**`${tolerance}`** [m]: determines the maximum tolerable distance between points  
-**`${visualize_rviz}`**: enable/disable marker generation for RViz visualization  
-**`${marker_size}`** [cm]: size of generated markers 

### ClusterTracker

Tracker nodelets inherit from this class. According to the parameters described above, it detects clusters, tracks their movement and publishes cluster and object data to specific ROS topics.

#### Output topics

-**`<tracker_name>/viz`**: *visualization_msgs::MarkerArray* message with markers for RViz 
-**`<tracker_name>/detections`**: custom *ObjectMessage* stream containing detected cluster/object IDs with their coordinates (centre of cluster)  
-**`<tracker_name>/cluster_<n>`**: *sensor_msgs::PointCloud2* message containing the data of the *n*-th cluster 

[//]: #
[f1tenth]: <https://f1tenth.org/index.html>
[sztaki]: <https://www.sztaki.hu/en>
[hokuyo]: <https://hokuyo-usa.com/products/lidar-obstacle-detection/ust-10lx>
[mynteye]: <https://www.mynteye.com/products/mynt-eye-stereo-camera>
[laser]: <https://github.com/ros-perception/pointcloud_to_laserscan/tree/lunar-devel>
[track]: <https://doi.org/10.5281/zenodo.3559186>
[ros]: <https://www.ros.org/>
[rosinstall]: <http://wiki.ros.org/ROS/Installation>
[opencv]: <https://github.com/opencv/opencv/releases/tag/3.4.5>
[boost]: <https://www.boost.org/>