//
// Created by lupasic on 08.10.16.
//

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <cmath>
#include <std_msgs/String.h>
#include <fstream>


class Converting
{
protected:
    ros::NodeHandle nh;
    int fl;
    ros::Subscriber curPcldata;
    std::vector<sensor_msgs::PointCloud> allClouds;
    std::ofstream fout;
    void putInFile(const sensor_msgs::PointCloud::ConstPtr &msg);

public:
    Converting(std::string name);
    void pclCallback(const sensor_msgs::PointCloud::ConstPtr &msg);

};

