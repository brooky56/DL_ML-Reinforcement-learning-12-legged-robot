//
// Created by lupasic on 08.10.16.
//

#include "converting.h"

Converting::Converting(std::string name)
{
    fl=0;
    std::string subscribe_pcl_topic, output_file;
    nh.getParam("/pcl_convert/point_cloud_topic", subscribe_pcl_topic);
    nh.getParam("/pcl_convert/output_file", output_file);
    fout.open(output_file.c_str());

    curPcldata = nh.subscribe(subscribe_pcl_topic, 100 ,&Converting::pclCallback, this);
}

void Converting::pclCallback(const sensor_msgs::PointCloud::ConstPtr &msg)
{
    allClouds.push_back(*msg);
    putInFile(msg);
    // ros::shutdown();

}

void Converting::putInFile(const sensor_msgs::PointCloud::ConstPtr &msg)
{
    // auto ct = msg->channels.begin()->values.begin();

    // for(auto it = msg->points.begin(); it != msg->points.end(); ++it, ++ct)
        for(auto it = msg->points.begin(); it != msg->points.end(); ++it)
    {

        geometry_msgs::Point32 cur_coords = *it;
        // float cur_params = *ct;
        fout << cur_coords.x << " " << cur_coords.y << " " << cur_coords.z << " "<< std::endl;
        //std::cout << cur_coords.x << " " << cur_coords.y << " " << cur_coords.z << " " << cur_params << std::endl;
    }
    // fout.close();
}