#pragma once
#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <geometry_msgs/Pose.h>

class ObstacleSpawner {
public:
    ObstacleSpawner(ros::NodeHandle& nh);

    void spawnBox(const geometry_msgs::Pose& pose);

private:
    ros::NodeHandle _nh;
    ros::ServiceClient _spawnClient;
    gazebo_msgs::SpawnModel _spawnModel;
    unsigned int _id = 0;  
};