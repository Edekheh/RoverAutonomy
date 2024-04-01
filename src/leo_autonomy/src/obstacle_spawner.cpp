#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <geometry_msgs/Pose.h>
#include "obstacle_spawner.h"

ObstacleSpawner::ObstacleSpawner(ros::NodeHandle& nh)
    : _nh(nh)
{
    _spawnClient = _nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
    _spawnModel.request.model_name = "box" + std::to_string(_id);
    _spawnModel.request.model_xml = "<sdf version='1.4'><model name='box'><pose>0 0 0.5 0 0 0</pose><link name='link'><collision name='collision'><geometry><box><size>1 1 1</size></box></geometry></collision><visual name='visual'><geometry><box><size>1 1 1</size></box></geometry></visual></link></model></sdf>";
    _id++;

}

void ObstacleSpawner::spawnBox(const geometry_msgs::Pose& pose)
{
    _spawnModel.request.initial_pose = pose;
    _spawnClient.call(_spawnModel);
    ROS_WARN_STREAM("Spawned with result: " << _spawnModel.response.status_message);
}