#pragma once
#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/TypeDefs.hpp"
#include "obstacle_spawner.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include <cstdlib>
#include <ctime>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <grid_map_core/iterators/CircleIterator.hpp>
#include <grid_map_core/iterators/LineIterator.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

class EnviromentCreator
{
  public:
    EnviromentCreator(ros::NodeHandle &nh, unsigned int obstacle_count);

  private:
    void wait_until_sim_time_starts();
    void init(unsigned int obstacle_count);
    void create_grid_map();
    void generate_line_obstacles();
    void generate_circle_obstacles(grid_map::Position position);
    void update_leo_pos();
    double _map_resolution = 0.1;
    double _map_width = 15.0;
    ros::NodeHandle _nh;
    ObstacleSpawner _obstacle_spawner;
    ros::ServiceClient _unpause_gz_client;
    gazebo_msgs::ModelStates _model_states;
    ros::Publisher _grid_map_pub;
    grid_map::GridMap _grid_map;
    ros::Timer _timer;
    ros::Subscriber _model_states_sub;
};
