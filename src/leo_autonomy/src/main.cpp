#include "enviroment_manager.h"
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_spawner");
    ros::NodeHandle nh;
    EnviromentCreator env_creator(nh, 30);
    ros::spin();
    return 0;
}
