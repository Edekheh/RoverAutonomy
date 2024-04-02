#include "obstacle_spawner.h"
#include <cstdlib>
#include <ctime>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

class EnviromentCreator
{
  public:
    EnviromentCreator(ros::NodeHandle &nh, unsigned int obstacle_count) : _nh(nh), _obstacle_spawner(nh)
    {
        wait_until_sim_time_starts();
        _unpause_gz_client = _nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
        init(obstacle_count);
        std_srvs::Empty srv;
        _unpause_gz_client.call(srv);
    }

  private:
    void wait_until_sim_time_starts()
    {
        bool ready = false;
        while (!ready)
        {
            ready = ros::service::exists("/gazebo/spawn_sdf_model", true) &&
                    ros::service::exists("/gazebo/unpause_physics", true);
        }
    }
    void init(unsigned int obstacle_count)
    {
        srand(time(nullptr));
        for (unsigned int i = 0; i < obstacle_count; i++)
        {
            geometry_msgs::Pose pose;
            float random_x = static_cast<float>(rand()) / static_cast<float>(RAND_MAX / 12) - 6;
            float random_y = static_cast<float>(rand()) / static_cast<float>(RAND_MAX / 12) - 6;
            if (random_x > -2 && random_x < 0 ) random_x -= 1;
            if (random_x > 0 && random_x < 2) random_x += 1;
            if (random_y > -2 && random_y < 0) random_y -= 1;
            if (random_y > 0 && random_y < 2) random_y += 1;
            pose.position.x = random_x;
            pose.position.y = random_y;
            _obstacle_spawner.spawnBox(pose);
        }
    }
    ros::NodeHandle _nh;
    ObstacleSpawner _obstacle_spawner;
    ros::ServiceClient _unpause_gz_client;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_spawner");
    ros::NodeHandle nh;
    EnviromentCreator env_creator(nh, 30);
    ros::spin();
    return 0;
}
