#ifndef TURTLE_SPAWN
#define TURTLE_SPAWN

#include <ros/ros.h>
#include <turtlesim/Spawn.h>

#define MAX_COORD 11.088889122
#define MAX_THETA 2*M_PI


class Spawn
{
  ros::NodeHandle node;
  turtlesim::Spawn srv;
  ros::ServiceClient add_turtle;
  std::string turtle_name;
  int turtle_limit, turtles_spawned;

public:
  Spawn();

  ~Spawn();

  void turtle_spawn();

};
#endif
