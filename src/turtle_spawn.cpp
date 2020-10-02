#include <ros/ros.h>
#include <turtlesim/Spawn.h>
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <math.h>
#include <learning_tf/turtle_spawn.h>

Spawn::Spawn()
{
  ros::service::waitForService("spawn");
  add_turtle = node.serviceClient<turtlesim::Spawn>("spawn");

  node.getParam("/turtle_limit", turtle_limit);
}

Spawn::~Spawn(){}

void Spawn::turtle_spawn()
{
  node.getParam("/turtles_spawned", turtles_spawned);

  if(turtles_spawned < 10)
  {
    std::srand(time(0));
    srv.request.x = float(std::rand())/float((RAND_MAX)) * MAX_COORD;
    srv.request.y = float(std::rand())/float((RAND_MAX)) * MAX_COORD;
    srv.request.theta = float(std::rand())/float((RAND_MAX)) * MAX_THETA;
    add_turtle.call(srv);

    turtle_name = srv.response.name;
    std::cout << "Turtle: "+turtle_name+" has spawn on x = " << srv.request.x <<" y = "<<srv.request.y<<" theta = "<<srv.request.theta << '\n';

    node.setParam("/turtles_spawned", ++turtles_spawned);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtle_spawn");
  Spawn spawn;
  ros::Rate rate(0.3);
  while(ros::ok())
  {
    spawn.turtle_spawn();
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
