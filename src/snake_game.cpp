#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/Kill.h>
#include <std_srvs/Empty.h>

#include <iostream>
#include <cstdlib>
#include <ctime>
#include <math.h>
#include <string>
#include <ros/package.h>

class Listener
{
  ros::NodeHandle node;
  bool follow;

  turtlesim::Spawn srv;
  std_srvs::Empty reset_srv, clear_srv;
  turtlesim::Kill kill_srv;
  ros::Publisher turtle_vel;
  tf::TransformListener listener;

  ros::ServiceClient add_turtle, reset_service, clear_service, kill_service;
  std::string turtle_name, last_turtle_, next_last_turtle;

  int added_turtles, level_turtles, turtle_limit, turtles_spawned;

  ros::Time start_time_to_follow;


public:
  Listener():
    follow(false)
    {
      node.getParam("/"+ros::this_node::getName()+"/turtle_name", turtle_name);
      node.getParam("/turtle_limit", turtle_limit);

      turtle_vel = node.advertise<geometry_msgs::Twist>("velocity_topic", 10);
    }

    ~Listener(){
      /*defaultParameters();
      std::string cmd = "bash -c \"rosparam dump -v "+ros::package::getPath("turtlesim_snake")+"/turtle.yaml\"";
      system(cmd.c_str());*/
    }

    void turtleTransform(const std::string &target_frame, const std::string &source_frame, tf::StampedTransform &tf)
    {
      try{
        ros::Time now = ros::Time::now();
        ros::Time past = now - ros::Duration(0.001);
        listener.waitForTransform(target_frame, now, source_frame, past, "/world", ros::Duration(3.0));
        listener.lookupTransform(target_frame, now, source_frame, past, "/world", tf);
      }
      catch (tf::TransformException &ex) {
        //ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
      }
    }

    float turtleDistance(const tf::StampedTransform &tf)
    {
      float turtleDistance = sqrt(pow(tf.getOrigin().x(), 2) + pow(tf.getOrigin().y(), 2));
      return turtleDistance;
    }

    void turtleVelocity(const int &distance, const tf::StampedTransform &tf)
    {
      geometry_msgs::Twist vel_msg;

      if(distance < 1.0)
      {
        vel_msg.angular.z = 0;
        vel_msg.linear.x = 0;
      }else{
        vel_msg.angular.z = 4.0 * atan2(tf.getOrigin().y(), tf.getOrigin().x());
        vel_msg.linear.x = 2.0 * sqrt(pow(tf.getOrigin().x(), 2) + pow(tf.getOrigin().y(), 2));
      }

      turtle_vel.publish(vel_msg);
    }

    void defaultParameters()
    {
      node.setParam("/last_turtle", "/turtle1");
      node.setParam("/added_turtles", 0);
      node.setParam("level_turtles", 3);
    }

    void colorChange()
    {
      int r, g, b;
      std::srand(time(0));
      r = std::rand() % 255 + 0;
      g = std::rand() % 255 + 0;
      b = std::rand() % 255 + 0;
      node.setParam("/sim/background_r", r);
      node.setParam("/sim/background_g", g);
      node.setParam("/sim/background_b", b);
      clear_service = node.serviceClient<std_srvs::Empty>("clear");
      clear_service.call(clear_srv);
    }

    void resetGame()
    {
      reset_service = node.serviceClient<std_srvs::Empty>("reset");
      node.getParam("/turtles_spawned", turtles_spawned);

      reset_service.call(reset_srv);

      node.setParam("/turtles_spawned", 0);

      for(int k = 1; k < turtles_spawned; k++)
      {
        std::string number = std::to_string(k+1);
        std::string kill_action = "bash -c \"rosnode kill /snake"+number+"\"";
        system(kill_action.c_str());
      }
      /*kill_service = node.serviceClient<turtlesim::Kill>("kill");
      for(int i = 1; i <= turtles_spawned; i++)
      {
        std::string number = std::to_string(i+1);
        kill_srv.request.name = "turtle"+number;
        kill_service.call(kill_srv);
      }*/
    }


    void step()
    {
      tf::StampedTransform transform, transform2;
      turtleTransform(turtle_name, "/turtle1", transform);
      float d = turtleDistance(transform);
      if(d == 0) return;

      if (d < 1.0 && !follow)
      {
        follow = true;
        colorChange();
        start_time_to_follow = ros::Time::now();

        node.getParam("/last_turtle", last_turtle_);
        node.getParam("/added_turtles", added_turtles);

        node.setParam("/last_turtle", turtle_name);
        node.setParam("/added_turtles", ++added_turtles);

        //std::string cmd = "bash -c \"rosparam dump -v "+ros::package::getPath("turtlesim_snake")+"/turtle.yaml\"";
        //system(cmd.c_str());
      }

      if(follow)
      {
        if(last_turtle_ == "/turtle1")
        {
          turtleVelocity(d, transform);
        }
        else
        {
          turtleTransform(turtle_name, last_turtle_, transform2);
          float d2 = turtleDistance(transform2);
          turtleVelocity(d2, transform2);
        }

        if (d < 0.5 && (ros::Time::now()-start_time_to_follow).toSec()>3)
        {
          defaultParameters();

          ROS_INFO("COLISION. YOU HAVE LOST. RESTARTING THE GAME...");

          resetGame();
        }

        node.getParam("/level_turtles", level_turtles);
        if(added_turtles == level_turtles && added_turtles < turtle_limit)
        {
          int new_level_turtles = level_turtles + 2;

          defaultParameters();
          node.setParam("level_turtles", new_level_turtles);

          ROS_INFO("LEVEL COMPLETE. LEVEL UP!");
          ROS_INFO("NEW OBJECTIVE: [%d] TURTLES", new_level_turtles);

          resetGame();
        }

        if(added_turtles == turtle_limit)
        {
          defaultParameters();
          std::string kill_action = "bash -c \"rosnode kill /teleop /turtle_spawn\"";
          system(kill_action.c_str());
          ROS_INFO("CONGRATULATIONS. YOU HAVE COMPLETE THE GAME");

          node.getParam("/turtles_spawned", turtles_spawned);
          kill_service = node.serviceClient<turtlesim::Kill>("kill");

          for(int i = 1; i <= turtles_spawned; i++)
          {
            std::string number = std::to_string(i+1);
            kill_srv.request.name = "turtle"+number;
            kill_service.call(kill_srv);
          }
        }
      }
    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "snake_game");
  Listener listener;

  ros::Rate rate(10.0);
  while(ros::ok())
  {
    listener.step();
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
