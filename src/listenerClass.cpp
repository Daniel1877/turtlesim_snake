#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>


class Listener
{
  ros::NodeHandle node;
  bool follow;
  bool started_;

  turtlesim::Spawn srv;
  ros::Publisher turtle_vel;
  ros::ServiceClient add_turtle;
  tf::TransformListener listener;

public:
  Listener():
    follow(false),
    started_(true)
    {
      ros::service::waitForService("spawn");
      add_turtle = node.serviceClient<turtlesim::Spawn>("spawn");
      srv.request.x = 10;
      srv.request.y = 10;
      add_turtle.call(srv);

      turtle_vel = node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);
    }

    ~Listener(){}

    void step()
    {
      tf::StampedTransform transform;
      try{
        ros::Time now = ros::Time::now();
        ros::Time past = now - ros::Duration(0.1);
        listener.waitForTransform("/turtle2", now,
                                  "/turtle1", past,
                                  "/world", ros::Duration(3.0));
        listener.lookupTransform("/turtle2", now,
                                 "/turtle1", past,
                                 "/world", transform);
      }
      catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
      }

      float d = sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2));
      if(d == 0) return;

      if (d < 1.0 && !follow) follow = true;

      if(follow)
      {
        geometry_msgs::Twist vel_msg;
        vel_msg.angular.z = 4.0 * atan2(transform.getOrigin().y(),
                                        transform.getOrigin().x());
        vel_msg.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
                                      pow(transform.getOrigin().y(), 2));
        turtle_vel.publish(vel_msg);
      }
    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_tf_listener");
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
