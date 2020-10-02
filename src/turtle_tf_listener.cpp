#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/Pose.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;

  ros::service::waitForService("spawn");
  ros::ServiceClient add_turtle =
    node.serviceClient<turtlesim::Spawn>("spawn");
  turtlesim::Spawn srv;
  srv.request.x = 10;
  srv.request.y = 10;
  srv.request.theta = 0;
  add_turtle.call(srv);

  ros::Publisher turtle_vel =
    node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);



  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (ros::ok()){
    tf::StampedTransform transform;
    try{
      ros::Time now = ros::Time::now();
      ros::Time past = now - ros::Duration(5.0);
     listener.waitForTransform("/turtle2", now,
                               "/turtle1", past,
                               "/world", ros::Duration(1.0));
     listener.lookupTransform("/turtle2", now,
                              "/turtle1", past,
                              "/world", transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    geometry_msgs::Twist vel_msg;
    vel_msg.angular.z = 4.0 * atan2(transform.getOrigin().y(),
                                    transform.getOrigin().x());
    vel_msg.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
                                  pow(transform.getOrigin().y(), 2));
    turtle_vel.publish(vel_msg);

    float d = sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2));
    std::cout << d << '\n';



    rate.sleep();
  }

  return 0;
};
