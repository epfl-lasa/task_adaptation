#include "ros/ros.h"
#include "TwoHandoversAdaptor.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "handover_adaptation_node");

  ros::NodeHandle nh;
  double frequency = 100.0;

  // Parameters
  std::string topic_target1_position;
  std::string topic_target2_position;
  std::string topic_target1_velocity;
  std::string topic_target2_velocity;
  std::string topic_task1_velocity;
  std::string topic_task2_velocity;
  std::string topic_adapted_velocity;
  std::string topic_robot_position;
  std::string topic_robot_velocity;


  if (!nh.getParam("topic_target1_position", topic_target1_position))
  {
    ROS_ERROR("Couldn't retrieve the topic name for the target 1 position. ");
    return -1;
  }

  if (!nh.getParam("topic_target2_position", topic_target2_position))
  {
    ROS_ERROR("Couldn't retrieve the topic name for the target 2 position. ");
    return -1;
  }

  if (!nh.getParam("topic_target1_velocity", topic_target1_velocity))
  {
    ROS_ERROR("Couldn't retrieve the topic name for the target 1 velocity. ");
    return -1;
  }

  if (!nh.getParam("topic_target2_velocity", topic_target2_velocity))
  {
    ROS_ERROR("Couldn't retrieve the topic name for the target 2 velocity. ");
    return -1;
  }

  if (!nh.getParam("topic_task1_velocity", topic_task1_velocity))
  {
    ROS_ERROR("Couldn't retrieve the topic name for the task 1 velocity. ");
    return -1;
  }

  if (!nh.getParam("topic_task2_velocity", topic_task2_velocity))
  {
    ROS_ERROR("Couldn't retrieve the topic name for the task 2 velocity. ");
    return -1;
  }

  if (!nh.getParam("topic_adapted_velocity", topic_adapted_velocity))
  {
    ROS_ERROR("Couldn't retrieve the topic name for the adapted velocity. ");
    return -1;
  }

  if (!nh.getParam("topic_robot_position", topic_robot_position))
  {
    ROS_ERROR("Couldn't retrieve the topic name for the robot position. ");
    return -1;
  }

  if (!nh.getParam("topic_robot_velocity", topic_robot_velocity))
  {
    ROS_ERROR("Couldn't retrieve the topic name for the robot velocity. ");
    return -1;
  }


  TwoHandoversAdaptor task_adaptor(nh,
                                   frequency,
                                   topic_target1_position,
                                   topic_target2_position,
                                   topic_target1_velocity,
                                   topic_target2_velocity,
                                   topic_task1_velocity,
                                   topic_task2_velocity,
                                   topic_robot_position,
                                   topic_robot_velocity,
                                   topic_adapted_velocity);

  task_adaptor.Run();

  return 0;
}