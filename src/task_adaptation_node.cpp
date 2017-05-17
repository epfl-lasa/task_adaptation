#include "ros/ros.h"
#include "TaskAdaptor.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "task_adaptation_node");

  ros::NodeHandle nh;
  double frequency = 250.0;

  // Parameters
  std::string topic_real_velocity;
  std::string topic_task1_velocity;
  std::string topic_task2_velocity;
  std::string topic_task3_velocity;
  std::string topic_task4_velocity;
  std::string topic_adapted_velocity;
  std::string topic_desired_force;




  std::vector<double> my_double_vector;


  if (!nh.getParam("topic_real_velocity", topic_real_velocity))
  {
    ROS_ERROR("Couldn't retrieve the topic name for the real velocity. ");
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

  if (!nh.getParam("topic_task3_velocity", topic_task3_velocity))
  {
    ROS_ERROR("Couldn't retrieve the topic name for the task 3 velocity. ");
    return -1;
  }

  if (!nh.getParam("topic_task4_velocity", topic_task4_velocity))
  {
    ROS_ERROR("Couldn't retrieve the topic name for the task 4 velocity. ");
    return -1;
  }

  if (!nh.getParam("topic_adapted_velocity", topic_adapted_velocity))
  {
    ROS_ERROR("Couldn't retrieve the topic name for the adapted velocity. ");
    return -1;
  }

  if (!nh.getParam("topic_desired_force", topic_desired_force))
  {
    ROS_ERROR("Couldn't retrieve the topic name for the desired force. ");
    return -1;
  }




  TaskAdaptor task_adaptor(nh,
                           frequency,
                           topic_real_velocity,
                           topic_task1_velocity,
                           topic_task2_velocity,
                           topic_task3_velocity,
                           topic_task4_velocity,
                           topic_adapted_velocity,
                           topic_desired_force);


  if (!task_adaptor.Init()) {
    return -1;
  }
  else {
    task_adaptor.Run();
  }


  return 0;
}