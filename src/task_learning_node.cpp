#include "ros/ros.h"
#include "TaskLearner.h"

#include <vector>


int main(int argc, char **argv)
{

  ros::init(argc, argv, "task_learning_node");

  ros::NodeHandle nh;
  double frequency = 250.0;

  // Parameters
  std::string topic_real_position;
  std::string topic_real_velocity;

  std::string topic_task1_velocity;
  std::string topic_task2_velocity;
  std::string topic_task3_velocity;
  std::string topic_task4_velocity;

  std::string topic_adapted_velocity;

  std::vector<double> x_lim;
  std::vector<double> y_lim;
  std::vector<double> z_lim;

  std::vector<int> N_grid_xyz;



  std::string topic_external_force;





  if (!nh.getParam("topic_real_position", topic_real_position))
  {
    ROS_ERROR("Couldn't retrieve the topic name for the real velocity. ");
    return -1;
  }

  if (!nh.getParam("topic_external_force", topic_external_force))
  {
    ROS_ERROR("Visualization: Couldn't retrieve the topic name for the external force. ");
    return -1;
  }

  if (!nh.getParam("topic_real_velocity", topic_real_velocity))
  {
    ROS_ERROR("Couldn't retrieve the topic name for the real position. ");
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

  if (!nh.getParam("x_lim", x_lim))  {
    ROS_ERROR("Couldn't retrieve limits for the X-axis. ");
    return -1;
  }

  if (!nh.getParam("y_lim", y_lim))  {
    ROS_ERROR("Couldn't retrieve limits for the Y-axis. ");
    return -1;
  }

  if (!nh.getParam("z_lim", z_lim))  {
    ROS_ERROR("Couldn't retrieve limits for the Z-axis. ");
    return -1;
  }

  if (!nh.getParam("N_grid_xyz", N_grid_xyz))  {
    ROS_ERROR("Couldn't retrieve the grid size for the dimenntions ");
    return -1;
  }



  TaskLearner task_learner(nh,
                           frequency,
                           topic_real_position,
                           topic_real_velocity,
                           topic_task1_velocity,
                           topic_task2_velocity,
                           topic_task3_velocity,
                           topic_task4_velocity,
                           topic_adapted_velocity,
                           topic_external_force,
                           x_lim,
                           y_lim,
                           z_lim,
                           N_grid_xyz);


  if (!task_learner.Init()) {
    return -1;
  }
  else {
    task_learner.Run();
  }


  return 0;
}