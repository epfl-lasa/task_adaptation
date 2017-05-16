#include "ros/ros.h"
#include "TaskAdaptor.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "task_adaptation_node");

  ros::NodeHandle nh;
  double frequency = 250.0;

  // Parameters
  std::string my_string;
  std::vector<double> my_double_vector;

  // if (!nh.getParam("state_topic_arm", state_topic_arm))
  // {
  //   ROS_ERROR("Couldn't retrieve the state_topic_arm. ");
  //   return -1;
  // }


  TaskAdaptor task_adaptor(nh, frequency);


  if (!task_adaptor.Init()) {
    return -1;
  }
  else {
    task_adaptor.Run();
  }


  return 0;
}