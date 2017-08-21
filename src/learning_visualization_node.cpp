#include "ros/ros.h"
#include "LearningVisualizer.h"

#include <vector>


int main(int argc, char **argv)
{

  ros::init(argc, argv, "learning_visualization_node");

  ros::NodeHandle nh;
  double frequency = 30.0;

  // Parameters
  std::string topic_real_position;
  std::string topic_beliefs;
  std::string topic_betas;

  std::vector<double> x_lim;
  std::vector<double> y_lim;
  std::vector<double> z_lim;

  std::vector<int> N_grid_xyz;


  if (!nh.getParam("topic_real_position", topic_real_position))
  {
    ROS_ERROR("Visualization: Couldn't retrieve the topic name for the real position. ");
    return -1;
  }

  if (!nh.getParam("topic_beliefs", topic_beliefs))
  {
    ROS_ERROR("Visualization: Couldn't retrieve the topic name for the beleifs. ");
    return -1;
  }

  if (!nh.getParam("topic_betas", topic_betas))
  {
    ROS_ERROR("Visualization: Couldn't retrieve the topic name for the beta values. ");
    return -1;
  }


  if (!nh.getParam("x_lim", x_lim))  {
    ROS_ERROR("Visualization: Couldn't retrieve limits for the X-axis. ");
    return -1;
  }

  if (!nh.getParam("y_lim", y_lim))  {
    ROS_ERROR("Visualization: Couldn't retrieve limits for the Y-axis. ");
    return -1;
  }

  if (!nh.getParam("z_lim", z_lim))  {
    ROS_ERROR("Visualization: Couldn't retrieve limits for the Z-axis. ");
    return -1;
  }

  if (!nh.getParam("N_grid_xyz", N_grid_xyz))  {
    ROS_ERROR("Visualization: Couldn't retrieve the grid size for the dimenntions ");
    return -1;
  }



  LearningVisualizer LearningVisualizer(nh,
                                        frequency,
                                        topic_real_position,
                                        topic_beliefs,
                                        topic_betas,
                                        x_lim,
                                        y_lim,
                                        z_lim,
                                        N_grid_xyz);


  if (!LearningVisualizer.Init()) {
    return -1;
  }
  else {
    LearningVisualizer.Run();
  }


  return 0;
}