#include <ros/ros.h>

#include "std_msgs/Float64MultiArray.h"

#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PointStamped.h"


class BeliefVisualizer
{

private:
	ros::NodeHandle nh_;

	std::vector<double> beliefs_;

	std::vector<double> target_pose_1_;
	std::vector<double> target_pose_2_;
	std::vector<double> target_pose_3_;
	std::vector<double> target_pose_4_;

	ros::Subscriber sub_target_1_;
	ros::Subscriber sub_target_2_;
	ros::Subscriber sub_target_3_;
	ros::Subscriber sub_target_4_;

	ros::Subscriber sub_beliefs_;



	ros::Publisher pub_marker_1_;
	ros::Publisher pub_marker_2_;
	ros::Publisher pub_marker_3_;
	ros::Publisher pub_marker_4_;


	visualization_msgs::Marker marker_1_;
	visualization_msgs::Marker marker_2_;
	visualization_msgs::Marker marker_3_;
	visualization_msgs::Marker marker_4_;



//geometry_msgs::PointStamped


public:
	BeliefVisualizer() {


		beliefs_.resize(5);

		target_pose_1_.resize(3);
		target_pose_2_.resize(3);
		target_pose_3_.resize(3);
		target_pose_4_.resize(3);

		marker_1_.header.frame_id = "world";
		marker_2_.header.frame_id = "world";
		marker_3_.header.frame_id = "world";
		marker_4_.header.frame_id = "world";

		marker_1_.type = visualization_msgs::Marker::SPHERE;
		marker_2_.type = visualization_msgs::Marker::SPHERE;
		marker_3_.type = visualization_msgs::Marker::SPHERE;
		marker_4_.type = visualization_msgs::Marker::SPHERE;

		double my_scale = 0.02;

		marker_1_.scale.x = my_scale;
		marker_1_.scale.y = my_scale;
		marker_1_.scale.z = my_scale;

		marker_2_.scale.x = my_scale;
		marker_2_.scale.y = my_scale;
		marker_2_.scale.z = my_scale;

		marker_3_.scale.x = my_scale;
		marker_3_.scale.y = my_scale;
		marker_3_.scale.z = my_scale;

		marker_4_.scale.x = my_scale;
		marker_4_.scale.y = my_scale;
		marker_4_.scale.z = my_scale;


		marker_1_.color.r = 1.0f;
		marker_1_.color.g = 0.0f;
		marker_1_.color.b = 0.0f;
		marker_1_.color.a = 1.0f;

		marker_2_.color.r = 0.0f;
		marker_2_.color.g = 1.0f;
		marker_2_.color.b = 0.0f;
		marker_2_.color.a = 1.0f;

		marker_3_.color.r = 0.0f;
		marker_3_.color.g = 0.3f;
		marker_3_.color.b = 1.0f;
		marker_3_.color.a = 1.0f;

		marker_4_.color.r = 0.9f;
		marker_4_.color.g = 0.2f;
		marker_4_.color.b = 0.0f;
		marker_4_.color.a = 1.0f;


		marker_1_.lifetime = ros::Duration();
		marker_2_.lifetime = ros::Duration();
		marker_3_.lifetime = ros::Duration();
		marker_4_.lifetime = ros::Duration();

		marker_1_.text = "Task1";


		sub_target_1_ = nh_.subscribe("/Task1/DS/target"  , 10, &BeliefVisualizer::GetTargetTask1, this);
		sub_target_2_ = nh_.subscribe("/Task2/DS/target"  , 10, &BeliefVisualizer::GetTargetTask2, this);
		sub_target_3_ = nh_.subscribe("/Task3/DS/target"  , 10, &BeliefVisualizer::GetTargetTask3, this);
		sub_target_4_ = nh_.subscribe("/Task4/DS/target"  , 10, &BeliefVisualizer::GetTargetTask4, this);

		sub_beliefs_ = nh_.subscribe("/task_adaptation/beliefs"  , 10, &BeliefVisualizer::UpdateBeliefs, this);



		pub_marker_1_ = nh_.advertise<visualization_msgs::Marker>("Task1/dynamic_marker", 1);
		pub_marker_2_ = nh_.advertise<visualization_msgs::Marker>("Task2/dynamic_marker", 1);
		pub_marker_3_ = nh_.advertise<visualization_msgs::Marker>("Task3/dynamic_marker", 1);
		pub_marker_4_ = nh_.advertise<visualization_msgs::Marker>("Task4/dynamic_marker", 1);




	}

	void GetTargetTask1(const geometry_msgs::PointStamped::ConstPtr& msg)
	{
		target_pose_1_[0] = msg->point.x;
		target_pose_1_[1] = msg->point.y;
		target_pose_1_[2] = msg->point.z;
		UpdateVisualization();
	}

	void GetTargetTask2(const geometry_msgs::PointStamped::ConstPtr& msg)
	{
		target_pose_2_[0] = msg->point.x;
		target_pose_2_[1] = msg->point.y;
		target_pose_2_[2] = msg->point.z;
		UpdateVisualization();
	}

	void GetTargetTask3(const geometry_msgs::PointStamped::ConstPtr& msg)
	{
		target_pose_3_[0] = msg->point.x;
		target_pose_3_[1] = msg->point.y;
		target_pose_3_[2] = msg->point.z;
		UpdateVisualization();
	}

	void GetTargetTask4(const geometry_msgs::PointStamped::ConstPtr& msg)
	{
		target_pose_4_[0] = msg->point.x;
		target_pose_4_[1] = msg->point.y;
		target_pose_4_[2] = msg->point.z;
		UpdateVisualization();
	}


	void UpdateBeliefs(const std_msgs::Float64MultiArray::ConstPtr& msg)
	{
		// target_pose_4_[0] = msg->point.x;
		// target_pose_4_[1] = msg->point.y;
		// target_pose_4_[2] = msg->point.z;

		beliefs_[0] = msg->data[0];
		beliefs_[1] = msg->data[1];
		beliefs_[2] = msg->data[2];
		beliefs_[3] = msg->data[3];
		beliefs_[4] = msg->data[4];

		UpdateVisualization();
	}





	void UpdateVisualization() {

		marker_1_.header.stamp = ros::Time::now();
		marker_2_.header.stamp = ros::Time::now();
		marker_3_.header.stamp = ros::Time::now();
		marker_4_.header.stamp = ros::Time::now();

		marker_1_.pose.position.x = target_pose_1_[0];
		marker_1_.pose.position.y = target_pose_1_[1];
		marker_1_.pose.position.z = target_pose_1_[2];

		marker_2_.pose.position.x = target_pose_2_[0];
		marker_2_.pose.position.y = target_pose_2_[1];
		marker_2_.pose.position.z = target_pose_2_[2];

		marker_3_.pose.position.x = target_pose_3_[0];
		marker_3_.pose.position.y = target_pose_3_[1];
		marker_3_.pose.position.z = target_pose_3_[2];

		marker_4_.pose.position.x = target_pose_4_[0];
		marker_4_.pose.position.y = target_pose_4_[1];
		marker_4_.pose.position.z = target_pose_4_[2];


		marker_1_.color.a = 0.3f + 0.7 * beliefs_[1];
		marker_2_.color.a = 0.3f + 0.7 * beliefs_[2];
		marker_3_.color.a = 0.3f + 0.7 * beliefs_[3];
		marker_4_.color.a = 0.3f + 0.7 * beliefs_[4];


		pub_marker_1_.publish(marker_1_);
		pub_marker_2_.publish(marker_2_);
		pub_marker_3_.publish(marker_3_);
		pub_marker_4_.publish(marker_4_);



	}


};












int main(int argc, char **argv)
{
	//Initiate ROS
	ros::init(argc, argv, "belief_visualizer");

	//Create an object of class JointMapper to read the robot joint angles and send it to robot_state_publisher and rviz
	BeliefVisualizer visualizer_object;

	ros::spin();

	return 0;
}
