#include <ros/ros.h>

#include "geometry_msgs/TwistStamped.h"










int main(int argc, char **argv)
{

	// variables
	geometry_msgs::TwistStamped Task1_velocity;
	geometry_msgs::TwistStamped Task2_velocity;
	geometry_msgs::TwistStamped Task3_velocity;
	geometry_msgs::TwistStamped Task4_velocity;
	geometry_msgs::TwistStamped Real_velocity;




	ros::init(argc, argv, "UnitaryTasks");
	ros::NodeHandle n;

	ros::Publisher pub_task1 = n.advertise<geometry_msgs::TwistStamped>("/testUnitary/task1/DesiredVelocity", 1000);
	ros::Publisher pub_task2 = n.advertise<geometry_msgs::TwistStamped>("/testUnitary/task2/DesiredVelocity", 1000);
	ros::Publisher pub_task3 = n.advertise<geometry_msgs::TwistStamped>("/testUnitary/task3/DesiredVelocity", 1000);
	ros::Publisher pub_task4 = n.advertise<geometry_msgs::TwistStamped>("/testUnitary/task4/DesiredVelocity", 1000);

	ros::Publisher pub_realVel = n.advertise<geometry_msgs::TwistStamped>("/testUnitary/real_velocity", 1000);

	ros::Rate loop_rate(1000);

	ros::Duration switch_rate(5);

	ros::Time::now();

	ros::Time time_switch = ros::Time::now() + switch_rate;


	while (ros::ok())
	{
		ros::Time time_now = ros::Time::now();

		Task1_velocity.header.stamp = time_now;
		Task1_velocity.header.frame_id = "/end_effector";
		Task1_velocity.twist.linear.x = 0.15;
		Task1_velocity.twist.linear.y = 0;
		Task1_velocity.twist.linear.z = 0.0;
		pub_task1.publish(Task1_velocity);


		Task2_velocity.header.stamp = time_now;
		Task2_velocity.header.frame_id = "/end_effector";
		Task2_velocity.twist.linear.x = -0.14;
		Task2_velocity.twist.linear.y = 0;
		Task2_velocity.twist.linear.z = 0;
		pub_task2.publish(Task2_velocity);

		Task3_velocity.header.stamp = time_now;
		Task3_velocity.header.frame_id = "/end_effector";
		Task3_velocity.twist.linear.x = 0;
		Task3_velocity.twist.linear.y = 0.15;
		Task3_velocity.twist.linear.z = 0;
		pub_task3.publish(Task3_velocity);

		Task4_velocity.header.stamp = time_now;
		Task4_velocity.header.frame_id = "/end_effector";
		Task4_velocity.twist.linear.x = 0;
		Task4_velocity.twist.linear.y = -0.14;
		Task4_velocity.twist.linear.z = 0;
		pub_task4.publish(Task4_velocity);


		if ( (ros::Time::now() - time_switch).toSec() > 0  ) {

			time_switch = ros::Time::now() + switch_rate;
			int task_number = rand() % 4 + 1;

			switch (task_number) {

			case 1 :
				Real_velocity = Task1_velocity;
				break;

			case 2 :
				Real_velocity = Task2_velocity;
				break;

			case 3 :
				Real_velocity = Task3_velocity;
				break;

			case 4 :
				Real_velocity = Task4_velocity;
				break;

			}
		}

		Real_velocity.header.stamp = time_now;
		pub_realVel.publish(Real_velocity);




		ros::spinOnce();

		loop_rate.sleep();

	}




	return 0;
}

