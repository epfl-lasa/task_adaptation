#include <iostream>
#include <stdio.h>
#include <math.h>

#include <ros/ros.h>

#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/WrenchStamped.h"

#include "cartesian_state_msgs/PoseTwist.h"


#include <dynamic_reconfigure/server.h>
#include <task_adaptation/task_adaptation_params.h>
// variables

geometry_msgs::TwistStamped msgAdaptedVelocity;
geometry_msgs::WrenchStamped msgWrenchControl;
cartesian_state_msgs::PoseTwist msgPoseTwist;



std::vector<float> RealVelocity;
std::vector<float> DesiredVelocity;
std::vector<float> ControlWrench;


std::vector<float> Task0_velocity;
std::vector<float> Task1_velocity;
std::vector<float> Task2_velocity;
std::vector<float> Task3_velocity;
std::vector<float> Task4_velocity;

bool flag_realVelocity_newdata;
bool flag_task1_newdata;
bool flag_task2_newdata;
bool flag_task3_newdata;
bool flag_task4_newdata;

//float belief_task0;
//float belief_task1;
//float belief_task2;
//float belief_task3;
//float belief_task4;

std::vector<float> Beliefs;

std::vector<float> UpdateBeliefsRaw;
std::vector<float> UpdateBeliefs;

double D_gain;
double epsilon;







void updateRealVelocity(const cartesian_state_msgs::PoseTwist::ConstPtr& msg);
void updateRealVelocity_world(const geometry_msgs::Twist::ConstPtr& msg);


void UpdateTask1(const geometry_msgs::TwistStamped::ConstPtr& msg);
void UpdateTask2(const geometry_msgs::TwistStamped::ConstPtr& msg);
void UpdateTask3(const geometry_msgs::TwistStamped::ConstPtr& msg);
void UpdateTask4(const geometry_msgs::TwistStamped::ConstPtr& msg);

void configCallback(task_adaptation::task_adaptation_params &config, uint32_t level);
void UpdateParamCallback(const task_adaptation::task_adaptation_params::ConstPtr &msg);



void DisplayInformation();


void UpdateDesiredVelocity();

void RawAdaptation();

float ComputeInnerSimilarity(float b, std::vector<float> RealVelocity);
float ComputeOutterSimilarity(std::vector<float> RealVelocity);

//float ComputeTotalInnerSimilarity();


void WinnerTakeAll();




int main(int argc, char **argv)
{



	// initializing the varibales
	RealVelocity.resize(3);
	DesiredVelocity.resize(3);
	ControlWrench.resize(6);

	Task0_velocity.resize(3);
	Task1_velocity.resize(3);
	Task2_velocity.resize(3);
	Task3_velocity.resize(3);
	Task4_velocity.resize(3);

	flag_realVelocity_newdata = false;
	flag_task1_newdata = false;
	flag_task2_newdata = false;
	flag_task3_newdata = false;
	flag_task4_newdata = false;



	Beliefs.resize(5);
	Beliefs[0] = 0;
	Beliefs[1] = 1;
	Beliefs[2] = 0;
	Beliefs[3] = 0;
	Beliefs[4] = 0;

	UpdateBeliefsRaw.resize(5);
	std::fill(UpdateBeliefsRaw.begin(), UpdateBeliefsRaw.end(), 0);

	UpdateBeliefs.resize(5);
	std::fill(UpdateBeliefs.begin(), UpdateBeliefs.end(), 0);

//	belief_task0 = 1;
//	belief_task1 = 0;
//	belief_task2 = 0;
//	belief_task3 = 0;
//	belief_task4 = 0;


	// task 0 is a null primitive
	Task0_velocity[0] = 0;
	Task0_velocity[1] = 0;
	Task0_velocity[2] = 0;



	// setup callback when application exits
//	atexit(close);


	ros::init(argc, argv, "taskAdaptation");
	ros::NodeHandle n;

//	ros::Publisher pub_velocity = n.advertise<geometry_msgs::TwistStamped>("TaskAdaptation/DesiredVelocity", 1000);
	ros::Publisher pub_adapted_velocity = n.advertise<geometry_msgs::TwistStamped>("/TaskAdaptation/adapted_velocity", 1);
	ros::Publisher pub_wrench_control   = n.advertise<geometry_msgs::WrenchStamped>("/wrench_control", 1);


//	ros::Subscriber sub_realVelocity = n.subscribe("TaskAdaptation/RealVelocity", 1000, updateRealVelocity);
	ros::Subscriber sub_realVelocity = n.subscribe("/ur5_cartesian_velocity_controller_sim/ee_state", 1000, updateRealVelocity);
	ros::Subscriber sub_realVelocity_world = n.subscribe("/ur5_cartesian_velocity_controller_sim/cart_vel_world", 1000, updateRealVelocity_world);


	ros::Subscriber sub_task1 = n.subscribe("TaskAdaptation/task1/DesiredVelocity", 1000, UpdateTask1);
	ros::Subscriber sub_task2 = n.subscribe("TaskAdaptation/task2/DesiredVelocity", 1000, UpdateTask2);
	ros::Subscriber sub_task3 = n.subscribe("TaskAdaptation/task3/DesiredVelocity", 1000, UpdateTask3);
	ros::Subscriber sub_task4 = n.subscribe("TaskAdaptation/task4/DesiredVelocity", 1000, UpdateTask4);



	D_gain  = 2;
	epsilon = 0.01;


	// Initialize node parameters from launch file or command line.
	// Use a private node handle so that multiple instances of the node can be run simultaneously
	// while using different parameters.
	std::string topic;
	int rate;
	ros::NodeHandle private_node_handle_("~");
	private_node_handle_.param("rate", rate, int(40));
	private_node_handle_.param("topic", topic, std::string("/TaskAdaptation/dynamic_params"));
	ros::Subscriber sub_message = n.subscribe(topic.c_str(), 1000, UpdateParamCallback);




	ros::Rate loop_rate(1000);


	ros::Time::now();
	ros::Duration freq_display(.4);
	ros::Time time_display = ros::Time::now() + freq_display;

	while (ros::ok())
	{


		if ( (ros::Time::now() - time_display).toSec() > 0  )
		{
			DisplayInformation();
			time_display = ros::Time::now() + freq_display;
		}



		// check if all tasks/primitives and the robot are alive
		bool flagAdapt = flag_realVelocity_newdata;
		flagAdapt = flagAdapt && flag_task1_newdata && flag_task2_newdata;
		flagAdapt = flagAdapt && flag_task3_newdata && flag_task4_newdata;

		if (flagAdapt && 0)
		{

			UpdateDesiredVelocity();

			RawAdaptation();

			WinnerTakeAll();


			for (int i = 0; i < Beliefs.size(); i++)
			{
				Beliefs[i] += epsilon * UpdateBeliefs[i];

				if (Beliefs[i] > 1)
					Beliefs[i] = 1;

				if (Beliefs[i] < 0)
					Beliefs[i] = 0;
			}



			// setting the flags to false and wait for new data to be read
			flag_realVelocity_newdata = false;
			flag_task1_newdata = false;
			flag_task2_newdata = false;
			flag_task3_newdata = false;
			flag_task4_newdata = false;






		}


		UpdateDesiredVelocity();

		msgAdaptedVelocity.header.stamp = ros::Time::now();
		msgAdaptedVelocity.twist.linear.x = DesiredVelocity[0];
		msgAdaptedVelocity.twist.linear.y = DesiredVelocity[1];
		msgAdaptedVelocity.twist.linear.z = DesiredVelocity[2];

		pub_adapted_velocity.publish(msgAdaptedVelocity);



		ControlWrench[0] = -D_gain * (RealVelocity[0] - DesiredVelocity[0]);
		ControlWrench[1] = -D_gain * (RealVelocity[1] - DesiredVelocity[1]);
		ControlWrench[2] = -D_gain * (RealVelocity[2] - DesiredVelocity[2]);

		if(ControlWrench[0] < -0.6)
			ControlWrench[0] = -0.6;
		if(ControlWrench[1] < -0.6)
			ControlWrench[1] = -0.6;
		if(ControlWrench[2] < -0.6)
			ControlWrench[2] = -0.6;

		if(ControlWrench[0] > 0.6)
			ControlWrench[0] = 0.6;
		if(ControlWrench[1] > 0.6)
			ControlWrench[1] = 0.6;
		if(ControlWrench[2] > 0.6)
			ControlWrench[2] = 0.6;


		msgWrenchControl.header.stamp = ros::Time::now();
		msgWrenchControl.header.frame_id = "world"; // just for visualization
		msgWrenchControl.wrench.force.x = ControlWrench[0];
		msgWrenchControl.wrench.force.y = ControlWrench[1];
		msgWrenchControl.wrench.force.z = ControlWrench[2];
		msgWrenchControl.wrench.torque.x = 0;
		msgWrenchControl.wrench.torque.y = 0;
		msgWrenchControl.wrench.torque.z = 0;

		pub_wrench_control.publish(msgWrenchControl);







		ros::spinOnce();
		loop_rate.sleep();
	}




	return 0;
}



// ---------------------------------------------------------------------------
//------------------- The raw adaptation takes place in this function --------
// ---------------------------------------------------------------------------

void RawAdaptation()
{
	std::fill(UpdateBeliefsRaw.begin(), UpdateBeliefsRaw.end(), 0);


	UpdateBeliefsRaw[1] -= ComputeOutterSimilarity(Task1_velocity);
	UpdateBeliefsRaw[1] -= 2 * ComputeInnerSimilarity(Beliefs[1], Task1_velocity);

	UpdateBeliefsRaw[2] -= ComputeOutterSimilarity(Task2_velocity);
	UpdateBeliefsRaw[2] -= 2 * ComputeInnerSimilarity(Beliefs[2], Task2_velocity);

	UpdateBeliefsRaw[3] -= ComputeOutterSimilarity(Task3_velocity);
	UpdateBeliefsRaw[3] -= 2 * ComputeInnerSimilarity(Beliefs[3], Task3_velocity);

	UpdateBeliefsRaw[4] -= ComputeOutterSimilarity(Task4_velocity);
	UpdateBeliefsRaw[4] -= 2 * ComputeInnerSimilarity(Beliefs[4], Task4_velocity);





	UpdateBeliefsRaw[0] -= ComputeOutterSimilarity(Task0_velocity);
//	UpdateBeliefsRaw[0] += 2 * Beliefs[0] * .03;

//	UpdateBeliefsRaw[0] -= 0.25 * ComputeInnerSimilarity(Beliefs[1],Task1_velocity);
//	UpdateBeliefsRaw[0] -= 0.25 * ComputeInnerSimilarity(Beliefs[2],Task2_velocity);
//	UpdateBeliefsRaw[0] -= 0.25 * ComputeInnerSimilarity(Beliefs[3],Task3_velocity);
//	UpdateBeliefsRaw[0] -= 0.25 * ComputeInnerSimilarity(Beliefs[4],Task4_velocity);

//	UpdateBeliefsRaw[0] += (Beliefs[1] - 1.0) * UpdateBeliefsRaw[1];
//	UpdateBeliefsRaw[0] += (Beliefs[2] - 1.0) * UpdateBeliefsRaw[2];
//	UpdateBeliefsRaw[0] += (Beliefs[3] - 1.0) * UpdateBeliefsRaw[3];
//	UpdateBeliefsRaw[0] += (Beliefs[4] - 1.0) * UpdateBeliefsRaw[4];

}


void WinnerTakeAll()
{
	// initializing the updates for the beliefs at zero
	std::fill(UpdateBeliefs.begin(), UpdateBeliefs.end(), 0);

	// fining the winner who has the biggest value for UpdateBeliefsRaw
	int winner_index = 0;

	for (int i = 1; i < UpdateBeliefsRaw.size(); i++)
	{
		if (UpdateBeliefsRaw[i] > UpdateBeliefsRaw[winner_index])
			winner_index = i;
	}

	// no update is required if the winner is already saturated
	if (Beliefs[winner_index] == 1)
		return;


	int runnerUp_index = 0;
	if (winner_index == 0)
		runnerUp_index = 1;

	for (int i = 0; i < UpdateBeliefsRaw.size(); i++)
	{
		if (i ==  winner_index)
			continue;

		if (UpdateBeliefsRaw[i] > UpdateBeliefsRaw[runnerUp_index])
			runnerUp_index = i;
	}

	// computing the middle point and removing form all raw updates
	float offset = 0.5 * (UpdateBeliefsRaw[winner_index] + UpdateBeliefsRaw[runnerUp_index]);

	for (int i = 0; i < UpdateBeliefsRaw.size(); i++)
		UpdateBeliefsRaw[i] -= offset;


	// computing the sum of updates and setting to zero for active one so we keep the sum beliefs at 1.
	float UpdateSum = 0;

	for (int i = 0; i < UpdateBeliefsRaw.size(); i++)
	{
		if (Beliefs[i] != 0 || UpdateBeliefsRaw[i] > 0)
			UpdateSum += UpdateBeliefsRaw[i];
	}



	for (int i = 0; i < UpdateBeliefsRaw.size(); i++)
	{
		UpdateBeliefs[i] = UpdateBeliefsRaw[i];

	}

	UpdateBeliefs[winner_index] -= UpdateSum;


}



float ComputeInnerSimilarity(float b, std::vector<float> task_velocity)
{
	std::vector<float> OtherTasks;
	OtherTasks.resize(3);

	OtherTasks[0] = DesiredVelocity[0] - b * task_velocity[0];
	OtherTasks[1] = DesiredVelocity[1] - b * task_velocity[1];
	OtherTasks[2] = DesiredVelocity[2] - b * task_velocity[2];

	float innerSimilarity = 0;

	innerSimilarity += OtherTasks[0] * task_velocity[0];
	innerSimilarity += OtherTasks[1] * task_velocity[1];
	innerSimilarity += OtherTasks[2] * task_velocity[2];


	return innerSimilarity;

}


float ComputeOutterSimilarity(std::vector<float> task_velocity)
{

	float outterSimiliary = 0;

	outterSimiliary += (RealVelocity[0] - task_velocity[0]) * (RealVelocity[0] - task_velocity[0]);
	outterSimiliary += (RealVelocity[1] - task_velocity[1]) * (RealVelocity[1] - task_velocity[1]);
	outterSimiliary += (RealVelocity[2] - task_velocity[2]) * (RealVelocity[2] - task_velocity[2]);

	return outterSimiliary;
}





void UpdateDesiredVelocity()
{
	// starting to zero
	std::fill(DesiredVelocity.begin(), DesiredVelocity.end(), 0);

	for (int dim = 0 ; dim < 3 ; dim++)
	{
		DesiredVelocity[dim] += Beliefs[1] * Task1_velocity[dim];
		DesiredVelocity[dim] += Beliefs[2] * Task2_velocity[dim];
		DesiredVelocity[dim] += Beliefs[3] * Task3_velocity[dim];
		DesiredVelocity[dim] += Beliefs[4] * Task4_velocity[dim];
	}

}



// ---------------------------------------------------------------------------
//------------------- Display relevant information in the console ------------
// ---------------------------------------------------------------------------
void DisplayInformation()
{
	//std::cout << RealVelocity[0] << "\t" <<  RealVelocity[1] << "\t" << RealVelocity[2]  << std::endl;

	std::cout << "Time = " << ros::Time::now() << std::endl;

	if (!flag_realVelocity_newdata)
		std::cout << "Real velocity is not received " << std::endl;

	if (!flag_task1_newdata)
		std::cout << "Task1 velocity is not received " << std::endl;

	if (!flag_task2_newdata)
		std::cout << "Task2 velocity is not received " << std::endl;

	if (!flag_task3_newdata)
		std::cout << "Task3 velocity is not received " << std::endl;

	if (!flag_task4_newdata)
		std::cout << "Task4 velocity is not received " << std::endl;

//	std::cout << "Beliefs are :  b0= " << Beliefs[0] <<
//			                 "\t b1= " << Beliefs[1] <<
//							 "\t b2= " << Beliefs[2] <<
//							 "\t b3= " << Beliefs[3] <<
//							 "\t b4= " << Beliefs[4] << std::endl;

	std::cout << "Real    velocity = [" << RealVelocity[0] 	<< " , " << RealVelocity[1] << " , " << RealVelocity[2] << "]" << std::endl;
	std::cout << "Adapted velocity = [" << DesiredVelocity[0] << " , " << DesiredVelocity[1] << " , " << DesiredVelocity[2] << "]" << std::endl;
	std::cout << "Control forces   = [" << ControlWrench[0] << " , " << ControlWrench[1] << " , " << ControlWrench[2] << "]" << std::endl;



	std::cout << "Task 0 : b =" << Beliefs[0] << "\t db_hat = " << UpdateBeliefsRaw[0] << "\t db = " << UpdateBeliefs[0] <<  std::endl;
	std::cout << "Task 1 : b =" << Beliefs[1] << "\t db_hat = " << UpdateBeliefsRaw[1] << "\t db = " << UpdateBeliefs[1] <<  std::endl;
	std::cout << "Task 2 : b =" << Beliefs[2] << "\t db_hat = " << UpdateBeliefsRaw[2] << "\t db = " << UpdateBeliefs[2] <<  std::endl;
	std::cout << "Task 3 : b =" << Beliefs[3] << "\t db_hat = " << UpdateBeliefsRaw[3] << "\t db = " << UpdateBeliefs[3] <<  std::endl;
	std::cout << "Task 4 : b =" << Beliefs[4] << "\t db_hat = " << UpdateBeliefsRaw[4] << "\t db = " << UpdateBeliefs[4] <<  std::endl;





	std::cout << "----------------------------------------------------- " << std::endl << std::endl;
}







// ---------------------------------------------------------------------------
//------------------- Reading the real velocity of the robot -----------------
// ---------------------------------------------------------------------------
void updateRealVelocity(const cartesian_state_msgs::PoseTwist::ConstPtr& msg)
{

	// RealVelocity[0] = msg->twist.linear.x;
	// RealVelocity[1] = msg->twist.linear.y;
	// RealVelocity[2] = msg->twist.linear.z;

	flag_realVelocity_newdata = true;

}

void updateRealVelocity_world(const geometry_msgs::Twist::ConstPtr& msg)
{

	RealVelocity[0] = msg->linear.x;
	RealVelocity[1] = msg->linear.y;
	RealVelocity[2] = msg->linear.z;

	flag_realVelocity_newdata = true;

}

// ---------------------------------------------------------------------------
//------------------- Reading the new desired velocity of each task ----------
// ---------------------------------------------------------------------------

void UpdateTask1(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	Task1_velocity[0] = msg->twist.linear.x;
	Task1_velocity[1] = msg->twist.linear.y;
	Task1_velocity[2] = msg->twist.linear.z;

	flag_task1_newdata = true;
}

void UpdateTask2(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	Task2_velocity[0] = msg->twist.linear.x;
	Task2_velocity[1] = msg->twist.linear.y;
	Task2_velocity[2] = msg->twist.linear.z;

	flag_task2_newdata = true;
}

void UpdateTask3(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	Task3_velocity[0] = msg->twist.linear.x;
	Task3_velocity[1] = msg->twist.linear.y;
	Task3_velocity[2] = msg->twist.linear.z;

	flag_task3_newdata = true;
}

void UpdateTask4(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	Task4_velocity[0] = msg->twist.linear.x;
	Task4_velocity[1] = msg->twist.linear.y;
	Task4_velocity[2] = msg->twist.linear.z;

	flag_task4_newdata = true;
}



/*--------------------------------------------------------------------
 * configCallback()
 * Callback function for dynamic reconfigure server.
 *------------------------------------------------------------------*/

void configCallback(task_adaptation::task_adaptation_params &config, uint32_t level)
{
	// Set class variables to new values. They should match what is input at the dynamic reconfigure GUI.
	std::string message = config.message.c_str();
	int D_gain = config.D_gain;
	int epsilon = config.D_gain;

	ROS_INFO_STREAM("configCallback: received update! messge : " << message << "  D_gain = " << D_gain << "  espsilon = " << epsilon);

}


void UpdateParamCallback(const task_adaptation::task_adaptation_params::ConstPtr &msg)
{
	std::string message = msg->message;
	int D_gain = msg->D_gain;
	int epsilon = msg->epsilon;

	ROS_INFO_STREAM("received update! messge : " << message << "  D_gain = " << D_gain << "  espsilon = " << epsilon);

}