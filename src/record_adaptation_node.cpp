#include <iostream>
#include <fstream>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <ros/ros.h>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Accel.h"
#include "geometry_msgs/Quaternion.h"


#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/WrenchStamped.h"
#include "std_msgs/Float64MultiArray.h"


using namespace std;



class AdaptationRecorder {

private:
	ros::NodeHandle nh_;
	ros::Rate loop_rate_;

	std::string recPath_;


	//real variables from the robot
	ofstream file_positionReal_;
	ofstream file_velocityReal_;
	ofstream file_accelerationReal_;
	ofstream file_forceReal_;


	//deireable velocities from motion-generation
	ofstream file_velocityDesired_;
	ofstream file_orientationDesired_;

	//commanded force by the controller
	ofstream file_forceDesired_;
	ofstream file_control_gains_;

	//for the task adaptation module
	ofstream file_velocityTask1_;
	ofstream file_velocityTask2_;
	ofstream file_velocityTask3_;
	ofstream file_velocityTask4_;

	ofstream file_beliefs_;
	ofstream file_adaptation_params_;



	ros::Subscriber sub_positionReal_;
	ros::Subscriber sub_velocityReal_;
	ros::Subscriber sub_accelerationReal_;
	ros::Subscriber sub_force_real_;


	ros::Subscriber sub_velocityDesired_;
	ros::Subscriber sub_orientationDesired_;

	ros::Subscriber sub_force_desired_;

	ros::Subscriber sub_task1_velocity_;
	ros::Subscriber sub_task2_velocity_;
	ros::Subscriber sub_task3_velocity_;
	ros::Subscriber sub_task4_velocity_;

	ros::Subscriber sub_tasks_beliefs_;
	ros::Subscriber sub_adaptation_params_;




public:
	AdaptationRecorder(ros::NodeHandle &n, double frequency)
		: nh_(n), loop_rate_(frequency) {

		ROS_INFO_STREAM("The recorder node is created at: " << nh_.getNamespace() << " with freq: " << frequency << "Hz");

	}

	void Initialize() {

		// Creating a recording directory
		std::string recPath_ = "./Recordings/";
		mkdir(recPath_.c_str(), 0777);

		// Creating a subdirectory for a specific subject
		recPath_ += "TestAdaptation/";
		mkdir(recPath_.c_str(), 0777);

		// Creating a subdirectory for a specific interaction based on time
		time_t rawtime;
		tm* timeinfo;
		char buffer [80];
		time(&rawtime);
		timeinfo = localtime(&rawtime);
		strftime(buffer, 80, "%Y-%m-%d-%H-%M-%S", timeinfo);
		recPath_ += string(buffer);
		mkdir(recPath_.c_str(), 0777);

		cout << "Recording to :" << recPath_.c_str() << endl;

		string recPathFile_positionReal = recPath_ + "/position_real.txt";
		file_positionReal_.open(recPathFile_positionReal);
		file_positionReal_  << "Time" << "\t" << "x" << "\t" << "y" << "\t" << "z"
		                    << "\t" << "rx" << "\t" << "ry" << "\t" << "rz" << "\t" << "rw" << "\n";
		sub_positionReal_ = nh_.subscribe("/lwr/ee_pose" , 1000, &AdaptationRecorder::GetPositionReal, this);

		string recPathFile_velocityReal = recPath_ + "/velocity_real.txt";
		file_velocityReal_.open(recPathFile_velocityReal);
		file_velocityReal_ << "Time" << "\t" << "x" << "\t" << "y" << "\t" << "z"
		                   << "\t" << "rx" << "\t" << "ry" << "\t" << "rz" << "\n";
		sub_velocityReal_ = nh_.subscribe("/lwr/ee_vel" , 1000, &AdaptationRecorder::GetVelocityReal, this);

		string recPathFile_acceleratonReal = recPath_ + "/acceleration_real.txt";
		file_accelerationReal_.open(recPathFile_acceleratonReal);
		file_accelerationReal_ << "Time" << "\t" << "x" << "\t" << "y" << "\t" << "z"
		                       << "\t" << "rx" << "\t" << "ry" << "\t" << "rz" << "\n";
		sub_accelerationReal_ = nh_.subscribe("/lwr/ee_vel" , 1000, &AdaptationRecorder::GetAccelerationReal, this);


		string recPathFile_forceReal = recPath_ + "/force_real.txt";
		file_forceReal_.open(recPathFile_forceReal);
		file_forceReal_ << "Time" << "\t" << "x" << "\t" << "y" << "\t" << "z"
		                << "\t" << "rx" << "\t" << "ry" << "\t" << "rz" << "\n";
		sub_force_real_ = nh_.subscribe("/lwr/joint_controllers/ee_ft" , 1000, &AdaptationRecorder::GetForceReal, this);


		string recPathFile_velocityDesired = recPath_ + "/velocity_desired.txt";
		file_velocityDesired_.open(recPathFile_velocityDesired);
		file_velocityDesired_ << "Time" << "\t" << "x" << "\t" << "y" << "\t" << "z" << "\n";
		sub_velocityDesired_ = nh_.subscribe("/lwr/joint_controllers/passive_ds_command_vel" , 1000, &AdaptationRecorder::GetVelocityDesired, this);

		string recPathFile_orientationsDesired = recPath_ + "/orientation_desired.txt";
		file_orientationDesired_.open(recPathFile_orientationsDesired);
		file_orientationDesired_ << "Time" << "\t" << "rx" << "\t" << "ry" << "\t" << "rz" << "\t" << "rw" << "\n";
		sub_orientationDesired_ = nh_.subscribe("/lwr/joint_controllers/passive_ds_command_orient" , 1000, &AdaptationRecorder::GetOrientationDesired, this);


		string recPathFile_forceDesired = recPath_ + "/force_desired.txt";
		file_forceDesired_.open(recPathFile_forceDesired);
		file_forceDesired_ << "Time" << "\t" << "x" << "\t" << "y" << "\t" << "z"
		                   << "\t" << "rx" << "\t" << "ry" << "\t" << "rz" << "\n";
		sub_force_desired_ = nh_.subscribe("/lwr/joint_controllers/force_desired" , 1000, &AdaptationRecorder::GetForceDesired, this);


		// file_control_gains_ : I will implement this later if needed



		string recPathFile_Task1Vel = recPath_ + "/Task1_velocityDesired.txt";
		file_velocityTask1_.open(recPathFile_Task1Vel);
		file_velocityTask1_ << "Time" << "\t" << "x" << "\t" << "y" << "\t" << "z" << "\n";
		sub_task1_velocity_ = nh_.subscribe("/Task1/filter/desired_velocity" , 1000, &AdaptationRecorder::GetTask1Velocity, this);

		string recPathFile_Task2Vel  = recPath_ + "/Task2_velocityDesired.txt";
		file_velocityTask2_.open(recPathFile_Task2Vel);
		file_velocityTask2_ << "Time" << "\t" << "x" << "\t" << "y" << "\t" << "z" << "\n";
		sub_task2_velocity_ = nh_.subscribe("/Task2/filter/desired_velocity" , 1000, &AdaptationRecorder::GetTask2Velocity, this);


		string recPathFile_Task3Vel  = recPath_ + "/Task3_velocityDesired.txt";
		file_velocityTask3_.open(recPathFile_Task3Vel);
		file_velocityTask3_ << "Time" << "\t" << "x" << "\t" << "y" << "\t" << "z" << "\n";
		sub_task3_velocity_ = nh_.subscribe("/Task3/filter/desired_velocity" , 1000, &AdaptationRecorder::GetTask3Velocity, this);


		string recPathFile_Task4Vel  = recPath_ + "/Task4_velocityDesired.txt";
		file_velocityTask4_.open(recPathFile_Task4Vel);
		file_velocityTask4_ << "Time" << "\t" << "x" << "\t" << "y" << "\t" << "z" << "\n";
		sub_task4_velocity_ = nh_.subscribe("/Task4/filter/desired_velocity" , 1000, &AdaptationRecorder::GetTask4Velocity, this);


		string recPathFile_TaskBelief = recPath_ + "/Tasks_beliefs.txt";
		file_beliefs_.open(recPathFile_TaskBelief);
		file_beliefs_ << "Time" << "\t" << "b0" << "\t" << "b1" << "\t" << "b2" << "\t" << "b3" << "\t" << "b4" << "\n";
		sub_tasks_beliefs_ = nh_.subscribe("/task_adaptation/beliefs" , 1000, &AdaptationRecorder::GetBeliefs, this);

		// string recPathFile_params = recPath_ + "/adaptation_params.txt";
		// file_adaptation_params_.open(recPathFile_params);
		// file_adaptation_params_ << "Time" << "\t" << "epsilon" << "\n";


		// string recPathFile_controlGain = recPath_ + "/control_gains.txt";
		// file_control_gains_.open(recPathFile_controlGain);
		// file_control_gains_ << "Time" << "\t" << "D0" << "\t" << "D1" << "\n";




	}


	void Run() {

		while (nh_.ok()) {

			ros::spinOnce();

			loop_rate_.sleep();
		}

	}


	void GetPositionReal(const geometry_msgs::Pose::ConstPtr& msg)
	{
		file_positionReal_  << ros::Time::now()	<< "\t"
		                    << msg->position.x 	<< "\t"
		                    << msg->position.y 	<< "\t"
		                    << msg->position.z 	<< "\t"
		                    << msg->orientation.x << "\t"
		                    << msg->orientation.y << "\t"
		                    << msg->orientation.z << "\t"
		                    << msg->orientation.w << "\n";
	}

	void GetVelocityReal(const geometry_msgs::Twist::ConstPtr& msg)
	{
		file_velocityReal_  << ros::Time::now()	<< "\t"
		                    << msg->linear.x 	<< "\t"
		                    << msg->linear.y 	<< "\t"
		                    << msg->linear.z 	<< "\t"
		                    << msg->angular.x 	<< "\t"
		                    << msg->angular.y 	<< "\t"
		                    << msg->angular.z 	<< "\n";
	}

	void GetAccelerationReal(const geometry_msgs::Accel::ConstPtr& msg)
	{
		file_accelerationReal_  << ros::Time::now()	<< "\t"
		                        << msg->linear.x 	<< "\t"
		                        << msg->linear.y 	<< "\t"
		                        << msg->linear.z 	<< "\t"
		                        << msg->angular.x 	<< "\t"
		                        << msg->angular.y 	<< "\t"
		                        << msg->angular.z 	<< "\n";
	}

	void GetForceReal(const geometry_msgs::WrenchStamped::ConstPtr& msg)
	{
		file_forceReal_  << ros::Time::now()	<< "\t"
		                 << msg->wrench.force.x 	<< "\t"
		                 << msg->wrench.force.y 	<< "\t"
		                 << msg->wrench.force.z 	<< "\t"
		                 << msg->wrench.torque.x	<< "\t"
		                 << msg->wrench.torque.y	<< "\t"
		                 << msg->wrench.torque.z	<< "\n";
	}

	void GetVelocityDesired(const geometry_msgs::Twist::ConstPtr& msg)
	{
		file_velocityDesired_  << ros::Time::now()	<< "\t"
		                       << msg->linear.x 	<< "\t"
		                       << msg->linear.y 	<< "\t"
		                       << msg->linear.z 	<< "\t"
		                       << msg->angular.x 	<< "\t"
		                       << msg->angular.y 	<< "\t"
		                       << msg->angular.z 	<< "\n";
	}

	void GetOrientationDesired(const geometry_msgs::Quaternion::ConstPtr& msg)
	{
		file_orientationDesired_  << ros::Time::now()	<< "\t"
		                          << msg->x 	<< "\t"
		                          << msg->y 	<< "\t"
		                          << msg->z 	<< "\t"
		                          << msg->w 	<< "\n";
	}

	void GetForceDesired(const geometry_msgs::WrenchStamped::ConstPtr& msg)
	{
		file_forceDesired_  << ros::Time::now()	<< "\t"
		                    << msg->wrench.force.x 	<< "\t"
		                    << msg->wrench.force.y 	<< "\t"
		                    << msg->wrench.force.z 	<< "\t"
		                    << msg->wrench.torque.x	<< "\t"
		                    << msg->wrench.torque.y	<< "\t"
		                    << msg->wrench.torque.z	<< "\n";
	}

	void GetTask1Velocity(const geometry_msgs::TwistStamped::ConstPtr& msg)
	{
		file_velocityTask1_  << ros::Time::now()	<< "\t"
		                     << msg->twist.linear.x 	<< "\t"
		                     << msg->twist.linear.y 	<< "\t"
		                     << msg->twist.linear.z 	<< "\n";
	}

	void GetTask2Velocity(const geometry_msgs::TwistStamped::ConstPtr& msg)
	{
		file_velocityTask2_  << ros::Time::now()	<< "\t"
		                     << msg->twist.linear.x 	<< "\t"
		                     << msg->twist.linear.y 	<< "\t"
		                     << msg->twist.linear.z 	<< "\n";
	}

	void GetTask3Velocity(const geometry_msgs::TwistStamped::ConstPtr& msg)
	{
		file_velocityTask3_  << ros::Time::now()	<< "\t"
		                     << msg->twist.linear.x 	<< "\t"
		                     << msg->twist.linear.y 	<< "\t"
		                     << msg->twist.linear.z 	<< "\n";
	}

	void GetTask4Velocity(const geometry_msgs::TwistStamped::ConstPtr& msg)
	{
		file_velocityTask4_  << ros::Time::now()	<< "\t"
		                     << msg->twist.linear.x 	<< "\t"
		                     << msg->twist.linear.y 	<< "\t"
		                     << msg->twist.linear.z 	<< "\n";
	}

	void GetBeliefs(const std_msgs::Float64MultiArray::ConstPtr& msg)
	{
		file_beliefs_  << ros::Time::now()	<< "\t"
		                     << msg->data[0] 	<< "\t"
		                     << msg->data[1] 	<< "\t"
		                     << msg->data[2] 	<< "\t"
		                     << msg->data[3] 	<< "\t"
		                     << msg->data[4] 	<< "\n";
	}
};






int main(int argc, char **argv)
{
	//Initiate ROS
	ros::init(argc, argv, "record_adaptation");



	ros::NodeHandle nh;
	double frequency = 100.0;
	AdaptationRecorder adaptation_recorder(nh, frequency);

	adaptation_recorder.Initialize();

	adaptation_recorder.Run();

	return 0;
}
