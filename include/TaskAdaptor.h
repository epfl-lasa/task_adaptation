#ifndef TASKADAPTOR_H
#define TASKADAPTOR_H





#include "ros/ros.h"

#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/WrenchStamped.h"

#include <dynamic_reconfigure/server.h>
#include <task_adaptation/task_adaptation_paramsConfig.h>




class TaskAdaptor {

private:

	// ros variables
	ros::NodeHandle nh_;
	ros::Rate loop_rate_;
	ros::Duration disp_rate_;


	ros::Subscriber sub_realVelocity_;

	ros::Subscriber sub_task1_;
	ros::Subscriber sub_task2_;
	ros::Subscriber sub_task3_;
	ros::Subscriber sub_task4_;

	ros::Publisher pub_adapted_velocity_;
	ros::Publisher pub_wrench_control_;

	geometry_msgs::TwistStamped  msgAdaptedVelocity_;
	geometry_msgs::WrenchStamped msgWrenchControl_;

	//dynamic reconfig settig
	dynamic_reconfigure::Server<task_adaptation::task_adaptation_paramsConfig> dyn_rec_srv_;
	dynamic_reconfigure::Server<task_adaptation::task_adaptation_paramsConfig>::CallbackType dyn_rec_f_;


	// task adaptation variables
	std::vector<float> RealVelocity_;
	std::vector<float> DesiredVelocity_;
	std::vector<float> ControlWrench_;


	std::vector<float> Task0_velocity_;
	std::vector<float> Task1_velocity_;
	std::vector<float> Task2_velocity_;
	std::vector<float> Task3_velocity_;
	std::vector<float> Task4_velocity_;

	bool flag_realVelocity_newdata_;
	bool flag_task1_newdata_;
	bool flag_task2_newdata_;
	bool flag_task3_newdata_;
	bool flag_task4_newdata_;

//float belief_task0;
//float belief_task1;
//float belief_task2;
//float belief_task3;
//float belief_task4;

	std::vector<float> Beliefs_;

	std::vector<float> UpdateBeliefsRaw_;
	std::vector<float> UpdateBeliefs_;

	double D_gain_, D_gain_hack_;
	double epsilon_, epsilon_hack_;






public:

	TaskAdaptor(ros::NodeHandle &n,
	            double frequency);

	bool Init();

	void Run();


private:

	void InitClassVariables();

	bool InitROS();

	void SetFlagsFalse();

	bool CheckNewData();

	void ComputeNewBeliefs();

	void PublishAdaptedVelocity();

	void ComputeDesiredForce();

	void PublishDesiredForce();

	void updateRealVelocity(const geometry_msgs::TwistStamped::ConstPtr& msg);
	void updateRealVelocity_world(const geometry_msgs::Twist::ConstPtr& msg);


	void UpdateTask1(const geometry_msgs::TwistStamped::ConstPtr& msg);
	void UpdateTask2(const geometry_msgs::TwistStamped::ConstPtr& msg);
	void UpdateTask3(const geometry_msgs::TwistStamped::ConstPtr& msg);
	void UpdateTask4(const geometry_msgs::TwistStamped::ConstPtr& msg);

	void DynCallback(task_adaptation::task_adaptation_paramsConfig& config, uint32_t level);
// void UpdateParamCallback(const task_adaptation::task_adaptation_params::ConstPtr _msg);



	void DisplayInformation();


	void UpdateDesiredVelocity();

	void RawAdaptation();

	float ComputeInnerSimilarity(float b, std::vector<float> RealVelocity);
	float ComputeOutterSimilarity(std::vector<float> RealVelocity);

//float ComputeTotalInnerSimilarity();


	void WinnerTakeAll();



};















#endif // TASKADAPTOR_H
