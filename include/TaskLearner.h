#ifndef TASKLEARNER_H
#define TASKLEARNER_H


#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"

#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/WrenchStamped.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"


#include <dynamic_reconfigure/server.h>
#include <task_adaptation/task_learning_paramsConfig.h>


class TaskLearner {

private:

	// ros variables
	ros::NodeHandle nh_;
	ros::Rate loop_rate_;
	ros::Duration disp_rate_;
	ros::Duration publish_beta_rate_;

	ros::Subscriber sub_realPosition_;
	ros::Subscriber sub_realVelocity_;
	ros::Subscriber sub_externalForce_;

	ros::Subscriber sub_task1_;
	ros::Subscriber sub_task2_;
	ros::Subscriber sub_task3_;
	ros::Subscriber sub_task4_;

	ros::Publisher pub_adapted_velocity_;
	ros::Publisher pub_wrench_control_;

	ros::Publisher pub_beliefs_;
	ros::Publisher pub_betas_;
	ros::Publisher pub_alpha2_;



	geometry_msgs::Twist  msgDesiredVelocity_;
// 	geometry_msgs::WrenchStamped msgWrenchControl_;

// 	//dynamic reconfig settig
	dynamic_reconfigure::Server<task_adaptation::task_learning_paramsConfig> dyn_rec_srv_;
	dynamic_reconfigure::Server<task_adaptation::task_learning_paramsConfig>::CallbackType dyn_rec_f_;

	// topic names
	std::string topic_real_position_;
	std::string topic_real_velocity_;
	std::string topic_task1_velocity_;
	std::string topic_task2_velocity_;
	std::string topic_task3_velocity_;
	std::string topic_task4_velocity_;
	std::string topic_adapted_velocity_;
	std::string topic_desired_force_;
	std::string topic_external_force_;


	std::vector<double> x_lim_;
	std::vector<double> y_lim_;
	std::vector<double> z_lim_;
	std::vector<int> N_grid_xyz_;


	// task learning variables
	int N_centeriods_;
	std::vector<std::vector<double>> Centers_;
	std::vector<double> activations_;
	std::vector<std::vector<double>> Beta_;
	std::vector<std::vector<double>> Beta_dot_;



	// task adaptation variables
	std::vector<float> RealPosition_;
	std::vector<float> RealVelocity_;
	std::vector<float> DesiredVelocity_;
	std::vector<float> ControlWrench_;

	// the null primitive always commands zero velocity
	// const std::vector<float> Task0_velocity_ = {0, 0, 0};

	std::vector<float> Task1_velocity_;
	std::vector<float> Task2_velocity_;
	std::vector<float> Task3_velocity_;
	std::vector<float> Task4_velocity_;

	// the flowoing are used to freeze the signals for learning

	std::vector<float> freeze_Task1_velocity_;
	std::vector<float> freeze_Task2_velocity_;
	std::vector<float> freeze_Task3_velocity_;
	std::vector<float> freeze_Task4_velocity_;
	std::vector<float> freeze_RealPosition_;
	std::vector<float> freeze_RealVelocity_;
	std::vector<float> freeze_DesiredVelocity_;



	// vectors to contain beliefs and their updates
	std::vector<bool>  flag_newdata_; // 0 for realvelocity and i for task_i
	std::vector<float> Beliefs_;
	std::vector<float> UpdateBeliefsRaw_;
	std::vector<float> UpdateBeliefs_;

	// adaptation gains
	double epsilon_;
	// double epsilon_hack_;

	// control gain for a simple velocity controller
	double D_gain_;
	// double D_gain_hack_;

	double alpha2_;

	double thresh_ext_force_;
	double ExtForce_norm2_;


public:

	TaskLearner(ros::NodeHandle &n,
	            double frequency,
	            std::string topic_real_position,
	            std::string topic_real_velocity,
	            std::string topic_task1_velocity,
	            std::string topic_task2_velocity,
	            std::string topic_task3_velocity,
	            std::string topic_task4_velocity,
	            std::string topic_adapted_velocity,
	            std::string topic_desired_force,
	            std::vector<double> x_lim,
	            std::vector<double> y_lim,
	            std::vector<double> z_lim,
	            std::vector<int> N_grid_xyz);

	bool Init();

	void Run();


// private:

	void InitClassVariables();
	bool InitROS();

	void ComputeActivation();
	void ComputeBeliefs();

	void ComputeDesiredVelocity();
	void PublishDesiredVelocity();


// 	bool CheckNewData();

// 	void ComputeNewBeliefs();

	void PublishBeliefs();
	void PublishBetas();

	void freezeTheSignals();


// 	void ComputeDesiredForce();

// 	void PublishDesiredForce();

	void updateRealVelocity(const geometry_msgs::Twist::ConstPtr& msg);

	void updateRealPosition(const geometry_msgs::Pose::ConstPtr& msg);

	void UpdateNormExternalForce(const geometry_msgs::WrenchStamped::ConstPtr& msg);


	// void UpdateRealVelocity_world(const geometry_msgs::Twist::ConstPtr& msg);


	void UpdateTask1(const geometry_msgs::TwistStamped::ConstPtr & msg);
	void UpdateTask2(const geometry_msgs::TwistStamped::ConstPtr & msg);
	void UpdateTask3(const geometry_msgs::TwistStamped::ConstPtr & msg);
	void UpdateTask4(const geometry_msgs::TwistStamped::ConstPtr & msg);




	void DynCallback(task_adaptation::task_learning_paramsConfig& config, uint32_t level);
// 	// void UpdateParamCallback(const task_adaptation::task_adaptation_params::ConstPtr _msg);

	void DisplayInformation();


	void RawAdaptation();

	void UpdateBeta();


	float ComputeInnerSimilarity(float b, std::vector<float> RealVelocity);

	float ComputeOutterSimilarity(std::vector<float> RealVelocity);

// 	void WinnerTakeAll();

	void SimpleWinnerTakeAll(); // it just removes the offest between first and second

// 	//float ComputeTotalInnerSimilarity();
};

#endif // TASKLEARNER_H
