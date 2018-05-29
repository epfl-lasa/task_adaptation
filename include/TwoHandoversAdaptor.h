#ifndef TWO_HANDOVERS_ADAPTOR_H
#define TWO_HANDOVERS_ADAPTOR_H


#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Float64MultiArray.h"

#include <dynamic_reconfigure/server.h>
#include <task_adaptation/TwoHandoversAdaptor_paramsConfig.h>


class TwoHandoversAdaptor {

private:

	// ros variables
	ros::NodeHandle nh_;
	ros::Rate loop_rate_;
	ros::Duration disp_rate_;

	ros::Subscriber sub_Target1_velocity_;
	ros::Subscriber sub_Target2_velocity_;
	ros::Subscriber sub_Task1_velocity_;
	ros::Subscriber sub_Task2_velocity_;

	ros::Publisher pub_adapted_velocity_;
	ros::Publisher pub_beliefs_;

	std::vector<float> Target1_velocity_;
	std::vector<float> Target2_velocity_;
	std::vector<float> Task1_velocity_;
	std::vector<float> Task2_velocity_;
	std::vector<float> DesiredVelocity_;


	//dynamic reconfig settig
	dynamic_reconfigure::Server<task_adaptation::TwoHandoversAdaptor_paramsConfig> dyn_rec_srv_;
	dynamic_reconfigure::Server<task_adaptation::TwoHandoversAdaptor_paramsConfig>::CallbackType dyn_rec_f_;




	// vectors to contain beliefs and their updates
	std::vector<bool>  flag_newdata_; // 0 for realvelocity and i for task_i
	std::vector<float> Beliefs_;
	std::vector<float> UpdateBeliefs_;

	// adaptation gains
	double epsilon_;

	ros::Time time_start_;


public:

	TwoHandoversAdaptor(ros::NodeHandle &n,
	                double frequency,
	                std::string topic_real_velocity,
	                std::string topic_task1_velocity,
	                std::string topic_task2_velocity,
	                std::string topic_adapted_velocity,
	                std::string topic_desired_force);


	void Run();


private:

	void Adaptation();
	void ComputeNewBeliefs();

	void UpdateDesiredVelocity();
	void PublishAdaptedVelocityStamped();
	void PublishAdaptedVelocity();

	void UpdateTarget1(const geometry_msgs::Twist::ConstPtr& msg);
	void UpdateTarget2(const geometry_msgs::Twist::ConstPtr& msg);

	void UpdateTask1(const geometry_msgs::TwistStamped::ConstPtr& msg);
	void UpdateTask2(const geometry_msgs::TwistStamped::ConstPtr& msg);

	bool CheckNewData();

	void DynCallback(task_adaptation::TwoHandoversAdaptor_paramsConfig& config, uint32_t level);

	void DisplayInformation();
	void PublishBeliefs();


};

#endif // TWO_HANDOVERS_ADAPTOR_H
