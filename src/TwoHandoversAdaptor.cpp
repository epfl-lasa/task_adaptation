#include "TwoHandoversAdaptor.h"


TwoHandoversAdaptor::TwoHandoversAdaptor(ros::NodeHandle &n,
        double frequency,
        std::string topic_target1_position,
        std::string topic_target2_position,
        std::string topic_target1_velocity,
        std::string topic_target2_velocity,
        std::string topic_Task1_velocity,
        std::string topic_Task2_velocity,
        std::string topic_robot_position,
        std::string topic_robot_velocity,
        std::string topic_adapted_velocity)
	:
	nh_(n),
	loop_rate_(frequency),
	disp_rate_(1) {

	ROS_INFO_STREAM("Handover adaptation node is created at: " << nh_.getNamespace() << " with freq: " << frequency << "Hz");

	sub_Target1_position_ = nh_.subscribe(topic_target1_position, 1,
	                                      &TwoHandoversAdaptor::UpdateTarget1position, this, ros::TransportHints().reliable().tcpNoDelay());
	sub_Target2_position_ = nh_.subscribe(topic_target2_position, 1,
	                                      &TwoHandoversAdaptor::UpdateTarget2position, this, ros::TransportHints().reliable().tcpNoDelay());
	sub_Target1_velocity_ = nh_.subscribe(topic_target1_velocity, 1,
	                                      &TwoHandoversAdaptor::UpdateTarget1velocity, this, ros::TransportHints().reliable().tcpNoDelay());
	sub_Target2_velocity_ = nh_.subscribe(topic_target2_velocity, 1,
	                                      &TwoHandoversAdaptor::UpdateTarget2velocity, this, ros::TransportHints().reliable().tcpNoDelay());
	sub_Task1_velocity_ = nh_.subscribe(topic_Task1_velocity, 1,
	                                    &TwoHandoversAdaptor::UpdateTask1, this, ros::TransportHints().reliable().tcpNoDelay());
	sub_Task2_velocity_ = nh_.subscribe(topic_Task2_velocity, 1,
	                                    &TwoHandoversAdaptor::UpdateTask2, this, ros::TransportHints().reliable().tcpNoDelay());
	sub_robot_position_ = nh_.subscribe(topic_robot_position, 1,
	                                    &TwoHandoversAdaptor::UpdateRobotPosition, this, ros::TransportHints().reliable().tcpNoDelay());
	sub_robot_velocity_ = nh_.subscribe(topic_robot_velocity, 1,
	                                    &TwoHandoversAdaptor::UpdateRobotVelocity, this, ros::TransportHints().reliable().tcpNoDelay());

	pub_adapted_velocity_ = nh_.advertise<geometry_msgs::Twist>(topic_adapted_velocity, 1);
	pub_beliefs_ = nh_.advertise<std_msgs::Float64MultiArray>("beliefs", 1);

	dyn_rec_f_ = boost::bind(&TwoHandoversAdaptor::DynCallback, this, _1, _2);
	dyn_rec_srv_.setCallback(dyn_rec_f_);

	if (nh_.ok()) {
		ros::spinOnce();
		ROS_INFO("The task adaption node is ready.");
	}
	else {
		ROS_ERROR("The ROS node has a problem.");
	}

	// initializing the varibales
	Target1_position_.resize(3);
	Target2_position_.resize(3);

	Target1_velocity_.resize(3);
	Target2_velocity_.resize(3);

	Task1_velocity_.resize(3);
	Task2_velocity_.resize(3);

	Robot_position_.resize(3);
	Robot_velocity_.resize(3);


	DesiredVelocity_.resize(3);

	Beliefs_.resize(2);
	std::fill(Beliefs_.begin(), Beliefs_.end(), 0);
	Beliefs_[0] = 1;

	flag_newdata_.resize(3);
	std::fill(flag_newdata_.begin(), flag_newdata_.end(), false);

	UpdateBeliefs_.resize(2);
	std::fill(UpdateBeliefs_.begin(), UpdateBeliefs_.end(), 0);

	epsilon_ = 3;
	posVelBias_ = 0;
	alpha_ = 0;


}





void TwoHandoversAdaptor::Run() {

	time_start_ = ros::Time::now();
	ros::Time time_display = ros::Time::now() + disp_rate_;

	while (nh_.ok()) {

		if ( (ros::Time::now() - time_display).toSec() > 0  )
		{
			DisplayInformation();
			time_display = ros::Time::now() + disp_rate_;
		}


		if (CheckNewData() && 1) {


			UpdateDesiredVelocity();

			Adaptation();

			ComputeNewBeliefs();
		}

		PublishBeliefs();


		UpdateDesiredVelocity();


		PublishAdaptedVelocity();


		ros::spinOnce();
		loop_rate_.sleep();
	}
}



bool TwoHandoversAdaptor::CheckNewData() {

	bool flagAdapt = true;

	for (int i = 0; i < flag_newdata_.size(); i++) {
		flagAdapt &= flag_newdata_[i];
	}

	if (flagAdapt) {
		std::fill(flag_newdata_.begin(), flag_newdata_.end(), false);
		return true;
	}

	return false;

}

// ---------------------------------------------------------------------------
//------------------- The adaptation takes place in this function --------
// ---------------------------------------------------------------------------

void TwoHandoversAdaptor::Adaptation() {

	std::fill(UpdateBeliefs_.begin(), UpdateBeliefs_.end(), 0);

	// for (int i = 0; i < 3; i++) {
	// 	double d1 = (Task1_velocity_[i] + Target1_velocity_[i]);
	// 	UpdateBeliefs_[0] -= d1 * d1;

	// 	double d2 = (Task2_velocity_[i] + Target2_velocity_[i]);
	// 	UpdateBeliefs_[1] -= d2 * d2;
	// }


	// update based on velocity information
	for (int i = 0; i < 3; i++) {
		UpdateBeliefs_[0] += (-1 * Task1_velocity_[i] ) * Target1_velocity_[i];

		UpdateBeliefs_[1] += (-1 * Task2_velocity_[i] ) * Target2_velocity_[i];
	}


	// update based on velocity information
	double d1 = 0;
	double d2 = 0;

	for (int i = 0; i < 3; i++) {
		d1 += Target1_position_[i] * Target1_position_[i];
		d2 += Target2_position_[i] * Target2_position_[i];
	}

	UpdateBeliefs_[0] += posVelBias_ * exp(-alpha_ * d1);
	UpdateBeliefs_[1] += posVelBias_ * exp(-alpha_ * d2);



	// winner take all
	double offset = 0.5 * (UpdateBeliefs_[0] + UpdateBeliefs_[1]);

	for (int i = 0; i < UpdateBeliefs_.size(); i++) {
		UpdateBeliefs_[i] -= offset;
	}

}


void TwoHandoversAdaptor::ComputeNewBeliefs() {

	for (int i = 0; i < Beliefs_.size(); i++)
	{
		Beliefs_[i] += epsilon_ * UpdateBeliefs_[i];

		if (Beliefs_[i] > 1)
			Beliefs_[i] = 1;

		if (Beliefs_[i] < 0)
			Beliefs_[i] = 0;
	}

	double sum_b = 0;

	for (int i = 0; i < Beliefs_.size(); i++) {
		sum_b += Beliefs_[i];
	}

	for (int i = 0; i < Beliefs_.size(); i++) {
		Beliefs_[i] /= sum_b;
	}


}

void TwoHandoversAdaptor::PublishBeliefs() {

	std_msgs::Float64MultiArray msg;

	msg.data.clear();

	for (int i = 0; i <= 4; i++) {
		msg.data.push_back(Beliefs_[i]);
	}

	pub_beliefs_.publish(msg);

}


void TwoHandoversAdaptor::UpdateDesiredVelocity()
{
	// starting to zero
	std::fill(DesiredVelocity_.begin(), DesiredVelocity_.end(), 0);

	for (int dim = 0 ; dim < 3 ; dim++)
	{
		DesiredVelocity_[dim] += Beliefs_[0] * Task1_velocity_[dim];
		DesiredVelocity_[dim] += Beliefs_[1] * Task2_velocity_[dim];
	}

}

// ---------------------------------------------------------------------------
//------------------- Receiving and sending signals --------------------------
// ---------------------------------------------------------------------------


void TwoHandoversAdaptor::PublishAdaptedVelocityStamped() {

	geometry_msgs::TwistStamped  msg;
	msg.header.stamp = ros::Time::now();
	msg.twist.linear.x = DesiredVelocity_[0];
	msg.twist.linear.y = DesiredVelocity_[1];
	msg.twist.linear.z = DesiredVelocity_[2];
	pub_adapted_velocity_.publish(msg);

}

void TwoHandoversAdaptor::PublishAdaptedVelocity() {

	geometry_msgs::Twist  msg;

	msg.linear.x = DesiredVelocity_[0];
	msg.linear.y = DesiredVelocity_[1];
	msg.linear.z = DesiredVelocity_[2];

	pub_adapted_velocity_.publish(msg);


}

void TwoHandoversAdaptor::UpdateTarget1position(const geometry_msgs::Pose::ConstPtr& msg) {

	Target1_position_[0] = msg->position.x - Robot_position_[0];
	Target1_position_[1] = msg->position.y - Robot_position_[1];
	Target1_position_[2] = msg->position.z - Robot_position_[2];

}

void TwoHandoversAdaptor::UpdateTarget2position(const geometry_msgs::Pose::ConstPtr& msg) {

	Target2_position_[0] = msg->position.x - Robot_position_[0];
	Target2_position_[1] = msg->position.y - Robot_position_[1];
	Target2_position_[2] = msg->position.z - Robot_position_[2];

}


void TwoHandoversAdaptor::UpdateTarget1velocity(const geometry_msgs::Twist::ConstPtr& msg) {

	Target1_velocity_[0] = msg->linear.x - Robot_velocity_[0];
	Target1_velocity_[1] = msg->linear.y - Robot_velocity_[1];
	Target1_velocity_[2] = msg->linear.z - Robot_velocity_[2];

	flag_newdata_[0] = true;
}

void TwoHandoversAdaptor::UpdateTarget2velocity(const geometry_msgs::Twist::ConstPtr& msg) {

	Target2_velocity_[0] = msg->linear.x - Robot_velocity_[0];
	Target2_velocity_[1] = msg->linear.y - Robot_velocity_[1];
	Target2_velocity_[2] = msg->linear.z - Robot_velocity_[2];

	flag_newdata_[1] = true;
}

void TwoHandoversAdaptor::UpdateTask1(const geometry_msgs::TwistStamped::ConstPtr& msg) {

	Task1_velocity_[0] = msg->twist.linear.x;
	Task1_velocity_[1] = msg->twist.linear.y;
	Task1_velocity_[2] = msg->twist.linear.z;

	flag_newdata_[2] = true;
}

void TwoHandoversAdaptor::UpdateTask2(const geometry_msgs::TwistStamped::ConstPtr& msg) {

	Task2_velocity_[0] = msg->twist.linear.x;
	Task2_velocity_[1] = msg->twist.linear.y;
	Task2_velocity_[2] = msg->twist.linear.z;

	flag_newdata_[3] = true;
}

void TwoHandoversAdaptor::UpdateRobotPosition(const geometry_msgs::Pose::ConstPtr& msg) {
	Robot_position_[0] = msg->position.x;
	Robot_position_[1] = msg->position.y;
	Robot_position_[2] = msg->position.z;

}
void TwoHandoversAdaptor::UpdateRobotVelocity(const geometry_msgs::Twist::ConstPtr& msg) {

	Robot_velocity_[0] = msg->linear.x;
	Robot_velocity_[1] = msg->linear.y;
	Robot_velocity_[2] = msg->linear.z;

}


/*--------------------------------------------------------------------
 * configCallback()
 * Callback function for dynamic reconfigure server.
 *------------------------------------------------------------------*/

void TwoHandoversAdaptor::DynCallback(task_adaptation::TwoHandoversAdaptor_paramsConfig& config, uint32_t level)
{
	// Set class variables to new values. They should match what is input at the dynamic reconfigure GUI.
	epsilon_ = config.epsilon;
	posVelBias_ = config.bias;
	alpha_ = config.alpha;

	ROS_INFO_STREAM("configCallback: received update!  espsilon = " << epsilon_);

}

/*--------------------------------------------------------------------
 * Display relevant information in the console
 *------------------------------------------------------------------*/

void TwoHandoversAdaptor::DisplayInformation()
{
	//std::cout << RealVelocity[0] << "\t" <<  RealVelocity[1] << "\t" << RealVelocity[2]  << std::endl;

	std::cout << "Time = " << ros::Time::now() - time_start_ << std::endl;

	std::cout << "Task 1 velocity = [" << Task1_velocity_[0] << " , " << Task1_velocity_[1] << " , " << Task1_velocity_[2] << "]" << std::endl;
	std::cout << "Task 2 velocity = [" << Task2_velocity_[0] << " , " << Task2_velocity_[1] << " , " << Task2_velocity_[2] << "]" << std::endl;

	std::cout << "Adapted velocity = [" << DesiredVelocity_[0] << " , " << DesiredVelocity_[1] << " , " << DesiredVelocity_[2] << "]" << std::endl;

	std::cout << "Adaptation rate  = " << epsilon_ << std::endl;

	std::cout << "Task 1 : b =" << Beliefs_[0] <<  "\t db = " << UpdateBeliefs_[0] <<  std::endl;
	std::cout << "Task 2 : b =" << Beliefs_[1] <<  "\t db = " << UpdateBeliefs_[1] <<  std::endl;


	if (!flag_newdata_[0])
		std::cout << "Target 1 velocity is not received " << std::endl;

	if (!flag_newdata_[1])
		std::cout << "Target 2 velocity is not received " << std::endl;

	if (!flag_newdata_[2])
		std::cout << "Task 1 velocity is not received " << std::endl;

	if (!flag_newdata_[3])
		std::cout << "Task 2 velocity is not received " << std::endl;

	std::cout << "----------------------------------------------------- " << std::endl << std::endl;

}