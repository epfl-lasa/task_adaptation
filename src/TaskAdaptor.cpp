#include "TaskAdaptor.h"


TaskAdaptor::TaskAdaptor(ros::NodeHandle &n,
                         double frequency,
                         std::string topic_real_velocity,
                         std::string topic_task1_velocity,
                         std::string topic_task2_velocity,
                         std::string topic_task3_velocity,
                         std::string topic_task4_velocity,
                         std::string topic_adapted_velocity,
                         std::string topic_desired_force)
	: nh_(n),
	  loop_rate_(frequency),
	  disp_rate_(0.4),
	  topic_real_velocity_(topic_real_velocity),
	  topic_task1_velocity_(topic_task1_velocity),
	  topic_task2_velocity_(topic_task2_velocity),
	  topic_task3_velocity_(topic_task3_velocity),
	  topic_task4_velocity_(topic_task4_velocity),
	  topic_adapted_velocity_(topic_adapted_velocity),
	  topic_desired_force_(topic_desired_force) {

	ROS_INFO_STREAM("Task adaptation node is created at: " << nh_.getNamespace() << " with freq: " << frequency << "Hz");
}




bool TaskAdaptor::Init() {

	InitClassVariables();

	if (!InitROS()) {
		ROS_ERROR_STREAM("ERROR intializing the ROS node");
		return false;
	}

	ROS_INFO("Task adaptation node is intialized.");
	return true;
}

void TaskAdaptor::Run() {

	ros::Time::now();
	ros::Time time_display = ros::Time::now() + disp_rate_;

	while (nh_.ok()) {

		if ( (ros::Time::now() - time_display).toSec() > 0  )
		{
			DisplayInformation();
			time_display = ros::Time::now() + disp_rate_;
		}

		if (CheckNewData() && 1) {

			UpdateDesiredVelocity();

			RawAdaptation();

			WinnerTakeAll();

			ComputeNewBeliefs();

			PublishBeliefs();
		}

		PublishBeliefs();


		UpdateDesiredVelocity();

		PublishAdaptedVelocity();

		// D_gain_hack_ = (1 - Beliefs_[0]) * D_gain_;

		ComputeDesiredForce();

		PublishDesiredForce();

		ros::spinOnce();

		loop_rate_.sleep();
	}
}


bool TaskAdaptor::InitROS() {

	sub_realVelocity_ = nh_.subscribe(topic_real_velocity_, 1000,
	                                  &TaskAdaptor::updateRealVelocity, this, ros::TransportHints().reliable().tcpNoDelay());

	sub_task1_ = nh_.subscribe(topic_task1_velocity_, 1000, &TaskAdaptor::UpdateTask1, this, ros::TransportHints().reliable().tcpNoDelay());
	sub_task2_ = nh_.subscribe(topic_task2_velocity_, 1000, &TaskAdaptor::UpdateTask2, this, ros::TransportHints().reliable().tcpNoDelay());
	sub_task3_ = nh_.subscribe(topic_task3_velocity_, 1000, &TaskAdaptor::UpdateTask3, this, ros::TransportHints().reliable().tcpNoDelay());
	sub_task4_ = nh_.subscribe(topic_task4_velocity_, 1000, &TaskAdaptor::UpdateTask4, this, ros::TransportHints().reliable().tcpNoDelay());

	pub_adapted_velocity_ = nh_.advertise<geometry_msgs::TwistStamped>(topic_adapted_velocity_, 1);
	pub_wrench_control_   = nh_.advertise<geometry_msgs::WrenchStamped>(topic_desired_force_, 1);
	pub_beliefs_ = nh_.advertise<std_msgs::Float64MultiArray>("beliefs", 1);

	dyn_rec_f_ = boost::bind(&TaskAdaptor::DynCallback, this, _1, _2);
	dyn_rec_srv_.setCallback(dyn_rec_f_);


	if (nh_.ok()) {
		ros::spinOnce();
		ROS_INFO("The task adaption node is ready.");
		return true;
	}
	else {
		ROS_ERROR("The ROS node has a problem.");
		return false;
	}
}


void TaskAdaptor::InitClassVariables() {

	// initializing the varibales
	RealVelocity_.resize(3);
	DesiredVelocity_.resize(3);
	ControlWrench_.resize(6);

	Task1_velocity_.resize(3);
	Task2_velocity_.resize(3);
	Task3_velocity_.resize(3);
	Task4_velocity_.resize(3);

	Beliefs_.resize(5);
	std::fill(Beliefs_.begin(), Beliefs_.end(), 0);
	Beliefs_[0] = 1;

	flag_newdata_.resize(5);
	std::fill(flag_newdata_.begin(), flag_newdata_.end(), false);

	UpdateBeliefsRaw_.resize(5);
	std::fill(UpdateBeliefsRaw_.begin(), UpdateBeliefsRaw_.end(), 0);

	UpdateBeliefs_.resize(5);
	std::fill(UpdateBeliefs_.begin(), UpdateBeliefs_.end(), 0);

	D_gain_  = 0;
	epsilon_ = 0;
	// epsilon_hack_ = epsilon_;
	// D_gain_hack_ = D_gain_;

}


bool TaskAdaptor::CheckNewData() {

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

void TaskAdaptor::ComputeNewBeliefs() {

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


	// if (Beliefs[0] > 0.5 )
	// 	epsilon_hack = epsilon * 5;
	// else
	// 	epsilon_hack = epsilon;

}

void TaskAdaptor::PublishBeliefs() {

	std_msgs::Float64MultiArray msg;

	msg.data.clear();

	for (int i = 0; i <= 4; i++) {
		msg.data.push_back(Beliefs_[i]);
	}

	msg.data.push_back(Beliefs_[3] + Beliefs_[4]);

	pub_beliefs_.publish(msg);

}


void TaskAdaptor::ComputeDesiredForce() {


	ControlWrench_[0] = -D_gain_ * (RealVelocity_[0] - DesiredVelocity_[0]);
	ControlWrench_[1] = -D_gain_ * (RealVelocity_[1] - DesiredVelocity_[1]);
	ControlWrench_[2] = -D_gain_ * (RealVelocity_[2] - DesiredVelocity_[2]);

	// if(ControlWrench[0] < -0.6)
	// 	ControlWrench[0] = -0.6;
	// if(ControlWrench[1] < -0.6)
	// 	ControlWrench[1] = -0.6;
	// if(ControlWrench[2] < -0.6)
	// 	ControlWrench[2] = -0.6;

	// if(ControlWrench[0] > 0.6)
	// 	ControlWrench[0] = 0.6;
	// if(ControlWrench[1] > 0.6)
	// 	ControlWrench[1] = 0.6;
	// if(ControlWrench[2] > 0.6)
	// 	ControlWrench[2] = 0.6;
}

void TaskAdaptor::PublishDesiredForce() {

	msgWrenchControl_.header.stamp = ros::Time::now();
	msgWrenchControl_.header.frame_id = "ur5_arm_base_link"; // just for visualization
	msgWrenchControl_.wrench.force.x = 0;//ControlWrench_[0];
	msgWrenchControl_.wrench.force.y = 0;//ControlWrench_[1];
	msgWrenchControl_.wrench.force.z = 0;//ControlWrench_[2];
	msgWrenchControl_.wrench.torque.x = 0;
	msgWrenchControl_.wrench.torque.y = 0;
	msgWrenchControl_.wrench.torque.z = 0;

	pub_wrench_control_.publish(msgWrenchControl_);

}


void TaskAdaptor::PublishAdaptedVelocity() {

	// msgAdaptedVelocity_.header.stamp = ros::Time::now();
	// msgAdaptedVelocity_.twist.linear.x = DesiredVelocity_[0];
	// msgAdaptedVelocity_.twist.linear.y = DesiredVelocity_[1];
	// msgAdaptedVelocity_.twist.linear.z = DesiredVelocity_[2];

	msgAdaptedVelocity_.header.stamp = ros::Time::now();
	msgAdaptedVelocity_.header.frame_id = "ur5_arm_base_link";
	msgAdaptedVelocity_.twist.linear.x = DesiredVelocity_[0];
	msgAdaptedVelocity_.twist.linear.y = DesiredVelocity_[1];
	msgAdaptedVelocity_.twist.linear.z = DesiredVelocity_[2];

	pub_adapted_velocity_.publish(msgAdaptedVelocity_);
}


void TaskAdaptor::updateRealVelocity(const geometry_msgs::Twist::ConstPtr& msg)
{

	RealVelocity_[0] = msg->linear.x;
	RealVelocity_[1] = msg->linear.y;
	RealVelocity_[2] = msg->linear.z;

	flag_newdata_[0] = true;

}


/*--------------------------------------------------------------------
 * Reading the new desired velocity of each task
 * and setting the flags to true for receiving the new data points
 *------------------------------------------------------------------*/

void TaskAdaptor::UpdateTask1(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	Task1_velocity_[0] = msg->twist.linear.x;
	Task1_velocity_[1] = msg->twist.linear.y;
	Task1_velocity_[2] = msg->twist.linear.z;

	flag_newdata_[1] = true;
}

void TaskAdaptor::UpdateTask2(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	Task2_velocity_[0] = msg->twist.linear.x;
	Task2_velocity_[1] = msg->twist.linear.y;
	Task2_velocity_[2] = msg->twist.linear.z;

	flag_newdata_[2] = true;
}

void TaskAdaptor::UpdateTask3(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	Task3_velocity_[0] = msg->twist.linear.x;
	Task3_velocity_[1] = msg->twist.linear.y;
	Task3_velocity_[2] = msg->twist.linear.z;

	flag_newdata_[3] = true;
}

void TaskAdaptor::UpdateTask4(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	Task4_velocity_[0] = msg->twist.linear.x;
	Task4_velocity_[1] = msg->twist.linear.y;
	Task4_velocity_[2] = msg->twist.linear.z;

	flag_newdata_[4] = true;
}


/*--------------------------------------------------------------------
 * configCallback()
 * Callback function for dynamic reconfigure server.
 *------------------------------------------------------------------*/

void TaskAdaptor::DynCallback(task_adaptation::task_adaptation_paramsConfig& config, uint32_t level)
{
	// Set class variables to new values. They should match what is input at the dynamic reconfigure GUI.
	D_gain_ = config.D_gain;
	epsilon_ = config.epsilon;

	ROS_INFO_STREAM("configCallback: received update! D_gain = " << D_gain_ << "  espsilon = " << epsilon_ );

}


void TaskAdaptor::UpdateDesiredVelocity()
{
	// starting to zero
	std::fill(DesiredVelocity_.begin(), DesiredVelocity_.end(), 0);

	for (int dim = 0 ; dim < 3 ; dim++)
	{
		DesiredVelocity_[dim] += Beliefs_[1] * Task1_velocity_[dim];
		DesiredVelocity_[dim] += Beliefs_[2] * Task2_velocity_[dim];
		DesiredVelocity_[dim] += Beliefs_[3] * Task3_velocity_[dim];
		DesiredVelocity_[dim] += Beliefs_[4] * Task4_velocity_[dim];
	}

}


// ---------------------------------------------------------------------------
//------------------- The raw adaptation takes place in this function --------
// ---------------------------------------------------------------------------

void TaskAdaptor::RawAdaptation()
{
	std::fill(UpdateBeliefsRaw_.begin(), UpdateBeliefsRaw_.end(), 0);

	double NullinnterSimilarity = 0;
	double TempInnerSimilarity;


	UpdateBeliefsRaw_[1] -= ComputeOutterSimilarity(Task1_velocity_);
	TempInnerSimilarity   = 2 * ComputeInnerSimilarity(Beliefs_[1], Task1_velocity_);
	UpdateBeliefsRaw_[1] -= TempInnerSimilarity;

	if (TempInnerSimilarity > NullinnterSimilarity) {
		NullinnterSimilarity = TempInnerSimilarity;
	}

	UpdateBeliefsRaw_[2] -= ComputeOutterSimilarity(Task2_velocity_);
	TempInnerSimilarity = 2 * ComputeInnerSimilarity(Beliefs_[2], Task2_velocity_);
	UpdateBeliefsRaw_[2] -= TempInnerSimilarity;

	if (TempInnerSimilarity > NullinnterSimilarity) {
		NullinnterSimilarity = TempInnerSimilarity;
	}

	UpdateBeliefsRaw_[3] -= ComputeOutterSimilarity(Task3_velocity_);
	TempInnerSimilarity   = 2 * ComputeInnerSimilarity(Beliefs_[3], Task3_velocity_);
	UpdateBeliefsRaw_[3] -= TempInnerSimilarity;

	if (TempInnerSimilarity > NullinnterSimilarity) {
		NullinnterSimilarity = TempInnerSimilarity;
	}

	UpdateBeliefsRaw_[4] -= ComputeOutterSimilarity(Task4_velocity_);
	TempInnerSimilarity   = 2 * ComputeInnerSimilarity(Beliefs_[4], Task4_velocity_);
	UpdateBeliefsRaw_[4] -= TempInnerSimilarity;

	if (TempInnerSimilarity > NullinnterSimilarity) {
		NullinnterSimilarity = TempInnerSimilarity;
	}

	UpdateBeliefsRaw_[0] -= ComputeOutterSimilarity(Task0_velocity_);
	UpdateBeliefsRaw_[0] -= NullinnterSimilarity;

	if (Beliefs_[0] < 0.2) {
		UpdateBeliefsRaw_[0] -= 1000;
	}

//	UpdateBeliefsRaw[0] -= 0.25 * ComputeInnerSimilarity(Beliefs[1],Task1_velocity);
//	UpdateBeliefsRaw[0] -= 0.25 * ComputeInnerSimilarity(Beliefs[2],Task2_velocity);
//	UpdateBeliefsRaw[0] -= 0.25 * ComputeInnerSimilarity(Beliefs[3],Task3_velocity);
//	UpdateBeliefsRaw[0] -= 0.25 * ComputeInnerSimilarity(Beliefs[4],Task4_velocity);

//	UpdateBeliefsRaw[0] += (Beliefs[1] - 1.0) * UpdateBeliefsRaw[1];
//	UpdateBeliefsRaw[0] += (Beliefs[2] - 1.0) * UpdateBeliefsRaw[2];
//	UpdateBeliefsRaw[0] += (Beliefs[3] - 1.0) * UpdateBeliefsRaw[3];
//	UpdateBeliefsRaw[0] += (Beliefs[4] - 1.0) * UpdateBeliefsRaw[4];

}


void TaskAdaptor::WinnerTakeAll()
{
	// initializing the updates for the beliefs at zero
	std::fill(UpdateBeliefs_.begin(), UpdateBeliefs_.end(), 0);

	// fining the winner who has the biggest value for UpdateBeliefsRaw
	int winner_index = 0;

	for (int i = 1; i < UpdateBeliefsRaw_.size(); i++)
	{
		if (UpdateBeliefsRaw_[i] > UpdateBeliefsRaw_[winner_index])
			winner_index = i;
	}

	// no update is required if the winner is already saturated
	if (Beliefs_[winner_index] == 1)
		return;

	int runnerUp_index = 0;

	if (winner_index == 0) {
		runnerUp_index = 1;
	}

	for (int i = 0; i < UpdateBeliefsRaw_.size(); i++)
	{
		if (i ==  winner_index) {
			continue;
		}

		if (UpdateBeliefsRaw_[i] > UpdateBeliefsRaw_[runnerUp_index]) {
			runnerUp_index = i;
		}
	}

	// computing the middle point and removing form all raw updates
	float offset = 0.5 * (UpdateBeliefsRaw_[winner_index] + UpdateBeliefsRaw_[runnerUp_index]);

	for (int i = 0; i < UpdateBeliefsRaw_.size(); i++) {
		UpdateBeliefsRaw_[i] -= offset;
	}


	// computing the sum of updates and setting to zero for active one so we keep the sum beliefs at 1.
	float UpdateSum = 0;

	for (int i = 0; i < UpdateBeliefsRaw_.size(); i++) {
		if (Beliefs_[i] != 0 || UpdateBeliefsRaw_[i] > 0)
			UpdateSum += UpdateBeliefsRaw_[i];
	}

	for (int i = 0; i < UpdateBeliefsRaw_.size(); i++) 	{
		UpdateBeliefs_[i] = UpdateBeliefsRaw_[i];
	}

	UpdateBeliefs_[winner_index] -= UpdateSum;

}


float TaskAdaptor::ComputeInnerSimilarity(float b, std::vector<float> task_velocity) {

	std::vector<float> OtherTasks;
	OtherTasks.resize(3);

	OtherTasks[0] = DesiredVelocity_[0] - b * task_velocity[0];
	OtherTasks[1] = DesiredVelocity_[1] - b * task_velocity[1];
	OtherTasks[2] = DesiredVelocity_[2] - b * task_velocity[2];

	float innerSimilarity = 0;

	innerSimilarity += OtherTasks[0] * task_velocity[0];
	innerSimilarity += OtherTasks[1] * task_velocity[1];
	innerSimilarity += OtherTasks[2] * task_velocity[2];

	return innerSimilarity;

}


float TaskAdaptor::ComputeOutterSimilarity(std::vector<float> task_velocity) {

	float outterSimiliary = 0;

	outterSimiliary += (RealVelocity_[0] - task_velocity[0]) * (RealVelocity_[0] - task_velocity[0]);
	outterSimiliary += (RealVelocity_[1] - task_velocity[1]) * (RealVelocity_[1] - task_velocity[1]);
	outterSimiliary += (RealVelocity_[2] - task_velocity[2]) * (RealVelocity_[2] - task_velocity[2]);

	return outterSimiliary;
}


/*--------------------------------------------------------------------
 * Display relevant information in the console
 *------------------------------------------------------------------*/

void TaskAdaptor::DisplayInformation()
{
	//std::cout << RealVelocity[0] << "\t" <<  RealVelocity[1] << "\t" << RealVelocity[2]  << std::endl;

	std::cout << "Time = " << ros::Time::now() << std::endl;

	if (!flag_newdata_[0])
		std::cout << "Real velocity is not received " << std::endl;

	if (!flag_newdata_[1])
		std::cout << "Task1 velocity is not received " << std::endl;

	if (!flag_newdata_[2])
		std::cout << "Task2 velocity is not received " << std::endl;

	if (!flag_newdata_[3])
		std::cout << "Task3 velocity is not received " << std::endl;

	if (!flag_newdata_[4])
		std::cout << "Task4 velocity is not received " << std::endl;

//	std::cout << "Beliefs are :  b0= " << Beliefs[0] <<
//			                 "\t b1= " << Beliefs[1] <<
//							 "\t b2= " << Beliefs[2] <<
//							 "\t b3= " << Beliefs[3] <<
//							 "\t b4= " << Beliefs[4] << std::endl;

	std::cout << "Real    velocity = [" << RealVelocity_[0] 	<< " , " << RealVelocity_[1] << " , " << RealVelocity_[2] << "]" << std::endl;
	std::cout << "Adapted velocity = [" << DesiredVelocity_[0] << " , " << DesiredVelocity_[1] << " , " << DesiredVelocity_[2] << "]" << std::endl;
	std::cout << "Control forces   = [" << ControlWrench_[0] << " , " << ControlWrench_[1] << " , " << ControlWrench_[2] << "]" << std::endl;

	std::cout << "Adaptation rate  = " << epsilon_ << " Control gain  = " << D_gain_ << std::endl;


	std::cout << "Task 0 : b =" << Beliefs_[0] << "\t db_hat = " << UpdateBeliefsRaw_[0] << "\t db = " << UpdateBeliefs_[0] <<  std::endl;
	std::cout << "Task 1 : b =" << Beliefs_[1] << "\t db_hat = " << UpdateBeliefsRaw_[1] << "\t db = " << UpdateBeliefs_[1] <<  std::endl;
	std::cout << "Task 2 : b =" << Beliefs_[2] << "\t db_hat = " << UpdateBeliefsRaw_[2] << "\t db = " << UpdateBeliefs_[2] <<  std::endl;
	std::cout << "Task 3 : b =" << Beliefs_[3] << "\t db_hat = " << UpdateBeliefsRaw_[3] << "\t db = " << UpdateBeliefs_[3] <<  std::endl;
	std::cout << "Task 4 : b =" << Beliefs_[4] << "\t db_hat = " << UpdateBeliefsRaw_[4] << "\t db = " << UpdateBeliefs_[4] <<  std::endl;

	std::cout << "----------------------------------------------------- " << std::endl << std::endl;
}
