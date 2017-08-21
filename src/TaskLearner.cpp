#include "TaskLearner.h"


TaskLearner::TaskLearner(ros::NodeHandle &n,
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
                         std::vector<int> N_grid_xyz )
	: nh_(n),
	  loop_rate_(frequency),
	  disp_rate_(0.4),
	  topic_real_position_(topic_real_position),
	  topic_real_velocity_(topic_real_velocity),
	  topic_task1_velocity_(topic_task1_velocity),
	  topic_task2_velocity_(topic_task2_velocity),
	  topic_task3_velocity_(topic_task3_velocity),
	  topic_task4_velocity_(topic_task4_velocity),
	  topic_adapted_velocity_(topic_adapted_velocity),
	  topic_desired_force_(topic_desired_force),
	  x_lim_(x_lim),
	  y_lim_(y_lim),
	  z_lim_(z_lim),
	  N_grid_xyz_(N_grid_xyz) {

	ROS_INFO_STREAM("Task-Learning node is created at: " << nh_.getNamespace() << " with freq: " << frequency << "Hz");
}




bool TaskLearner::Init() {

	InitClassVariables();

	if (!InitROS()) {
		ROS_ERROR_STREAM("ERROR intializing the ROS node");
		return false;
	}

	ROS_INFO("Task-Learning node is intialized.");
	return true;
}

void TaskLearner::Run() {

	ros::Time::now();
	ros::Time time_display = ros::Time::now() + disp_rate_;

	ROS_INFO("Task-Learning node is running ...");

	while (nh_.ok()) {

		if ( (ros::Time::now() - time_display).toSec() > 0  ) {
			time_display = ros::Time::now() + disp_rate_;
			DisplayInformation();
			PublishBetas();
		}

		ComputeActivation();

		ComputeBeliefs();
		PublishBeliefs();

		ComputeDesiredVelocity();
		PublishDesiredVelocity();

		RawAdaptation();
		SimpleWinnerTakeAll();
		UpdateBeta();


		ros::spinOnce();
		loop_rate_.sleep();
	}
}


bool TaskLearner::InitROS() {

	sub_realVelocity_ = nh_.subscribe(topic_real_velocity_, 1000,
	                                  &TaskLearner::updateRealVelocity, this, ros::TransportHints().reliable().tcpNoDelay());

	sub_realPosition_ = nh_.subscribe(topic_real_position_, 1000,
	                                  &TaskLearner::updateRealPosition, this, ros::TransportHints().reliable().tcpNoDelay());

	sub_task1_ = nh_.subscribe(topic_task1_velocity_, 1000, &TaskLearner::UpdateTask1, this, ros::TransportHints().reliable().tcpNoDelay());
	sub_task2_ = nh_.subscribe(topic_task2_velocity_, 1000, &TaskLearner::UpdateTask2, this, ros::TransportHints().reliable().tcpNoDelay());
	sub_task3_ = nh_.subscribe(topic_task3_velocity_, 1000, &TaskLearner::UpdateTask3, this, ros::TransportHints().reliable().tcpNoDelay());
	sub_task4_ = nh_.subscribe(topic_task4_velocity_, 1000, &TaskLearner::UpdateTask4, this, ros::TransportHints().reliable().tcpNoDelay());

	// ONE TOPIC TO READ REAL POSITION


	pub_adapted_velocity_ = nh_.advertise<geometry_msgs::Twist>(topic_adapted_velocity_, 1);
	pub_wrench_control_   = nh_.advertise<geometry_msgs::WrenchStamped>(topic_desired_force_, 1);

	pub_beliefs_ = nh_.advertise<std_msgs::Float64MultiArray>("beliefs", 1);
	pub_betas_   = nh_.advertise<std_msgs::Float64MultiArray>("betas", 1);

	dyn_rec_f_ = boost::bind(&TaskLearner::DynCallback, this, _1, _2);
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


void TaskLearner::InitClassVariables() {

	// initializing the varibales

	RealPosition_.resize(3);
	RealVelocity_.resize(3);
	DesiredVelocity_.resize(3);
	ControlWrench_.resize(6);

	Task1_velocity_.resize(3);
	Task2_velocity_.resize(3);
	Task3_velocity_.resize(3);
	Task4_velocity_.resize(3);

	Beliefs_.resize(4);
	std::fill(Beliefs_.begin(), Beliefs_.end(), 0.25f);
// 	Beliefs_[0] = 1;

	flag_newdata_.resize(5);
	std::fill(flag_newdata_.begin(), flag_newdata_.end(), false);

	UpdateBeliefsRaw_.resize(4);
	std::fill(UpdateBeliefsRaw_.begin(), UpdateBeliefsRaw_.end(), 0);

	UpdateBeliefs_.resize(4);
	std::fill(UpdateBeliefs_.begin(), UpdateBeliefs_.end(), 0);

	D_gain_  = 0;
	epsilon_ = 0;
	// epsilon_hack_ = epsilon_;
	// D_gain_hack_ = D_gain_;



	// intialize the centriods
	N_centeriods_ = N_grid_xyz_[0] * N_grid_xyz_[1] * N_grid_xyz_[2];

	ROS_WARN_STREAM( N_centeriods_ << " centriods are being created");

	Centers_.resize(N_centeriods_);

	for (int i = 0; i < N_centeriods_; i++) {
		Centers_[i].resize(3); // x-y-z
	}


	int counter = 0;
	for (int i = 0; i < N_grid_xyz_[0]; i++) {
		for (int j = 0; j < N_grid_xyz_[1]; j++) {
			for (int k = 0; k < N_grid_xyz_[2]; k++) {

				Centers_[counter][0] = x_lim_[0] + (x_lim_[1] - x_lim_[0]) / (double)(N_grid_xyz_[0] - 1) * i;
				Centers_[counter][1] = y_lim_[0] + (y_lim_[1] - y_lim_[0]) / (double)(N_grid_xyz_[1] - 1) * j;
				Centers_[counter][2] = z_lim_[0] + (z_lim_[1] - z_lim_[0]) / (double)(N_grid_xyz_[2] - 1) * k;

				counter++;
			}
		}
	}


	activations_.resize(N_centeriods_);

	for (int i = 0; i < N_centeriods_; i++) {
		activations_[i] = 1.0f / (double)N_centeriods_; // x-y-z
	}

	Beta_.resize(N_centeriods_);
	Beta_dot_.resize(N_centeriods_);


	for (int i = 0; i < N_centeriods_; i++) {
		Beta_[i].resize(4); // for 4 tasks
		Beta_dot_[i].resize(4);
		for (int j = 0; j < 4; j++) {
			Beta_[i][j] = 0.25f; // equally between tasks
			Beta_dot_[i][j] = 0.0f;
		}
	}


	sigma2_ = 0.008;

// for (int i = 0; i < N_centeriods_; ++i) {
// 	std::cout << Centers_[i][0] << "\t" << Centers_[i][1] << "\t" << Centers_[i][2] << std::endl;
// }

// ROS_WARN_STREAM("counted up to " <<  counter);


}


// bool TaskLearner::CheckNewData() {

// 	bool flagAdapt = true;

// 	for (int i = 0; i < flag_newdata_.size(); i++) {
// 		flagAdapt &= flag_newdata_[i];
// 	}

// 	if (flagAdapt) {
// 		std::fill(flag_newdata_.begin(), flag_newdata_.end(), false);
// 		return true;
// 	}

// 	return false;

// }

void TaskLearner::ComputeActivation() {

// 	activation.zero(); // a global vector of size N_BASIS
	double sum_activation = 0;

	for (int i = 0; i < N_centeriods_; i++) {
		double norm2 = 0;
		for (int j = 0; j < 3; j++) // x-y-z
		{
			norm2 += pow(Centers_[i][j] - RealPosition_[j], 2);
		}

		activations_[i] = exp(-0.5 * norm2 / sigma2_);

		sum_activation += activations_[i];
	}

	// normalizing the activations
	for (int i = 0; i < N_centeriods_; i++)
	{
		activations_[i] /= sum_activation;
	}


}

void TaskLearner::ComputeBeliefs() {

	std::fill(Beliefs_.begin(), Beliefs_.end(), 0.0f);

	for (int i = 0; i < N_centeriods_; i++) {
		for (int j = 0; j < 4 ; j++) {
			Beliefs_[j] += activations_[i] * Beta_[i][j];
		}
	}


	// The sum is already close to 1, but we need to ensure it will stay close to 1
	double sum_beliefs = 0;
	for (int j = 0; j < 4 ; j++) {

		// this is not suppose to happen, but only for reliability
		if ( Beliefs_[j] < 0) {
			ROS_WARN("Computation instability in Beliefs");
			Beliefs_[j] = 0;
		}
		sum_beliefs += Beliefs_[j];
	}

	for (int j = 0; j < 4 ; j++) {
		Beliefs_[j] /= sum_beliefs;
	}
}

void TaskLearner::UpdateBeta() {

	for (int i = 0; i < N_centeriods_; i++) {
		for (int j = 0; j < 4; j++) {
			Beta_dot_[i][j] = activations_[i] * UpdateBeliefs_[j];
		}
	}

	for (int i = 0; i < N_centeriods_; i++) {
		double beta_sum = 0;
		for (int j = 0; j < 4; j++) {
			Beta_[i][j]  +=  epsilon_ *  Beta_dot_[i][j];

			if (Beta_[i][j] < 0) {
				Beta_[i][j] = 0;
			}

			beta_sum += Beta_[i][j];
		}
		for (int j = 0; j < 4; j++) {
			Beta_[i][j]  /=  beta_sum;
		}
	}
}




void TaskLearner::PublishBeliefs() {

	std_msgs::Float64MultiArray msg;

	msg.data.clear();

	for (int i = 0; i < 4; i++) {
		msg.data.push_back(Beliefs_[i]);
	}

	pub_beliefs_.publish(msg);

}


void TaskLearner::PublishBetas() {

	std_msgs::Float64MultiArray msg;

	msg.data.clear();

	for (int i = 0; i < N_centeriods_; i++) {
		for (int j = 0; j < 4; j++) {
					msg.data.push_back(Beta_[i][j]);
		}
	}

	pub_betas_.publish(msg);
}


// void TaskLearner::ComputeDesiredForce() {


// 	ControlWrench_[0] = -D_gain_ * (RealVelocity_[0] - DesiredVelocity_[0]);
// 	ControlWrench_[1] = -D_gain_ * (RealVelocity_[1] - DesiredVelocity_[1]);
// 	ControlWrench_[2] = -D_gain_ * (RealVelocity_[2] - DesiredVelocity_[2]);

// 	// if(ControlWrench[0] < -0.6)
// 	// 	ControlWrench[0] = -0.6;
// 	// if(ControlWrench[1] < -0.6)
// 	// 	ControlWrench[1] = -0.6;
// 	// if(ControlWrench[2] < -0.6)
// 	// 	ControlWrench[2] = -0.6;

// 	// if(ControlWrench[0] > 0.6)
// 	// 	ControlWrench[0] = 0.6;
// 	// if(ControlWrench[1] > 0.6)
// 	// 	ControlWrench[1] = 0.6;
// 	// if(ControlWrench[2] > 0.6)
// 	// 	ControlWrench[2] = 0.6;
// }

// void TaskLearner::PublishDesiredForce() {

// 	msgWrenchControl_.header.stamp = ros::Time::now();
// 	msgWrenchControl_.header.frame_id = "world"; // just for visualization
// 	msgWrenchControl_.wrench.force.x = ControlWrench_[0];
// 	msgWrenchControl_.wrench.force.y = ControlWrench_[1];
// 	msgWrenchControl_.wrench.force.z = ControlWrench_[2];
// 	msgWrenchControl_.wrench.torque.x = 0;
// 	msgWrenchControl_.wrench.torque.y = 0;
// 	msgWrenchControl_.wrench.torque.z = 0;

// 	pub_wrench_control_.publish(msgWrenchControl_);

// }




void TaskLearner::updateRealPosition(const geometry_msgs::Pose::ConstPtr& msg) {

	RealPosition_[0] = msg->position.x;
	RealPosition_[1] = msg->position.y;
	RealPosition_[2] = msg->position.z;

}



void TaskLearner::updateRealVelocity(const geometry_msgs::Twist::ConstPtr& msg) {

	RealVelocity_[0] = msg->linear.x;
	RealVelocity_[1] = msg->linear.y;
	RealVelocity_[2] = msg->linear.z;

	flag_newdata_[0] = true;
}




/*--------------------------------------------------------------------
 * Reading the new desired velocity of each task
 * and setting the flags to true for receiving the new data points
 *------------------------------------------------------------------*/

void TaskLearner::UpdateTask1(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	Task1_velocity_[0] = msg->twist.linear.x;
	Task1_velocity_[1] = msg->twist.linear.y;
	Task1_velocity_[2] = msg->twist.linear.z;

	flag_newdata_[1] = true;
}

void TaskLearner::UpdateTask2(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	Task2_velocity_[0] = msg->twist.linear.x;
	Task2_velocity_[1] = msg->twist.linear.y;
	Task2_velocity_[2] = msg->twist.linear.z;

	flag_newdata_[2] = true;
}

void TaskLearner::UpdateTask3(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	Task3_velocity_[0] = msg->twist.linear.x;
	Task3_velocity_[1] = msg->twist.linear.y;
	Task3_velocity_[2] = msg->twist.linear.z;

	flag_newdata_[3] = true;
}

void TaskLearner::UpdateTask4(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	Task4_velocity_[0] = msg->twist.linear.x;
	Task4_velocity_[1] = msg->twist.linear.y;
	Task4_velocity_[2] = msg->twist.linear.z;

	flag_newdata_[4] = true;
}


// --------------------------------------------------------------------
//  * configCallback()
//  * Callback function for dynamic reconfigure server.
//  *------------------------------------------------------------------

void TaskLearner::DynCallback(task_adaptation::task_learning_paramsConfig& config, uint32_t level)
{
	// Set class variables to new values. They should match what is input at the dynamic reconfigure GUI.
	D_gain_ = config.D_gain;
	epsilon_ = config.epsilon;
	sigma2_ = config.sigma2;

	ROS_INFO_STREAM("configCallback: received update! D_gain = " << D_gain_ << "  espsilon = " << epsilon_ << " Sigma2" << sigma2_);

}


void TaskLearner::ComputeDesiredVelocity() {

	// starting to zero
	std::fill(DesiredVelocity_.begin(), DesiredVelocity_.end(), 0);

	for (int dim = 0 ; dim < 3 ; dim++)
	{
		DesiredVelocity_[dim] += Beliefs_[0] * Task1_velocity_[dim];
		DesiredVelocity_[dim] += Beliefs_[1] * Task2_velocity_[dim];
		DesiredVelocity_[dim] += Beliefs_[2] * Task3_velocity_[dim];
		DesiredVelocity_[dim] += Beliefs_[3] * Task4_velocity_[dim];
	}
}

void TaskLearner::PublishDesiredVelocity() {

	// msgDesiredVelocity_.header.stamp = ros::Time::now();
	// msgDesiredVelocity_.twist.linear.x = DesiredVelocity_[0];
	// msgDesiredVelocity_.twist.linear.y = DesiredVelocity_[1];
	// msgDesiredVelocity_.twist.linear.z = DesiredVelocity_[2];


	msgDesiredVelocity_.linear.x = DesiredVelocity_[0];
	msgDesiredVelocity_.linear.y = DesiredVelocity_[1];
	msgDesiredVelocity_.linear.z = DesiredVelocity_[2];

	pub_adapted_velocity_.publish(msgDesiredVelocity_);
}


// // ---------------------------------------------------------------------------
// //------------------- The raw adaptation takes place in this function --------
// // ---------------------------------------------------------------------------

void TaskLearner::RawAdaptation()
{
	std::fill(UpdateBeliefsRaw_.begin(), UpdateBeliefsRaw_.end(), 0);

	double NullinnterSimilarity = 0;
	double TempInnerSimilarity;


	UpdateBeliefsRaw_[0] -= ComputeOutterSimilarity(Task1_velocity_);
	TempInnerSimilarity   = 2 * ComputeInnerSimilarity(Beliefs_[0], Task1_velocity_);
	UpdateBeliefsRaw_[0] -= TempInnerSimilarity;

	if (TempInnerSimilarity > NullinnterSimilarity) {
		NullinnterSimilarity = TempInnerSimilarity;
	}

	UpdateBeliefsRaw_[1] -= ComputeOutterSimilarity(Task2_velocity_);
	TempInnerSimilarity = 2 * ComputeInnerSimilarity(Beliefs_[1], Task2_velocity_);
	UpdateBeliefsRaw_[1] -= TempInnerSimilarity;

	if (TempInnerSimilarity > NullinnterSimilarity) {
		NullinnterSimilarity = TempInnerSimilarity;
	}

	UpdateBeliefsRaw_[2] -= ComputeOutterSimilarity(Task3_velocity_);
	TempInnerSimilarity   = 2 * ComputeInnerSimilarity(Beliefs_[2], Task3_velocity_);
	UpdateBeliefsRaw_[2] -= TempInnerSimilarity;

	if (TempInnerSimilarity > NullinnterSimilarity) {
		NullinnterSimilarity = TempInnerSimilarity;
	}

	UpdateBeliefsRaw_[3] -= ComputeOutterSimilarity(Task4_velocity_);
	TempInnerSimilarity   = 2 * ComputeInnerSimilarity(Beliefs_[3], Task4_velocity_);
	UpdateBeliefsRaw_[3] -= TempInnerSimilarity;

	if (TempInnerSimilarity > NullinnterSimilarity) {
		NullinnterSimilarity = TempInnerSimilarity;
	}

	// UpdateBeliefsRaw_[0] -= ComputeOutterSimilarity(Task0_velocity_);
	// UpdateBeliefsRaw_[0] -= NullinnterSimilarity;

	// if(Beliefs_[0] < 0.2){
	// 	UpdateBeliefsRaw_[0] -= 1000;
	// }

//	UpdateBeliefsRaw[0] -= 0.25 * ComputeInnerSimilarity(Beliefs[1],Task1_velocity);
//	UpdateBeliefsRaw[0] -= 0.25 * ComputeInnerSimilarity(Beliefs[2],Task2_velocity);
//	UpdateBeliefsRaw[0] -= 0.25 * ComputeInnerSimilarity(Beliefs[3],Task3_velocity);
//	UpdateBeliefsRaw[0] -= 0.25 * ComputeInnerSimilarity(Beliefs[4],Task4_velocity);

//	UpdateBeliefsRaw[0] += (Beliefs[1] - 1.0) * UpdateBeliefsRaw[1];
//	UpdateBeliefsRaw[0] += (Beliefs[2] - 1.0) * UpdateBeliefsRaw[2];
//	UpdateBeliefsRaw[0] += (Beliefs[3] - 1.0) * UpdateBeliefsRaw[3];
//	UpdateBeliefsRaw[0] += (Beliefs[4] - 1.0) * UpdateBeliefsRaw[4];



}

void TaskLearner::SimpleWinnerTakeAll() {

	// initializing the updates for the beliefs at zero
	std::fill(UpdateBeliefs_.begin(), UpdateBeliefs_.end(), 0);

	// fining the winner who has the biggest value for UpdateBeliefsRaw
	int winner_index = 0;

	for (int i = 1; i < UpdateBeliefsRaw_.size(); i++)
	{
		if (UpdateBeliefsRaw_[i] > UpdateBeliefsRaw_[winner_index])
			winner_index = i;
	}

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
		UpdateBeliefs_[i] = UpdateBeliefsRaw_[i] - offset;
	}

}



// void TaskLearner::WinnerTakeAll()
// {
// 	// initializing the updates for the beliefs at zero
// 	std::fill(UpdateBeliefs_.begin(), UpdateBeliefs_.end(), 0);

// 	// fining the winner who has the biggest value for UpdateBeliefsRaw
// 	int winner_index = 0;

// 	for (int i = 1; i < UpdateBeliefsRaw_.size(); i++)
// 	{
// 		if (UpdateBeliefsRaw_[i] > UpdateBeliefsRaw_[winner_index])
// 			winner_index = i;
// 	}

// 	// no update is required if the winner is already saturated
// 	if (Beliefs_[winner_index] == 1)
// 		return;

// 	int runnerUp_index = 0;

// 	if (winner_index == 0) {
// 		runnerUp_index = 1;
// 	}

// 	for (int i = 0; i < UpdateBeliefsRaw_.size(); i++)
// 	{
// 		if (i ==  winner_index) {
// 			continue;
// 		}

// 		if (UpdateBeliefsRaw_[i] > UpdateBeliefsRaw_[runnerUp_index]) {
// 			runnerUp_index = i;
// 		}
// 	}

// 	// computing the middle point and removing form all raw updates
// 	float offset = 0.5 * (UpdateBeliefsRaw_[winner_index] + UpdateBeliefsRaw_[runnerUp_index]);

// 	for (int i = 0; i < UpdateBeliefsRaw_.size(); i++) {
// 		UpdateBeliefsRaw_[i] -= offset;
// 	}


// 	// computing the sum of updates and setting to zero for active one so we keep the sum beliefs at 1.
// 	float UpdateSum = 0;

// 	for (int i = 0; i < UpdateBeliefsRaw_.size(); i++) {
// 		if (Beliefs_[i] != 0 || UpdateBeliefsRaw_[i] > 0)
// 			UpdateSum += UpdateBeliefsRaw_[i];
// 	}

// 	for (int i = 0; i < UpdateBeliefsRaw_.size(); i++) 	{
// 		UpdateBeliefs_[i] = UpdateBeliefsRaw_[i];
// 	}

// 	UpdateBeliefs_[winner_index] -= UpdateSum;

// }


float TaskLearner::ComputeInnerSimilarity(float b, std::vector<float> task_velocity) {

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


float TaskLearner::ComputeOutterSimilarity(std::vector<float> task_velocity) {

	float outterSimiliary = 0;

	outterSimiliary += (RealVelocity_[0] - task_velocity[0]) * (RealVelocity_[0] - task_velocity[0]);
	outterSimiliary += (RealVelocity_[1] - task_velocity[1]) * (RealVelocity_[1] - task_velocity[1]);
	outterSimiliary += (RealVelocity_[2] - task_velocity[2]) * (RealVelocity_[2] - task_velocity[2]);

	return outterSimiliary;
}


/*--------------------------------------------------------------------
 * Display relevant information in the console
 *------------------------------------------------------------------*/

void TaskLearner::DisplayInformation()
{
	//std::cout << RealVelocity[0] << "\t" <<  RealVelocity[1] << "\t" << RealVelocity[2]  << std::endl;

	std::cout << "Time = " << ros::Time::now() << std::endl;

	// if (!flag_newdata_[0])
	// 	std::cout << "Real velocity is not received " << std::endl;

	// if (!flag_newdata_[1])
	// 	std::cout << "Task1 velocity is not received " << std::endl;

	// if (!flag_newdata_[2])
	// 	std::cout << "Task2 velocity is not received " << std::endl;

	// if (!flag_newdata_[3])
	// 	std::cout << "Task3 velocity is not received " << std::endl;

	// if (!flag_newdata_[4])
	// 	std::cout << "Task4 velocity is not received " << std::endl;

//	std::cout << "Beliefs are :  b0= " << Beliefs[0] <<
//			                 "\t b1= " << Beliefs[1] <<
//							 "\t b2= " << Beliefs[2] <<
//							 "\t b3= " << Beliefs[3] <<
//							 "\t b4= " << Beliefs[4] << std::endl;

	std::cout << "Real    velocity = [" << RealVelocity_[0] 	<< " , " << RealVelocity_[1] << " , " << RealVelocity_[2] << "]" << std::endl;
	std::cout << "Adapted velocity = [" << DesiredVelocity_[0] << " , " << DesiredVelocity_[1] << " , " << DesiredVelocity_[2] << "]" << std::endl;
	std::cout << "Control forces   = [" << ControlWrench_[0] << " , " << ControlWrench_[1] << " , " << ControlWrench_[2] << "]" << std::endl;

	std::cout << "Adaptation rate  = " << epsilon_ << " Control gain  = " << D_gain_ << std::endl;


	std::cout << "Task 1 : b =" << Beliefs_[0] << "\t db_hat = " << UpdateBeliefsRaw_[0] << "\t db = " << UpdateBeliefs_[0] <<  std::endl;
	std::cout << "Task 2 : b =" << Beliefs_[1] << "\t db_hat = " << UpdateBeliefsRaw_[1] << "\t db = " << UpdateBeliefs_[1] <<  std::endl;
	std::cout << "Task 3 : b =" << Beliefs_[2] << "\t db_hat = " << UpdateBeliefsRaw_[2] << "\t db = " << UpdateBeliefs_[2] <<  std::endl;
	std::cout << "Task 4 : b =" << Beliefs_[3] << "\t db_hat = " << UpdateBeliefsRaw_[3] << "\t db = " << UpdateBeliefs_[3] <<  std::endl;


	ROS_INFO_STREAM("Beliefs : " << Beliefs_[0] << " " << Beliefs_[1] << " " << Beliefs_[2] << " " << Beliefs_[3]);
	ROS_INFO_STREAM("centroid : " << Centers_[0][0] << " " << Centers_[0][1] << " " << Centers_[0][2] );
	ROS_INFO_STREAM("Real posisition: " << RealPosition_[0] << " " << RealPosition_[1] << " " << RealPosition_[2]);
	ROS_INFO_STREAM("activations:" << activations_[0]);
	ROS_INFO_STREAM("activations:" << activations_[1]);

	ROS_INFO_STREAM("--------------");

	std::cout << "----------------------------------------------------- " << std::endl << std::endl;
}
