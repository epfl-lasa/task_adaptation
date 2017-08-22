#include "LearningVisualizer.h"


LearningVisualizer::LearningVisualizer(ros::NodeHandle &n,
                                       double frequency,
                                       std::string topic_real_position,
                                       std::string topic_beliefs,
                                       std::string topic_betas,
                                       std::vector<double> x_lim,
                                       std::vector<double> y_lim,
                                       std::vector<double> z_lim,
                                       std::vector<int> N_grid_xyz,
                                       std::string topic_task1_velocity,
                                       std::string topic_task2_velocity,
                                       std::string topic_task3_velocity,
                                       std::string topic_task4_velocity)
	: nh_(n),
	  loop_rate_(frequency),
	  topic_real_position_(topic_real_position),
	  topic_beliefs_(topic_beliefs),
	  topic_betas_(topic_betas),
	  x_lim_(x_lim),
	  y_lim_(y_lim),
	  z_lim_(z_lim),
	  N_grid_xyz_(N_grid_xyz),
	  topic_task1_velocity_(topic_task1_velocity),
	  topic_task2_velocity_(topic_task2_velocity),
	  topic_task3_velocity_(topic_task3_velocity),
	  topic_task4_velocity_(topic_task4_velocity) {

	ROS_INFO_STREAM("Task-Learning node is created at: " << nh_.getNamespace() << " with freq: " << frequency << "Hz");
}




bool LearningVisualizer::Init() {

	InitClassVariables();

	// we just need to this once, except when sigma changes
	ComputeActivation();


	if (!InitROS()) {
		ROS_ERROR_STREAM("ERROR intializing the ROS node");
		return false;
	}

	ROS_INFO("Task-Learning node is intialized.");
	return true;
}

void LearningVisualizer::Run() {

	ros::Time::now();

	ROS_INFO("Learning-Visualization node is running ...");



	while (nh_.ok()) {


		UpdateVisualization();

		// ComputeActivation();

		ComputeBeliefs();

		ComputePublishFuturePath();

		ros::spinOnce();
		loop_rate_.sleep();
	}
}


bool LearningVisualizer::InitROS() {


	sub_target_1_ = nh_.subscribe("/Tasks/Task1/DS/target"  , 10, &LearningVisualizer::GetTargetTask1, this);
	sub_target_2_ = nh_.subscribe("/Tasks/Task2/DS/target"  , 10, &LearningVisualizer::GetTargetTask2, this);
	sub_target_3_ = nh_.subscribe("/Tasks/Task3/DS/target"  , 10, &LearningVisualizer::GetTargetTask3, this);
	sub_target_4_ = nh_.subscribe("/Tasks/Task4/DS/target"  , 10, &LearningVisualizer::GetTargetTask4, this);

	sub_task1_velocity_ = nh_.subscribe(topic_task1_velocity_, 10, &LearningVisualizer::UpdateTask1, this, ros::TransportHints().reliable().tcpNoDelay());
	sub_task2_velocity_ = nh_.subscribe(topic_task2_velocity_, 10, &LearningVisualizer::UpdateTask2, this, ros::TransportHints().reliable().tcpNoDelay());
	sub_task3_velocity_ = nh_.subscribe(topic_task3_velocity_, 10, &LearningVisualizer::UpdateTask3, this, ros::TransportHints().reliable().tcpNoDelay());
	sub_task4_velocity_ = nh_.subscribe(topic_task4_velocity_, 10, &LearningVisualizer::UpdateTask4, this, ros::TransportHints().reliable().tcpNoDelay());


	sub_beliefs_ = nh_.subscribe(topic_beliefs_  , 10, &LearningVisualizer::UpdateBeliefs, this);
	sub_beta_ = nh_.subscribe(topic_betas_  , 10, &LearningVisualizer::UpdateBetas, this);



	pub_marker_1_ = nh_.advertise<visualization_msgs::Marker>("/Learning/attractor_1", 1);
	pub_marker_2_ = nh_.advertise<visualization_msgs::Marker>("/Learning/attractor_2", 1);
	pub_marker_3_ = nh_.advertise<visualization_msgs::Marker>("/Learning/attractor_3", 1);
	pub_marker_4_ = nh_.advertise<visualization_msgs::Marker>("/Learning/attractor_4", 1);

	pub_surface_ = nh_.advertise<visualization_msgs::Marker>("/Learning/surface_marker", 1);

	pub_pointCloud_ = nh_.advertise<sensor_msgs::PointCloud>("/Learning/point_cloud", 10);

	pub_DesiredPath_ = nh_.advertise<nav_msgs::Path>("/Learning/DesiredPath", 1);


	sub_realPosition_ = nh_.subscribe(topic_real_position_, 1000,
	                                  &LearningVisualizer::updateRealPosition, this, ros::TransportHints().reliable().tcpNoDelay());



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


void LearningVisualizer::InitClassVariables() {




	// draw a static surface for the table


	surface_.header.frame_id = "world";
	surface_.header.stamp = ros::Time();
	surface_.ns = "marker_test_triangle_list";
	surface_.id = 0;
	surface_.type = visualization_msgs::Marker::TRIANGLE_LIST;
	surface_.action = visualization_msgs::Marker::ADD;
	surface_.pose.position.x = 0.0;
	surface_.pose.position.y = 0.0;
	surface_.pose.position.z = 0.0;
	surface_.pose.orientation.x = 0.0;
	surface_.pose.orientation.y = 0.0;
	surface_.pose.orientation.z = 0.0;
	surface_.pose.orientation.w = 1.0;
	surface_.scale.x = 1.0;
	surface_.scale.y = 1.0;
	surface_.scale.z = 1.0;
	surface_.color.a = 1.0;

	geometry_msgs::Point p1, p2, p3, p4, p5, p6;
	p1.x = -0.36;
	p1.y = -0.48;
	p1.z = 0.20;
	p2.x = -0.72;
	p2.y = -0.48;
	p2.z = 0.20;
	p3.x = -0.72;
	p3.y = 0.24;
	p3.z = 0.20;
	p4.x = -0.72;
	p4.y = 0.24;
	p4.z = 0.20;
	p5.x = -0.36;
	p5.y = 0.24;
	p5.z = 0.20;
	p6.x = -0.36;
	p6.y = -0.48;
	p6.z = 0.20;

	std_msgs::ColorRGBA c;
	c.r = 0.6;
	c.g = 0.6;
	c.b = 0.6;
	c.a = 0.5;

	for (int k = 0; k < 6; k++)
	{
		surface_.colors.push_back(c);
	}

	surface_.points.push_back(p1);
	surface_.points.push_back(p2);
	surface_.points.push_back(p3);
	surface_.points.push_back(p4);
	surface_.points.push_back(p5);
	surface_.points.push_back(p6);




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
	marker_3_.color.g = 0.0f;
	marker_3_.color.b = 1.0f;
	marker_3_.color.a = 1.0f;

	marker_4_.color.r = 1.0f;
	marker_4_.color.g = 1.0f;
	marker_4_.color.b = 1.0f;
	marker_4_.color.a = 1.0f;


	marker_1_.lifetime = ros::Duration();
	marker_2_.lifetime = ros::Duration();
	marker_3_.lifetime = ros::Duration();
	marker_4_.lifetime = ros::Duration();





	// preparing the pointCloud
	myPointCloud_.header.frame_id = "world";
	myPointCloud_.header.stamp = ros::Time::now();

	myPointCloud_.channels.resize(2);
	myPointCloud_.channels[0].name = "rgb";
	myPointCloud_.channels[1].name = "intensity";

	int nx = 12, ny = 25, nz = 12;
	myPointCloud_.points.resize(nx * ny * nz);
	myPointCloud_.channels[0].values.resize(nx * ny * nz);
	myPointCloud_.channels[1].values.resize(nx * ny * nz);




	//intial color
	uint8_t r = 0, g = 0, b = 0;    // Example: Red color
	uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);

	int pc = 0;
	for (int i = 0; i < nx; i++) {
		for (int j = 0; j < ny; j++) {
			for (int k = 0; k < nz; k++) {

				myPointCloud_.points[pc].x = x_lim_[0] + (x_lim_[1] - x_lim_[0]) / (double)(nx - 1) * i;
				myPointCloud_.points[pc].y = y_lim_[0] + (y_lim_[1] - y_lim_[0]) / (double)(ny - 1) * j;
				myPointCloud_.points[pc].z = z_lim_[0] + (z_lim_[1] - z_lim_[0]) / (double)(nz - 1) * k;

				myPointCloud_.channels[0].values[pc] = *reinterpret_cast<float*>(&rgb);

				myPointCloud_.channels[1].values[pc] = 1;

				pc++;
			}
		}
	}





	// initializing the varibales

	RealPosition_.resize(3);

	Task1_velocity_.resize(3);
	Task2_velocity_.resize(3);
	Task3_velocity_.resize(3);
	Task4_velocity_.resize(3);



	Beliefs_.resize(4);
	std::fill(Beliefs_.begin(), Beliefs_.end(), 0.25f);
// 	Beliefs_[0] = 1;

	// intialize the centriods
	N_centeriods_ = N_grid_xyz_[0] * N_grid_xyz_[1] * N_grid_xyz_[2];

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


	activations_.resize(myPointCloud_.points.size());

	for (int i = 0; i < myPointCloud_.points.size(); i++) {
		activations_[i].resize(N_centeriods_);
		for (int j = 0; j < N_centeriods_; j++) {
			activations_[i][j] = 1.0f / (double)N_centeriods_;
		}
	}


	Beta_.resize(N_centeriods_);
	for (int i = 0; i < N_centeriods_; i++) {
		Beta_[i].resize(4); // for 4 tasks
		for (int j = 0; j < 4; j++) {
			Beta_[i][j] = 0.25f; // equally between tasks
		}
	}

	BeliefCloud_.resize(myPointCloud_.points.size());

	for (int i = 0; i < myPointCloud_.points.size(); i++) {
		BeliefCloud_[i].resize(4);
		for (int j = 0; j < 4; j++) {
			BeliefCloud_[i][j] = 0.25f; //equal among primitives
		}
	}


	sigma2_ = 0.008;



}



void LearningVisualizer::ComputeActivation() {

// 	activation.zero(); // a global vector of size N_BASIS
	double sum_activation = 0;

	for (int i = 0; i < myPointCloud_.points.size(); i++) {
		double sum_activation = 0;

		for (int j = 0; j < N_centeriods_; j++) {
			double norm2 = 0;
			norm2 += pow(Centers_[j][0] - myPointCloud_.points[i].x, 2);
			norm2 += pow(Centers_[j][1] - myPointCloud_.points[i].y, 2);
			norm2 += pow(Centers_[j][2] - myPointCloud_.points[i].z, 2);

			activations_[i][j] = exp(-0.5 * norm2 / sigma2_);

			sum_activation += activations_[i][j];
		}

		for (int j = 0; j < N_centeriods_; j++)
		{
			activations_[i][j] /= sum_activation;
		}


	}

}

void LearningVisualizer::ComputeBeliefs() {



	for (int i = 0; i < myPointCloud_.points.size(); i++) {
		for (int j = 0; j < 3; j++) {
			BeliefCloud_[i][j] = 0;

			for (int k = 1; k < N_centeriods_; k++) {
				BeliefCloud_[i][j] += activations_[i][k] * Beta_[k][j];
			}

			if (BeliefCloud_[i][j] < 0) {
				BeliefCloud_[i][j] = 0;
			}
		}

		//may check for sum (over j for each i ) to one

	}

	for (int i = 0; i < myPointCloud_.points.size(); i++) {


		uint8_t r = 255 * BeliefCloud_[i][0];  // from primitive one to red
		uint8_t g = 255 * BeliefCloud_[i][1];  // from primitive one to green
		uint8_t b = 255 * BeliefCloud_[i][2];  // from primitive one to blue

		uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);

		myPointCloud_.channels[0].values[i] = *reinterpret_cast<float*>(&rgb);


	}


}




void LearningVisualizer::updateRealPosition(const geometry_msgs::Pose::ConstPtr& msg) {

	RealPosition_[0] = msg->position.x;
	RealPosition_[1] = msg->position.y;
	RealPosition_[2] = msg->position.z;


	ROS_WARN("reading the real position");
}



void LearningVisualizer::UpdateVisualization() {

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


	marker_1_.color.a = 0.3f + 0.7 * Beliefs_[0];
	marker_2_.color.a = 0.3f + 0.7 * Beliefs_[1];
	marker_3_.color.a = 0.3f + 0.7 * Beliefs_[2];
	marker_4_.color.a = 0.3f + 0.7 * Beliefs_[3];


	pub_marker_1_.publish(marker_1_);
	pub_marker_2_.publish(marker_2_);
	pub_marker_3_.publish(marker_3_);
	pub_marker_4_.publish(marker_4_);

	pub_surface_.publish(surface_);

	pub_pointCloud_.publish(myPointCloud_);



}

void LearningVisualizer::GetTargetTask1(const geometry_msgs::PointStamped::ConstPtr& msg)
{
	target_pose_1_[0] = msg->point.x;
	target_pose_1_[1] = msg->point.y;
	target_pose_1_[2] = msg->point.z;

}

void LearningVisualizer::GetTargetTask2(const geometry_msgs::PointStamped::ConstPtr& msg)
{
	target_pose_2_[0] = msg->point.x;
	target_pose_2_[1] = msg->point.y;
	target_pose_2_[2] = msg->point.z;
}

void LearningVisualizer::GetTargetTask3(const geometry_msgs::PointStamped::ConstPtr& msg)
{
	target_pose_3_[0] = msg->point.x;
	target_pose_3_[1] = msg->point.y;
	target_pose_3_[2] = msg->point.z;
}

void LearningVisualizer::GetTargetTask4(const geometry_msgs::PointStamped::ConstPtr& msg)
{
	target_pose_4_[0] = msg->point.x;
	target_pose_4_[1] = msg->point.y;
	target_pose_4_[2] = msg->point.z;
}


void LearningVisualizer::UpdateBeliefs(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	// target_pose_4_[0] = msg->point.x;
	// target_pose_4_[1] = msg->point.y;
	// target_pose_4_[2] = msg->point.z;

	Beliefs_[0] = msg->data[0];
	Beliefs_[1] = msg->data[1];
	Beliefs_[2] = msg->data[2];
	Beliefs_[3] = msg->data[3];
	// beliefs_[4] = msg->data[4];
}

void LearningVisualizer::UpdateBetas(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	// target_pose_4_[0] = msg->point.x;
	// target_pose_4_[1] = msg->point.y;
	// target_pose_4_[2] = msg->point.z;

	// beliefs_[0] = msg->data[0];
	// beliefs_[1] = msg->data[1];
	// beliefs_[2] = msg->data[2];
	// beliefs_[3] = msg->data[3];
	// beliefs_[4] = msg->data[4];

	int counter = 0;
	for (int i = 0; i < N_centeriods_; i++) {
		for (int j = 0; j < 4; j++) {

			Beta_[i][j] = msg->data[counter];
			counter++;
		}
	}


}




// --------------------------------------------------------------------
//  * configCallback()
//  * Callback function for dynamic reconfigure server.
//  *------------------------------------------------------------------



void LearningVisualizer::ComputePublishFuturePath() {




	// geometry_msgs::PointStamped msg;

	// msg.header.frame_id = "world";
	// msg.header.stamp = ros::Time::now();
	// msg.point.x = target_pose_[0] + target_offset_[0];
	// msg.point.y = target_pose_[1] + target_offset_[1];
	// msg.point.z = target_pose_[2] + target_offset_[2];

	// pub_target_.publish(msg);

	// // create a temporary message


	// setting the header of the path
	msg_DesiredPath_.header.stamp = ros::Time::now();
	msg_DesiredPath_.header.frame_id = "world";

	std::vector<double> simulated_pos = RealPosition_;
	std::vector<double> simulated_vel;
	simulated_vel.resize(3);

	std::vector<double> sim_activation;
	sim_activation.resize(N_centeriods_);

	std::vector<double> sim_beliefs;
	sim_beliefs.resize(4);

	double dt = .2;

	msg_DesiredPath_.poses.resize(MAX_FRAME);


	for (int frame = 0; frame < MAX_FRAME; frame++) {

		double sum_activation = 0;
		for (int i = 0; i < N_centeriods_; i++) {

			double norm2 = 0;
			norm2 += pow(Centers_[i][0] - simulated_pos[0], 2);
			norm2 += pow(Centers_[i][1] - simulated_pos[1], 2);
			norm2 += pow(Centers_[i][2] - simulated_pos[2], 2);

			sim_activation[i] = exp(-0.5 * norm2 / sigma2_);

			sum_activation += sim_activation[i];
		}

		for (int i = 0; i < N_centeriods_; i++)	{
			sim_activation[i] /= sum_activation;
		}

		for (int j = 0; j < 3; j++) {
			sim_beliefs[j] = 0;
			for (int i = 0; i < N_centeriods_; i++) {
				sim_beliefs[j] += sim_activation[i] * Beta_[i][j];
			}
			if (sim_beliefs[j] < 0) {
				sim_beliefs[j] = 0;
			}
		}

		for (int d = 0; d < 2; d++) {
			simulated_vel[d] = 0;
			simulated_vel[d] += sim_beliefs[0] * Task1_velocity_[d];
			simulated_vel[d] += sim_beliefs[1] * Task2_velocity_[d];
			simulated_vel[d] += sim_beliefs[2] * Task3_velocity_[d];
			simulated_vel[d] += sim_beliefs[3] * Task4_velocity_[d];

		}

		simulated_pos[0] +=  simulated_vel[0] * dt;
		simulated_pos[1] +=  simulated_vel[1] * dt;
		simulated_pos[2] +=  simulated_vel[2] * dt;

		msg_DesiredPath_.poses[frame].header.stamp = ros::Time::now();
		msg_DesiredPath_.poses[frame].header.frame_id = "world";
		msg_DesiredPath_.poses[frame].pose.position.x = simulated_pos[0];
		msg_DesiredPath_.poses[frame].pose.position.y = simulated_pos[1];
		msg_DesiredPath_.poses[frame].pose.position.z = simulated_pos[2];

	}


	pub_DesiredPath_.publish(msg_DesiredPath_);




	//may check for sum (over j for each i ) to one

// }


// double sum_activation = 0;

// for (int i = 0; i < myPointCloud_.points.size(); i++) {
// 	double sum_activation = 0;

// 	for (int j = 0; j < N_centeriods_; j++) {
// 		double norm2 = 0;
// 		norm2 += pow(Centers_[j][0] - myPointCloud_.points[i].x, 2);
// 		norm2 += pow(Centers_[j][1] - myPointCloud_.points[i].y, 2);
// 		norm2 += pow(Centers_[j][2] - myPointCloud_.points[i].z, 2);

// 		activations_[i][j] = exp(-0.5 * norm2 / sigma2_);

// 		sum_activation += activations_[i][j];
// 	}

// 	for (int j = 0; j < N_centeriods_; j++)
// 	{
// 		activations_[i][j] /= sum_activation;
// 	}



// for (int frame = 0; frame < MAX_FRAME; frame++)
// {


// 	simulated_vel = SED_GMM_->getVelocity(simulated_pose - target_pose_ - target_offset_);

// 	simulated_pose[0] +=  simulated_vel[0] * dt_ * 20;
// 	simulated_pose[1] +=  simulated_vel[1] * dt_ * 20;
// 	simulated_pose[2] +=  simulated_vel[2] * dt_ * 20;

// 	msg_DesiredPath_.poses[frame].header.stamp = ros::Time::now();
// 	msg_DesiredPath_.poses[frame].header.frame_id = "world";
// 	msg_DesiredPath_.poses[frame].pose.position.x = simulated_pose[0];
// 	msg_DesiredPath_.poses[frame].pose.position.y = simulated_pose[1];
// 	msg_DesiredPath_.poses[frame].pose.position.z = simulated_pose[2];

// 	pub_DesiredPath_.publish(msg_DesiredPath_);


// }



// // starting to zero
// std::fill(DesiredVelocity_.begin(), DesiredVelocity_.end(), 0);

// for (int dim = 0 ; dim < 3 ; dim++)
// {
// 	DesiredVelocity_[dim] += Beliefs_[0] * Task1_velocity_[dim];
// 	DesiredVelocity_[dim] += Beliefs_[1] * Task2_velocity_[dim];
// 	DesiredVelocity_[dim] += Beliefs_[2] * Task3_velocity_[dim];
// 	DesiredVelocity_[dim] += Beliefs_[3] * Task4_velocity_[dim];
// }
}



/*--------------------------------------------------------------------
 * Reading the new desired velocity of each task
 * and setting the flags to true for receiving the new data points
 *------------------------------------------------------------------*/

void LearningVisualizer::UpdateTask1(const geometry_msgs::TwistStamped::ConstPtr & msg)
{
	Task1_velocity_[0] = msg->twist.linear.x;
	Task1_velocity_[1] = msg->twist.linear.y;
	Task1_velocity_[2] = msg->twist.linear.z;

}

void LearningVisualizer::UpdateTask2(const geometry_msgs::TwistStamped::ConstPtr & msg)
{
	Task2_velocity_[0] = msg->twist.linear.x;
	Task2_velocity_[1] = msg->twist.linear.y;
	Task2_velocity_[2] = msg->twist.linear.z;

}

void LearningVisualizer::UpdateTask3(const geometry_msgs::TwistStamped::ConstPtr & msg)
{
	Task3_velocity_[0] = msg->twist.linear.x;
	Task3_velocity_[1] = msg->twist.linear.y;
	Task3_velocity_[2] = msg->twist.linear.z;

}

void LearningVisualizer::UpdateTask4(const geometry_msgs::TwistStamped::ConstPtr & msg)
{
	Task4_velocity_[0] = msg->twist.linear.x;
	Task4_velocity_[1] = msg->twist.linear.y;
	Task4_velocity_[2] = msg->twist.linear.z;

}