
/*
 * Copyright (C) 2016, Yu
*/


#include "motion_planning.h"



TeleopIMU::TeleopIMU():
pnh_("~/fcu")
{
	pnh_.param("k_stick", k_stick_, 25);
	pnh_.param("k_stick_yaw", k_stick_yaw_, 120);


    pub=n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",1);


    llcmd_pub_acc = n.advertise<asctec_hl_comm::mav_ctrl>("fcu/control",1); //command to HL_inteface
   // llcmb_pubrate.Rate(20);//20HZ
    llcmd_pub_vel = n.advertise<asctec_hl_comm::mav_ctrl>("fcu/control",1); //command to HL_interface
    ext_state=n.advertise<sensor_fusion_comm::ExtState>("fcu/state", 1); //external state, to interface of asctec
    pub2=n.advertise<geometry_msgs::PoseStamped>("command/pose",1); //command to quadrotor
    pub3 = n.advertise<geometry_msgs::TwistStamped>("command/twist",1); //velocity command to quadrotor

	//publish gps position and velocity information:
	position_gps = n.advertise<nav_msgs::Odometry>("gpsinformation",1);


    //should run rosrun xsens_driver mtnode.py in order to run the xsens_driver node,
    //this node will advertise /imu/data
    //topic
	sub=n.subscribe<sensor_msgs::Imu>("/imu/data",10,&TeleopIMU::callBack,this);

	//the topic name is still under discussion, from the SLAM module
	pose_sub_ = n.subscribe("robot_pose", 1, &TeleopIMU::poseCallback, this);

	//the topic name is still under discussion, from the SLAM module
	odometry_sub_ = n.subscribe<nav_msgs::Odometry>("/stereo_odometer/odometry", 1, &TeleopIMU::odometryCallback, this);
	rcdata_sub_ = n.subscribe<asctec_hl_comm::mav_rcdata>("fcu/rcdata", 1, &TeleopIMU::rcdataCallback, this);
	gps_custom_sub_ =n.subscribe<asctec_hl_comm::GpsCustom> ("fcu/gps_custom", 1, &TeleopIMU::gpsdataCallback, this);
	imu_custom_sub_ =n.subscribe<asctec_hl_comm::mav_imu>  ("fcu/imu_custom", 1, &TeleopIMU::imudataCallback, this);
	flag_cmd_sub = n.subscribe<asctec_mav_motion_planning::flag_cmd>("flag_cmd", 1, &TeleopIMU::flagcmdCallback, this); //flag determine which device will sends the position cmd
	cmdfromgene_sub = n.subscribe<asctec_hl_comm::mav_ctrl>("trajtodriver", 1, &TeleopIMU::minimumcmdCallback, this);


	//config_motion = asctec_mav_motion_planning::motion_planning_paraConfig::__getDefault__();
	  // bring up dynamic reconfigure
	motionconf_srv_ = new ReconfigureServer(pnh_);
	ReconfigureServer::CallbackType f = boost::bind(&TeleopIMU::cbmotionConfig, this, _1, _2);
	motionconf_srv_->setCallback(f);

	//determine if the RC data transmit position commands:
	flag_rc_cmd=1;


	//record the time
	time_doby_last=0;
	time = 0;

	//outdoor or indoor:
    //out door or indoor,
    //1: outdoor, GPS provides the position information
    //2: indoor, SLAM module provides the position information
    flag_pose_source = 2;


    //initialize the commands:
    global_position_cmd.x = 0;
    global_position_cmd.y = 0;
    global_position_cmd.z = 0;
    global_position_cmd.yaw = 0;

    //initialize the variables in the TLC position control
    {
    	omega_n_filter_out_trans[0] = 10.0;
    	omega_n_filter_out_trans[1] = 10.0;
    	omega_n_filter_out_trans[2] = 10.0;
    	xi_filter_out_trans[0] = 1.414;
    	xi_filter_out_trans[1] = 1.414;
    	xi_filter_out_trans[2] = 1.414;

    	omega_n_filter_in_trans[0] = 10.0;
    	omega_n_filter_in_trans[1] = 10.0;
    	omega_n_filter_in_trans[2] = 10.0;
		xi_filter_in_trans[0] = 1.414;
		xi_filter_in_trans[1] = 1.414;
		xi_filter_in_trans[2] = 1.414;

		xi_filter_out[0] = 1.414;
		xi_filter_out[1] = 1.414;
		xi_filter_out[2] = 1.414;
		omega_n_filter_out[0] = 10.0;
		omega_n_filter_out[1] = 10.0;
		omega_n_filter_out[2] = 10.0;

		xi_filter_in[0] = 1.414;
		xi_filter_in[1] = 1.414;
		xi_filter_in[2] = 1.414;
		omega_n_filter_in[0] = 20.0;
		omega_n_filter_in[1] = 20.0;
		omega_n_filter_in[2] = 20.0;

    	for (int i=0;i<3;i++) {
    		a_1_out_trans[i] = omega_n_filter_out_trans[i]*omega_n_filter_out_trans[i];
    		a_2_out_trans[i] = 2*xi_filter_out_trans[i]*omega_n_filter_out_trans[i];

    		a_1_in_trans[i] = omega_n_filter_in_trans[i]*omega_n_filter_in_trans[i];
    		a_2_in_trans[i] = 2*xi_filter_in_trans[i]*omega_n_filter_in_trans[i];

    		a_1_out[i] = omega_n_filter_out[i]*omega_n_filter_out[i];
    		a_2_out[i] = 2*xi_filter_out[i]*omega_n_filter_out[i];

    		a_1_in[i] = omega_n_filter_in[i]*omega_n_filter_in[i];
    		a_2_in[i] = 2*xi_filter_in[i]*omega_n_filter_in[i];
    	}

    	ksi_roll_out=2.6;
    	ksi_pitch_out=2.6;
    	ksi_yaw_out=1.7;
    	ksi_roll_in=3.4;
    	ksi_pitch_in=3.4;
    	ksi_yaw_in=2.3;

    	omega_roll_out=0.8;
    	omega_pitch_out=0.8;
    	omega_yaw_out=0.8;
    	omega_roll_in=3.5;
    	omega_pitch_in=3.5;
    	omega_yaw_in=3.2;

    	ksi_trans_out_x=1.6;
    	ksi_trans_out_y=1.6;
    	ksi_trans_out_z=2;
    	omega_trans_out_x=0.15;
    	omega_trans_out_y=0.15;
    	omega_trans_out_z=0.15;

    	ksi_trans_in_x=1.6;
    	ksi_trans_in_y=1.6;
    	ksi_trans_in_z=2;
    	omega_trans_in_x=0.4;
    	omega_trans_in_y=0.4;
    	omega_trans_in_z=0.4;

		//sampling time:
		T_sampling = 0.05;
		time_scale_position = 1;  //the scale of the sampling time of the position time compared to attitude loop

		m = 1.35;
		g_ = 9.8;
    }


    //initial instant of SLAM
    slam_int = 0;
    yaw_ini_slam = 0;
    slam_int_instant = 0;

    state_feedback.pose.orientation.x=0;
    state_feedback.pose.orientation.y=0;
    state_feedback.pose.orientation.z=0;
    state_feedback.pose.orientation.w=1;


}


void TeleopIMU::flagcmdCallback(const asctec_mav_motion_planning::flag_cmdConstPtr&  flagcmd){

	if (flagcmd->flag==1)
		flag_rc_cmd=0; //position cmd from RC transmitter is not used
	else if (flagcmd->flag==2)
		flag_rc_cmd=1; //position cmd from RC transmitter is used
}

void TeleopIMU::minimumcmdCallback(const asctec_hl_comm::mav_ctrlConstPtr& msg){

	//commands for the position control,
	//ENU frame to NED frame:
	P_nom[0] = msg->x;
	P_nom[1] = -msg->y;
	P_nom[2] = -msg->z;
	gamma_nom[2] = -msg->yaw;

}



void TeleopIMU::gpsdataCallback(const asctec_hl_comm::GpsCustomConstPtr& gpsdata){

 //use the GPS data as the external position and velocity

	//only used in test


//	std_msgs/Header header
//	geometry_msgs/Pose pose
//	geometry_msgs/Vector3 velocity


// gpsdata:
	LLA[0]= (gpsdata->latitude)/180.0*M_PI;
	LLA[1]= (gpsdata->longitude)/180.0*M_PI;
	LLA[2]= gpsdata->altitude;

	TeleopIMU::LLP_Euclidean(LLA);

	if (flag_pose_source==1) //outdoor, velocity
	{
		state_feedback.velocity.x=gpsdata->velocity_x;
		state_feedback.velocity.y=gpsdata->velocity_y;

		ROS_INFO_STREAM("feedback position x: "<<(state_feedback.velocity.x));
		ROS_INFO_STREAM("feedback position y: "<<(state_feedback.velocity.y));
		ROS_INFO_STREAM("feedback position z: "<<(state_feedback.velocity.z));
	}


	nav_msgs::Odometry msg;
	msg.pose.pose.position = state_feedback.pose.position;
	msg.twist.twist.linear = state_feedback.velocity;

	position_gps.publish(msg);

}

void TeleopIMU::imudataCallback(const asctec_hl_comm::mav_imuConstPtr& imudata){
//use the GPS height as the external height and z-velocity
	//only used in test

	if (flag_pose_source==1) //outdoor, velocity
	{
		state_feedback.pose.position.z= imudata->height;
		state_feedback.velocity.z=imudata->differential_height;
		state_feedback.pose.orientation =imudata->orientation;
	}

	//record the initial orientation
	if (slam_int_instant == 1)
	{
		slam_int_instant = 0;

		//below, get the yaw angle
		double quaternion[4];
		double R_temp[9];
		double gamma_temp[3];

		//notice the order of quaternion:
		quaternion[1] = imudata->orientation.x;
		quaternion[2] = imudata->orientation.y;
		quaternion[3] = imudata->orientation.z;
		quaternion[0] = imudata->orientation.w;

		math_function::quaternion_to_R(&quaternion[0], &R_temp[0]);
		//ENU frame
		math_function::RtoEulerangle(&R_temp[0], &gamma_temp[0]);

		//the initial yaw angle when the SLAM data is available, ENU frame
		yaw_ini_slam = gamma_temp[2];

	}

	//publish the external state for position control purpose

	////////test only////////////////////
//	state_feedback.pose.orientation.w=1;
//	state_feedback.pose.position.x=1000;
//	state_feedback.pose.position.y=25300;
//	state_feedback.pose.position.z=566;
	/////////////////////////////////////

	ext_state.publish(state_feedback);

	//test the frequency of the data:
//	int64_t ts_usec;
//	double ts_sec;
//	double time_body;
//	double T_sampling;
//
//	ts_usec = (uint64_t)(ros::WallTime::now().toSec() * 1.0e6);
//	time_body =(double)(ts_usec-time)/1.0e6;  //actual time used in calculation
//	T_sampling=time_body-time_doby_last;
//	time_doby_last=time_body;
//
// 	ROS_INFO_STREAM("current time (time_body)"<<(time_body));
// 	ROS_INFO_STREAM("current time (T_sampling)"<<(T_sampling));
}




void TeleopIMU::rcdataCallback(const asctec_hl_comm::mav_rcdataConstPtr& rcdata){
	//k_stick_yaw_
	//int k_stick_;

	//the command is in real angle and real angular velocity
	//roll and pitch angle cmd  =real_angle*1000/K_stick
	//yaw cmd=real_anglular_velocity*1000/k_stick_yaw,

	int64_t ts_usec;
	double ts_sec;
	double time_body;
	double T_sampling;

	{
		//the feedback information of the position, velocity, and yaw angle:
		//should be transformed from ENU to NED
		//the control law is expressed in NED
		//the feedback information is expressed in ENU:
		P_sen[0] = state_feedback.pose.position.x;
		P_sen[1] = -state_feedback.pose.position.y;
		P_sen[2] = -state_feedback.pose.position.z;

		V_sen[0] = state_feedback.velocity.x;
		V_sen[1] = -state_feedback.velocity.y;
		V_sen[2] = -state_feedback.velocity.z;

		//below, get the yaw angle
		double quaternion[4];
		double R_temp[9];
		double gamma_temp[3];

		//notice the order of quaternion:
		quaternion[1] = state_feedback.pose.orientation.x;
		quaternion[2] = state_feedback.pose.orientation.y;
		quaternion[3] = state_feedback.pose.orientation.z;
		quaternion[0] = state_feedback.pose.orientation.w;

		math_function::quaternion_to_R(&quaternion[0], &R_temp[0]);
		//ENU frame
		math_function::RtoEulerangle(&R_temp[0], &gamma_temp[0]);

		//NED frame:
		gamma_sen[0] = gamma_temp[0];
		gamma_sen[1] = -gamma_temp[1];
		gamma_sen[2] = -gamma_temp[2];
	}


	//notice T_sampling, very important:
	ts_usec = (uint64_t)(ros::WallTime::now().toSec() * 1.0e6);
	time_body =(double)(ts_usec-time)/1.0e6;  //actual time used in calculation
	T_sampling = time_body-time_doby_last;
	time_doby_last = time_body;

	//T_sampling = 0.05;

 	//ROS_INFO_STREAM("current time (time_body)"<<(time_body));
 	ROS_INFO_STREAM("current time (T_sampling)"<<(T_sampling));


	asctec_hl_comm::mav_ctrl msg;

	if  ((rcdata->channel[5]) < 1800 )
	{
		 msg.type = asctec_hl_comm::mav_ctrl::acceleration;

		 //note the rcdata is not the same with the command sent to LL from HL
		 //msg.x msg.y units: rad

		 msg.x =  (rcdata->channel[0]-2047) *k_stick_/1000.0*M_PI/180.0;  //pitch
		 msg.y =  (-rcdata->channel[1] + 2047) *k_stick_/1000.0*M_PI/180.0;   //opposite direction, roll
		 msg.yaw = (-rcdata->channel[3] + 2047) *k_stick_yaw_/1000.0*M_PI/180.0;   //opposite direction

		 msg.z = rcdata->channel[2]/4096.0;
	}

	if (((rcdata->channel[5]) > 1800 ) & ((rcdata->channel[5]) < 2500))
	{
		msg.type = asctec_hl_comm::mav_ctrl::velocity_body;


//		  ctrlLL.x = helper::clamp<short>(-2047, 2047, (short)(msg.x / config_.max_velocity_xy * 2047.0));
//		  ctrlLL.y = helper::clamp<short>(-2047, 2047, (short)(msg.y / config_.max_velocity_xy * 2047.0));
//		  ctrlLL.yaw = helper::clamp<short>(-2047, 2047, (short)(msg.yaw / config_.max_velocity_yaw* 2047.0));
//		  ctrlLL.z = helper::clamp<short>(-2047, 2047, (short)(msg.z / config_.max_velocity_z * 2047.0)) + 2047; // "zero" is still 2047!

		msg.x = -(rcdata->channel[0]-2047) /2047.0*config_motion.max_velocity_xy;
		msg.y = (-rcdata->channel[1] + 2047) /2047.0*config_motion.max_velocity_xy;
		msg.yaw = (-rcdata->channel[3] + 2047) /2047.0*config_motion.max_velocity_yaw;
		msg.z = ( rcdata->channel[2]-2047)/2047.0*config_motion.max_velocity_z;

	}


	if ((rcdata->channel[5]) > 4000 )
	{
		//msg.type = asctec_hl_comm::mav_ctrl::position;
		global_position_cmd.type = asctec_hl_comm::mav_ctrl::position;

		if (flag_rc_cmd == 1)
		{
			//position cmd is from RC transmitter, transmitter sends velocity commands:

			global_position_cmd.x =  global_position_cmd.x - T_sampling* (rcdata->channel[0]-2047) /2047.0*config_motion.max_velocity_xy;
			global_position_cmd.y =  global_position_cmd.y + T_sampling* (-rcdata->channel[1] + 2047) /2047.0*config_motion.max_velocity_xy;
			global_position_cmd.yaw =  global_position_cmd.yaw  + T_sampling* (-rcdata->channel[3] + 2047) /2047.0*config_motion.max_velocity_yaw;
			global_position_cmd.z = global_position_cmd.z + T_sampling* ( rcdata->channel[2]-2047)/2047.0*config_motion.max_velocity_z;

			//unit:m/s
			global_position_cmd.v_max_xy = 5;
			global_position_cmd.v_max_z= 5;



			//modified on Feb. 13, 2017, cancel this massege:
			//msg = global_position_cmd;

			//added on Feb. 2017, position control run in PC

			//ENU frame to NED frame:
			P_nom[0] = global_position_cmd.x;
			P_nom[1] = -global_position_cmd.y;
			P_nom[2] = -global_position_cmd.z;
			gamma_nom[2] = -global_position_cmd.yaw;
		}



		//run the position control law, all expressed in NED frame
		compute_V_nom();
		compute_F_nom();
		translate_outerloop_controller();
		translate_innerloop_controller();
		compute_gamma_nom();
		compute_omega_nom();
		rotate_outerloop_controller();

		//generate the thrust and attitude commands, notice, should be in in ENU frame:

		msg.type = asctec_hl_comm::mav_ctrl::acceleration;

		//note the rcdata is not the same with the command sent to LL from HL
		//msg.x =  gamma_nom[1];  //pitch angle
		//msg.y =  -gamma_nom[0];   //opposite direction, roll angle
		//msg.yaw = -omega_com[2];   //opposite direction, yaw rate
		//msg.z = 0.5*G/(m*g_);
	}



	if ( ((rcdata_last.channel[5])<4000) & ((rcdata->channel[5])>4000))
	{
		//initialize the original point, set the current position as the original point

		LLA_0 = LLA;

		//in GPS environment, the following function is used:
		TeleopIMU::LLP_Euclidean(LLA);

		//record the initial time
		time=(uint64_t)(ros::WallTime::now().toSec() * 1.0e6);

		{
			for (int i=0;i<3;i++)    //commanded value, control value and sensed value initialization
			{
				//P_com[i] = 0;
				P_nom[i] = 0;
				//P_sen[i] = 0;
				P_err[i] = 0;
				P_err_int[i] = 0;

				V_nom[i] = 0;
				V_com[i] = 0;
				V_ctrl[i] = 0;
				//V_sen[i] = 0;
				V_err[i] = 0;
				V_err_int[i] = 0;

				f_z_com=0;

				F_nom[i] = 0;
				F_com[i] = 0;
				F_ctrl[i] = 0;

				gamma_com[i] = 0;
				gamma_nom[i] = 0;
				//gamma_sen[i] = 0;
				gamma_err[i] = 0;
				gamma_err_int[i] = 0;
				gamma_ctrl[i]=0;

				omega_nom[i] = 0;
				omega_com[i] = 0;
				omega_ctrl[i] = 0;
				//omega_sen[i] = 0;
				omega_err[i] = 0;
				omega_err_int[i] = 0;
			}

			for (int i=0;i<3;i++)  //the temp variables in the psedodifferentiator
			{
				V_nom_filter_m2[i]=0;
				V_nom_filter_m1[i]=0;
				P_nom_filter_m2[i]=0;
				P_nom_filter_m1[i]=0;
				omega_nom_filter_m2[i]=0;
				omega_nom_filter_m1[i]=0;
				gamma_nom_filter_m2[i]=0;
				gamma_nom_filter_m1[i]=0;
			}

			phi_nom=0;     //the variables remember the value after filter in the inverse functions.
			theta_nom=0;
			psi_nom=0;
			p_nom=0;
			q_nom=0;
			r_nom=0;
			Px_nom=0;
			Py_nom=0;
			Pz_nom=0;
			Vx_nom=0;
			Vy_nom=0;
			Vz_nom=0;  //the variables remember the value after filter in the inverse functions.
			G=0;//initial total thrust

			//yaw angle:
			gamma_com[2] = gamma_sen[2];
			gamma_nom[2] = gamma_sen[2];
			gamma_nom_filter_m1[2]=gamma_sen[2];

		//initialize when transition from 3DOF to 6 DOF
			yaw_6DOF_init = gamma_sen[2]; //when transition, record the current yaw angle
			gamma_com[2] = yaw_6DOF_init;  //initialize the commands of yaw angle
			gamma_nom[2] = yaw_6DOF_init;   //initialize the commands of yaw angle
			//reset_yaw_control(); //initialize the commands of yaw angle

			G = rcdata->channel[2]/4096.0/0.5*m; //obtain the relative mass when transition from 3dof to 6dof
			G_6dof_init = G;  //record the thrust in the transition instant when from 3 DOF to 6 DOF
			m = G_6dof_init/g_;  //revise the mass accoording to the current total thrust command

			//computeR(&gamma_com[0], &Rcom_filter_m1[0][0]); //set the initial value of rotation matrix
	//		for(int i=0;i<3;i++)
	//		{
	//			for(int j=0;j<3;j++)
	//			{
	//				Rcom_filter_m1[i][j]=R_com[i][j];
	//			}
	//		}

			P_nom[0] = P_sen[0];
			P_nom[1] = P_sen[1];
			P_nom[2] = P_sen[2];

			Px_nom = P_sen[0];
			Py_nom = P_sen[1];
			Pz_nom = P_sen[2];

			Vx_nom = 0;
			Vy_nom = 0;
			Vz_nom = 0;

			P_nom_filter_m1[0] = P_sen[0]; //the filter value
			P_nom_filter_m1[1] = P_sen[1];
			P_nom_filter_m1[2] = P_sen[2];

			P_nom_filter_m2[0] = 0;
			P_nom_filter_m2[1] = 0;
			P_nom_filter_m2[2] = 0;

			V_nom_filter_m1[0] = 0;
			V_nom_filter_m1[1] = 0;
			V_nom_filter_m1[2] = 0;

			V_nom_filter_m2[0] = 0;
			V_nom_filter_m2[1] = 0;
			V_nom_filter_m2[2] = 0;
		}

	}

	if (((rcdata->channel[5])<4000) | (flag_rc_cmd != 1))
	{

		global_position_cmd.x = state_feedback.pose.position.x;
		global_position_cmd.y = state_feedback.pose.position.y;
		global_position_cmd.z = state_feedback.pose.position.z;

		//below, get the yaw angle
		double quaternion[4];
		double R_temp[9];
		double gamma_temp[3];

		//notice the order of quaternion:
		quaternion[1] = state_feedback.pose.orientation.x;
		quaternion[2] = state_feedback.pose.orientation.y;
		quaternion[3] = state_feedback.pose.orientation.z;
		quaternion[0] = state_feedback.pose.orientation.w;

		math_function::quaternion_to_R(&quaternion[0], &R_temp[0]);
		//ENU frame
		math_function::RtoEulerangle(&R_temp[0], &gamma_temp[0]);

		ROS_INFO_STREAM("feedback roll/degree: "<<gamma_temp[0]/3.14159265*180.0);
		ROS_INFO_STREAM("feedback pitch/degree: "<<gamma_temp[1]/3.14159265*180.0);
		ROS_INFO_STREAM("feedback yaw/degree: "<<gamma_temp[2]/3.14159265*180.0);

        //ENU frame, in rad
		global_position_cmd.yaw = (float)gamma_temp[2];
	}


	if (flag_rc_cmd == 1)
	{
		//only receive the commands from RC transmitter
		//ROS_INFO_STREAM("cmd , x: "<<msg.x);
		//ROS_INFO_STREAM("cmd  , y: "<<msg.y);
		//ROS_INFO_STREAM("cmd  , yaw: "<<msg.yaw);
		//ROS_INFO_STREAM("cmd  , z: "<<msg.z);
		//ROS_INFO_STREAM("global_position_cmd  , z: "<<global_position_cmd.z);
		//llcmd_pub_vel.publish(msg);
	}

    ext_state.publish(state_feedback);

    rcdata_last = *rcdata; //record the rcdata of last time

}





void TeleopIMU::callBack(const sensor_msgs::Imu::ConstPtr& imu)
{
    geometry_msgs::Twist vel;


    vel.linear.x = imu->angular_velocity.x;
    vel.linear.y = imu->angular_velocity.y;
    vel.linear.z = imu->angular_velocity.z;
    vel.angular.x = imu->angular_velocity.x;
    vel.angular.y = imu->angular_velocity.y;
    vel.angular.z = imu->angular_velocity.z;


    //vel.linear = '[2.0, 0.0, 0.0]';
   // vel.angular = '[2.0, 0.0, 0.0]';
    pub.publish(vel);

    //added on October, 10, 2016
    //the message is command to the simulated quadrotor in gazebo
    geometry_msgs::PoseStamped postoquadrotor;

    postoquadrotor.pose.position.x = 1;
    postoquadrotor.pose.position.y=1;
    postoquadrotor.pose.position.z =1;
    postoquadrotor.pose.orientation.x= imu->orientation.x;
    postoquadrotor.pose.orientation.y=imu->orientation.y;
    postoquadrotor.pose.orientation.z=imu->orientation.z;
    postoquadrotor.pose.orientation.w=imu->orientation.w;
    postoquadrotor.header.seq = imu->header.seq;
    postoquadrotor.header.stamp = imu->header.stamp;
   // postoquadrotor.header.frame_id = imu->header.frame_id;
    postoquadrotor.header.frame_id  =   "world";

    //publish the massege
    pub2.publish(postoquadrotor);

    geometry_msgs::TwistStamped velquadrotor;

    velquadrotor.header.frame_id ="world";
    velquadrotor.header.stamp =imu->header.stamp;
    velquadrotor.header.seq = imu->header.seq;

    velquadrotor.twist.linear.x = imu->angular_velocity.x;
    velquadrotor.twist.linear.y = imu->angular_velocity.y;
    velquadrotor.twist.linear.z = imu->angular_velocity.z;
    velquadrotor.twist.angular.x = imu->angular_velocity.x;
    velquadrotor.twist.angular.y = imu->angular_velocity.y;
    velquadrotor.twist.angular.z = imu->angular_velocity.z;

    //publish the massage
     pub3.publish(velquadrotor);

}

void TeleopIMU::send_acc_ctrl(void){

	//send the command in acc mode


	asctec_hl_comm::mav_ctrl msg;

	//the command is in real angle and real angular velocity
	//roll and pitch angle cmd  =real_angle*1000/K_stick

	//yaw cmd=real_anglular_velocity*1000/k_stick_yaw,


    msg.x = 1000;
    msg.y = 1000;
    msg.z = 0.5;

    msg.yaw = 1000* M_PI / 180.0;
    msg.v_max_xy = -4000; // use max velocity from config
    msg.v_max_z = 4000;


      msg.type = asctec_hl_comm::mav_ctrl::acceleration;
//    else if (type == "vel")
//      msg.type = asctec_hl_comm::mav_ctrl::velocity;
//    else if (type == "pos")
//      msg.type = asctec_hl_comm::mav_ctrl::position;
//    else if (type == "vel_b")
//      msg.type = asctec_hl_comm::mav_ctrl::velocity_body;
//    else if (type == "pos_b")
//      msg.type = asctec_hl_comm::mav_ctrl::position_body;
//    else
//    {
//      ROS_ERROR("Command type not recognized");
//      usage();
//      return -1;
//    }

    pub = n.advertise<asctec_hl_comm::mav_ctrl> ("fcu/control", 1);
//    for (int i = 0; i < 15; i++)
//    {
//      pub.publish(msg);
//      if (!ros::ok())
//        return 0;
//      r.sleep();
//    }

    while (ros::ok()){
    	llcmd_pub_acc.publish(msg);

    	ros::spinOnce();

    }
}


void TeleopIMU::send_velo_control(void){

	//send velocity command



}


void TeleopIMU::cbmotionConfig(asctec_mav_motion_planning::motion_planning_paraConfig & config, uint32_t level){

	ROS_INFO_STREAM("max_velocity_xy: "<<config.max_velocity_xy);

	//config = config_motion;

	config_motion = config;

	ROS_INFO_STREAM("max_velocity_xy: "<<config_motion.max_velocity_xy);

}



void TeleopIMU::LLP_Euclidean(Eigen::Vector3d & LLA)
//calculate the three position coordinate in the
//Locally North-East-Down tangent plane Euclidean linearized coordinate system
//note the definition of the origin of the NED LLP frame
{
	double lla_mit_g[3];
	double lla_mit_g_0[3];
	double P_sen_llp[3];  //the position in NED represent
	double R_earth=6378140; // 6378140m, the long radius of the earth
	double e_earth=0.0033539376905456;
	double tempP[3];
	double Rn, Re;  //Rn is the north direction curve radius, Re is the east direction curve radius
	int i;

	for(i=0;i<3;i++)
	{
		lla_mit_g[i]=LLA(i);
		lla_mit_g_0[i]=LLA_0(i);
	}

	tempP[0]= 1- e_earth*e_earth * sin(lla_mit_g[0])* sin(lla_mit_g[0]);
	Rn = R_earth*(1-e_earth*e_earth) / tempP[0]*sqrt(tempP[0]);
	Re = R_earth  / sqrt(tempP[0]);
	tempP[0]= Re*(lla_mit_g[0]-lla_mit_g_0[0]);
	tempP[1]= Rn* cos(lla_mit_g[0]) * (lla_mit_g[1]-lla_mit_g_0[1]);
	tempP[2]= -(lla_mit_g[2] -lla_mit_g_0[2]);
	for(i=0; i<3; i++)
	{
		P_sen_llp[i]= tempP[i];
	}

	if (flag_pose_source ==1) //outdoor, position, in ENU :
	{
		state_feedback.pose.position.x = P_sen_llp[0];
		state_feedback.pose.position.y = -P_sen_llp[1];
		//state_feedback.pose.position.z = -P_sen[2];
	}

}


void TeleopIMU::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose){

	if (flag_pose_source == 2) //indoor, position
	{
		//state_feedback.pose.position = pose->position;
		//state_feedback.pose.orientation = pose->orientation;

		//add the intial yaw angle:
		//below, get the yaw angle
		double quaternion[4];
		double R_temp[9];
		double gamma_temp[3];

		//notice the order of quaternion:
		quaternion[1] = pose->pose.orientation.x;
		quaternion[2] = pose->pose.orientation.y;
		quaternion[3] = pose->pose.orientation.z;
		quaternion[0] = pose->pose.orientation.w;

		math_function::quaternion_to_R(&quaternion[0], &R_temp[0]);
		//ENU frame
		math_function::RtoEulerangle(&R_temp[0], &gamma_temp[0]);

		//revise the yaw angle considering the initial angle
		gamma_temp[2] = gamma_temp[2] + yaw_ini_slam;

		//convert it to quaternion:
		double R_temp_2[9];
		double quaternion_2[4];

		math_function::computeR(&gamma_temp[0], &R_temp_2[0]);
		math_function::computequaternion(&R_temp_2[0], &quaternion_2[0]);

		state_feedback.pose.orientation.x = quaternion_2[1];
		state_feedback.pose.orientation.y = quaternion_2[2];
		state_feedback.pose.orientation.z = quaternion_2[3];
		state_feedback.pose.orientation.w = quaternion_2[0];

        //transform the pose
		state_feedback.pose.position.x = cos(yaw_ini_slam)*(pose->pose.position.x) - sin(yaw_ini_slam)*(pose->pose.position.y);
		state_feedback.pose.position.y = sin(yaw_ini_slam)*(pose->pose.position.x) + cos(yaw_ini_slam)*(pose->pose.position.y);
		state_feedback.pose.position.z =  pose->pose.position.z;

	}


	if (slam_int == 0)
	{
		//record the yaw angle only at the initial time
		slam_int = 1;
		slam_int_instant = 1;

	}

}


void TeleopIMU::odometryCallback(const nav_msgs::OdometryConstPtr& odometry){

	if (flag_pose_source == 2) //indoor, linear velocity
	{
		//state_feedback.velocity = odometry->twist.twist.linear;

		//transform from SLAM original frame to NWU frame:
		state_feedback.velocity.x = cos(yaw_ini_slam)*(odometry->twist.twist.linear.x) - sin(yaw_ini_slam)*(odometry->twist.twist.linear.y);
		state_feedback.velocity.y = sin(yaw_ini_slam)*(odometry->twist.twist.linear.x) + cos(yaw_ini_slam)*(odometry->twist.twist.linear.y);
		state_feedback.velocity.z =  odometry->twist.twist.linear.z;

		//transform from body-frame to reference frame:
		state_feedback.velocity.x =  odometry->twist.twist.linear.x;
		state_feedback.velocity.y =  odometry->twist.twist.linear.y;
		state_feedback.velocity.z =  odometry->twist.twist.linear.z;

		//convert it to quaternion:
		double R_temp[9];
		double q_temp[4];

		//notice the order of quaternion:
		q_temp[1] = state_feedback.pose.orientation.x;
		q_temp[2] = state_feedback.pose.orientation.y;
		q_temp[3] = state_feedback.pose.orientation.z;
		q_temp[0] = state_feedback.pose.orientation.w;

		math_function::quaternion_to_R(&q_temp[0], &R_temp[0]);

		state_feedback.velocity.x = R_temp[0]* state_feedback.velocity.x + R_temp[1]*state_feedback.velocity.y + R_temp[2]*state_feedback.velocity.z;
		state_feedback.velocity.y = R_temp[3]* state_feedback.velocity.x + R_temp[4]*state_feedback.velocity.y + R_temp[5]*state_feedback.velocity.z;
		state_feedback.velocity.z = R_temp[6]* state_feedback.velocity.x + R_temp[7]*state_feedback.velocity.y + R_temp[8]*state_feedback.velocity.z;

	}
}





int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_mav");

    ros::NodeHandle nh("motion_mav");

    TeleopIMU teopuav;

    ros::spin();

}



//added on Feb., 2016, position control
void TeleopIMU::translate_innerloop_controller(void)
	 //?????T_sampling
{
	int i;
//	 float ksi_trans_in_x=1.414f;
//	 float ksi_trans_in_y=1.414f;
//	 float ksi_trans_in_z=2.828f;
//	 float omega_trans_in_x=1.0f;
//	 float omega_trans_in_y=1.0f;
//	 float omega_trans_in_z=1.0f;

	 double alfa_412;
	 double alfa_422;
	 double alfa_432;
	 double alfa_411;
	 double alfa_421;
	 double alfa_431;

	 for(i=0;i<3;i++)
	 {
		 V_err[i]=V_com[i]-V_sen[i];
		 V_err_int[i]=V_err_int[i]+V_err[i]*(T_sampling*time_scale_position);  /*P_err,P_err_int ??????,????0*/
	 }

	 alfa_412=2*ksi_trans_in_x*omega_trans_in_x;
	 alfa_422=2*ksi_trans_in_y*omega_trans_in_y;
	 alfa_432=2*ksi_trans_in_z*omega_trans_in_z;
	 alfa_411=omega_trans_in_x*omega_trans_in_x;
	 alfa_421=omega_trans_in_y*omega_trans_in_y;
	 alfa_431=omega_trans_in_z*omega_trans_in_z;

	 F_ctrl[0] = V_err_int[0]* m * alfa_411 + V_err[0]* m * alfa_412;
	 F_ctrl[1] = V_err_int[1]* m * alfa_421 + V_err[1]* m * alfa_422;
	 F_ctrl[2] = V_err_int[2]* m * alfa_431 + V_err[2]* m * alfa_432;


	 for(i=0;i<3;i++)
		 F_com[i]=F_ctrl[i]+F_nom[i];  //command ??????????

	 //F_com[2]= -(G_6dof-G_6dof_init);	 //test mode, do not close the Z position
	 //if (command_source_flag!=0) //tune the altitude  controller
	 	//F_com[2]=F_ctrl[2]+F_nom[2];  //command ??????????
}


void TeleopIMU::translate_outerloop_controller(void)
	 /*control timing is T_sampling*/
{
	int i;
//	 float ksi_trans_out_x=1.414f;
//	 float ksi_trans_out_y=1.414f;
//	 float ksi_trans_out_z=2.828f;
//	 float omega_trans_out_x=0.25f;
//	 float omega_trans_out_y=0.25f;
//	 float omega_trans_out_z=0.25f;

	double alfa_312;
	double alfa_322;
	double alfa_332;
	double alfa_311;
	double alfa_321;
	double alfa_331;


	 for(i=0;i<3;i++)
	 {
		 P_err[i]=P_nom[i]-P_sen[i];
		 P_err_int[i]=P_err_int[i]+P_err[i]*(T_sampling*time_scale_position);  /*P_err,P_err_int ??????,????0*/
	 }

	 alfa_312=2*ksi_trans_out_x*omega_trans_out_x;
	 alfa_322=2*ksi_trans_out_y*omega_trans_out_y;
	 alfa_332=2*ksi_trans_out_z*omega_trans_out_z;
	 alfa_311=omega_trans_out_x*omega_trans_out_x;
	 alfa_321=omega_trans_out_y*omega_trans_out_y;
	 alfa_331=omega_trans_out_z*omega_trans_out_z;

	 V_ctrl[0] = P_err_int[0]*  alfa_311 + P_err[0]* alfa_312;
	 V_ctrl[1] = P_err_int[1]* alfa_321 + P_err[1]*  alfa_322;
	 V_ctrl[2] = P_err_int[2]*  alfa_331 + P_err[2]*   alfa_332;

	 for(i=0;i<3;i++)
		 V_com[i]=V_ctrl[i]+V_nom[i];  /*command ??????????*/
 }


void TeleopIMU::compute_F_nom(void)
{
	//T_sampling
	int i;
	double d_Vx_nom;
	double d_Vy_nom;
	double d_Vz_nom;
	double Vx_nom_last, Vy_nom_last, Vz_nom_last;

	Vx_nom_last=Vx_nom;
	Vy_nom_last=Vy_nom;
	Vz_nom_last=Vz_nom;

	///////filter /////////////
	for(i=0;i<3;i++)
	{V_nom_filter_m2[i]=(a_1_in_trans[i]*V_nom[i]-
	a_1_in_trans[i]*V_nom_filter_m1[i]-a_2_in_trans[i]*V_nom_filter_m2[i])*(T_sampling*time_scale_position)+V_nom_filter_m2[i];
	V_nom_filter_m1[i]=V_nom_filter_m2[i]*(T_sampling*time_scale_position)+V_nom_filter_m1[i];
	}
	//// ////////////

	Vx_nom=V_nom_filter_m1[0];
	Vy_nom=V_nom_filter_m1[1];
	Vz_nom=V_nom_filter_m1[2];

	d_Vx_nom=(Vx_nom-Vx_nom_last)/(T_sampling*time_scale_position);
	d_Vy_nom=(Vy_nom-Vy_nom_last)/(T_sampling*time_scale_position);
	d_Vz_nom=(Vz_nom-Vz_nom_last)/(T_sampling*time_scale_position);

	F_nom[0]=m*d_Vx_nom;
	F_nom[1]=m*d_Vy_nom;
	F_nom[2]=m*d_Vz_nom;
	//F_nom[0]=0;F_nom[1]=0;F_nom[2]=0;//tuning only
}


void TeleopIMU::compute_V_nom(void)
{
	//T_sampling?????
	int i;
	double d_Px_nom=0;
	double d_Py_nom=0;
	double d_Pz_nom=0; /*??????????????,????*/
	double Px_nom_last, Py_nom_last, Pz_nom_last;

	Px_nom_last=Px_nom;
	Py_nom_last=Py_nom;
	Pz_nom_last=Pz_nom;  /*?????????????,????*/

	/////////????//////////////
	for(i=0;i<3;i++)
	{
		P_nom_filter_m2[i]=(a_1_out_trans[i]*P_nom[i]-a_1_out_trans[i]*P_nom_filter_m1[i]-
												a_2_out_trans[i]*P_nom_filter_m2[i])*(T_sampling*time_scale_position)+P_nom_filter_m2[i];
		P_nom_filter_m1[i]=P_nom_filter_m2[i]*(T_sampling*time_scale_position)+P_nom_filter_m1[i];
	}
	/////////????//////////////

	Px_nom=P_nom_filter_m1[0];
	Py_nom=P_nom_filter_m1[1];
	Pz_nom=P_nom_filter_m1[2];   //the nominal position after filter

	d_Px_nom=(Px_nom-Px_nom_last)/(T_sampling*time_scale_position);
	d_Py_nom=(Py_nom-Py_nom_last)/(T_sampling*time_scale_position);
	d_Pz_nom=(Pz_nom-Pz_nom_last)/(T_sampling*time_scale_position);  //????,??????????

	V_nom[0]=d_Px_nom;
	V_nom[1]=d_Py_nom;
	V_nom[2]=d_Pz_nom;
//	V_nom[0]=0;V_nom[1]=0;V_nom[2]=0;//tuning only
}




void TeleopIMU::compute_gamma_nom(void)
{
	 //T_sampling?????
	int i;
//	float Fcom_exg[3];
//	float x_b[3]; //the x axis of the body fixed frame
//	float y_b[3]; //the y axis of the body fixed frame
//	float z_b[3]; //the z axis of the body fixed frame
//	float a_1[3]; //the project vector of the x axis of the body fixed frame on the xy plane of
//	//earth frame
	double tempfloat_norm;

	//gamma_nom[2]=yaw_6DOF_init;   //psi can be set to a value if needed

	Rzb_sens[0]=cos(gamma_sen[0])*sin(gamma_sen[1])*cos(gamma_sen[2])+sin(gamma_sen[0])*sin(gamma_sen[2]);
	Rzb_sens[1]=cos(gamma_sen[0])*sin(gamma_sen[1])*sin(gamma_sen[2])-sin(gamma_sen[0])*cos(gamma_sen[2]);
	Rzb_sens[2]=cos(gamma_sen[0])*cos(gamma_sen[1]);

	Fcom_exg[0]=F_com[0];
	Fcom_exg[1]=F_com[1];
	Fcom_exg[2]=F_com[2]-m*g_;	//F-[0; 0; m*g]

	tempfloat_norm = norm_vector(&Fcom_exg[0]);  // norm(F-[0; 0; m*g]);

	G=-dot_product(&Fcom_exg[0], &Rzb_sens[0]);  //the total thrust commands, G=-(F- [0; 0; m*g] )\cdot Rzb_sens
	f_z_com = -G;   //the total thrust, G>0
	//f_z_com<0, the total force gererated by the propellers algong the z direction of body frame

//	f_z_com=-norm_vector(&Fcom_exg[0]);  //T = norm(F-[0; 0; m*g]);
//    //f_z_com<0, the total force gererated by the propellers algong the z direction of body frame
//	G = -f_z_com;   //the total thrust, G>0

	for(i=0; i<3; i++)
	{
		z_b[i]= 0- Fcom_exg[i] / tempfloat_norm;  //z_b= -(F- [0; 0; m*g] )/ norm( F- [0; 0; m*g]);
	}


	a_1[0]= cos(gamma_nom[2]);
	a_1[1]= sin(gamma_nom[2]);
	a_1[2]= 0;   //a_1= [cos(psi); sin(psi); 0];

	//y_b = cross(z_b, a_1) / norm(cross(z_b, a_1));
	cross_vector(&z_b[0], &a_1[0], &y_b[0]);
	tempfloat_norm = norm_vector(&y_b[0]);
	for(i=0; i<3; i++)
	{
		y_b[i] = y_b[i] / tempfloat_norm;
		//y_b[i] = y_b[i];
	}

	// x_b = cross(y_b, z_b);
	cross_vector(&y_b[0], &z_b[0], &x_b[0]);
	tempfloat_norm = norm_vector(&x_b[0]);
	for(i=0; i<3; i++)
	{
		x_b[i] = x_b[i] / tempfloat_norm;
	}

	gamma_nom[1] = 0- asin(x_b[2]);  // theta = -asin(R(3,1));
	gamma_nom[0]= asin(y_b[2]/cos(gamma_nom[1])); //phi = asin (R(3,2)/cos_theta);

	for(i=0; i<2; i++)
	{
		gamma_com[i]= gamma_nom[i];
	}

	//in simulation test, in real time, should be commented:
//	gamma_com[0]=0;
//	gamma_com[0]=0;
}




void TeleopIMU::compute_omega_nom(void)
{
	//T_sampling?????
    int i;
	float d_phi_nom;
	float d_theta_nom;
	float d_psi_nom; /*????????????,????*/

	float phi_nom_last,theta_nom_last,psi_nom_last;

//	float xi_filter_out[3]={1.414f,1.414f,1.414f};  //????
//	float omega_n_filter_out[3]={5.0f,5.0f,5.0f};  //????
//	float a_1_out[3];
//	float a_2_out[3];  //????
//
//
//	for(i=0;i<3;i++)
//	{a_1_out[i]=omega_n_filter_out[i]*omega_n_filter_out[i];
//	a_2_out[i]=2*xi_filter_out[i]*omega_n_filter_out[i];
//	}  //????

	phi_nom_last=phi_nom;
	theta_nom_last=theta_nom;
	psi_nom_last=psi_nom;     /*??????????????(????????????),????*/

	/////////????//////////////
	for(i=0;i<3;i++)
	{gamma_nom_filter_m2[i]=(a_1_out[i]*gamma_nom[i]-a_1_out[i]*gamma_nom_filter_m1[i]-a_2_out[i]*gamma_nom_filter_m2[i])*T_sampling+gamma_nom_filter_m2[i];
	gamma_nom_filter_m1[i]=gamma_nom_filter_m2[i]*T_sampling+gamma_nom_filter_m1[i];
	}
	/////////????//////////////

	phi_nom=gamma_nom_filter_m1[0];
	theta_nom=gamma_nom_filter_m1[1];
	psi_nom=gamma_nom_filter_m1[2];   /*???????,????,???0,??????????*/

	d_phi_nom=(phi_nom-phi_nom_last)/T_sampling;
	d_theta_nom=(theta_nom-theta_nom_last)/T_sampling;
	d_psi_nom=(psi_nom-psi_nom_last)/T_sampling;   //??????,???????,?????

	omega_nom[0]=d_phi_nom-d_psi_nom*sin(gamma_nom[1]);
	omega_nom[1]=d_theta_nom*cos(gamma_nom[0])+d_psi_nom*sin(gamma_nom[0])*cos(gamma_nom[1]);
	omega_nom[2]=-d_theta_nom*sin(gamma_nom[0])+d_psi_nom*cos(gamma_nom[0])*cos(gamma_nom[1]);
	//omega_nom[0]=0;omega_nom[1]=0;omega_nom[2]=0;//tuning only
}




void TeleopIMU::rotate_outerloop_controller(void)
//?????T,?????,??
{
	int i,j,k;

// 	float ksi_roll_out=m_setpadlg.m_zita_phi;
// 	float ksi_pitch_out=m_setpadlg.m_zita_theta;
// 	float ksi_yaw_out=m_setpadlg.m_zita_psi;
// 	float omega_roll_out=m_setpadlg.m_omega_phi;
// 	float omega_pitch_out=m_setpadlg.m_omega_theta;
// 	float omega_yaw_out=m_setpadlg.m_omega_psi;

	float alfa_112;
	float alfa_122;
	float alfa_132;
	float alfa_111;
	float alfa_121;
	float alfa_131;

	float K_I1[3][3];
	float K_p1[3][3];  /*????????,????*/

	float temp1[3]={0,0,0},temp2[3]={0,0,0};  //????omega_ctrl[i]?P???I??

	for(i=0;i<2;i++)    /*??????????,??????*/
	{
		gamma_err[i]=gamma_com[i]-gamma_sen[i];
		gamma_err_int[i]=gamma_err_int[i]+gamma_err[i]*T_sampling;  /*gamma_err,gamma_err_int ??????,????0*/
	}

	gamma_err[2] = error_yaw(gamma_com[2], gamma_sen[2]);
	gamma_err_int[2]=gamma_err_int[2]+gamma_err[2]*T_sampling;

//	if(if6dofcontroller==0)   //if the current controller is 3 DOF controller, then use the velocity control for yaw channel
//	{
//  	gamma_err[2]=0;
//	    gamma_err_int[2]=0;    //if the yaw channel uses the angular velocity control, then uncomment these codes
//	}

	alfa_112=2*ksi_roll_out*omega_roll_out;
	alfa_122=2*ksi_pitch_out*omega_pitch_out;
	alfa_132=2*ksi_yaw_out*omega_yaw_out;
	alfa_111=omega_roll_out*omega_roll_out;
	alfa_121=omega_pitch_out*omega_pitch_out;
	alfa_131=omega_yaw_out*omega_yaw_out;

	K_I1[0][0]=alfa_111;
	K_I1[0][1]=0.0f;
	K_I1[0][2]=-alfa_131*sin(gamma_nom[1]);
	K_I1[1][0]=0.0f;
	K_I1[1][1]=alfa_121*cos(gamma_nom[0]);
	K_I1[1][2]=alfa_131*sin(gamma_nom[0])*cos(gamma_nom[1]);
	K_I1[2][0]=0.0f;
	K_I1[2][1]=-alfa_121*sin(gamma_nom[0]);
	K_I1[2][2]=alfa_131*cos(gamma_nom[0])*cos(gamma_nom[1]);

	K_p1[0][0]=alfa_112;
	K_p1[0][1]=omega_nom[1]*sin(gamma_nom[0])+omega_nom[2]*cos(gamma_nom[0]);
	K_p1[0][2]=-alfa_132*sin(gamma_nom[1]);
	K_p1[1][0]=-omega_nom[2];
	K_p1[1][1]=alfa_122*cos(gamma_nom[0])+(omega_nom[1]*sin(gamma_nom[0])+omega_nom[2]*cos(gamma_nom[0]))*sin(gamma_nom[0])*sin(gamma_nom[1])/cos(gamma_nom[1]);
	K_p1[1][2]=alfa_132*sin(gamma_nom[0])*cos(gamma_nom[1]);
	K_p1[2][0]=omega_nom[1];
	K_p1[2][1]=-alfa_122*sin(gamma_nom[0])+(omega_nom[1]*sin(gamma_nom[0])+omega_nom[2]*cos(gamma_nom[0]))*cos(gamma_nom[0])*sin(gamma_nom[1])/cos(gamma_nom[1]);
	K_p1[2][2]=alfa_132*cos(gamma_nom[0])*cos(gamma_nom[1]);

	for(j=0;j<3;j++)
	{
		for(k=0;k<3;k++)
			temp1[j]=temp1[j]+K_p1[j][k]*gamma_err[k];
	}      /*a?b??????????,??????,???????????0,?????????*/
	for(j=0;j<3;j++)
	{
		for(k=0;k<3;k++)
			temp2[j]=temp2[j]+K_I1[j][k]*gamma_err_int[k];
	}     /*a?b??????????,??????,???????????0,?????????*/
	for(i=0;i<3;i++)
		omega_ctrl[i]=temp1[i]+temp2[i];

	for(i=0;i<3;i++)
		omega_com[i]=omega_ctrl[i]+omega_nom[i];  /*command ??????????*/

//	if(if6dofcontroller==0)   //if the current controller is 3 DOF controller, then use the velocity control for yaw channel
//	omega_com[2] = -(InputPWM[3]-1519)/50.0f*0.017453f;  //the angular velocity of pitch channel is given by the RC transmitter
}



double TeleopIMU::error_yaw(double yaw_com, double yaw_sens)
{
	//calculate the error of the yaw angle
	double error;
	int i=0;
	error = yaw_com - yaw_sens;

//	if ((error> 3.14159265f)  &&  (error<= 9.42477796f))
//		error = error - 6.2831853f;
//	else if ((error> 9.42477796f ) &&  (error<=  15.7079632679490f))
//		error = error - 12.566370f;
//	else if ((error< -3.14159265f)  &&  (error>= -9.42477796f))
//		error = error + 6.2831853f;
//	else if ((error< -9.42477796f ) &&  (error>=  -15.7079632679490f))
//		error = error + 12.566370f;

	if (error>3.14159265f)
	{
		i=(int)(error/(6.283185307179586f)-0.5f);
		i=i+1;
		error=error-i*6.283185307179586f;
	}
	else if (error<-3.14159265f)
	{
		i=(int)(error/(6.283185307179586f)-0.5f);
		error=error-i*6.283185307179586f;
	}

	return error;
}





double TeleopIMU::norm_vector(double *p)
{
	//the norm of a three dimensional vector
	double y;
	y= 	sqrt((*p)*(*p)+(*(p+1)) * (*(p+1))+ (*(p+2)) * (*(p+2)));
	return y;
}



void TeleopIMU::cross_vector(double *a1, double *a2, double *result)
{   // result = a1 \times a2
	//the   Vector cross product.
//	 a2*b3 - a3*b2
// a3*b1 - a1*b3
// a1*b2 - a2*b1
	double temp[3];
	int i;
	temp[0] = (*(a1+1))* (*(a2+2)) - (*(a1+2))*(*(a2+1));
	temp[1] = (*(a1+2))* (*(a2+0)) - (*(a1+0))*(*(a2+2));
	temp[2] = (*(a1+0))* (*(a2+1)) - (*(a1+1))*(*(a2+0));
	for(i=0; i<3; i++)
	{
		*(result+i)= temp[i];
	}
}


double TeleopIMU::dot_product(double *a1, double *a2)
{
	//the   dot product of the two input vectors

	double temp;
	temp  = (*(a1+0))* (*(a2+0)) + (*(a1+1))*(*(a2+1))+  (*(a1+2))*(*(a2+2));

	return temp;
}











// %EndTag(FULLTEXT)%






