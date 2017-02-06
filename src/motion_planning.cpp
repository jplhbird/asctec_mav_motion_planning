
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
	pose_sub_ = n.subscribe("posefromslam", 1, &TeleopIMU::poseCallback, this);

	//the topic name is still under discussion, from the SLAM module
	odometry_sub_ = n.subscribe<nav_msgs::Odometry>("odometryfromslam", 1, &TeleopIMU::odometryCallback, this);
	rcdata_sub_ = n.subscribe<asctec_hl_comm::mav_rcdata>("fcu/rcdata", 1, &TeleopIMU::rcdataCallback, this);
	gps_custom_sub_ =n.subscribe<asctec_hl_comm::GpsCustom> ("fcu/gps_custom", 1, &TeleopIMU::gpsdataCallback, this);
	imu_custom_sub_ =n.subscribe<asctec_hl_comm::mav_imu>  ("fcu/imu_custom", 1, &TeleopIMU::imudataCallback, this);
	flag_cmd_sub = n.subscribe<asctec_mav_motion_planning::flag_cmd>("flag_cmd", 1, &TeleopIMU::flagcmdCallback, this); //flag determine which device will sends the position cmd



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


}


void TeleopIMU::flagcmdCallback(const asctec_mav_motion_planning::flag_cmdConstPtr&  flagcmd){

	if (flagcmd->flag==1)
		flag_rc_cmd=0; //position cmd from RC transmitter is not used
	else if (flagcmd->flag==2)
		flag_rc_cmd=1; //position cmd from RC transmitter is used
}



void TeleopIMU::gpsdataCallback(const asctec_hl_comm::GpsCustomConstPtr& gpsdata){

 //use the GPS data as the external position and velocity

	//only used in test



//	std_msgs/Header header
//	geometry_msgs/Pose pose
//	geometry_msgs/Vector3 velocity



// gpsdata:
	LLA[0]= gpsdata->latitude;
	LLA[1]= gpsdata->longitude;
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

	//publish the external state for position control purpose
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


	//notice T_sampling, very important:
	ts_usec = (uint64_t)(ros::WallTime::now().toSec() * 1.0e6);
	time_body =(double)(ts_usec-time)/1.0e6;  //actual time used in calculation
	T_sampling = time_body-time_doby_last;
	time_doby_last = time_body;

	T_sampling = 0.05;

 	ROS_INFO_STREAM("current time (time_body)"<<(time_body));
 	ROS_INFO_STREAM("current time (T_sampling)"<<(T_sampling));


	asctec_hl_comm::mav_ctrl msg;

	if  ((rcdata->channel[5]) < 1800 )
	{
		 msg.type = asctec_hl_comm::mav_ctrl::acceleration;

		 //note the rcdata is not the same with the command sent to LL from HL

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
		}

		msg = global_position_cmd;
	}



	if ( ((rcdata_last.channel[5])<4000) & ((rcdata->channel[5])>4000))
	{
		//initialize the original point, set the current position as the original point

		LLA_0 = LLA;

		//in GPS environment, the following function is used:
		TeleopIMU::LLP_Euclidean(LLA);

		//record the initial time
		time=(uint64_t)(ros::WallTime::now().toSec() * 1.0e6);
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

        //ENU frame, in rad
		global_position_cmd.yaw = (float)gamma_temp[2];
	}


	if (flag_rc_cmd == 1)
	{
		//only receive the commands from RC transmitter
		ROS_INFO_STREAM("cmd , x: "<<msg.x);
		ROS_INFO_STREAM("cmd  , y: "<<msg.y);
		ROS_INFO_STREAM("cmd  , yaw: "<<msg.yaw);
		ROS_INFO_STREAM("cmd  , z: "<<msg.z);
		ROS_INFO_STREAM("global_position_cmd  , z: "<<global_position_cmd.z);
		llcmd_pub_vel.publish(msg);
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


	float P_sen[3];  //the position in NED represent

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
		P_sen[i]= (float)tempP[i];
	}

	if (flag_pose_source ==1) //outdoor, position, in ENU :
	{
		state_feedback.pose.position.x = P_sen[0];
		state_feedback.pose.position.y = -P_sen[1];
		state_feedback.pose.position.z = -P_sen[2];
	}

}


void TeleopIMU::poseCallback(const geometry_msgs::Pose::ConstPtr& pose){

	if (flag_pose_source == 2) //indoor, position
	{
		state_feedback.pose.position = pose->position;
		state_feedback.pose.orientation = pose->orientation;
	}
}


void TeleopIMU::odometryCallback(const nav_msgs::OdometryConstPtr& odometry){

	if (flag_pose_source == 2) //indoor, linear velocity
	{
		state_feedback.velocity = odometry->twist.twist.linear;
	}

}





int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_mav");

    ros::NodeHandle nh("motion_mav");

    TeleopIMU teopuav;

    ros::spin();

}


// %EndTag(FULLTEXT)%






