
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

		ROS_INFO_STREAM("feedback roll/degree"<<gamma_temp[0]/3.14159265*180.0);
		ROS_INFO_STREAM("feedback pitch/degree"<<gamma_temp[1]/3.14159265*180.0);
		ROS_INFO_STREAM("feedback yaw/degree"<<gamma_temp[2]/3.14159265*180.0);

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






