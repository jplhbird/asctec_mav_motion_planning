
/*
 * minimum trajectory generation,
 *
 *  Created on: Dec 21, 2016
 *      Author: yu
 */


#include "minimum_snap_traj.h"




Minimumsnap::Minimumsnap()
{
	int i;

	taj_pub = nh_minsnap.advertise<asctec_hl_comm::mav_ctrl>("fcu/control",1); //command to HL_interface

	control_pub = nh_minsnap.advertise<asctec_hl_comm::mav_ctrl>("fcu/control",1); //command to HL_interface

	flag_cmd_pub = nh_minsnap.advertise<asctec_mav_motion_planning::flag_cmd>("flag_cmd",1); //flag determine which device will sends the position cmd

	//the topic name is still under discussion, from the SLAM module
	pose_sub_ = nh_minsnap.subscribe("posefromslam", 1, &Minimumsnap::poseCallback, this);

	//the topic name is still under discussion:
	cmd_sub_ = nh_minsnap.subscribe<nav_msgs::Path>("positioncmd", 1, &Minimumsnap::cmdCallback, this);

	rcdata_sub_ = nh_minsnap.subscribe<asctec_hl_comm::mav_rcdata>("fcu/rcdata", 1, &Minimumsnap::rcdataCallback, this);
	imu_custom_sub_ =nh_minsnap.subscribe<asctec_hl_comm::mav_imu>  ("fcu/imu_custom", 1, &Minimumsnap::imudataCallback, this);

	//gps environment:
	ext_state_sub_=nh_minsnap.subscribe<sensor_fusion_comm::ExtState>("fcu/state", 1, &Minimumsnap::extstateCallback, this); //external state, to interface of asctec


	//init the flag values
	flag.calcmd=0;


	//the initial time once the commanded position is received
	begin_init.flag=0;
	begin_init.time=0;

	current_point=0;
	Pnomflag =1;
	for(i=0;i<4;i++)
	{
		time_current[i]=0;
	}
	time_doby_last=0;

	i_jump_no=0;

	i_mapcruise=0;
	T_sampling =0.05;
	yaw_6DOF_init=0;

	flag_pc_cmd = 0;

	//outdoor or indoor:
    //out door or indoor,
    //1: outdoor, GPS provides the position information
    //2: indoor, SLAM module provides the position information
    flag_pose_source=2;

}

Minimumsnap::~Minimumsnap()
{

}


void Minimumsnap::rcdataCallback(const asctec_hl_comm::mav_rcdataConstPtr& rcdata){
	//serve as a trigger information, run the motion planning algorithm in this frequency
	//record the current time:

	int64_t ts_usec;
	double ts_sec;
	double time_body;


	if (begin_init.flag==0)
	{
		begin_init.time=(uint64_t)(ros::WallTime::now().toSec() * 1.0e6);
		begin_init.flag=1;

	}


	//notice T_sampling, very important:
	ts_usec = (uint64_t)(ros::WallTime::now().toSec() * 1.0e6);
	time_body =((double)(ts_usec-begin_init.time))/1.0e6;  //actual time used in calculation
	T_sampling=time_body-time_doby_last;
	time_doby_last=time_body;

//	T_sampling = 0.05;


 	ROS_INFO_STREAM("current time (time_body)"<<(time_body));
// 	ROS_INFO_STREAM("current time (T_sampling)"<<(T_sampling));

	//map cruise trajectory, the trajectory follows minimum snap
	//use the following function:
	//trajectory planning of a strait line from start point to end point, minimum snap trajectory
	//float minimumsnap_line(float t0, float alpha, float x0, float xf, float time)
 	if (flag_pc_cmd==1)
 	{
 		ROS_INFO_STREAM("current time (T_sampling)"<<(T_sampling));
		//record the yaw angle at the beginning of each line:
		if (i_jump_no==20)
		{
			reset_yaw_control();
			//time need to rotate:
			timearray__mapcruise[2*current_point]= time_body+ abs(yaw_mapcruise[current_point]-yaw_6DOF_init)/0.1745f;

			time_current[0]=time_body+ abs(yaw_mapcruise[current_point]-yaw_6DOF_init)/0.1745f;
			i_jump_no=30;
		}


		if ((time_body<=time_current[0]) && (i_jump_no==30))
		{
			//rotate the yaw angle to the set angle:
			rotate_yaw_mapcruise(current_point);
			//i_jump_no=40;
		}

		if ((time_body>time_current[0])&& (time_body<=time_current[0]+2))
		{
								//time need to line:
			timearray__mapcruise[2*current_point+1]= time_body+
			sqrt(
			(P_ini_cruise[0]- points_mapcruise[0][current_point])*(P_ini_cruise[0]- points_mapcruise[0][current_point])+
			(P_ini_cruise[1]-points_mapcruise[1][current_point])*(P_ini_cruise[1]- points_mapcruise[1][current_point])+
			(P_ini_cruise[2]-points_mapcruise[2][current_point])*(P_ini_cruise[2]- points_mapcruise[2][current_point])
			)/velocity_mapcruise[current_point];

			time_current[2]= time_body+
			sqrt(
			(P_ini_cruise[0]- points_mapcruise[0][current_point])*(P_ini_cruise[0]- points_mapcruise[0][current_point])+
			(P_ini_cruise[1]-points_mapcruise[1][current_point])*(P_ini_cruise[1]- points_mapcruise[1][current_point])+
			(P_ini_cruise[2]-points_mapcruise[2][current_point])*(P_ini_cruise[2]- points_mapcruise[2][current_point])
			)/velocity_mapcruise[current_point];

			//time_body=time_body+T_sampling;

			i_jump_no=50;

		}



		//line:
		if ((time_body>time_current[0]+2)&& (time_body<=time_current[2]) && (i_jump_no==50))
		{
			int j;
			//minimum snap trajectory:
			for(j=0;j<3;j++)
			{
				//float minimumsnap_line(float t0, float alpha, float x0, float xf, float time)
				P_nom[j]= minimumsnap_line(timearray__mapcruise[2*current_point]+2,
				timearray__mapcruise[2*current_point+1]-timearray__mapcruise[2*current_point]-2,
				P_ini_cruise[j], points_mapcruise[j][current_point], time_body);
			}
			//time_body =time_body+T_sampling;
			//i_jump_no=100;
		}

		if ((time_body>time_current[2])&& (time_body<=5.0f+time_current[2]) && (i_jump_no==50))
		{
			//hold on:
			if((time_body>=4.8f+time_current[2])&&(i_jump_no==50))
			{
				//next line:
				current_point++;

				i_jump_no=20;

				time_current[0]=0;
				time_current[2]=0;
				time_current[1]=0;
			}

			if(current_point>=i_mapcruise)
			{
				//finish cruise:
				Pnomflag =1;

				asctec_mav_motion_planning::flag_cmd flag_topic;
				flag_topic.flag=2; //1: position is give by PC, 2: position is give by RC transmitter
				flag_cmd_pub.publish(flag_topic);

				flag_pc_cmd=0;
			}

			//time_body =time_body+T_sampling;
		}

		asctec_hl_comm::mav_ctrl msg;

		//important, notice the unit and the definition of the coordinate frame:
		msg.x = P_nom[0];
		msg.y = P_nom[1];
		msg.z = P_nom[2];
		msg.yaw = gamma_com[2];
		msg.type = asctec_hl_comm::mav_ctrl::position;
		//unit:m/s
		msg.v_max_xy = 5;
		msg.v_max_z= 5;

		taj_pub.publish(msg);

		//show it in terminal:
		ROS_INFO_STREAM("cmd , x: "<<msg.x);
		ROS_INFO_STREAM("cmd , y: "<<msg.y);
		ROS_INFO_STREAM("cmd , yaw: "<<msg.yaw);
		ROS_INFO_STREAM("cmd , z: "<<msg.z);
	}

}


void Minimumsnap::imudataCallback(const asctec_hl_comm::mav_imuConstPtr&   imudata){

	double quaternion[4];
	double R_temp[9];
	double gamma_temp[3];


	//notice the order of quaternion:
	quaternion[1] =imudata->orientation.x;
	quaternion[2] =imudata->orientation.y;
	quaternion[3] =imudata->orientation.z;
	quaternion[0] =imudata->orientation.w;

	math_function::quaternion_to_R(&quaternion[0], &R_temp[0]);

	//ENU frame
	math_function::RtoEulerangle(&R_temp[0], &gamma_temp[0]);

	//expressed in ENU definition, important:
	gamma_sen[0]= (float)gamma_temp[0];
	gamma_sen[1]= (float)gamma_temp[1];
	gamma_sen[2]= (float)gamma_temp[2];


	//used to test:
//	gamma_sen[0]=gamma_sen[0]/3.14159265359*180;
//	gamma_sen[1]=gamma_sen[1]/3.14159265359*180;
//	gamma_sen[2]=gamma_sen[2]/3.14159265359*180;
//
//
//	ROS_INFO_STREAM("r: "<<(R_temp[8]));
//
//	ROS_INFO_STREAM("q: "<<(quaternion[0]));
//
//	ROS_INFO_STREAM("phi: "<<(gamma_sen[0]));
//	ROS_INFO_STREAM("theta: "<<(gamma_sen[1]));
//	ROS_INFO_STREAM("psi: "<<(gamma_sen[2]));


}



//trajectory planning of a strait line from start point to end point, minimum snap trajectory
float Minimumsnap::minimumsnap_line(float t0, float alpha, float x0, float xf, float time)
{
	//t0: start time
	//alpha: time factor, the total time
	//x0: start state
	//xf: final state
	//time: current time
	//Return: the planned current state
	float x_c;
	float  tau_i;
	float x_tilde;

	//non-dimensional time:
	tau_i=(time-t0)/alpha;

	// non-dimensional state:
	//9-order polynomials:
	//coefficients, rise order: A= [0; 0; 0; 7 ;-0.00594poses2;-20.96748 ;20.9286 ;-5.922 ;-0.0423 ;0.0091;];

	//9-order polynomial non-dimensional state:
	x_tilde  =  7.0f*      tau_i*tau_i*tau_i
							-0.005942f*tau_i*tau_i*tau_i*tau_i
							-20.96748f*tau_i*tau_i*tau_i*tau_i*tau_i
							+ 20.9286f*tau_i*tau_i*tau_i*tau_i*tau_i*tau_i
							-5.922f *  tau_i*tau_i*tau_i*tau_i*tau_i*tau_i*tau_i
							-0.0423f*  tau_i*tau_i*tau_i*tau_i*tau_i*tau_i*tau_i*tau_i
							 +0.0091f *tau_i*tau_i*tau_i*tau_i*tau_i*tau_i*tau_i*tau_i*tau_i;

	//dimensional state:
	x_c = x0+ (xf-x0) * x_tilde;

	return(x_c);
}



void Minimumsnap::poseCallback(const geometry_msgs::Pose::ConstPtr& pose){


	//outdoor or indoor:
    //out door or indoor,
    //1: outdoor, GPS provides the position information
    //2: indoor, SLAM module provides the position information
    if (flag_pose_source == 2)
    {
		P_sen[0]=pose->position.x;
		P_sen[1]=pose->position.y;
		P_sen[2]=pose->position.z;


    }

}


void Minimumsnap::extstateCallback(const sensor_fusion_comm::ExtStateConstPtr& ext_state){

	//outdoor or indoor:
    //out door or indoor,
    //1: outdoor, GPS provides the position information
    //2: indoor, SLAM module provides the position information
    if (flag_pose_source == 1)
    {
		P_sen[0]=ext_state->pose.position.x;
		P_sen[1]=ext_state->pose.position.y;
		P_sen[2]=ext_state->pose.position.z;
    }

}






void Minimumsnap::cmdCallback(const nav_msgs::PathConstPtr& positioncmd){

	flag.calcmd=1;

	begin_init.flag=0;

	i_mapcruise = positioncmd->poses.size();

	for(int i = 0; i < positioncmd->poses.size(); i++)
	{
		geometry_msgs::Pose wp = positioncmd->poses[i].pose;

//		float points_mapcruise[3][20];
//		float velocity_mapcruise[20];
//		float yaw_mapcruise[20];
//		float timearray__mapcruise[40];

		points_mapcruise[0][i] = wp.position.x;
		points_mapcruise[1][i] = wp.position.y;
		points_mapcruise[2][i] = wp.position.z;

		double quaternion[4];
		double R_temp[9];
		double gamma_temp[3];


		//notice the order of quaternion:
		quaternion[1] =wp.orientation.x;
		quaternion[2] =wp.orientation.y;
		quaternion[3] =wp.orientation.z;
		quaternion[0] =wp.orientation.w;

		math_function::quaternion_to_R(&quaternion[0], &R_temp[0]);
		math_function::RtoEulerangle(&R_temp[0], &gamma_temp[0]);

		//expressed in ENU definition, important, according to the definition of the body-fixed frame, we should modified the following
//		gamma_temp[1]= -gamma_temp[1];
//		gamma_temp[2]= -gamma_temp[2];

		//commanded yaw angle, unit is in rad:
		yaw_mapcruise[i]=gamma_temp[2];

		//commanded speed, should be adjusted according to the actual situation
		velocity_mapcruise[i]=1;
	}


	Pnomflag =100; //exclude other commands
	//time_body=0;
	current_point=0;
	i_jump_no=20;
	for(int i=0;i<4;i++)
	{
		time_current[i]=0.0f;
	}


	asctec_mav_motion_planning::flag_cmd flag_topic;
	flag_topic.flag=1; //1: position is give by PC, 2: position is give by RC transmitter
	flag_cmd_pub.publish(flag_topic);

	flag_pc_cmd=1; //activate the flag determine the computer sends the commands,

	ROS_INFO_STREAM("received path commands, flag_pc_cmd is set to)"<<(flag_pc_cmd));

}


void Minimumsnap::rotate_yaw_mapcruise(int i)
{
	float timp_elapse;

	//rotate the yaw angle to the set angle: yaw_mapcruise[i]

	gamma_nom[2]= gamma_nom[2] + (yaw_mapcruise[i]-yaw_6DOF_init)/abs(yaw_mapcruise[i]-yaw_6DOF_init)
	*0.1745f* T_sampling;  //10deg/s ,
	gamma_com[2] = gamma_nom[2];

	//time_body =time_body+T_sampling;
}


void Minimumsnap::reset_yaw_control(void)
{
	//reset the yaw angle command to be equal to the current sensed yaw angle, in the range of [-pi,pi]
	//notice all keep continuous of the reset process
	//notice that before the reset, the commanded yaw angle many not in the range of [-pi,pi]
	//should keep stable during the reset process

	{
		yaw_6DOF_init = gamma_sen[2]; //when transition, record the current yaw angle
		gamma_com[2] = yaw_6DOF_init;  //initialize the commands of yaw angle
		gamma_nom[2] = yaw_6DOF_init;   //initialize the commands of yaw angle
	}


	P_ini_cruise[0]= P_sen[0];
	P_ini_cruise[1]= P_sen[1];
	P_ini_cruise[2]= P_sen[2];

	//below, record the current tracking error in the control, in planning, it may not be used:
/*
  //commanded value, control value and sensed value initialization
	{
		gamma_err[2] = 0;
		gamma_err_int[2] = 0;
		gamma_ctrl[2]=0;
	}

  //the middle variables in the pusedodifferentiator
	{
		gamma_nom_filter_m2[2]=0;
		gamma_nom_filter_m1[2]=yaw_6DOF_init;
	}

	psi_nom=yaw_6DOF_init;
	*/
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "minimum_snap");

    ros::NodeHandle nh("minimum_snap");

    Minimumsnap minimumfun;

    ros::spin();

}




