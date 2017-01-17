
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

	//the topic name is still under discussion, from the SLAM module
	pose_sub_ = nh_minsnap.subscribe("pose", 1, &Minimumsnap::poseCallback, this);

	//the topic name is still under discussion:
	cmd_sub_ = nh_minsnap.subscribe<nav_msgs::Path>("positioncmd", 1, &Minimumsnap::cmdCallback, this);


	rcdata_sub_ = nh_minsnap.subscribe<asctec_hl_comm::mav_rcdata>("fcu/rcdata", 1, &Minimumsnap::rcdataCallback, this);

	imu_custom_sub_ =nh_minsnap.subscribe<asctec_hl_comm::mav_imu>  ("fcu/imu_custom", 1, &Minimumsnap::imudataCallback, this);



	//init the flag values
	flag.calcmd=0;


	//the initial time once the commanded position is received
	begin_init.flag=0;

	current_point=0;
	Pnomflag =1;
	for(i=0;i<4;i++)
	{
		time_current[i]=0;
	}
	time_doby_last=0;




}

Minimumsnap::~Minimumsnap()
{

}


void Minimumsnap::rcdataCallback(const asctec_hl_comm::mav_rcdataConstPtr& rcdata){
	//serve as a trigger information, run the motion planning algorithm in this frequency
	//record the current time:

	int64_t ts_usec;
	float ts_sec;

	float time_body;


	if (begin_init.flag==0)
	{
		begin_init.time=(uint64_t)(ros::WallTime::now().toSec() * 1.0e6);
		begin_init.flag=1;

	}


	ts_usec = (uint64_t)(ros::WallTime::now().toSec() * 1.0e6);

	time_body =((float)(ts_usec-begin_init.time))/1.0e6;  //actual time used in calculation


	T_sampling=time_body-time_doby_last;

	time_doby_last=time_body;


 	ROS_INFO_STREAM("current time (ts_sec)"<<(time_body));
 	ROS_INFO_STREAM("current time (ts_usec)"<<(ts_usec));

	//map cruise trajectory, the trajectory follows minimum snap
	//use the following function:
	//trajectory planning of a strait line from start point to end point, minimum snap trajectory
	//float minimumsnap_line(float t0, float alpha, float x0, float xf, float time)
	{
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
			}

			//time_body =time_body+T_sampling;
		}

	}

}


void Minimumsnap::imudataCallback(const asctec_hl_comm::mav_imuConstPtr&   imudata){



	float quaternion[4];


	//notice the order of quaternion:
	quaternion[0] =imudata->orientation.x;
	quaternion[1] =imudata->orientation.y;
	quaternion[2] =imudata->orientation.z;
	quaternion[3] =imudata->orientation.w;


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
	//coefficients, rise order: A= [0; 0; 0; 7 ;-0.005942;-20.96748 ;20.9286 ;-5.922 ;-0.0423 ;0.0091;];

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




}



void Minimumsnap::cmdCallback(const nav_msgs::PathConstPtr& positioncmd){

	flag.calcmd=1;

	begin_init.flag=0;


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
	//notice all keep continous of the reset process
	//notice that before the reset, the commanded yaw angle many not in the range of [-pi,pi]
	//shoul keep stabel dring the reset process

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

  //the middel variables in the psedodifferentiator
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




