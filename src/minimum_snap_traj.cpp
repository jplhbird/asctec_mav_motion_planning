
/*
 * minimum trajectory generation,
 *
 *  Created on: Dec 21, 2016
 *      Author: yu
 */


#include "minimum_snap_traj.h"




Minimumsnap::Minimumsnap(ros::NodeHandle & nh):
		nh_minsnap(nh)

{

	taj_pub = nh_minsnap.advertise<asctec_hl_comm::mav_ctrl>("fcu/control",1); //command to HL_interface

	control_pub = nh_minsnap.advertise<asctec_hl_comm::mav_ctrl>("fcu/control",1); //command to HL_interface

	//the topic name is still under discussion, from the SLAM module
	pose_sub_ = nh_minsnap.subscribe("pose", 1, &Minimumsnap::poseCallback, this);

	//the topic name is still under discussion:
	cmd_sub_ = nh_minsnap.subscribe<nav_msgs::Path>("positioncmd", 1, &Minimumsnap::cmdCallback, this);


	rcdata_sub_ = nh_minsnap.subscribe<asctec_hl_comm::mav_rcdata>("fcu/rcdata", 1, &Minimumsnap::rcdataCallback, this);



	//init the flag values
	flag.calcmd=0;


}

Minimumsnap::~Minimumsnap()
{

}


void Minimumsnap::rcdataCallback(const asctec_hl_comm::mav_rcdataConstPtr& rcdata){
	//serve as a trigger information, run the motion planning algorithm in this frequency


	//record the current time:

	int64_t ts_usec;
	float ts_sec;


	ts_usec = (uint64_t)(ros::WallTime::now().toSec() * 1.0e6);

	ts_sec =((float)ts_usec)/1.0e6;

//	ROS_INFO_STREAM("current time (sec)"<<(ts_sec));


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



}




