
/*
 * minimum trajectory generation,
 *
 *  Created on: Dec 21, 2016
 *      Author: yu
 */


#include "minimun_snap_traj.h"




Minimumsnap::Minimumsnap(ros::NodeHandle & nh):
		nh_minsnap(nh)

{

	taj_pub = nh_minsnap.advertise<asctec_hl_comm::mav_ctrl>("fcu/control",1); //command to HL_interface

	control_pub = nh_minsnap.advertise<asctec_hl_comm::mav_ctrl>("fcu/control",1); //command to HL_interface

	//the topic name is still under discussion:
	pose_sub_ = nh_minsnap.subscribe("pose", 1, &Minimumsnap::poseCallback, this);

	//the topic name is still under discussion:
	cmd_sub_ = nh_minsnap.subscribe<nav_msgs::Path>("positioncmd", 1, &Minimumsnap::cmdCallback, this);


	//init the flag values
	flag.calcmd=0;


}

Minimumsnap::~Minimumsnap()
{

}



//trajectory planning of a strait line from start point to end point, munimum snap trajectory
float Minimumsnap::minimumsnap_line(float t0, float alpha, float x0, float xf, float time)
{
	//t0: start time
	//alpha: time factor, the total time
	//x0: start state
	//xf: final state
	//time: current time
	//returen: the planned current state
	float x_c;
	float  tau_i;
	float x_tilde;

	//non-dimensional time:
	tau_i=(time-t0)/alpha;

	// non-dimensional state:
	//9-order polynomials:
	//coeffients, accend order: A= [0; 0; 0; 7 ;-0.005942;-20.96748 ;20.9286 ;-5.922 ;-0.0423 ;0.0091;];

	//9-order pllinomials non-dimensional state:
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




