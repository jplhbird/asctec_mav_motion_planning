/*
 * mimimun_snap_traj.h
 *
 *  Created on: Dec 21, 2016
 *      Author: yu
 */

#ifndef ASCTEC_MAV_MOTION_PLANNING_SRC_MIMIMUN_SNAP_TRAJ_H_
#define ASCTEC_MAV_MOTION_PLANNING_SRC_MIMIMUN_SNAP_TRAJ_H_



#include "ros/ros.h"
#include "std_msgs/String.h"


// message includes
#include <asctec_hl_comm/mav_rcdata.h>
#include <asctec_hl_comm/mav_ctrl.h>
#include <asctec_hl_comm/mav_imu.h>
#include <asctec_hl_comm/mav_status.h>
#include <asctec_hl_comm/GpsCustom.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Path.h>


// dynamic reconfigure includes
#include <dynamic_reconfigure/server.h>
#include <asctec_mav_motion_planning/motion_planning_paraConfig.h>




class Minimumsnap{
public:
	Minimumsnap();
	~Minimumsnap();

private:

	ros::NodeHandle nh_minsnap;

    void poseCallback(const geometry_msgs::Pose::ConstPtr& pose);

	void cmdCallback(const nav_msgs::Path::ConstPtr& positioncmd);

	void rcdataCallback(const asctec_hl_comm::mav_rcdataConstPtr& rcdata);

	void imudataCallback(const asctec_hl_comm::mav_imuConstPtr&   imudata);

    //the function used to calculate the minimum trajectory from start point to end point:
    float minimumsnap_line(float t0, float alpha, float x0, float xf, float time);

    void quaternion_to_R(float *q, float *r);
    void RtoEulerangle(float *r, float *angle);


    ros::Publisher taj_pub;  //publish the calculated minimum trajectory
    ros::Publisher control_pub;  //publish the calculated minimum trajectory
    ros::Subscriber rcdata_sub_; //subscribe the rcdata from /fcu
    ros::Subscriber imu_custom_sub_;
    ros::Subscriber pose_sub_;  //subscribe the current position of the UAV
    ros::Subscriber cmd_sub_;  //subscribe the commanded position of the UAV



    struct
	{
    	int flag;
    	int64_t time;
    	int i_jump_no;

    	int current_point;
    	float timearray__mapcruise[200];

	}begin_init;




	//2017 Jan. added for map cruise:
	float points_mapcruise[3][20];
	float velocity_mapcruise[20];
	float yaw_mapcruise[20];
	float timearray__mapcruise[40];
	int i_mapcruise;  //no of commanded target points
	double ll_mapcruise[3];
	float point_current[3];
	float P_ini_cruise[3];
	int current_point;
	int  i_jump_no;
	float time_current[4];
	float yaw_6DOF_init;
	void reset_yaw_control();
	void rotate_yaw_mapcruise(int i);
	unsigned int Pnomflag; //flag determining which trajectory is to be used

	float T_sampling;
	float time_doby_last;

	float P_nom[3];
	float gamma_nom[3];
	float gamma_com[3];
	float P_sen[3];
	float gamma_sen[3];




    struct
    {
    	int calcmd;
    	float omega[3];
    	float vel[3];
    	float pos[3];
    }flag;  //the flag used to determine which function is to run


};





#endif /* ASCTEC_MAV_MOTION_PLANNING_SRC_MIMIMUN_SNAP_TRAJ_H_ */
