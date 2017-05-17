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
#include "math_function.h"


// message includes
#include <asctec_hl_comm/mav_rcdata.h>
#include <asctec_hl_comm/mav_ctrl.h>
#include <asctec_hl_comm/mav_imu.h>
#include <asctec_hl_comm/mav_status.h>
#include <asctec_hl_comm/GpsCustom.h>
#include <sensor_fusion_comm/ExtState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud.h>
#include <asctec_mav_motion_planning/flag_cmd.h>
#include <qpOASES.hpp>


// dynamic reconfigure includes
#include <dynamic_reconfigure/server.h>
#include <asctec_mav_motion_planning/motion_planning_paraConfig.h>


class Minimumsnap{
public:
	Minimumsnap();
	~Minimumsnap();

//    void quaternion_to_R(double *q, double *r);
//    void RtoEulerangle(double *r, double *angle);

private:

	ros::NodeHandle nh_minsnap;

    void poseCallback(const geometry_msgs::Pose::ConstPtr& pose);

	void cmdCallback(const nav_msgs::Path::ConstPtr& positioncmd);

	void rcdataCallback(const asctec_hl_comm::mav_rcdataConstPtr& rcdata);

	void imudataCallback(const asctec_hl_comm::mav_imuConstPtr&   imudata);

	void extstateCallback(const sensor_fusion_comm::ExtStateConstPtr& ext_state);

	//void virtual_dynamics()

    //the function used to calculate the minimum trajectory from start point to end point:
    float minimumsnap_line(float t0, float alpha, float x0, float xf, float time);


    ros::Publisher taj_pub;  //publish the calculated minimum trajectory
    ros::Publisher control_pub;  //publish the calculated minimum trajectory
    ros::Subscriber rcdata_sub_; //subscribe the rcdata from /fcu
    ros::Subscriber imu_custom_sub_;
    ros::Subscriber pose_sub_;  //subscribe the current position of the UAV, from SLAM module
    ros::Subscriber cmd_sub_;  //subscribe the commanded position of the UAV

    //determine if the cmd is transmitted from RC transmitter or PC
    //flag_cmd = 1: position command is give by PC
    //flag_cmd = 2: position command is give by RC transmitter
    ros::Publisher flag_cmd_pub;

    ros::Subscriber ext_state_sub_;

    struct
	{
    	int flag;
    	int64_t time;
    	int i_jump_no;

    	int current_point;
    	float timearray__mapcruise[200];

	}begin_init;




	//2017 Jan. added for map cruise:
	float points_mapcruise[3][200];
	float velocity_mapcruise[200];
	float yaw_mapcruise[200];
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

	//2017 May, added for the initialization of SLAM
	int slam_int; //flag determine if it is the initial time of SLAM
	double yaw_ini_slam; //the yaw angle at the initial time, when SLAM is available

	double T_sampling;
	double time_doby_last;

	float P_nom[3];
	float gamma_nom[3];
	float gamma_com[3];
	float P_sen[3];
	float gamma_sen[3];

    //determine if the computer sends the position commands
    int flag_pc_cmd;

    struct
    {
    	int calcmd;
    	float omega[3];
    	float vel[3];
    	float pos[3];
    }flag;  //the flag used to determine which function is to run


	//outdoor or indoor:
    //out door or indoor,
    //1: outdoor, GPS provides the position information
    //2: indoor, SLAM module provides the position information
    int flag_pose_source;

    //2017, April, added for obstacle avoiding
//
//    double t1[20000];  //the time array for the virtual dynamics
//    double y1[12][20000];  //the state output for the virtual dynamics
//    double tspan; //the time horizon used to calculate the obstacle avoiding algorithm
//    double y0[12]; //the intial state of the virtual dynamics
//    double obsta;
//    double pathpara;
//    double T_traj;
//    double u_virtual[3][20000]; //the control for the virtual dynamics




    struct trajd
	{
		double trajd_0d[3]; //desired trajectory
		double trajd_1d[3]; //1-st derivative of desired trajectory
		double trajd_2d[3]; //2-nd derivative of desired trajectory
		double trajd_3d[3]; //3-rd derivative of desired trajectory
		double trajd_4d[3]; //4-th derivative of desired trajectory
	};

    struct pathpara_str
    {
    	double start[3];
    	double end[3];
    	double v; //the average velocity from start point to end point
    };


    void virtual_dynamics(const double *y0, const double *u_virtual, pathpara_str &pathpara,
    		const sensor_msgs::PointCloud &obstac, double * tspan, double *t1, double*y1, int * n_dtime);
    void virtual_Control(const trajd& trajd_i, const double * y, const sensor_msgs::PointCloud &obstac,  double *u_virutal);
    void cbf_planning(const double *u,  double *out);
    void virtual_dynamics_onestep(double *x, const double *u, const double * deltat);
    void traj_snap_strait(const double *t0, const double *alpha, const double *x0, const double *xf, const double *t, double *out);
    void traj_gen(const double *t, const pathpara_str &pathpara, const double *T_orig, double *out);


    //for obstacle avoiding

    int flag_pc_cmd_obs;
	int current_point_obs;
	int i_mapcruise_obs;

	float points_mapcruise_obs[3][20000];
	float yaw_mapcruise_obs[20000];
	int flag_arrive;    //indicates if it arrives
	float accuracy_arrive;  //the accuracy indicate if arrive to the desired goal
	int flag_to_global;  //transmit to global planner
	float current_goal[3]; //the current goal
	float P_sen_obs[3]; //record the sensed position


//    function [t1,y1]=virtual_dynamics(quad_3d_ode, tspan, y0, options, obsta, pathpara, T_traj, current_hdl)
//
//
//    % quad_3d_ode(t, y, obsta, pathpara, T_orig, ctrl_hdl)
//    % [t1, y1] = ode45(@quad_3d_ode, tspan, y0, options, obsta, pathpara, T_traj, current_hdl);
//
//    dt=0.01;
//
//    n_state=length(y0);
//
//    t1=tspan(1):dt:tspan(end);
//    n_time=length(t1);
//
//    y1=zeros(n_time, n_state);
//    y=y0;
//
//    for i=2:n_time
//        t=t1(i);
//        dy = feval(quad_3d_ode, t, y, obsta, pathpara, T_traj, current_hdl);
//        y=y+dy*dt;
//        y1(i,:)=y';
//    end

};





#endif /* ASCTEC_MAV_MOTION_PLANNING_SRC_MIMIMUN_SNAP_TRAJ_H_ */
