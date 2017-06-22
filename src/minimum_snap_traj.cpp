
/*
 * minimum trajectory generation,
 *
 *  Created on: Dec 21, 2016
 *      Author: yu
 */


#include "minimum_snap_traj.h"
#include <qpOASES.hpp>



Minimumsnap::Minimumsnap()
{
	int i;

	taj_pub = nh_minsnap.advertise<asctec_hl_comm::mav_ctrl>("fcu/control",1); //command to HL_interface

	control_pub = nh_minsnap.advertise<asctec_hl_comm::mav_ctrl>("fcu/control",1); //command to HL_interface

	flag_cmd_pub = nh_minsnap.advertise<asctec_mav_motion_planning::flag_cmd>("flag_cmd",1); //flag determine which device will sends the position cmd

	//the topic name is still under discussion, from the SLAM module
	pose_sub_ = nh_minsnap.subscribe("robot_pose", 1, &Minimumsnap::poseCallback, this);

	//the topic name is still under discussion:
	cmd_sub_ = nh_minsnap.subscribe<nav_msgs::Path>("positioncmd", 1, &Minimumsnap::cmdCallback, this);

	rcdata_sub_ = nh_minsnap.subscribe<asctec_hl_comm::mav_rcdata>("fcu/rcdata", 1, &Minimumsnap::rcdataCallback, this);
	imu_custom_sub_ =nh_minsnap.subscribe<asctec_hl_comm::mav_imu>  ("fcu/imu_custom", 1, &Minimumsnap::imudataCallback, this);

	//gps environment:
	ext_state_sub_=nh_minsnap.subscribe<sensor_fusion_comm::ExtState>("fcu/state", 1, &Minimumsnap::extstateCallback, this); //external state, to interface of asctec

	//Subscribe the point cloud2, the topic name should be modified according to the actual situation:
	pc_sub_ =nh_minsnap.subscribe<sensor_msgs::PointCloud2>  ("pcl_output", 1, &Minimumsnap::pcCallback, this);  //published from the test_path_cmd

	// published from the simulation module
	//pc_sub_ =nh_minsnap.subscribe<sensor_msgs::PointCloud2>  ("/pelican/vi_sensor/camera_depth/depth/points", 1, &Minimumsnap::pcCallback, this);

	//publish the point cloud, we transform the point cloud 2 to point cloud, and make some process to it
	point_could_pub_ = nh_minsnap.advertise<sensor_msgs::PointCloud>("point_could_pub",1);


	//init the flag values
	flag.calcmd=0;


	//the initial time once the commanded position is received
	begin_init.flag=0;
	begin_init.time=0;

	current_point=-1;
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
    flag_pose_source = 1;

    //initial instant of SLAM
    slam_int = 0;
    yaw_ini_slam = 0;

    //for obstacle avoiding:
    flag_pc_cmd_obs = 0;
    current_point_obs = -1;
    accuracy_arrive = 0.5;
    flag_arrive = 0;
    i_mapcruise_obs = 0;
    flag_to_global = 0;

    enable_obs_avoid = 1;  //during test, this flag can enable or disable the obstacle avoiding module

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

	//T_sampling = 0.05;


 //	ROS_INFO_STREAM("current time (time_body)"<<(time_body));
// 	ROS_INFO_STREAM("current time (T_sampling)"<<(T_sampling));

	//map cruise trajectory, the trajectory follows minimum snap
	//use the following function:
	//trajectory planning of a strait line from start point to end point, minimum snap trajectory
	//float minimumsnap_line(float t0, float alpha, float x0, float xf, float time)

	if (enable_obs_avoid == 0)
	{
		if (flag_pc_cmd==1)
		{
	// 		ROS_INFO_STREAM("current time (T_sampling)"<<(T_sampling));
	//		//record the yaw angle at the beginning of each line:
	//		if (i_jump_no==20)
	//		{
	//			reset_yaw_control();
	//			//time need to rotate:
	//			timearray__mapcruise[2*current_point]= time_body+ abs(yaw_mapcruise[current_point]-yaw_6DOF_init)/0.1745;
	//
	//			time_current[0]=time_body+ abs(yaw_mapcruise[current_point]-yaw_6DOF_init)/0.1745;
	//			i_jump_no=30;
	//
	//			ROS_INFO_STREAM("yaw_mapcruise[i]"<<(yaw_mapcruise[current_point]));
	//			ROS_INFO_STREAM("yaw_6DOF_init[i]"<<(yaw_6DOF_init));
	//		}
	//
	//
	//		if ((time_body<=time_current[0]) && (i_jump_no==30))
	//		{
	//			//rotate the yaw angle to the set angle:
	//			rotate_yaw_mapcruise(current_point);
	//			//i_jump_no=40;
	//			ROS_INFO_STREAM("rotate_yaw_mapcruise");
	//
	//		}
	//
	//		if ((time_body>time_current[0])&& (time_body<=time_current[0]+2.0))
	//		{
	//								//time need to line:
	//			timearray__mapcruise[2*current_point+1]= time_body+
	//			sqrt(
	//			(P_ini_cruise[0]- points_mapcruise[0][current_point])*(P_ini_cruise[0]- points_mapcruise[0][current_point])+
	//			(P_ini_cruise[1]-points_mapcruise[1][current_point])*(P_ini_cruise[1]- points_mapcruise[1][current_point])+
	//			(P_ini_cruise[2]-points_mapcruise[2][current_point])*(P_ini_cruise[2]- points_mapcruise[2][current_point])
	//			)/velocity_mapcruise[current_point];
	//
	//			time_current[2]= time_body+
	//			sqrt(
	//			(P_ini_cruise[0]- points_mapcruise[0][current_point])*(P_ini_cruise[0]- points_mapcruise[0][current_point])+
	//			(P_ini_cruise[1]-points_mapcruise[1][current_point])*(P_ini_cruise[1]- points_mapcruise[1][current_point])+
	//			(P_ini_cruise[2]-points_mapcruise[2][current_point])*(P_ini_cruise[2]- points_mapcruise[2][current_point])
	//			)/velocity_mapcruise[current_point];
	//
	//			//time_body=time_body+T_sampling;
	//
	//			i_jump_no=50;
	//
	//		}
	//
	//
	//
	//		//line:
	//		if ((time_body>time_current[0]+2.0)&& (time_body<=time_current[2]) && (i_jump_no==50))
	//		{
	//			int j;
	//			//minimum snap trajectory:
	//			for(j=0;j<3;j++)
	//			{
	//				//float minimumsnap_line(float t0, float alpha, float x0, float xf, float time)
	//				P_nom[j]= minimumsnap_line(timearray__mapcruise[2*current_point]+2,
	//				timearray__mapcruise[2*current_point+1]-timearray__mapcruise[2*current_point]-2,
	//				P_ini_cruise[j], points_mapcruise[j][current_point], time_body);
	//			}
	//			//time_body =time_body+T_sampling;
	//			//i_jump_no=100;
	//		}
	//
	//		if ((time_body>time_current[2])&& (time_body<=5.0+time_current[2]) && (i_jump_no==50))
	//		{
	//			//hold on:
	//			if((time_body>=4.8+time_current[2])&&(i_jump_no==50))
	//			{
	//				//next line:
	//				current_point++;
	//
	//				i_jump_no=20;
	//
	//				time_current[0]=0;
	//				time_current[2]=0;
	//				time_current[1]=0;
	//			}
	//
	//			if(current_point>=i_mapcruise)
	//			{
	//				//finish cruise:
	//				Pnomflag =1;
	//
	//				asctec_mav_motion_planning::flag_cmd flag_topic;
	//				flag_topic.flag=2; //1: position is give by PC, 2: position is give by RC transmitter
	//				flag_cmd_pub.publish(flag_topic);
	//
	//				flag_pc_cmd=0;
	//				time_body =0;
	//				time_doby_last=0;
	//
	//
	//			}
	//
	//			//time_body =time_body+T_sampling;
	//		}

			asctec_hl_comm::mav_ctrl msg;

			if(current_point==-1)
			{
				//important, notice the unit and the definition of the coordinate frame:
				msg.x = 999.999;  //must give a value to P_nom, or it will be zero
				msg.y = 999.999;
				msg.z = 999.999;
				msg.yaw = 0;
				msg.type = asctec_hl_comm::mav_ctrl::position;
				//unit:m/s
				msg.v_max_xy = 5;
				msg.v_max_z= 5;

				//taj_pub.publish(msg);

				//publish the msg for the self-developed driver
				control_pub.publish(msg);

				//show it in terminal:
				ROS_INFO_STREAM("beginning");

				current_point++;
			}


			else if(current_point < i_mapcruise)
			{
				//important, notice the unit and the definition of the coordinate frame:
				msg.x = points_mapcruise[0][current_point];  //must give a value to P_nom, or it will be zero?
				msg.y = points_mapcruise[1][current_point];
				msg.z = points_mapcruise[2][current_point];
				msg.yaw = yaw_mapcruise[current_point];
				msg.type = asctec_hl_comm::mav_ctrl::position;
				//unit:m/s
				msg.v_max_xy = 5;
				msg.v_max_z= 5;

				//taj_pub.publish(msg);

				//publish the msg for the self-developed driver
				control_pub.publish(msg);

				//show it in terminal:
				ROS_INFO_STREAM("current no "<<current_point);
				ROS_INFO_STREAM("cmd, x: "<<msg.x);
				ROS_INFO_STREAM("cmd, y: "<<msg.y);
				ROS_INFO_STREAM("cmd, z: "<<msg.z);
				ROS_INFO_STREAM("cmd, yaw: "<<msg.yaw);

				current_point++;

			}
			else if (current_point >= i_mapcruise)
			{
				current_point = -1;
				flag_pc_cmd = 0;


				//important, notice the unit and the definition of the coordinate frame:
				msg.x = 99.999;  //must give a value to P_nom, or it will be zero?
				msg.y = 99.999;
				msg.z = 99.999;
				msg.yaw = -(200.0-360000.0)/360000.0*2*M_PI-2*M_PI; //notice the transformation process, the accuracy information for the low level controller
				msg.type = asctec_hl_comm::mav_ctrl::position;
				//unit:m/s
				msg.v_max_xy = 5;
				msg.v_max_z= 5;

				//taj_pub.publish(msg);

				//publish the msg for the self-developed driver
				control_pub.publish(msg);

				//show it in terminal:
				ROS_INFO_STREAM("end");
			}
		}
	}



 	//obstacle avoiding code:
 	if(enable_obs_avoid == 1 )
 	{ //obstacle avoiding code:

		if (flag_pc_cmd_obs==1)
		{
			asctec_hl_comm::mav_ctrl msg;

			if(current_point_obs==-1)
			{
				current_point_obs++;
			}


			else if(current_point_obs < i_mapcruise)
			{
				//for each path points, execute the obstacle avoiding:
				{
					double y_ini[12]; //initial state of virtual dynamics
					double u_virtual[3]; // input of the virtual dynamics
					pathpara_str pathpara_received; //set the path parameters for the virtual dynamics
					//sensor_msgs::PointCloud obstacle_received;
					double tspan_output;
					double t_array[20000];
					double y_array[20000][12];
					int n_distime;

					for (int aa =0; aa < 3; aa++)
					{
						y_ini[aa] =  P_sen_obs[aa];
						u_virtual[aa] = 0;

						pathpara_received.start[aa] = P_sen_obs[aa];
						pathpara_received.end[aa] = points_mapcruise[aa][current_point_obs];
						pathpara_received.v = 1;
					}

					for (int aa =3; aa < 12; aa++)
					{
						y_ini[aa] =  0;
					}

					virtual_dynamics(&y_ini[0], &u_virtual[0], pathpara_received, obstacle_received, &tspan_output,  & t_array[0], & y_array[0][0], &n_distime);

					//send the waypoint at each path points from the global planner
					msg.x = 999.999;  //must give a value to P_nom, or it will be zero
					msg.y = 999.999;
					msg.z = 999.999;
					msg.yaw = 0;
					msg.type = asctec_hl_comm::mav_ctrl::position;
					//unit:m/s
					msg.v_max_xy = 5;
					msg.v_max_z= 5;

					//taj_pub.publish(msg);
					//publish the msg for the self-developed driver
					control_pub.publish(msg);

					//show it in terminal:
					ROS_INFO_STREAM("beginning, obstacle avoiding ");
					ROS_INFO_STREAM("sensed x position"<<P_sen_obs[0]);
					ROS_INFO_STREAM("expected z position"<<points_mapcruise[2][current_point_obs]);

					for (int bb =0; bb <n_distime; bb = bb+10)
					{
						//everything 1 second
						for (int i=0; i<3; i++){
							points_mapcruise_obs[i][bb] = y_array[bb][i];
						}

						//important, notice the unit and the definition of the coordinate frame:
						msg.x = points_mapcruise_obs[0][bb];  //must give a value to P_nom, or it will be zero?
						msg.y = points_mapcruise_obs[1][bb];
						msg.z = points_mapcruise_obs[2][bb];
						msg.yaw = yaw_mapcruise_obs[bb];
						msg.type = asctec_hl_comm::mav_ctrl::position;
						//unit:m/s
						msg.v_max_xy = 5;
						msg.v_max_z= 5;

						//taj_pub.publish(msg);

						//publish the msg for the self-developed driver
						control_pub.publish(msg);

						//show it in terminal:
						ROS_INFO_STREAM("current no "<<bb);
						ROS_INFO_STREAM("cmd, x: "<<msg.x);
						ROS_INFO_STREAM("cmd, y: "<<msg.y);
						ROS_INFO_STREAM("cmd, z: "<<msg.z);
						ROS_INFO_STREAM("cmd, yaw: "<<msg.yaw);
					}

					//end of the commands:
					msg.x = 99.999;  //must give a value to P_nom, or it will be zero?
					msg.y = 99.999;
					msg.z = 99.999;
					msg.yaw = -(200.0-360000.0)/360000.0*2*M_PI-2*M_PI; //notice the transformation process, th-08

					//unit:m/s
					msg.v_max_xy = 5;
					msg.v_max_z= 5;

					//taj_pub.publish(msg);

					//publish the msg for the self-developed driver
					control_pub.publish(msg);

					//show it in terminal:
					ROS_INFO_STREAM("transmit finished, obstacle avoiding");

				}

				current_point_obs++;
			}
			else if (current_point_obs >= i_mapcruise)
			{
				current_point_obs = -1;
				flag_pc_cmd_obs = 0;

				//record the final goal:
				for(int i=0; i < 3; i++)
				{
					current_goal[i] = points_mapcruise_obs[i][i_mapcruise-1];
				}
			}
		}

		float norm_final;
		norm_final = sqrt(  (current_goal[0]- P_sen_obs[0])*(current_goal[0]- P_sen_obs[0]) +
				(current_goal[1]- P_sen_obs[1])*(current_goal[1]- P_sen_obs[1]) +
				(current_goal[2]- P_sen_obs[2])*(current_goal[2]- P_sen_obs[2]) );

		if (norm_final <= accuracy_arrive)
		{
			flag_arrive = 1; //arrive;
		}
		else
		{
			flag_arrive = 0;
		}
 	}//end of obstacle avoiding code, added on May, 4, 2017
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


	//the virtual control used to avoid obstacles:


	//traj_snap_strait(const double *t0, const double *alpha, const double *x0, const double *xf, const double *t, double *out)

	double cmd_traj[3][5];
	double starttime;
	double duration;
	double startpath[3];
	double endpath[3];
	double time_array_cbf[20000];
	int n_time;


//	for (int time = 0; time < n_time; time++)
//	{
//		for (int i = 0; i < 3; i++)
//		{
//			traj_snap_strait(&starttime, &duration, &startpath[i], &endpath[i],  &time_array_cbf[time],  &cmd_traj[i][0]);
//		}
//	}

//    x = traj_snap(0, T_orig, pathpara.start(1), pathpara.end(1), t);
//    y = traj_snap(0, T_orig, pathpara.start(2), pathpara.end(2), t);
//    z = traj_snap(0, T_orig, pathpara.start(3), pathpara.end(3), t);

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


	ROS_INFO_STREAM("t0"<<(t0));
	ROS_INFO_STREAM("alpha"<<(alpha));
	ROS_INFO_STREAM("xo"<<(x0));
	ROS_INFO_STREAM("xf"<<(xf));
	ROS_INFO_STREAM("time"<<(time));


}



void Minimumsnap::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose){


	//outdoor or indoor:
    //out door or indoor,
    //1: outdoor, GPS provides the position information
    //2: indoor, SLAM module provides the position information
    if (flag_pose_source == 2)
    {
		P_sen[0]=pose->pose.position.x;
		P_sen[1]=pose->pose.position.y;
		P_sen[2]=pose->pose.position.z;

    }

    P_sen_obs[0]= pose->pose.position.x;
    P_sen_obs[1]= pose->pose.position.y;
    P_sen_obs[2]= pose->pose.position.z;

    if (slam_int ==0 )
    {
    	slam_int = 1;
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

//    P_sen_obs[0]=ext_state->pose.position.x;
//    P_sen_obs[1]=ext_state->pose.position.y;
//    P_sen_obs[2]=ext_state->pose.position.z;

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
		yaw_mapcruise[i]=(float)gamma_temp[2];

		ROS_INFO_STREAM("yaw_mapcruise[i])"<<(yaw_mapcruise[i]));
		ROS_INFO_STREAM("points_mapcruise[2][i])"<<(points_mapcruise[2][i]));

		printf( "planned quaternion = [ %e, %e, %e, %e ]\n\n",
				wp.orientation.x,wp.orientation.y, wp.orientation.z, wp.orientation.w);

		printf( "planned quaternion = [ %e, %e, %e, %e ]\n\n",
				quaternion[0],quaternion[1], quaternion[2], quaternion[3]);

		printf( "planned matrix = [ %e, %e, %e, %e, %e, %e, %e, %e, %e ]\n\n",
				R_temp[0], R_temp[1], R_temp[2], R_temp[3], R_temp[4], R_temp[5], R_temp[6], R_temp[7], R_temp[8]);

		printf( "planned Euler angles = [ %e, %e, %e]\n\n",gamma_temp[0], gamma_temp[1], gamma_temp[2]);

		//commanded speed, should be adjusted according to the actual situation
		velocity_mapcruise[i]=1;
	}


	Pnomflag =100; //exclude other commands
	//time_body=0;
	current_point=-1;
	i_jump_no=20;
	for(int i=0;i<4;i++)
	{
		time_current[i]=0.0f;
	}

	asctec_mav_motion_planning::flag_cmd flag_topic;
	flag_topic.flag=1; //1: position is give by PC, 2: position is give by RC transmitter
	flag_cmd_pub.publish(flag_topic);

	flag_pc_cmd=1; //activate the flag determine the computer sends the commands,
	time_doby_last=0;

	ROS_INFO_STREAM("received path commands, flag_pc_cmd is set to)"<<(flag_pc_cmd));


	{
		//added on May, 3, 2017, add the obstacle avoiding:
	//	void Minimumsnap::virtual_dynamics(const double *y0, const double *u_virtual, pathpara_str &pathpara,
	//			const sensor_msgs::PointCloud &obstac, double * tspan,  double *t1, double*y1);


		current_point_obs = -1;
		flag_pc_cmd_obs = 1;

		for (int j = 0; j< i_mapcruise-1; j++)
		{
			double y_ini[12]; //initial state of virtual dynamics
			double u_virtual[3]; // input of the virtual dynamics
			pathpara_str pathpara_received; //set the path parameters for the virtual dynamics
			//sensor_msgs::PointCloud obstacle_received;
			double tspan_output;
			double t_array[20000];
			double y_array[20000][12];
			int n_discretetime;

			for (int aa =0; aa < 3; aa++)
			{
				y_ini[aa] =  points_mapcruise[aa][j];
				u_virtual[aa] = 0;

				pathpara_received.start[aa] = points_mapcruise[aa][j];
				pathpara_received.end[aa] = points_mapcruise[aa][j+1];
				pathpara_received.v = 1;

			}

			for (int aa =3; aa < 12; aa++)
			{
				y_ini[aa] =  0;
			}

			//virtual_dynamics(&y_ini[0], &u_virtual[0], pathpara_received, obstacle_received, &tspan_output,  & t_array[0], & y_array[0][0], &n_discretetime);

			for (int bb =0; bb <20000; bb = bb+100)
			{
				for (int i=0; i<3; i++){
					points_mapcruise_obs[i][bb] = y_array[bb][i];
				}
			}

			//record the waypoint at each path points from the global planner


		}

	}

}


void Minimumsnap::rotate_yaw_mapcruise(int i)
{
	float timp_elapse;

	//rotate the yaw angle to the set angle: yaw_mapcruise[i]

	if(yaw_mapcruise[i] > (yaw_6DOF_init+0.001))
	{
		gamma_nom[2]= gamma_nom[2] + 0.1745* T_sampling;  //10deg/s ,
	}
	else if(yaw_mapcruise[i] < (yaw_6DOF_init-0.001))
	{
		gamma_nom[2]= gamma_nom[2] - 0.1745* T_sampling;  //10deg/s ,
	}


//	gamma_nom[2]= gamma_nom[2] + 0.1745* T_sampling;

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

	P_nom[0] = P_sen[0];
	P_nom[1] = P_sen[1];
	P_nom[2] = P_sen[2];

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


void Minimumsnap::virtual_dynamics(const double *y0, const double *u_virtual, pathpara_str &pathpara,
		const sensor_msgs::PointCloud &obstac, double * tspan,  double *t1, double*y1, int * n_dtime)
{
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

	double dt =0.01;  //sampling time, can be modified according to the actual conditions
	int n_state=12; //the dimension of state space
	double y[12]; //the state of the virtual dynamics, each
	double u[3]; //the input of the virtual dynamics, each time instant
	//double traj[12]; //the reference trajectory for the virtual dynamics, each time instant

	double norm;  //the distance from the beginning point to the end point
	norm = sqrt(    (pathpara.end[0]-pathpara.start[0])*(pathpara.end[0]-pathpara.start[0])+
			(pathpara.end[1]-pathpara.start[1])*(pathpara.end[1]-pathpara.start[1]) +
			(pathpara.end[2]-pathpara.start[2])*(pathpara.end[2]-pathpara.start[2]) );

	double T_orig =  norm/pathpara.v;  //calculation average time using the minimum snap trajectory



	*tspan = T_orig + 50;  //the actual time > average time
	int n_time = int((* tspan)/dt);

	* n_dtime = n_time;

	*t1 = 0;
	for(int i = 1; i < n_time; i++)
	{
		*(t1+i) = *(t1+i-1)+dt;  //time array
 	}

	for(int i=0; i<12; i++)
	{//the initial state of the virtual dynamics
		y[i]=*(y0+i);

	}

	for(int i = 0; i < n_time; i++)
	{
		trajd trajd_i;

		//current time:
		double t;
		t=*(t1+i);


		//traj_gen(const double *t, const pathpara_str &pathpara, const double *T_orig, double *out)
		//pathpara_str pathpara;
		double out_traj_gen[15];
		traj_gen(&t, pathpara, &T_orig, &out_traj_gen[0]);
		for (int k=0; k<3; k++){
			trajd_i.trajd_0d[k] = out_traj_gen[k];
			trajd_i.trajd_1d[k] = out_traj_gen[k+3];
			trajd_i.trajd_2d[k] = out_traj_gen[k+6];
			trajd_i.trajd_3d[k] = out_traj_gen[k+9];
			trajd_i.trajd_4d[k] = out_traj_gen[k+12];

		//	ROS_INFO_STREAM("NO. "<<i);
		//	ROS_INFO_STREAM("panned trajectory"<<trajd_i.trajd_0d[k]);
		}



	    int n_ob;


	    //test code:
//		for (int i=0; i<3; i++){
//			 trajd_i.trajd_0d[i] =0.5;
//		     trajd_i.trajd_1d[i]=0.5;
//			  trajd_i.trajd_2d[i]=0.5;
//			 trajd_i.trajd_3d[i]=0.5;
//			 trajd_i.trajd_4d[i]=0.5;
//		}
//		for (int i=0; i<12; i++){
//			y[i] = 0.9;
//		}

	     printf( "\n distance   = [ %e ] \n", norm );


	    virtual_Control(trajd_i, &y[0], obstac, &u[0]);

	//    printf( "\n test the control  = [ %e, %e, %e ] \n", u[0], u[1], u[2]  );




		virtual_dynamics_onestep(&y[0], &u[0], & dt); //dynamics at each time

		for (int row=0; row<12; row++)
		{
			//record the state at each instant
			*(y1+i*12+row)=y[row];    //double y1[n][12]
		}
	}
}


void Minimumsnap::traj_gen(const double *t, const pathpara_str &pathpara, const double *T_orig, double *out){

//	v=pathpara.v;
//	direction=(pathpara.end-pathpara.start)/norm(pathpara.end-pathpara.start);

	double v;
	v=pathpara.v;

	double direction[3];
	double norm;

	norm = sqrt(    (pathpara.end[0]-pathpara.start[0])*(pathpara.end[0]-pathpara.start[0])+
			(pathpara.end[1]-pathpara.start[1])*(pathpara.end[1]-pathpara.start[1]) +
			(pathpara.end[2]-pathpara.start[2])*(pathpara.end[2]-pathpara.start[2]) );

	for (int i=0; i<3; i++)
	{
		direction[i] = (pathpara.end[i]-pathpara.start[i])/norm;
	}


//	if (t<=T_orig)
//	    pos = pathpara.start+direction*v*t;
//	    vel = direction*v;
//	    acc = [0; 0; 0];
//	    dacc =[0; 0; 0];
//	    d2acc = [0; 0; 0];
//
//	    x = traj_snap(0, T_orig, pathpara.start(1), pathpara.end(1), t);
//	    y = traj_snap(0, T_orig, pathpara.start(2), pathpara.end(2), t);
//	    z = traj_snap(0, T_orig, pathpara.start(3), pathpara.end(3), t);
//	    pos = [x(1); y(1); z(1)];
//	    vel = [x(2); y(2); z(2)];
//	    acc = [x(3); y(3); z(3)];
//	    dacc = [x(4); y(4); z(4)];
//	    d2acc = [x(5); y(5); z(5)];
//	else
//	    pos=pathpara.end;
//	    vel = [0; 0; 0];
//	    acc = [0; 0; 0];
//	    dacc =[0; 0; 0];
//	    d2acc = [0; 0; 0];
//	end

	double pos[3], vel[3], acc[3], dacc[3], d2acc[3];
	double starttime = 0;
	double duration = *T_orig;
	double x0, xf;
	double out_snap[5];

	if (*t<=*T_orig)
	{
	    double t_i;
	    t_i = *t;
		for (int i = 0; i<3; i++)
		{

			x0 = pathpara.start[i];
			xf = pathpara.end[i];

			traj_snap_strait(&starttime, &duration, &x0, &xf, &t_i, &out_snap[0]);
			pos[i] = out_snap[0];
			vel[i] = out_snap[1];
			acc[i] = out_snap[2];
			dacc[i] = out_snap[3];
			d2acc[i] = out_snap[4];

//			ROS_INFO_STREAM("x0 "<<x0);
//			ROS_INFO_STREAM("xf "<<xf);
//			ROS_INFO_STREAM("t_i "<<t_i);
//			ROS_INFO_STREAM("duration "<<duration);
//			ROS_INFO_STREAM("starttime "<<starttime);
//			ROS_INFO_STREAM("pos[i] "<<pos[i]);

		}

	}
	else
	{
		for (int i = 0; i<3; i++)
		{
			 pos[i] = pathpara.end[i];
			 vel[i] = 0;
			 acc[i] = 0;
			 dacc[i] = 0;
			 d2acc[i] = 0;
		}
	}


	for (int i = 0; i < 3; i++){
			*(out+i) = pos[i];
			*(out+i+3) = vel[i];
			*(out+i+6) = acc[i];
			*(out+i+9) = dacc[i];
			*(out+i+12) = d2acc[i];
	}

}



void Minimumsnap::virtual_dynamics_onestep(double *x, const double *u, const double * deltat){
	//Virtual dynamics at each step
	//x: state
	//u: input

	double dt;  //sampling time
	double dx[12];
	double x_i[12]; //state

	dt = * deltat;


	//the derivative of the states:
	for(int j=0; j<9; j++)
	{
		dx[j]= *(x+j+3);
	}
	for(int j=0; j<3; j++)
	{
		dx[j+9] = u[j];
	}

	for(int j=0; j<12; j++)
	{
		x_i[j] = *(x+j); //state
		*(x+j) = x_i[j] + dx[j]*dt;  //Evolution of states
	}
}



void Minimumsnap::virtual_Control(const trajd& trajd_i, const double * y, const sensor_msgs::PointCloud & obstac, double *u_virutal ){

	//commanded trajectory and its derivative
	double rhat[3], rhatd[3], rhatdd[3], rhatddd[3], rhatdddd[3];
	for (int i=0; i<3; i++){
		rhat[i] = trajd_i.trajd_0d[i];
		rhatd[i] = trajd_i.trajd_1d[i];
		rhatdd[i] = trajd_i.trajd_2d[i];
		rhatddd[i] = trajd_i.trajd_3d[i];
		rhatdddd[i] = trajd_i.trajd_4d[i];

//		rhat[i] = 2;
//		rhatd[i] = 2;
//		rhatdd[i] = 2;
//		rhatddd[i] = 2;
//		rhatdddd[i] = 2;
	}

	printf( "\n commanded trajectory = [ %e, %e, %e ] \n", rhat[0], rhat[1], rhat[2] );


	//sensed trajectory
	double r[3], rd[3], rdd[3],  rddd[3];

	for (int i=0; i<3; i++){
		r[i] = *(y+i);
		rd[i] = *(y+i+3);
		rdd[i] = *(y+i+6);
		rddd[i] = *(y+i+9);

		//thest the code
//		r[i] = 1;
//		rd[i] = 1;
//		rdd[i] = 1;
//		rddd[i] = 1;
	}

	printf( "\n sensed trajectory = [ %e, %e, %e ] \n", r[0], r[1], r[2] );

	//parameters in the controller
	int k1, k2, k3, k4;
	k1=1;
	k2=4;
	k3=6;
	k4=4;


//	%nominal input
//	v_nom=rhatdddd-k1*(r-rhat)-k2*(rd-rhatd)-k3*(rdd-rhatdd)-k4*(rddd-rhatddd);

	double v_nom[3];
	for (int i=0; i<3; i++)
	{
		v_nom[i] = rhatdddd[i]-k1*(r[i]-rhat[i])-k2*(rd[i]-rhatd[i])-k3*(rdd[i]-rhatdd[i])-k4*(rddd[i]-rhatddd[i]);
	}

	//printf( "\n v_nom= [ %e, %e, %e ] \n", v_nom[0], v_nom[1], v_nom[2] );

	//parameter:
	double Ds=0.5;
	// coefficient of z:
	double cz=1;

//	n_ob=size(obsta,1);
//	A_n=zeros(n_ob,3);
//	b_n=zeros(n_ob,1);

	int n_ob;  //No of point clouds
	n_ob = obstac.points.size();


	double A_n[n_ob][3];
	double b_n[n_ob];

	//%safety function
	//h_n=zeros(n_ob,1);

	double h_n[n_ob];

	for (int i_obst=0; i_obst<n_ob; i_obst++)
	{

		//%obstacles:
		double ob[3];
//		for (int j=0; j<3; j++)
//		{
//			ob[j]=*(obsta+3*i_obst+j);
//		}

		ob[0]=obstac.points[i_obst].x;
		ob[1]=obstac.points[i_obst].y;
		ob[2]=obstac.points[i_obst].z;

	    //%parameter:
	    double k_ob[4]={k1/0.2, k2/0.2, k3/0.2, k4/0.2};


	    //calculate the safety related functions
	    double h_x[4], h_y[4], h_z[4];
	    double u_x[4], u_y[4], u_z[4];
	    u_x[0]=r[0]-ob[0]; u_x[1] =rd[0]; u_x[2] =rdd[0];  u_x[3] =rddd[0];
	    u_y[0]=r[1]-ob[1]; u_y[1] =rd[1]; u_y[2] =rdd[1];  u_y[3] =rddd[1];
	    u_z[0]=r[2]-ob[2]; u_z[1] =rd[2]; u_z[2] =rdd[2];  u_z[3] =rddd[2];

	    cbf_planning(&u_x[0],  &h_x[0]);
	    cbf_planning(&u_y[0],  &h_y[0]);
	    cbf_planning(&u_z[0],  &h_z[0]);

	    double h, dh, ddh, dddh;
	    h=h_x[0]+h_y[0]+h_z[0]/(cz*cz*cz*cz)-Ds*Ds*Ds*Ds;
	    dh = h_x[1]+h_y[1]+h_z[1]/(cz*cz*cz*cz);
	    ddh = h_x[2]+h_y[2]+h_z[2]/(cz*cz*cz*cz);
	    dddh = h_x[3]+h_y[3]+h_z[3]/(cz*cz*cz*cz);


//	    printf( "\n h_x= [ %e, %e, %e, %e ] \n", h_x[0], h_x[1], h_x[2], h_x[3]);
//	    printf( "\n  h_y= [ %e, %e, %e, %e ] \n", h_y[0], h_y[1], h_y[2], h_y[3]);
//	    printf( "\n  h_z= [ %e, %e, %e, %e ] \n", h_z[0], h_z[1], h_z[2], h_z[3]);
//	    printf( "\n  u_z= [ %e, %e, %e, %e ] \n", u_z[0], u_z[1], u_z[2], u_z[3]);
//	    printf( "\n  u_y= [ %e, %e, %e, %e ] \n", u_y[0], u_y[1], u_y[2], u_y[3]);
//	    printf( "\n  u_x= [ %e, %e, %e, %e ] \n", u_x[0], u_x[1], u_x[2], u_x[3]);
//
//		printf( "\n h, dh, ddh, dddh= [ %e, %e, %e , %e] \n", h, dh, ddh, dddh);


	    //	cbf_planning(const double *u,  double *out);

//	    h_x=cbf_planning([r(1)-ob(1);rd(1);rdd(1);rddd(1)]);
//	    h_y=cbf_planning([r(2)-ob(2);rd(2);rdd(2);rddd(2)]);
//	    h_z=cbf_planning([r(3)-ob(3);rd(3);rdd(3);rddd(3)]);
//
//	    h=h_x.h+h_y.h+h_z.h/cz^4-Ds^4;
//	    dh=h_x.dh+h_y.dh+h_z.dh/cz^4;
//	    ddh=h_x.ddh+h_y.ddh+h_z.ddh/cz^4;
//	    dddh=h_x.dddh+h_y.dddh+h_z.dddh/cz^4;

	   // relative_r=r-ob;

	    // relative position from the UAV to the obstacle
	    double relative_r[3];
	    for (int i=0; i<3; i++)
	    {
	    	relative_r[i] = r[i] - ob[i];
	    }

//	    %optimal problem:
//	    %variable is v-vnom
//	    A=[-4*relative_r(1)^3, -4*relative_r(2)^3, -4/cz^4*relative_r(3)^3];
//	    b=k_ob*[h;dh;ddh;dddh]+...
//	      (24*rd.^4+144*relative_r.*(rd.^2).*rdd+36*(relative_r.^2).*(rdd.^2)+48*(relative_r.^2).*rd.*rddd)'*ones(3,1);
//	    b=b-A*v_nom;
//	    A_n(i,:)=A;
//	    b_n(i)=b;
//	    h_n(i)=h;

	    double A[3], b;

	    A[0]=-4*relative_r[0]*relative_r[0]*relative_r[0];
	    A[1]=-4*relative_r[1]*relative_r[1]*relative_r[1];
	    A[2]=-4/cz*cz*cz*cz*relative_r[2]*relative_r[2]*relative_r[2];

		b = k_ob[0]*h+k_ob[1]*dh+k_ob[2]*ddh+k_ob[3]*dddh;

		//24*rd.^4+144*relative_r.*(rd.^2).*rdd+36*(relative_r.^2).*(rdd.^2)+48*(relative_r.^2).*rd.*rddd);
		double tempb[3];
	    for (int i=0; i<3; i++)
	    {
	    	tempb[i]=24*rd[i]*rd[i]*rd[i]*rd[i] +
	    			144*relative_r[i]*(rd[i]*rd[i])*rdd[i]+
					36*(relative_r[i]*relative_r[i])*(rdd[i]*rdd[i])+
					48*(relative_r[i]*relative_r[i])*rd[i]*rddd[i];
	    }

	    b = b + tempb[0] + tempb[1] + tempb[2];
	    b = b-(A[0]*v_nom[0]+A[1]*v_nom[1]+A[2]*v_nom[2]);


	    b_n[i_obst]=b;

	    for (int i=0; i<3; i++)
	    {
	    	A_n[i_obst][i]=A[i];  //double A_n[n_ob][3];
	    }
	}


	//notice T_sampling, very important:
	double ts_usec;
	ts_usec = (uint64_t)(ros::WallTime::now().toSec() * 1.0e6);
 	ROS_INFO_STREAM("time in front of QP solver: )"<<(ts_usec));

   //using the opensource package solve QP problem
	USING_NAMESPACE_QPOASES
	SQProblem cbfqp(3,1);

	real_t H1_[3*3] ={1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
	real_t f1_[3] = {0.0, 0.0, 0.0};
	real_t A1_[n_ob*3];
	real_t b1_[n_ob];

	real_t lb_input_[3]; //lower bound of the control
	real_t ub_input_[3]; //upper bound of the control

	double alpha_lb = 10; //used to calculate the upper and lower bound for the control

	for(int ilb = 0; ilb < 3; ilb ++){
		lb_input_[ilb] = -alpha_lb - v_nom[ilb];
		ub_input_[ilb] = alpha_lb - v_nom[ilb];
	}



	for (int iqp = 0; iqp < n_ob; iqp ++ ){
		for (int cqp = 0; cqp < 3; cqp ++ ){
			A1_[iqp*3+cqp] = A_n[iqp][cqp];

		}

		b1_[iqp] = b_n[iqp];

		printf( "\n b matrix = [ %e  ]", b1_[iqp] );
		printf( "\n A matrix = [ %e, %e , %e   ]", A1_[iqp*3+0], A1_[iqp*3+1], A1_[iqp*3+2]);
	}


	real_t  *ptrnll = NULL;

	// Solve first QP
	int nWSR1 = 10;

	//example.hotstart( H_new,g_new,A_new,lb_new,ub_new,lbA_new,ubA_new, nWSR,0);
	//A1_*x<=b1_
	cbfqp.init(H1_,f1_,A1_,lb_input_,ub_input_,ptrnll,b1_, nWSR1, 0 );

	// Get and print solution of second QP.
	real_t uOpt[3];
	cbfqp.getPrimalSolution( uOpt );
	printf( "\n virtual control = [ %e, %e, %e ];  objVal = %e\n\n", uOpt[0], uOpt[1], uOpt[2], cbfqp.getObjVal() );


	for (int i=0; i<3; i++)
	{
		*(u_virutal+i) = uOpt[i] + v_nom[i];

		//*(u_virutal+i) = v_nom[i];

	}

	ts_usec = (uint64_t)(ros::WallTime::now().toSec() * 1.0e6);
 	ROS_INFO_STREAM("time after QP solver"<<(ts_usec));


}



void Minimumsnap::traj_snap_strait(const double *t0, const double *alpha, const double *x0, const double *xf, const double *t, double *out){
//minimum-snap trajectory for a strait line
	double A_colve[10] = {0,
			0,
			0,
			0,
			0,
			126,
			-420,
			540,
			-315,
			70};
	double A_colve_1d[10] = {0,
			0,
			0,
			0,
			0,
			630,
			-2520,
			3780,
			-2520,
			630};

	double A_colve_2d[10] = {0,
			0,
			0,
			0,
			0,
			2520,
			-12600,
			22680,
			-17640,
			5040};

	double A_colve_3d[10] = {0,
	0,
	0,
	0,
	0,
	7560,
	-50400,
	113400,
	-105840,
	35280};

	double A_colve_4d[10] = {0,
			0,
			0,
			0,
			0,
			15120,
			-151200,
			453600,
			-529200,
			211680};

	double tau_i;

	tau_i = ((*t)-(*t0))/(*alpha);

//	%non-dimension
//	x_tilde  = A_colve.'*[tau_i^0; tau_i^1; tau_i^2; tau_i^3; tau_i^4; tau_i^5; tau_i^6; tau_i^7; tau_i^8; tau_i^9];
//	x_tilde_d  = A_colve_1d'*[0; tau_i^0; tau_i^1; tau_i^2; tau_i^3; tau_i^4; tau_i^5; tau_i^6; tau_i^7; tau_i^8];
//	x_tilde_dd  = A_colve_2d'*[0; 0; tau_i^0; tau_i^1; tau_i^2; tau_i^3; tau_i^4; tau_i^5; tau_i^6; tau_i^7];
//	x_tilde_ddd  = A_colve_3d'*[0; 0; 0; tau_i^0; tau_i^1; tau_i^2; tau_i^3; tau_i^4; tau_i^5; tau_i^6];
//	x_tilde_dddd  = A_colve_4d'*[0; 0; 0; 0; tau_i^0; tau_i^1; tau_i^2; tau_i^3; tau_i^4; tau_i^5];


	double x_tilde, x_tilde_d, x_tilde_dd, x_tilde_ddd, x_tilde_dddd;


	x_tilde = A_colve[0]*1 +
			A_colve[1]*tau_i  +
			A_colve[2]*tau_i*tau_i  +
			A_colve[3]*tau_i*tau_i*tau_i  +
			A_colve[4]*tau_i*tau_i*tau_i*tau_i +
			A_colve[5]*tau_i*tau_i*tau_i*tau_i*tau_i  +
			A_colve[6]*tau_i*tau_i*tau_i*tau_i*tau_i*tau_i  +
			A_colve[7]*tau_i*tau_i*tau_i*tau_i*tau_i*tau_i*tau_i  +
			A_colve[8]*tau_i*tau_i*tau_i*tau_i*tau_i*tau_i*tau_i*tau_i  +
			A_colve[9]*tau_i*tau_i*tau_i*tau_i*tau_i*tau_i*tau_i*tau_i*tau_i;

	x_tilde_d = A_colve_1d[0]*0 +
			A_colve_1d[1]*1  +
			A_colve_1d[2]*tau_i  +
			A_colve_1d[3]*tau_i*tau_i  +
			A_colve_1d[4]*tau_i*tau_i*tau_i +
			A_colve_1d[5]*tau_i*tau_i*tau_i*tau_i  +
			A_colve_1d[6]*tau_i*tau_i*tau_i*tau_i*tau_i  +
			A_colve_1d[7]*tau_i*tau_i*tau_i*tau_i*tau_i*tau_i  +
			A_colve_1d[8]*tau_i*tau_i*tau_i*tau_i*tau_i*tau_i*tau_i  +
			A_colve_1d[9]*tau_i*tau_i*tau_i*tau_i*tau_i*tau_i*tau_i*tau_i;

	x_tilde_dd = A_colve_2d[0]*0 +
			A_colve_2d[1]*0  +
			A_colve_2d[2]*1  +
			A_colve_2d[3]*tau_i  +
			A_colve_2d[4]*tau_i*tau_i +
			A_colve_2d[5]*tau_i*tau_i*tau_i  +
			A_colve_2d[6]*tau_i*tau_i*tau_i*tau_i  +
			A_colve_2d[7]*tau_i*tau_i*tau_i*tau_i*tau_i  +
			A_colve_2d[8]*tau_i*tau_i*tau_i*tau_i*tau_i*tau_i  +
			A_colve_2d[9]*tau_i*tau_i*tau_i*tau_i*tau_i*tau_i*tau_i;

	x_tilde_ddd = A_colve_3d[0]*0 +
			A_colve_3d[1]*0  +
			A_colve_3d[2]*0  +
			A_colve_3d[3]*1  +
			A_colve_3d[4]*tau_i +
			A_colve_3d[5]*tau_i*tau_i  +
			A_colve_3d[6]*tau_i*tau_i*tau_i  +
			A_colve_3d[7]*tau_i*tau_i*tau_i*tau_i  +
			A_colve_3d[8]*tau_i*tau_i*tau_i*tau_i*tau_i  +
			A_colve_3d[9]*tau_i*tau_i*tau_i*tau_i*tau_i*tau_i;


	x_tilde_dddd = A_colve_4d[0]*0 +
			A_colve_4d[1]*0  +
			A_colve_4d[2]*0  +
			A_colve_4d[3]*0  +
			A_colve_4d[4]*1 +
			A_colve_4d[5]*tau_i  +
			A_colve_4d[6]*tau_i*tau_i  +
			A_colve_4d[7]*tau_i*tau_i*tau_i  +
			A_colve_4d[8]*tau_i*tau_i*tau_i*tau_i  +
			A_colve_4d[9]*tau_i*tau_i*tau_i*tau_i*tau_i;


	double x_i, x_d, x_dd, x_ddd, x_dddd;

	x_i = (*x0)+ ((*xf)-(*x0)) * x_tilde;
	x_d = x_tilde_d/(*alpha);
	x_dd = x_tilde_dd/((*alpha)*(*alpha));
	x_ddd = x_tilde_ddd/((*alpha)*(*alpha)*(*alpha));
	x_dddd = x_tilde_dddd/((*alpha)*(*alpha)*(*alpha)*(*alpha));

	*out = x_i;
	*(out+1) = x_d;
	*(out+2) = x_dd;
	*(out+3) = x_ddd;
	*(out+4) = x_dddd;

	ROS_INFO_STREAM("*x0 "<<*x0);
	ROS_INFO_STREAM("*xf "<<*xf);
	ROS_INFO_STREAM("*t "<<*t);
	ROS_INFO_STREAM("*t0 "<<*t0);
	ROS_INFO_STREAM("*alpha "<<*alpha);
	ROS_INFO_STREAM("**out "<<*(out+3));

}


void Minimumsnap::cbf_planning(const double *u,  double *out)
{
	//compute h, dot h ,ddot h, dddot h
	//input
	double x, dx, ddx, dddx;
	x=*u;
	dx=*(u+1);
	ddx=*(u+2);
	dddx=*(u+3);

	double h, dh, ddh, dddh;
	h=x*x*x*x;
	dh=4*x*x*x*dx;
	ddh=12*x*x*dx*dx+4*x*x*x*ddx;
	dddh=24*x*dx*dx*dx+36*x*x*dx*ddx+4*x*x*x*dddx;

	*out=h;
	*(out+1)=dh;
	*(out+2)=ddh;
	*(out+3)=dddh;
}

void Minimumsnap::pcCallback(const sensor_msgs::PointCloud2ConstPtr& pc2_obsta)
{
	sensor_msgs::PointCloud pc_obsta;
	sensor_msgs::PointCloud2 pc2;
	pc2.data =  pc2_obsta->data;
	pc2.fields =  pc2_obsta->fields;
	pc2.header =  pc2_obsta->header;
	pc2.height =  pc2_obsta->height;
	pc2.is_bigendian =  pc2_obsta->is_bigendian;
	pc2.is_dense =  pc2_obsta->is_dense;
	pc2.point_step =  pc2_obsta->point_step;
	pc2.row_step =  pc2_obsta->row_step;
	pc2.width =  pc2_obsta->width;

	//convert the pointcloud2 to pointcloud data:
	sensor_msgs::convertPointCloud2ToPointCloud(pc2,  obstacle_received);


	int n_ob;  //No of point clouds
	n_ob = obstacle_received.points.size();


	double A_n[n_ob][3];
	double b_n[n_ob];

	//%safety function
	//h_n=zeros(n_ob,1);

	double h_n[n_ob];

	for (int i_obst=0; i_obst<n_ob; i_obst++)
	{

		//%obstacles:
		double ob[3];
//		for (int j=0; j<3; j++)
//		{
//			ob[j]=*(obsta+3*i_obst+j);
//		}

		ob[0]=obstacle_received.points[i_obst].x;
		ob[1]=obstacle_received.points[i_obst].y;
		ob[2]=obstacle_received.points[i_obst].z;
	}

	//obtain the closest points
	process_pcl(obstacle_received);
	point_could_pub_.publish(obstacle_received);
}

void Minimumsnap::process_pcl(sensor_msgs::PointCloud &obstac){
	//process the original point cloud data

	int n_ob;  //No of point clouds
	n_ob = obstac.points.size();

	double A_n[n_ob][3];
	double distance[n_ob];
	int no_points[n_ob];

	geometry_msgs::Point32 point;
	std::vector<geometry_msgs::Point32> process_point;

	for (int i_obst=0; i_obst<n_ob; i_obst++)
	{
		distance[i_obst] = sqrt( obstac.points[i_obst].x * obstac.points[i_obst].x +
				obstac.points[i_obst].y * obstac.points[i_obst].y + obstac.points[i_obst].z * obstac.points[i_obst].z);
	}

	double temp;

	//from small to big, only the distance:
	for (int i = 0; i < n_ob - 1; i++)
		for (int j = 0; j < n_ob - 1 - i; j++)
			if (distance[j] > distance[j + 1]) {
				temp = distance[j];
				distance[j] = distance[j + 1];
				distance[j + 1] = temp;
			}

	//the order of the points:
	for (int i = 0; i < n_ob; i++){
		for (int j = 0; j< n_ob; j++)
		if(distance[i] ==  sqrt( obstac.points[j].x * obstac.points[j].x +
				obstac.points[j].y * obstac.points[j].y + obstac.points[j].z * obstac.points[j].z))
		{
			no_points[i] = j;
		}
	}

	int resize_pcl = 10;  //resize the point cloud data

	if (resize_pcl < n_ob){
		//the smallest distance:
		for (int i=0; i<resize_pcl; i++){
			int no_p = no_points[i];
			point =obstac.points[no_p];
			process_point.push_back(point);
		}

		obstacle_received.points.resize(process_point.size());

		if(!process_point.empty()){
			obstacle_received.header.frame_id = 'test';
		}

		for(unsigned int i=0; i < process_point.size(); i++){
			obstacle_received.points[i] = process_point[i];
		}
	}
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "minimum_snap");

    ros::NodeHandle nh("minimum_snap");

    Minimumsnap minimumfun;

    ros::spin();

}




