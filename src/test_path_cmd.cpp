/*
 * test_path_cmd.cpp
 *
 *  Created on: Jan 26, 2017
 *      Author: yushu
 */




//other header files
#include "minimum_snap_traj.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "math_function.h"

//#include "sensor_msgs/Imu.h"


// message includes
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
#include <asctec_hl_comm/mav_rcdata.h>
#include <asctec_hl_comm/mav_ctrl.h>
#include <asctec_hl_comm/mav_imu.h>
#include <asctec_hl_comm/mav_status.h>
#include <asctec_hl_comm/GpsCustom.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_fusion_comm/ExtState.h>
#include <asctec_mav_motion_planning/flag_cmd.h>


// dynamic reconfigure includes
#include <dynamic_reconfigure/server.h>
#include <asctec_mav_motion_planning/motion_planning_paraConfig.h>

//matrix:
#include <Eigen/Eigen>


int main(int argc, char **argv)
{

	//test the motion planning module

  ros::init(argc, argv, "test_path_cmd");

  ros::NodeHandle n;
  ros::Publisher path_sim_pub = n.advertise<nav_msgs::Path>("positioncmd", 1);    //test only, publish the path information


   ros::Rate loop_rate(0.1);


  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg;

    {
    	float cmdp[5][3]={{1,1,3},{3,5,6},{8,7,2},{9,7,2},{5,1,4}};
    	float yawcmd[5] = {1,1.2,1.6,0,-1};

    	nav_msgs::Path gui_path;
    	geometry_msgs::PoseStamped pose;
    	std::vector<geometry_msgs::PoseStamped> plan;
		for (int i=0; i<5; i++){
			  pose.pose.position.x = cmdp[i][0];
			  pose.pose.position.y = cmdp[i][1];
			  pose.pose.position.z = cmdp[i][2];

			  //quaternion, note the order
			  double ea[3]={0,0,0};
			  ea[2]=yawcmd[i];

			  double R_temp[9];
			  double quaternion[4];
			  math_function::computeR(&ea[0], &R_temp[0]);
			  math_function::computequaternion(&R_temp[0], &quaternion[0]);

			  pose.pose.orientation.x = quaternion[1];
			  pose.pose.orientation.y = quaternion[2];
			  pose.pose.orientation.z = quaternion[3];
			  pose.pose.orientation.w = quaternion[0];
			  plan.push_back(pose);


				printf( "planned yaw = [ %e, %e, %e ]\n\n",
						ea[0],ea[1],ea[2]);

				printf( "planned matrix = [ %e, %e, %e, %e, %e, %e, %e, %e, %e ]\n\n",
						R_temp[0], R_temp[1], R_temp[2], R_temp[3], R_temp[4], R_temp[5], R_temp[6], R_temp[7], R_temp[8]);

				printf( "planned quaternion = [ %e, %e, %e, %e ]\n\n",
										pose.pose.orientation.x,pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);



//				//quaternion[0] = cos(0.5); quaternion[1]= sin(0.5)*0; quaternion[2]= sin(0.5)*0; quaternion[3]= sin(0.5);
//				math_function::quaternion_to_R(&quaternion[0], &R_temp[0]);
//				math_function::RtoEulerangle(&R_temp[0], &ea[0]);
//
//
//				printf( "planned quaternion reverse = [ %e, %e, %e, %e ]\n\n",
//						quaternion[0],quaternion[1], quaternion[2], quaternion[3]);
//
//
//				printf( "planned matrix reverse = [ %e, %e, %e, %e, %e, %e, %e, %e, %e ]\n\n",
//						R_temp[0], R_temp[1], R_temp[2], R_temp[3], R_temp[4], R_temp[5], R_temp[6], R_temp[7], R_temp[8]);
//
//
//				printf( "planned yaw reverse = [ %e, %e, %e ]\n\n",
//						ea[0],ea[1],ea[2]);




			}

		gui_path.poses.resize(plan.size());

		if(!plan.empty()){
			  gui_path.header.frame_id = 'test';
			  gui_path.header.stamp = plan[0].header.stamp;
		}

		for(unsigned int i=0; i < plan.size(); i++){
			  gui_path.poses[i] = plan[i];
		}



		path_sim_pub.publish(gui_path);
		ROS_INFO_STREAM("send path points");

    }


    ros::spinOnce();
    loop_rate.sleep();

    ++count;

    if(count >=2)
    	break;


  }


  return 0;
}
