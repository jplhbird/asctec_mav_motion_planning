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

  ros::init(argc, argv, "test_path_cmd");

  ros::NodeHandle n;
  ros::Publisher path_sim_pub = n.advertise<nav_msgs::Path>("positioncmd", 1);    //test only, publish the path information


   ros::Rate loop_rate(0.1);


  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg;

    {
    	float cmdp[5][3]={{1,1,3},{3,5,6},{8,7,2},{20,9,10},{5,1,5}};

    	nav_msgs::Path gui_path;
    	geometry_msgs::PoseStamped pose;
    	std::vector<geometry_msgs::PoseStamped> plan;
		for (int i=0; i<5; i++){
			  pose.pose.position.x = cmdp[i][0];
			  pose.pose.position.y = cmdp[i][1];
			  pose.pose.position.z = cmdp[i][2];
			  pose.pose.orientation.x = 0.0;
			  pose.pose.orientation.y = 0.0;
			  pose.pose.orientation.z = 0.0;
			  pose.pose.orientation.w = 1.0;
			  plan.push_back(pose);
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
  }


  return 0;
}
