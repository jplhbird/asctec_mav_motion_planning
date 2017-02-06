/*
 * sim_interface.cpp
 *
 *  Created on: Jan 18, 2017
 *      Author: yushu
 */



#include "sim_interface.h"



sim_interface::sim_interface(){

	rcdata_pub_ = n_sim.advertise<asctec_hl_comm::mav_rcdata>("fcu/rcdata",1);
	imu_pub_= n_sim.advertise<asctec_hl_comm::mav_imu>("fcu/imu_custom", 1);
	pose_pub_= n_sim.advertise<geometry_msgs::Pose>("posefromslam", 1);
	odometry_pub_ = n_sim.advertise<nav_msgs::Odometry>("odometryfromslam", 10);
	cmd_sim_pub = n_sim.advertise<geometry_msgs::PoseStamped>("/pelican/command/pose", 10);

	path_sim_pub = n_sim.advertise<nav_msgs::Path>("positioncmd", 1);  //test only



	cmd_asctec = n_sim.subscribe<asctec_hl_comm::mav_ctrl>("fcu/control", 1, &sim_interface::cmdcallback, this);
	odometry_sub_ = n_sim.subscribe<nav_msgs::Odometry>("/pelican/odometry_sensor1/odometry", 1, &sim_interface::odometry_sim_callback, this);



//    {
//	 nav_msgs::Path gui_path;
//	  geometry_msgs::PoseStamped pose;
//	  std::vector<geometry_msgs::PoseStamped> plan;
//	for (int i=0; i<5; i++){
//	      pose.pose.position.x = i;
//	      pose.pose.position.y = -i;
//	      pose.pose.position.z = i;
//	      pose.pose.orientation.x = 0.0;
//	      pose.pose.orientation.y = 0.0;
//	      pose.pose.orientation.z = 0.0;
//	      pose.pose.orientation.w = 1.0;
//	      plan.push_back(pose);
//	    }
//
//	gui_path.poses.resize(plan.size());
//
//	if(!plan.empty()){
//	      gui_path.header.frame_id = 'test';
//	      gui_path.header.stamp = plan[0].header.stamp;
//	}
//
//	for(unsigned int i=0; i < plan.size(); i++){
//	      gui_path.poses[i] = plan[i];
//	}
//
//
//	path_sim_pub.publish(gui_path);
//	ROS_INFO_STREAM("send path points");
//    }

}

sim_interface::~sim_interface(){

}






void sim_interface::cmdcallback(const asctec_hl_comm::mav_ctrlConstPtr& cmddata){

	  geometry_msgs::PoseStamped posecmd;

	  posecmd.pose.position.x = cmddata->x;
	  posecmd.pose.position.y = cmddata->y;
	  posecmd.pose.position.z = cmddata->z;


	  double R_temp[9];
	  double ea[3]={0,0,0};
	  double quaternion[4];

	  ea[2]=cmddata->yaw;

	  math_function::computeR(&ea[0], &R_temp[0]);
	  math_function::computequaternion(&R_temp[0], &quaternion[0]);

	  posecmd.pose.orientation.x = quaternion[1];
	  posecmd.pose.orientation.y = quaternion[2];
	  posecmd.pose.orientation.z = quaternion[3];
	  posecmd.pose.orientation.w = quaternion[0];

	  printf( "planned quaternion = [ %e, %e, %e, %e ]\n\n",
			  posecmd.pose.orientation.x,posecmd.pose.orientation.y, posecmd.pose.orientation.z, posecmd.pose.orientation.w);

	  cmd_sim_pub.publish(posecmd);
}

void sim_interface::odometry_sim_callback(const nav_msgs::OdometryConstPtr&  odometrydata){
	geometry_msgs::Pose slampose;
	nav_msgs::Odometry slamodometry;
	asctec_hl_comm::mav_imu sim_imu;

	slampose = odometrydata->pose.pose;
	slamodometry =  *odometrydata;

 	pose_pub_.publish(slampose);
 	odometry_pub_.publish(slamodometry);

 	sim_imu.orientation = odometrydata->pose.pose.orientation;

 	sim_imu.height = odometrydata->pose.pose.position.z;
 	sim_imu.differential_height = odometrydata->twist.twist.linear.z;

 	imu_pub_.publish(sim_imu);

	asctec_hl_comm::mav_rcdata rcdata;
 //simulate the rcdata
	rcdata.channel[0]= 2047;
	rcdata.channel[1]= 2047;
	rcdata.channel[2]= 2200;
	rcdata.channel[3]= 2047;
	rcdata.channel[4]= 4095;
	rcdata.channel[5]= 4095;

	rcdata_pub_.publish(rcdata);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "sim_interface");

 //   ros::NodeHandle nh("motion_mav");

    sim_interface sim_asctec;
    ros::spin();

}
