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
	cmd_sim_pub = n_sim.advertise<geometry_msgs::PoseStamped>("/firefly/command/pose", 10);

	cmd_asctec = n_sim.subscribe<asctec_hl_comm::mav_ctrl>("fcu/control", 1, &sim_interface::cmdcallback, this);
	odometry_sub_ = n_sim.subscribe<nav_msgs::Odometry>("/firefly/odometry_sensor1/odometry", 1, &sim_interface::odometry_sim_callback, this);


}

sim_interface::~sim_interface(){

}


void sim_interface::cmdcallback(const asctec_hl_comm::mav_ctrlConstPtr& cmddata){

}

void sim_interface::odometry_sim_callback(const nav_msgs::OdometryConstPtr&  odometrydata){

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "sim_interface");

 //   ros::NodeHandle nh("motion_mav");

    sim_interface sim_asctec;

 //   Minimumsnap minimumfun;

    ros::spin();

}
