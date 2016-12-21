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

#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"

// message includes
#include <asctec_hl_comm/mav_rcdata.h>
#include <asctec_hl_comm/mav_ctrl.h>
#include <asctec_hl_comm/mav_imu.h>
#include <asctec_hl_comm/mav_status.h>
#include <asctec_hl_comm/GpsCustom.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3Stamped.h>


// dynamic reconfigure includes
#include <dynamic_reconfigure/server.h>
#include <asctec_mav_motion_planning/motion_planning_paraConfig.h>




class Minimumsnap{
public:
	Minimumsnap(ros::NodeHandle & nh);
	~Minimumsnap();

private:

	ros::NodeHandle nh_minsnap;

    void poseCallback(const geometry_msgs::Pose::ConstPtr& pose);

  //  void cmdCallback(const nav_msgs::PathConstPtr& positioncmd);

    //the function used to calculate the minimum trajectory from start point to end point:
    float minimumsnap_line(float t0, float alpha, float x0, float xf, float time);


    ros::Publisher taj_pub;  //publish the calculated minimum trajectory
    ros::Publisher control_pub;  //publish the calculated minimum trajectory
    ros::Subscriber pose_sub_;  //subscribe the current position of the UAV
    ros::Subscriber cmd_sub_;  //subscribe the commanded position of the UAV

};





#endif /* ASCTEC_MAV_MOTION_PLANNING_SRC_MIMIMUN_SNAP_TRAJ_H_ */
