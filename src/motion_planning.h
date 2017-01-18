/*
 * motion_planning.h
 *
 *  Created on: Dec 21, 2016
 *      Author: yu
 */

#ifndef ASCTEC_MAV_MOTION_PLANNING_SRC_MOTION_PLANNING_H_
#define ASCTEC_MAV_MOTION_PLANNING_SRC_MOTION_PLANNING_H_

//other header files
#include "minimum_snap_traj.h"

#include "ros/ros.h"
#include "std_msgs/String.h"

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


/**
 * This tutorial demonstrates simple usage of mit-g-710, using it to command the turtle in the ROS package
 */


typedef dynamic_reconfigure::Server<asctec_mav_motion_planning::motion_planning_paraConfig> ReconfigureServer;

//using namespace std;
class TeleopIMU{
public:
    TeleopIMU();
private:
    void callBack(const sensor_msgs::Imu::ConstPtr& imu);

    void rcdataCallback(const asctec_hl_comm::mav_rcdataConstPtr& rcdata);

    void gpsdataCallback(const asctec_hl_comm::GpsCustomConstPtr&   gpsdata);

    void imudataCallback(const asctec_hl_comm::mav_imuConstPtr&   imudata);

    void flagcmdCallback(const asctec_mav_motion_planning::flag_cmdConstPtr&  flagcmd);

	//the topic name is still under discussion, from the SLAM module
	void poseCallback(const geometry_msgs::Pose::ConstPtr& pose);

	//the topic name is still under discussion, from the SLAM module
	void odometryCallback(const nav_msgs::OdometryConstPtr& odometry);

    //send command in acc mode
    void send_acc_ctrl(void);

    //send command in velocity mode:
    void send_velo_control(void);

    //convert the position from LLA to NED coordinate
    void LLP_Euclidean(Eigen::Vector3d & LLA);

    ros::NodeHandle n;
    ros::NodeHandle pnh_;

    ros::Publisher pub;
    ros::Publisher pub2;

    ros::Publisher pub3;

    ros::Publisher llcmd_pub_acc;

    ros::Publisher llcmd_pub_vel;

    ros::Publisher ext_state;

    ros::Subscriber pose_sub_;  //subscribe the current position of the UAV, from SLAM module
    ros::Subscriber odometry_sub_;  //the topic name is still under discussion, from the SLAM module


    ros::Subscriber rcdata_sub_;

    ros::Subscriber gps_custom_sub_;
    ros::Subscriber imu_custom_sub_;

    ros::Subscriber sub;

    ros::Subscriber flag_cmd_sub;


    sensor_fusion_comm::ExtState state_feedback;

    Eigen::Vector3d LLA; //latitude, longitude, altitude, this order
    Eigen::Vector3d LLA_0;  //latitude, longitude, altitude, this order, the original point

    asctec_hl_comm::mav_rcdata rcdata_last; //record the rcdata last time

    //determine if the RC transmitter sends the position commands
    int flag_rc_cmd;

    //out door or indoor,
    //1: outdoor, GPS provides the position information
    //2: indoor, SLAM module provides the position information
    int flag_pose_source;


    asctec_hl_comm::mav_ctrl global_position_cmd; //global position commands, used to calculate the position commands from RC transmitter

    int64_t time;
    double time_body, time_doby_last;


    /// gain from AutoPilot values to 1/1000 degree for the input from the pitch and roll "stick"
    /**
     * This value should be equal to the one that can be found when reading the controller parameters of the LLP by the AscTec AutoPilot software.
     * It is only relevant, when this interface is used to send roll/pitch (or x/y velocity when in GPS mode) commands to the LLP.
     */
    int k_stick_;

    /// gain from AutoPilot values to 1/1000 degree for the input from the yaw "stick"
    /**
     * This value should be equal to the one that can be found when reading the controller parameters of the LLP by the AscTec AutoPilot software.
     * It is only relevant, when this interface is used to send yaw commands to the LLP.
     */
    int k_stick_yaw_;


   // ros::Rate llcmb_pubrate;

    // dynamic reconfigure
    ReconfigureServer *motionconf_srv_;
    void cbmotionConfig(asctec_mav_motion_planning::motion_planning_paraConfig & config, uint32_t level);

    asctec_mav_motion_planning::motion_planning_paraConfig config_motion;



};





#endif /* ASCTEC_MAV_MOTION_PLANNING_SRC_MOTION_PLANNING_H_ */
