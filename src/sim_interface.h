/*
 * sim_interface.h
 *
 *  Created on: Jan 18, 2017
 *      Author: yushu
 */

#ifndef ASCTEC_MAV_MOTION_PLANNING_SRC_SIM_INTERFACE_H_
#define ASCTEC_MAV_MOTION_PLANNING_SRC_SIM_INTERFACE_H_



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



//using namespace std;
class sim_interface{
public:
	sim_interface();
	~sim_interface();

	ros::Publisher path_sim_pub; //test only, publish the path information
private:

    void cmdcallback(const asctec_hl_comm::mav_ctrlConstPtr& cmddata);
    void odometry_sim_callback(const nav_msgs::OdometryConstPtr&  odometrydata);


    ros::NodeHandle n_sim;

    ros::Publisher rcdata_pub_;  //simulate the rcdata from RC transmitter
    ros::Publisher imu_pub_;  //simulate the imu data from asctec UAV
    ros::Publisher pose_pub_;  //simulate the pose from slam module
    ros::Publisher odometry_pub_;  //simulate the pose from slam module
    ros::Publisher cmd_sim_pub;  //pose cmd to the simulator UAV




    ros::Subscriber cmd_asctec; //subscribe the commands to asctec
    ros::Subscriber path_sub_;  //subscribe the path from path planning modules
    ros::Subscriber odometry_sub_;  //  from the simulator

};








#endif /* ASCTEC_MAV_MOTION_PLANNING_SRC_SIM_INTERFACE_H_ */
