/*
 * motion_planning.h
 *
 *  Created on: Dec 21, 2016
 *      Author: yu
 */

#ifndef ASCTEC_MAV_MOTION_PLANNING_SRC_MOTION_PLANNING_H_
#define ASCTEC_MAV_MOTION_PLANNING_SRC_MOTION_PLANNING_H_



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

    //send command in acc mode
    void send_acc_ctrl(void);

    //send command in velocity mode:
    void send_velo_control(void);

    ros::NodeHandle n;
    ros::NodeHandle pnh_;

    ros::Publisher pub;
    ros::Publisher pub2;

    ros::Publisher pub3;

    ros::Publisher llcmd_pub_acc;

    ros::Publisher llcmd_pub_vel;


    ros::Subscriber rcdata_sub_;

    ros::Subscriber sub;

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
