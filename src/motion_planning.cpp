
/*
 * Copyright (C) 2016, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include "motion_planning.h"
//
//#include "ros/ros.h"
//#include "std_msgs/String.h"

//#include "sensor_msgs/Imu.h"
//#include "geometry_msgs/Twist.h"
//#include "geometry_msgs/PoseStamped.h"
//#include "geometry_msgs/TwistStamped.h"
//
//
//
//// message includes
//#include <asctec_hl_comm/mav_rcdata.h>
//#include <asctec_hl_comm/mav_ctrl.h>
//#include <asctec_hl_comm/mav_imu.h>
//#include <asctec_hl_comm/mav_status.h>
//#include <asctec_hl_comm/GpsCustom.h>
//#include <sensor_msgs/Imu.h>
//#include <sensor_msgs/NavSatFix.h>
//#include <geometry_msgs/Vector3Stamped.h>
//
//
//// dynamic reconfigure includes
//#include <dynamic_reconfigure/server.h>
//#include <asctec_mav_motion_planning/motion_planning_paraConfig.h>
//
//
//
///**
// * This tutorial demonstrates simple usage of mit-g-710, using it to command the turtle in the ROS package
// */
//
//
//typedef dynamic_reconfigure::Server<asctec_mav_motion_planning::motion_planning_paraConfig> ReconfigureServer;
//
//using namespace std;
//class TeleopIMU{
//public:
//    TeleopIMU();
//private:
//    void callBack(const sensor_msgs::Imu::ConstPtr& imu);
//
//    void rcdataCallback(const asctec_hl_comm::mav_rcdataConstPtr& rcdata);
//
//    //send command in acc mode
//    void send_acc_ctrl(void);
//
//    //send command in velocity mode:
//    void send_velo_control(void);
//
//    ros::NodeHandle n;
//    ros::NodeHandle pnh_;
//
//    ros::Publisher pub;
//    ros::Publisher pub2;
//
//    ros::Publisher pub3;
//
//    ros::Publisher llcmd_pub_acc;
//
//    ros::Publisher llcmd_pub_vel;
//
//
//    ros::Subscriber rcdata_sub_;
//
//    ros::Subscriber sub;
//
//    /// gain from AutoPilot values to 1/1000 degree for the input from the pitch and roll "stick"
//    /**
//     * This value should be equal to the one that can be found when reading the controller parameters of the LLP by the AscTec AutoPilot software.
//     * It is only relevant, when this interface is used to send roll/pitch (or x/y velocity when in GPS mode) commands to the LLP.
//     */
//    int k_stick_;
//
//    /// gain from AutoPilot values to 1/1000 degree for the input from the yaw "stick"
//    /**
//     * This value should be equal to the one that can be found when reading the controller parameters of the LLP by the AscTec AutoPilot software.
//     * It is only relevant, when this interface is used to send yaw commands to the LLP.
//     */
//    int k_stick_yaw_;
//
//
//   // ros::Rate llcmb_pubrate;
//
//    // dynamic reconfigure
//    ReconfigureServer *motionconf_srv_;
//    void cbmotionConfig(asctec_mav_motion_planning::motion_planning_paraConfig & config, uint32_t level);
//
//    asctec_mav_motion_planning::motion_planning_paraConfig config_motion;
//
//
//
//};

TeleopIMU::TeleopIMU():
pnh_("~/fcu")
{
	pnh_.param("k_stick", k_stick_, 25);
	pnh_.param("k_stick_yaw", k_stick_yaw_, 120);


    pub=n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",1);


    llcmd_pub_acc = n.advertise<asctec_hl_comm::mav_ctrl>("fcu/control",1); //command to HL_inteface
   // llcmb_pubrate.Rate(20);//20HZ

    llcmd_pub_vel = n.advertise<asctec_hl_comm::mav_ctrl>("fcu/control",1); //command to HL_interface



    pub2=n.advertise<geometry_msgs::PoseStamped>("command/pose",1); //command to quadrotor

    pub3 = n.advertise<geometry_msgs::TwistStamped>("command/twist",1); //velocity command to quadrotor

	sub=n.subscribe<sensor_msgs::Imu>("/imu/data",10,&TeleopIMU::callBack,this);
    //should run rosrun xsens_driver mtnode.py in order to run the xsens_driver node,
    //this node will advertise /imu/data
    //topic


	rcdata_sub_ = n.subscribe<asctec_hl_comm::mav_rcdata>("fcu/rcdata", 1, &TeleopIMU::rcdataCallback, this);


	  // bring up dynamic reconfigure
	motionconf_srv_ = new ReconfigureServer(pnh_);
	ReconfigureServer::CallbackType f = boost::bind(&TeleopIMU::cbmotionConfig, this, _1, _2);
	motionconf_srv_->setCallback(f);


   // send_acc_ctrl();
}






void TeleopIMU::rcdataCallback(const asctec_hl_comm::mav_rcdataConstPtr& rcdata){
	//k_stick_yaw_

	//int k_stick_;


	//the command is in real angle and real angular velocity
	//roll and pitch angle cmd  =real_angle*1000/K_stick

	//yaw cmd=real_anglular_velocity*1000/k_stick_yaw,

	asctec_hl_comm::mav_ctrl msg;

	if  ((rcdata->channel[5]) < 1800 )
	{
		 msg.type = asctec_hl_comm::mav_ctrl::acceleration;

		 //note the rcdata is not the same with the command sent to LL from HL

		 msg.x =  (rcdata->channel[0]-2047) *k_stick_/1000.0*M_PI/180.0;
		 msg.y =  (-rcdata->channel[1] + 2047) *k_stick_/1000.0*M_PI/180.0;   //opposite direction
		 msg.yaw = (-rcdata->channel[3] + 2047) *k_stick_yaw_/1000.0*M_PI/180.0;   //opposite direction

		 msg.z = rcdata->channel[2]/4096.0;

		 //test only:
//		 msg.x = 0;
//		 msg.y = 0;
//		 msg.yaw = 0;
//		 msg.z = 0;
	}

	if (((rcdata->channel[5]) > 1800 ) & ((rcdata->channel[5]) < 2500))
	{
		msg.type = asctec_hl_comm::mav_ctrl::velocity_body;


//		  ctrlLL.x = helper::clamp<short>(-2047, 2047, (short)(msg.x / config_.max_velocity_xy * 2047.0));
//		  ctrlLL.y = helper::clamp<short>(-2047, 2047, (short)(msg.y / config_.max_velocity_xy * 2047.0));
//		  ctrlLL.yaw = helper::clamp<short>(-2047, 2047, (short)(msg.yaw / config_.max_velocity_yaw* 2047.0));
//		  ctrlLL.z = helper::clamp<short>(-2047, 2047, (short)(msg.z / config_.max_velocity_z * 2047.0)) + 2047; // "zero" is still 2047!

		msg.x = (rcdata->channel[0]-2047) /2047*config_motion.max_velocity_xy;
		msg.y = (-rcdata->channel[1] + 2047) /2047.0*config_motion.max_velocity_xy;
		msg.yaw = (-rcdata->channel[3] + 2047) /2047.0*config_motion.max_velocity_yaw;
		msg.z = ( rcdata->channel[2]-2047)/2047.0*config_motion.max_velocity_z;

	}
//	if ((rcdata->channel[5]) > 4000 )
//	{
//		msg.type = asctec_hl_comm::mav_ctrl::velocity_body;
//
//	}


	ROS_INFO_STREAM("k_stick: "<< k_stick_);

	ROS_INFO_STREAM("k_stick_yaw: "<< k_stick_yaw_);

	ROS_INFO_STREAM("channel 0: "<<(rcdata->channel[0]));
	ROS_INFO_STREAM("channel 1: "<<(rcdata->channel[1]));
	ROS_INFO_STREAM("channel 2: "<<(rcdata->channel[2]));
	ROS_INFO_STREAM("channel 3: "<<(rcdata->channel[3]));
	ROS_INFO_STREAM("channel 4: "<<(rcdata->channel[4]));
	ROS_INFO_STREAM("channel 5: "<<(rcdata->channel[5]));
	ROS_INFO_STREAM("channel 6: "<<(rcdata->channel[6]));

	ROS_INFO_STREAM("cmd from HL to LL, x: "<<msg.x);
	ROS_INFO_STREAM("cmd from HL to LL, y: "<<msg.y);
	ROS_INFO_STREAM("cmd from HL to LL, yaw: "<<msg.yaw);

    llcmd_pub_vel.publish(msg);

}



void TeleopIMU::callBack(const sensor_msgs::Imu::ConstPtr& imu)
{
    geometry_msgs::Twist vel;
//    geometry_msgs/Vector3 linear
//      float64 x
//      float64 y
//      float64 z
//    geometry_msgs/Vector3 angular
//      float64 x
//      float64 y
//      float64 z
 //   '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'

    vel.linear.x = imu->angular_velocity.x;
    vel.linear.y = imu->angular_velocity.y;
    vel.linear.z = imu->angular_velocity.z;
    vel.angular.x = imu->angular_velocity.x;
    vel.angular.y = imu->angular_velocity.y;
    vel.angular.z = imu->angular_velocity.z;


    //vel.linear = '[2.0, 0.0, 0.0]';
   // vel.angular = '[2.0, 0.0, 0.0]';
    pub.publish(vel);

    //added on October, 10, 2016
    //the message is command to the simulated quadrotor in gazebo
    geometry_msgs::PoseStamped postoquadrotor;
//    std_msgs/Header header
//      uint32 seq
//      time stamp
//      string frame_id
//    geometry_msgs/Pose pose
//      geometry_msgs/Point position
//        float64 x
//        float64 y
//        float64 z
//      geometry_msgs/Quaternion orientation
//        float64 x
//        float64 y
//        float64 z
//        float64 w

    postoquadrotor.pose.position.x = 1;
    postoquadrotor.pose.position.y=1;
    postoquadrotor.pose.position.z =1;
    postoquadrotor.pose.orientation.x= imu->orientation.x;
    postoquadrotor.pose.orientation.y=imu->orientation.y;
    postoquadrotor.pose.orientation.z=imu->orientation.z;
    postoquadrotor.pose.orientation.w=imu->orientation.w;
    postoquadrotor.header.seq = imu->header.seq;
    postoquadrotor.header.stamp = imu->header.stamp;
   // postoquadrotor.header.frame_id = imu->header.frame_id;
    postoquadrotor.header.frame_id  =   "world";



    //publish the massege
    pub2.publish(postoquadrotor);



    geometry_msgs::TwistStamped velquadrotor;
//    std_msgs/Header header
//      uint32 seq
//      time stamp
//      string frame_id
//    geometry_msgs/Twist twist
//      geometry_msgs/Vector3 linear
//        float64 x
//        float64 y
//        float64 z
//      geometry_msgs/Vector3 angular
//        float64 x
//        float64 y
//        float64 z

    velquadrotor.header.frame_id ="world";
    velquadrotor.header.stamp =imu->header.stamp;
    velquadrotor.header.seq = imu->header.seq;

    velquadrotor.twist.linear.x = imu->angular_velocity.x;
    velquadrotor.twist.linear.y = imu->angular_velocity.y;
    velquadrotor.twist.linear.z = imu->angular_velocity.z;
    velquadrotor.twist.angular.x = imu->angular_velocity.x;
    velquadrotor.twist.angular.y = imu->angular_velocity.y;
    velquadrotor.twist.angular.z = imu->angular_velocity.z;

    //publish the massege
     pub3.publish(velquadrotor);

}

void TeleopIMU::send_acc_ctrl(void){

	//send the command in acc mode


	asctec_hl_comm::mav_ctrl msg;

	//the command is in real angle and real angular velocity
	//roll and pitch angle cmd  =real_angle*1000/K_stick

	//yaw cmd=real_anglular_velocity*1000/k_stick_yaw,


    msg.x = 1000;
    msg.y = 1000;
    msg.z = 0.5;

    msg.yaw = 1000* M_PI / 180.0;
    msg.v_max_xy = -4000; // use max velocity from config
    msg.v_max_z = 4000;


      msg.type = asctec_hl_comm::mav_ctrl::acceleration;
//    else if (type == "vel")
//      msg.type = asctec_hl_comm::mav_ctrl::velocity;
//    else if (type == "pos")
//      msg.type = asctec_hl_comm::mav_ctrl::position;
//    else if (type == "vel_b")
//      msg.type = asctec_hl_comm::mav_ctrl::velocity_body;
//    else if (type == "pos_b")
//      msg.type = asctec_hl_comm::mav_ctrl::position_body;
//    else
//    {
//      ROS_ERROR("Command type not recognized");
//      usage();
//      return -1;
//    }

    pub = n.advertise<asctec_hl_comm::mav_ctrl> ("fcu/control", 1);
//    for (int i = 0; i < 15; i++)
//    {
//      pub.publish(msg);
//      if (!ros::ok())
//        return 0;
//      r.sleep();
//    }

    while (ros::ok()){
    	llcmd_pub_acc.publish(msg);
    	//ROS_INFO(msg);
    	ros::spinOnce();
    //	llcmb_pubrate.sleep();
    	  // %EndTag(RATE_SLEEP)%
    }
}


void TeleopIMU::send_velo_control(void){

	//send velocity command



}


void TeleopIMU::cbmotionConfig(asctec_mav_motion_planning::motion_planning_paraConfig & config, uint32_t level){

	ROS_INFO_STREAM("max_velocity_xy: "<<config.max_velocity_xy);

	config_motion = config;

	ROS_INFO_STREAM("max_velocity_xy: "<<config_motion.max_velocity_xy);

}





int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_mav");
    TeleopIMU teleop_turtle;

    ros::spin();
}
// %EndTag(FULLTEXT)%



