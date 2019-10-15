/*
 * Copyright (C) 2018 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author:  Nadia Figueroa
 * email:   nadia.figueroafernandez@epfl.ch
 * website: lasa.epfl.ch
 *
 * This work was supported by the EU project Cogimon H2020-ICT-23-2014.
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */


#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <vector>
#include "eigen3/Eigen/Dense"
#include "Quaternions.h"
#include "geometry_msgs/Pose.h"

using namespace std;
using namespace Eigen;


/****************   Global Variables **********************/
geometry_msgs::Pose  msg_real_pose_;

/* Initialize Linear Position DS parameters */
MatrixXd A_p(MatrixXd::Zero(3,3));
VectorXd x_att(VectorXd::Zero(3,1));


/* Initialize Linear Quaternion DS parameters */
MatrixXd A_o(MatrixXd::Zero(3,3));
VectorXd q_att(VectorXd::Zero(4,1));

/* Define Initialize pOSE */
VectorXd x_curr(VectorXd::Zero(3,1)), q_curr(VectorXd::Zero(4,1));
VectorXd x_des(VectorXd::Zero(3,1)), q_des(VectorXd::Zero(4,1));
double dt(0.005), ds_velocity_limit(0.25);

/* Subscribers/Publishers*/
geometry_msgs::Twist      msg_desired_velocity_;
geometry_msgs::Quaternion msg_desired_quaternion_;
ros::Subscriber sub_robot_pose_;
ros::Publisher  pub_desired_twist_ ;
ros::Publisher  pub_desired_quat_ ;
/*********   [will put in a class after testing] *********/


void robotPoseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
  
    /* Reading current robot state */
    msg_real_pose_ = *msg;

    x_curr(0) = msg_real_pose_.position.x;
    x_curr(1) = msg_real_pose_.position.y;
    x_curr(2) = msg_real_pose_.position.z;
    q_curr(0) = msg_real_pose_.orientation.w;
    q_curr(1) = msg_real_pose_.orientation.x;
    q_curr(2) = msg_real_pose_.orientation.y;
    q_curr(3) = msg_real_pose_.orientation.z;

    /* Parameters for simulation */ 
    double pos_error(0), quat_error(0);
    VectorXd x_dot(VectorXd::Zero(3,1)), omega(VectorXd::Zero(3,1));  

    /* Visualization Variables */ 
    static tf::TransformBroadcaster br;
    tf::Transform pose_des, target;
    target.setOrigin(tf::Vector3(x_att(0), x_att(1), x_att(2)));
    target.setRotation(tf::Quaternion(q_att(1),q_att(2),q_att(3),q_att(0)));
    /********************************************************************/
    /* Position Dynamics Computation */
    x_dot  = A_p * (x_curr-x_att);    
    if (x_dot.norm() > ds_velocity_limit)
        x_dot = x_dot.normalized() * ds_velocity_limit;    
    pos_error = (x_curr-x_att).norm();
    ROS_INFO_STREAM( "Translational Velocity: " << x_dot(0) << " " << x_dot(1) << " " << x_dot(2));    
    if (pos_error < 0.02){
        ROS_INFO_STREAM("******** POSITION ATTRACTOR REACHED ********");
        x_dot(0)=0; x_dot(1)=0; x_dot(2)=0;
    }

    /* Quaternion Dynamics Computation */
    omega  =   A_o * quat_diff(q_curr, q_att);
    if (omega.norm() > ds_velocity_limit*5)
        omega = omega.normalized() * ds_velocity_limit*5;
    quat_error = quat_diff(q_curr, q_att).norm();
    ROS_INFO_STREAM( "Angular Velocity: " << omega(0) << " " << omega(1) << " " << omega(2));    
    if (quat_error < 0.05){
        ROS_INFO_STREAM("******** QUATERNION ATTRACTOR REACHED ********");
        omega(0)=0; omega(1)=0; omega(2)=0;
    }
    /********************************************************************/

    /* Publish velocity commands */
    x_des = x_curr + x_dot*dt;
    q_des = quat_mul(quat_exp(0.5 * quat_deriv(omega * dt * 10 )), q_curr);

    msg_desired_velocity_.linear.x  = x_dot(0);
    msg_desired_velocity_.linear.y  = x_dot(1);
    msg_desired_velocity_.linear.z  = x_dot(2);
    msg_desired_velocity_.angular.x = omega(0);
    msg_desired_velocity_.angular.y = omega(1);
    msg_desired_velocity_.angular.z = omega(2);
    pub_desired_twist_.publish(msg_desired_velocity_);

    msg_desired_quaternion_.w    = q_des(0);
    msg_desired_quaternion_.x    = q_des(1);
    msg_desired_quaternion_.y    = q_des(2);
    msg_desired_quaternion_.z    = q_des(3);  
    pub_desired_quat_.publish(msg_desired_quaternion_);

    /* Monitor convergence */
    ROS_INFO_STREAM( "Position Error: " << pos_error << " " << "Orientation Error:" << quat_error );    
            

    /* Broadcast Transforms */
    pose_des.setOrigin(tf::Vector3(x_des(0), x_des(1), x_des(2)));
    pose_des.setRotation(tf::Quaternion(q_des(1),q_des(2),q_des(3),q_des(0)));
    br.sendTransform(tf::StampedTransform(pose_des, ros::Time::now(),  "lwr_base_link", "desired_pose"));
    br.sendTransform(tf::StampedTransform(target, ros::Time::now(),  "lwr_base_link", "attractor"));

}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_quatDS_node");
    ros::NodeHandle nh;
    ros::NodeHandle _nh("~");

    /****************** Define initial position and orientation ******************/
    /* Define DS Parameters */
    A_p(0,0) = -1;
    A_p(1,1) = -2;
    A_p(2,2) = -3;

    A_o(0,0) = -10;
    A_o(1,1) = -10;
    A_o(2,2) = -10;

    x_att << -0.419, -0.0468, 0.15059;
    q_att << -0.04616,-0.124,0.991007,-0.018758;    

    sub_robot_pose_     = nh.subscribe("/lwr/ee_pose", 1000, robotPoseCallback);
    pub_desired_twist_  = nh.advertise<geometry_msgs::Twist>("/lwr/joint_controllers/passive_ds_command_vel", 1);
    pub_desired_quat_   = nh.advertise<geometry_msgs::Quaternion>("/lwr/joint_controllers/passive_ds_command_orient", 1);
    ros::spin();
    // Stop the node's resources
    ros::shutdown();
    // Exit tranquilly
    return 0;

}