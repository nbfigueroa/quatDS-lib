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

using namespace std;
using namespace Eigen;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_quatDS_node");
    ros::NodeHandle nh;
    ros::NodeHandle _nh("~");

    /* Initialize Linear Position DS parameters */
    MatrixXd A_p;
    A_p = MatrixXd::Zero(3,3);
	A_p(0,0) = -0.5*1;
	A_p(1,1) = -0.5*2;
	A_p(2,2) = -0.5*3;
	VectorXd x_att(VectorXd::Zero(3,1));


    /* Initialize Linear Quaternion DS parameters */
    MatrixXd A_o;
	A_o = MatrixXd::Zero(3,3);
	A_o(0,0) = -1.5;
	A_o(1,1) = -1.5;
	A_o(2,2) = -1.5;
	VectorXd q_att(VectorXd::Zero(4,1));

	/* Define attractors */
    x_att << -0.419, -0.0468, 0.15059;
    q_att << -0.04616,-0.124,0.991007,-0.018758;

    /* Parameters for simulation */ 
    int i (1);
    double dt(0.0025), pos_error(0), quat_error(0);
    VectorXd x_dot(VectorXd::Zero(3,1)), omega(VectorXd::Zero(3,1));
    VectorXd x_curr(VectorXd::Zero(3,1)), q_curr(VectorXd::Zero(4,1));
    static tf::TransformBroadcaster br;
    tf::Transform pose_curr, target, init;
    target.setOrigin(tf::Vector3(x_att(0), x_att(1), x_att(2)));
    target.setRotation(tf::Quaternion(q_att(1),q_att(2),q_att(3),q_att(0)));


    /* Define initial position and orientation */
    int init_pose = 2;
    if (init_pose == 1){
            /* From go_center position (from kuka-lags-tasks) */
            x_curr << -0.398287790221, 0.276403682489, 0.40357671952;
            q_curr << 0.590064946744, 0.679998360383, -0.300609096301, -0.314737604554;
    }else if(init_pose == 2){
            /* From go_left position (from simple examples) */
            x_curr << -0.3687019795, -0.31078379649, 0.353700792986;
            q_curr << 0.711304, -0.021037765273, -0.702077440469, -0.0262812481386;
    }else if (init_pose == 3){
            /* From go_top_center position (from kuka-lags-tasks) */
            x_curr << -0.674814265398, 0.214883445878, 0.614376695163;
            q_curr <<  0.746608, 0.563370280825, -0.201834619809, -0.290607963541;
    }

    
    init.setOrigin(tf::Vector3(x_curr(0), x_curr(1), x_curr(2)));
    init.setRotation(tf::Quaternion(q_curr(1),q_curr(2),q_curr(3),q_curr(0)));

    /* Simulation loop */
    while (ros::ok()){

    	/* Position Dynamics Computation */
        x_dot  = A_p * (x_curr-x_att);
		x_curr = x_curr + x_dot*dt;
		pos_error = (x_curr-x_att).norm();
		

        /* Quaternion Dynamics Computation */
        omega  = A_o * quat_diff(q_curr, q_att);
        q_curr = quat_mul(quat_exp(0.5 * quat_deriv(omega * dt)), q_curr);
        quat_error = quat_diff(q_curr, q_att).norm();


        /* Monitor convergence */
        cout << "[iter=" << i<< " ] Position Error: " << pos_error << 
                 " " << "Orientation Error:" << quat_error << endl;    
		if (pos_error < 0.005 && quat_error < 0.01)
			break;
    	i++;


    	/* Create transform to broadcast */
        pose_curr.setOrigin(tf::Vector3(x_curr(0), x_curr(1), x_curr(2)));
		pose_curr.setRotation(tf::Quaternion(q_curr(1),q_curr(2),q_curr(3),q_curr(0)));
		br.sendTransform(tf::StampedTransform(pose_curr, ros::Time::now(),  "world", "robot_pose"));
        br.sendTransform(tf::StampedTransform(target, ros::Time::now(),  "world", "attractor"));
        br.sendTransform(tf::StampedTransform(init, ros::Time::now(),  "world", "init"));

		ros::spinOnce();
		ros::Duration(dt).sleep();

    }

    // Stop the node's resources
    ros::shutdown();
    // Exit tranquilly
    return 0;

}