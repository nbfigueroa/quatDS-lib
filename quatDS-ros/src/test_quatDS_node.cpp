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

    /* Test simulation */ 
    int i (1), max_iter(500);
    double dt(0.075), pos_error(0), quat_error(0);
    VectorXd x_dot(VectorXd::Zero(3,1)), omega(VectorXd::Zero(3,1));
    VectorXd x_curr(VectorXd::Zero(3,1)), q_curr(VectorXd::Zero(4,1));
    
    /* Define initial position and orientation */
    x_curr << -0.4486, 0.3119, 0.43699;
    q_curr << 0.69736, -0.0454,-0.713,0.05638;
    while (i<max_iter){

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
		if (pos_error < 0.0025 && quat_error < 0.025)
			break;
    	i++;
    }

    // Stop the node's resources
    ros::shutdown();
    // Exit tranquilly
    return 0;

}