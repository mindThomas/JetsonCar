/* Copyright (C) 2018-2019 Thomas Jespersen, TKJ Electronics. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the MIT License
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the MIT License for further details.
 *
 * Contact information
 * ------------------------------------------
 * Thomas Jespersen, TKJ Electronics
 * Web      :  http://www.tkjelectronics.dk
 * e-mail   :  thomasj@tkjelectronics.dk
 * ------------------------------------------
 */

#include <signal.h>
#include <condition_variable>
#include <stdlib.h>
#include <stdio.h>

#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/format.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "omp.h"

#include <iostream>
#include <cstring>
#include <string>

#include <boost/math/quaternion.hpp> // see https://www.boost.org/doc/libs/1_66_0/libs/math/doc/html/quaternions.html

void QuaternionIntegration_ForwardEuler(
        double dt,
        double omeg_x, double omeg_y, double omeg_z,
        double& q0, double& q1, double& q2, double& q3);
void QuaternionIntegration_Exponential(
        double dt,
        double omeg_x, double omeg_y, double omeg_z,
        double& q0, double& q1, double& q2, double& q3);
void normalizeQuaternion(double& q0, double& q1, double& q2, double& q3);

#define deg2rad(x) (M_PI*x/180.f)
#define rad2deg(x) (180.f*x/M_PI)

int main(int argc, char** argv ) {    
    std::string argv_str(realpath(argv[0], 0));
    std::string base = argv_str.substr(0, argv_str.find_last_of("/"));

    double angularVelocity[3] = {0.3, 0.2, 0.1};
    double dt = 1.0/200;

    // Create initial quaternion for testing - using Boost Quaternions: https://www.boost.org/doc/libs/1_66_0/libs/math/doc/html/quaternions.html
    boost::math::quaternion<double> q0(0.9848,0.1736,0,0); // w,x,y,z
    q0 = q0 / sqrt(boost::math::norm(q0)); // normalize

    // Convert angular velocity vector into a quaternion
    boost::math::quaternion<double> q_omeg(0, angularVelocity[0], angularVelocity[1], angularVelocity[2]);   // use as body angular velocity

    std::cout << "Quaternion are printed in order: \t(w, x, y, z)" << std::endl << std::endl;


    /* Inertial vs Body angular velocity test */
    std::cout << ">>>>> Inertial vs Body angular velocity test <<<<<" << std::endl;
    boost::math::quaternion<double> q_body = q0 * boost::math::exp(0.5*dt*q_omeg); // compute the new quaternion using the quaternion exponential
    boost::math::quaternion<double> q_inertial = boost::math::exp(0.5*dt*q_omeg) * q0; // compute the new quaternion using the quaternion exponential

    // Print results for comparison
    std::cout << "q0 \t\t=\t" << q0 << std::endl;
    std::cout << "q_omeg \t\t=\t" << q_omeg << std::endl;
    std::cout << "q_body \t\t=\t" << q_body << std::endl;
    std::cout << "q_inertial \t=\t" << q_inertial << std::endl;

    std::cout << std::endl;


    /* Integration method test - methods are inspired from https://www.ashwinnarayan.com/post/how-to-integrate-quaternions/  */
    std::cout << ">>>>> Integration method test <<<<<" << std::endl;
    // Method 1: Quaternion Exponential
    boost::math::quaternion<double> q1_exp = q0 * boost::math::exp(0.5*dt*q_omeg);

    // Method 2: Manual Quaternion Exponential
    boost::math::quaternion<double> q_exp_delta;
    if (norm(q_omeg) > 0)
        q_exp_delta = cos(0.5 * dt * sqrt(norm(q_omeg))) + q_omeg / sqrt(norm(q_omeg)) * sin(0.5 * dt * sqrt(norm(q_omeg)));
    else
        q_exp_delta = boost::math::quaternion<double>(1, 0, 0, 0); // unit quaternion since the angular velocity is zero (no movement)

    boost::math::quaternion<double> q2_exp_manual = q0 * q_exp_delta;

    // Method 3: Manual Quaternion Exponential
    double q_1[4] = {q0.R_component_1(), q0.R_component_2(), q0.R_component_3(), q0.R_component_4()};
    QuaternionIntegration_Exponential(dt, angularVelocity[0],angularVelocity[1],angularVelocity[2], q_1[0],q_1[1],q_1[2],q_1[3]);
    boost::math::quaternion<double> q3_exp_manual(q_1[0],q_1[1],q_1[2],q_1[3]);

    // Method 4: Delta Quaternion
    boost::math::quaternion<double> q_delta(1, 0.5*dt*angularVelocity[0], 0.5*dt*angularVelocity[1], 0.5*dt*angularVelocity[2]);
    boost::math::quaternion<double> q4_delta = q0 * q_delta; // compute the new quaternion using body angular velocity to construct the delta quaternion (hence right multiplication)

    // Method 5: Forward Euler
    boost::math::quaternion<double> dq = 0.5 * q0 * q_omeg; // quaternion derivative using body angular velocity
    //boost::math::quaternion<double> dq = 0.5 * q_omeg * q0; // quaternion derivative using inertial angular velocity
    boost::math::quaternion<double> q5_euler = q0 + dt*dq;
    q5_euler = q5_euler / sqrt(boost::math::norm(q5_euler)); // normalize

    // Method 6: Forward Euler 2
    double q_2[4] = {q0.R_component_1(), q0.R_component_2(), q0.R_component_3(), q0.R_component_4()};
    QuaternionIntegration_ForwardEuler(dt, angularVelocity[0],angularVelocity[1],angularVelocity[2], q_2[0],q_2[1],q_2[2],q_2[3]);
    boost::math::quaternion<double> q6_euler(q_2[0],q_2[1],q_2[2],q_2[3]);

    // Print results for comparison
    std::cout << "Angular Velocity \t=\t(" << angularVelocity[0] << ", " << angularVelocity[1] << ", " << angularVelocity[2] << ")" << std::endl;
    std::cout << "Boost Exponential \t=\t" << q1_exp << std::endl;
    std::cout << "Manual Exponential \t=\t" << q2_exp_manual << std::endl;
    std::cout << "Delta \t\t\t=\t" << q4_delta << std::endl;
    std::cout << "Forward Euler \t\t=\t" << q5_euler << std::endl;
    std::cout << "Manual Exponential my \t=\t" << q3_exp_manual << std::endl;
    std::cout << "Forward Euler my \t=\t" << q6_euler << std::endl;

    return 0;
}

void QuaternionIntegration_ForwardEuler(
        double dt,
        double omeg_x, double omeg_y, double omeg_z,
        double& q0, double& q1, double& q2, double& q3)
{
    // The angular velocity is given in body frame
    /* Forward Euler method
     * q_new = q + dt * dq
     * dq = 0.5 * q0 * q_omeg
     * q_omeg = [0,omeg_x,omeg_y,omeg_z]
     */

    // dq = 0.5*Phi(q0)*[0,omeg_x,omeg_y,omeg_z]
    double dq0 = 0.5 * (-q1*omeg_x -q2*omeg_y -q3*omeg_z);
    double dq1 = 0.5 * (+q0*omeg_x -q3*omeg_y +q2*omeg_z);
    double dq2 = 0.5 * (+q3*omeg_x +q0*omeg_y -q1*omeg_z);
    double dq3 = 0.5 * (-q2*omeg_x +q1*omeg_y +q0*omeg_z);

    q0 += dt*dq0;
    q1 += dt*dq1;
    q2 += dt*dq2;
    q3 += dt*dq3;

    normalizeQuaternion(q0, q1, q2, q3);
}

void QuaternionIntegration_Exponential(
        double dt,
        double omeg_x, double omeg_y, double omeg_z,
        double& q0, double& q1, double& q2, double& q3)
{
    // The angular velocity is given in body frame
    /* Quaternion Exponential method
     * q_new = q0 * exp(1/2*dt*q_omeg)
     * q_omeg = [0,omeg_x,omeg_y,omeg_z]
     */

    double omeg_norm = sqrt(omeg_x*omeg_x + omeg_y*omeg_y + omeg_z*omeg_z);

    double q_exp0, q_exp1, q_exp2, q_exp3;

    if (omeg_norm > 0) {
        double sinOmeg = sin(0.5 * dt * omeg_norm);
        q_exp0 = cos(0.5 * dt * omeg_norm);
        q_exp1 = sinOmeg * omeg_x / omeg_norm;
        q_exp2 = sinOmeg * omeg_y / omeg_norm;
        q_exp3 = sinOmeg * omeg_z / omeg_norm;
    } else {
        // unit quaternion since the angular velocity is zero (no movement)
        q_exp0 = 1.0;
        q_exp1 = 0.0;
        q_exp2 = 0.0;
        q_exp3 = 0.0;
    }

    // q_new = Phi(q0) * q_exp
    double q0_new = +q0*q_exp0 -q1*q_exp1 -q2*q_exp2 -q3*q_exp3;
    double q1_new = +q1*q_exp0 +q0*q_exp1 -q3*q_exp2 +q2*q_exp3;
    double q2_new = +q2*q_exp0 +q3*q_exp1 +q0*q_exp2 -q1*q_exp3;
    double q3_new = +q3*q_exp0 -q2*q_exp1 +q1*q_exp2 +q0*q_exp3;

    // Update quaternion
    q0 = q0_new;
    q1 = q1_new;
    q2 = q2_new;
    q3 = q3_new;
}

void normalizeQuaternion(double& q0, double& q1, double& q2, double& q3)
{
    double norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 /= norm;
    q1 /= norm;
    q2 /= norm;
    q3 /= norm;
}
