/* Copyright (C) 2020 Thomas Jespersen, TKJ Electronics. All rights reserved.
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

#ifndef ROS_CALLBACKS_H
#define ROS_CALLBACKS_H

#include <memory>
#include <mutex>

#include "lspc/LSPC.h"
#include "lspc/message_types/MessageTypes.h"

// For CLion to update/capture the changes made to generated services, message types and parameters, open the "build" folder and run "make"
/* Include generated Services */
#include <jetsoncar_interfaces/Reboot.h>
#include <jetsoncar_interfaces/EnterBootloader.h>
#include <jetsoncar_interfaces/SetPID.h>
#include <jetsoncar_interfaces/SetRateLimits.h>

/* Include generated Message Types */
#include <jetsoncar_interfaces/Encoders.h>
#include <jetsoncar_interfaces/Setpoint.h>

/* Include generated Dynamic Reconfigure parameters */
#include <dynamic_reconfigure/server.h>
#include <jetsoncar_driver/ParametersConfig.h>

// Global variables
extern bool MATLAB_log;
extern std::timed_mutex reconfigureMutex;
extern std::shared_ptr<dynamic_reconfigure::Server<jetsoncar_driver::ParametersConfig>> reconfigureServer;
extern jetsoncar_driver::ParametersConfig reconfigureConfig;

void ROS_Callback_Setpoint(const jetsoncar_interfaces::Setpoint::ConstPtr& msg, std::shared_ptr<std::timed_mutex> lspcMutex, std::shared_ptr<lspc::Socket *> lspcObj);
bool ROS_Service_Reboot(jetsoncar_interfaces::Reboot::Request &req, jetsoncar_interfaces::Reboot::Response &res, std::shared_ptr<std::timed_mutex> lspcMutex, std::shared_ptr<lspc::Socket *> lspcObj);
bool ROS_Service_EnterBootloader(jetsoncar_interfaces::EnterBootloader::Request &req, jetsoncar_interfaces::EnterBootloader::Response &res, std::shared_ptr<std::timed_mutex> lspcMutex, std::shared_ptr<lspc::Socket *> lspcObj);
bool ROS_Service_SetPID(jetsoncar_interfaces::SetPID::Request &req, jetsoncar_interfaces::SetPID::Response &res, std::shared_ptr<std::timed_mutex> lspcMutex, std::shared_ptr<lspc::Socket *> lspcObj);
bool ROS_Service_SetRateLimits(jetsoncar_interfaces::SetRateLimits::Request &req, jetsoncar_interfaces::SetRateLimits::Response &res, std::shared_ptr<std::timed_mutex> lspcMutex, std::shared_ptr<lspc::Socket *> lspcObj);
void ROS_ReconfigureCallback(jetsoncar_driver::ParametersConfig &config, uint32_t level, std::shared_ptr<std::timed_mutex> lspcMutex, std::shared_ptr<lspc::Socket *> lspcObj);
void ROS_WatchdogTimeoutCallback(const ros::TimerEvent& e);

#endif // ROS_CALLBACKS_H
