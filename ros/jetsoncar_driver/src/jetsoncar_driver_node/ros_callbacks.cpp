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

#include <jetsoncar_driver/jetsoncar_driver_node/ros_callbacks.h>

// Global variables
bool MATLAB_log = false;
std::timed_mutex reconfigureMutex;
std::shared_ptr<dynamic_reconfigure::Server<jetsoncar_driver::ParametersConfig>> reconfigureServer;
jetsoncar_driver::ParametersConfig reconfigureConfig;

void ROS_Callback_Setpoint(const jetsoncar_interfaces::Setpoint::ConstPtr& msg, std::shared_ptr<std::timed_mutex> lspcMutex, std::shared_ptr<lspc::Socket *> lspcObj) {
    if (!lspcMutex->try_lock_for(std::chrono::milliseconds(100))) return; // could not get lock

    if ((*lspcObj)->isOpen()) {
        //ROS_DEBUG_STREAM("Sending cmd_vel to Kugle as both VelocityReference_Heading and AngularVelocityReference_Body");
        lspc::MessageTypesFromPC::Setpoint_t payload;
        payload.angular_velocity = msg->angular_velocity;
        payload.steering = msg->steering_angle;
        std::vector<uint8_t> payloadPacked((uint8_t *)&payload, (uint8_t *)&payload+sizeof(payload)); // this method of "serializing" requires that PC runs Little Endian (which most PC processors do = Intel x86, AMD 64 etc.)
        (*lspcObj)->send(lspc::MessageTypesFromPC::Setpoint, payloadPacked);
    }

    lspcMutex->unlock();
}

bool ROS_Service_Reboot(jetsoncar_interfaces::Reboot::Request &req, jetsoncar_interfaces::Reboot::Response &res, std::shared_ptr<std::timed_mutex> lspcMutex, std::shared_ptr<lspc::Socket *> lspcObj)
{
    res.acknowledged = false;
    if (!(*lspcObj) || !lspcMutex->try_lock_for(std::chrono::milliseconds(100))) {
        return true; // could not get lock
    }

    if (!(*lspcObj)->isOpen()) {
        lspcMutex->unlock();
        return true; // connection is not open
    }

    std::vector<uint8_t> empty;
    (*lspcObj)->send(lspc::MessageTypesFromPC::Reboot, empty);
    lspcMutex->unlock();

    res.acknowledged = true;
    return true;
}

bool ROS_Service_EnterBootloader(jetsoncar_interfaces::EnterBootloader::Request &req, jetsoncar_interfaces::EnterBootloader::Response &res, std::shared_ptr<std::timed_mutex> lspcMutex, std::shared_ptr<lspc::Socket *> lspcObj)
{
    res.acknowledged = false;
    if (!lspcMutex->try_lock_for(std::chrono::milliseconds(100))) {
        return true; // could not get lock
    }

    if (!(*lspcObj) || !(*lspcObj)->isOpen()) {
        lspcMutex->unlock();
        return true; // connection is not open
    }

    std::vector<uint8_t> empty;
    (*lspcObj)->send(lspc::MessageTypesFromPC::EnterBootloader, empty);
    lspcMutex->unlock();

    res.acknowledged = true;
    return true;
}

bool ROS_Service_SetPID(jetsoncar_interfaces::SetPID::Request &req, jetsoncar_interfaces::SetPID::Response &res, std::shared_ptr<std::timed_mutex> lspcMutex, std::shared_ptr<lspc::Socket *> lspcObj)
{
    res.acknowledged = false;

    if (!lspcMutex->try_lock_for(std::chrono::milliseconds(100))) return false; // could not get lock

    if (!(*lspcObj) || !(*lspcObj)->isOpen()) {
        lspcMutex->unlock();
        return false; // connection is not open
    }

    lspc::MessageTypesFromPC::SetPID_t payload;
    payload.P = req.P;
    payload.I = req.I;
    payload.D = req.D;
    std::vector<uint8_t> payloadPacked((uint8_t *)&payload, (uint8_t *)&payload+sizeof(payload)); // this method of "serializing" requires that PC runs Little Endian (which most PC processors do = Intel x86, AMD 64 etc.)
    (*lspcObj)->send(lspc::MessageTypesFromPC::SetPID, payloadPacked);

    lspcMutex->unlock();

    res.acknowledged = true;
    return true;
}

bool ROS_Service_SetRateLimits(jetsoncar_interfaces::SetRateLimits::Request &req, jetsoncar_interfaces::SetRateLimits::Response &res, std::shared_ptr<std::timed_mutex> lspcMutex, std::shared_ptr<lspc::Socket *> lspcObj)
{
    res.acknowledged = false;

    if (!lspcMutex->try_lock_for(std::chrono::milliseconds(100))) return false; // could not get lock

    if (!(*lspcObj) || !(*lspcObj)->isOpen()) {
        lspcMutex->unlock();
        return false; // connection is not open
    }

    lspc::MessageTypesFromPC::SetRateLimits_t payload;
    payload.max_acceleration = req.max_acceleration;
    payload.max_deceleration = req.max_deceleration;
    std::vector<uint8_t> payloadPacked((uint8_t *)&payload, (uint8_t *)&payload+sizeof(payload)); // this method of "serializing" requires that PC runs Little Endian (which most PC processors do = Intel x86, AMD 64 etc.)
    (*lspcObj)->send(lspc::MessageTypesFromPC::SetRateLimits, payloadPacked);

    lspcMutex->unlock();

    res.acknowledged = true;
    return true;
}

void ROS_ReconfigureCallback(jetsoncar_driver::ParametersConfig &config, uint32_t level, std::shared_ptr<std::timed_mutex> lspcMutex, std::shared_ptr<lspc::Socket *> lspcObj)
{
    ROS_DEBUG("Reconfigure Request");
    if (!reconfigureMutex.try_lock_for(std::chrono::milliseconds(10))) return; // could not get lock

    if (config.P != reconfigureConfig.P ||
        config.I != reconfigureConfig.I ||
        config.D != reconfigureConfig.D) {
        // Call the service to update the parameters
        jetsoncar_interfaces::SetPID::Request req;
        jetsoncar_interfaces::SetPID::Response res;
        req.P = config.P;
        req.I = config.I;
        req.D = config.D;
        if (ROS_Service_SetPID(req, res, lspcMutex, lspcObj))
        {
            if (res.acknowledged)
                ROS_INFO("PID parameters updated successfully");
            else
                ROS_WARN("PID parameters could not be updated");
        }
        else
        {
            ROS_ERROR("Failed to call service SetPID");
        }
    }

    if (config.max_acceleration != reconfigureConfig.max_acceleration ||
        config.max_deceleration != reconfigureConfig.max_deceleration) {
        // Call the service to update the parameters
        jetsoncar_interfaces::SetRateLimits::Request req;
        jetsoncar_interfaces::SetRateLimits::Response res;
        req.max_acceleration = config.max_acceleration;
        req.max_deceleration = config.max_deceleration;        
        if (ROS_Service_SetRateLimits(req, res, lspcMutex, lspcObj))
        {
            if (res.acknowledged)
                ROS_INFO("Rate limits updated successfully");
            else
                ROS_WARN("Rate limits could not be updated");
        }
        else
        {
            ROS_ERROR("Failed to call service SetRateLimits");
        }
    }    

    MATLAB_log = config.MATLAB_log;

    reconfigureConfig = config;
    reconfigureMutex.unlock();
    reconfigureServer->updateConfig(reconfigureConfig);
}
