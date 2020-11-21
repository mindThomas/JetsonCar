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

#include "MainTask.h"
#include "cmsis_os.h"
#include "Priorities.h"
#include "ProcessorInit.h"
#include "Parameters.h"

/* Include Periphiral drivers */
//#include "ADC.h"
//#include "EEPROM.h"
#include "Encoder.h"
//#include "I2C.h"
#include "InputCapture.h"
#include "IO.h"
#include "PWM.h"
#include "RCReceiver.h"
#include "Servo.h"
//#include "SMBus.h"
//#include "SPI.h"
#include "Timer.h"
//#include "UART.h"
#include "USBCDC.h"
//#include "Watchdog.h"

/* Include Device drivers */
//#include "Battery.h"
//#include "ESCON.h"
//#include "IMU.h"
//#include "LCD.h"
#include "LSPC.hpp"
//#include "MPU9250.h"
//#include "MTI200.h"
//#include "QuadratureKnob.h"

/* Include Module libraries */
//#include "LQR.h"
//#include "SlidingMode.h"
#include "Debug.h"
#include "CPULoad.h"
//#include "COMEKF.h"
//#include "MadgwickAHRS.h"
//#include "QEKF.h"
//#include "VelocityEKF.h"
//#include "Parameters.h"
//#include "PowerManagement.h"
//#include "Joystick.h"
//#include "ModuleTemplate.h"
#include "SpeedController.h"

/* Include Application-layer libraries */
#include "LightAndSoundHandler.h"
//#include "Communication.h"
//#include "FrontPanel.h"
//#include "PathFollowingController.h"
//#include "ApplicationTemplate.h"

/* Include Misc libraries */
#include "RateLimiter.hpp"

/* Miscellaneous includes */
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cstring>
#include <string>
#include <vector>
#include <cmath>

void jetColor(const float v, const float vmin, const float vmax, float RGB[3]);
void Reboot_Callback(void * param, const std::vector<uint8_t>& payload);
void EnterBootloader_Callback(void * param, const std::vector<uint8_t>& payload);
void SetPID_Callback(void * param, const std::vector<uint8_t>& payload);
void Setpoint_Callback(void * param, const std::vector<uint8_t>& payload);

int32_t encoderFront = 0;
int32_t encoderBack = 0;

float throttle_in = 0;
float steering_in = 0;

typedef struct {
	SemaphoreHandle_t semaphore = NULL;
	uint32_t timestamp = 0;
	enum {
		MANUAL,
		AUTO
	} mode = MANUAL;
	lspc::MessageTypesFromPC::Setpoint_t setpoint = {.angular_velocity = 0, .steering = 0};
} ControllerSetpoint_t;

#define MOTOR_SYSID	 false

void MainTask(void * pvParameters)
{
	/* Use this task to:
	 * - Create objects for each module
	 *     (OBS! It is very important that objects created with "new"
	 *      only happens within a thread due to the usage of the FreeRTOS managed heap)
	 * - Link any modules together if necessary
	 * - Create message exchange queues and/or semaphore
	 * - (Create) and start threads related to modules
	 *
	 * Basically anything related to starting the system should happen in this thread and NOT in the main() function !!!
	 */

	/* Initialize communication */
	USBCDC * usb = new USBCDC(USBCDC_TRANSMITTER_PRIORITY, "JetsonCar Serial Port", 0x5544);
	LSPC * lspcUSB = new LSPC(usb, LSPC_RECEIVER_PRIORITY, LSPC_TRANSMITTER_PRIORITY); // very important to use "new", otherwise the object gets placed on the stack which does not have enough memory!
	Debug::AssignDebugCOM(lspcUSB); // pair debug module with configured LSPC module to enable "Debug::print" functionality
	CPULoad * cpuLoad = new CPULoad(*lspcUSB, CPULOAD_PRIORITY);

	/* Initialize hardware periphirals */
	RCReceiver * rc_throttle = new RCReceiver(InputCapture::TIMER4, InputCapture::CH3);
	RCReceiver * rc_steering = new RCReceiver(InputCapture::TIMER4, InputCapture::CH4);
	Servo * throttle = new Servo(PWM::TIMER1, PWM::CH1);
	Servo * servo_front = new Servo(PWM::TIMER9, PWM::CH1, -1.0f, 1.0f, 1.2f, 1.8f);
	Encoder * encoder_front = new Encoder(Encoder::TIMER5);
	Encoder * encoder_back = new Encoder(Encoder::TIMER2, true);
	IO * buzzer = new IO(GPIOA, GPIO_PIN_4);
	PWM * red = new PWM(PWM::TIMER8, PWM::CH1, 1000, 100);
	PWM * green = new PWM(PWM::TIMER8, PWM::CH2);
	PWM * blue = new PWM(PWM::TIMER8, PWM::CH3);

	/* Register general (system wide) LSPC callbacks */
	lspcUSB->registerCallback(lspc::MessageTypesFromPC::Reboot, &Reboot_Callback);
	lspcUSB->registerCallback(lspc::MessageTypesFromPC::EnterBootloader, &EnterBootloader_Callback);

	/* Initialize microseconds timer */
	Timer * microsTimer = new Timer(Timer::TIMER11, 1000000); // create a 1 MHz counting timer used for micros() timing

	/* Create setpoint semaphores */
	ControllerSetpoint_t ControllerSetpoint;
	ControllerSetpoint.semaphore = xSemaphoreCreateBinary();
	if (ControllerSetpoint.semaphore == NULL) {
		ERROR("Could not create reference semaphore");
		return;
	}
	vQueueAddToRegistry(ControllerSetpoint.semaphore, "Reference");
	xSemaphoreGive( ControllerSetpoint.semaphore ); // give the semaphore the first time
	lspcUSB->registerCallback(lspc::MessageTypesFromPC::Setpoint, &Setpoint_Callback, (void *)&ControllerSetpoint);

	/* Initialize modules */
#if MOTOR_SYSID
	throttle->Disable();
	osDelay(100);
	throttle->Set(0);
	osDelay(100);
	throttle->Disable();
	osDelay(100);
	throttle->Set(0);
	osDelay(1000);
#else
	struct {
		float a = 21.9663f;
		float b = 5.9356f;
		float c = 0.2154f;
	} feedforward_params;
	SpeedController * controller = new SpeedController(lspcUSB, *microsTimer, *throttle, *encoder_back, ENCODER_TICKS_REV * GEAR_RATIO,
			[feedforward_params](float setpoint)->float{
				return copysignf(1.f, setpoint) * (feedforward_params.c - logf(1.f - fabs(setpoint) / feedforward_params.a) / feedforward_params.b);
			},
			SPEED_CONTROLLER_PRIORITY);  // 12 ticks pr. encoder/motor rev,  gearing ratio of 40  =  480 ticks pr. rev
	RateLimiter rateLimit(0.05f, 15.0f, 25.0f);
	lspcUSB->registerCallback(lspc::MessageTypesFromPC::SetPID, &SetPID_Callback, (void *)controller);

	controller->Enable();
	controller->SetSpeed(5);
	servo_front->Set(0.0f);
#endif

	/******* APPLICATION LAYERS *******/
	//LightAndSoundHandler * LightAndSound = new LightAndSoundHandler(*red, *green, *blue, *buzzer);
	//if (!LightAndSound) ERROR("Could not initialize light and sound handler");


	bool EnableTest = false;
	float SpeedValue = 0.0f;
	bool Up_nDown = true;
	uint16_t stepWait = 0;
	ControllerSetpoint_t ControllerSetpointLocal;
	bool RC_Active = false;

	xSemaphoreTake( ControllerSetpoint.semaphore, ( TickType_t ) portMAX_DELAY); // lock for reading
	ControllerSetpointLocal.setpoint = ControllerSetpoint.setpoint;
	xSemaphoreGive( ControllerSetpoint.semaphore ); // give semaphore back

	while (1)
	{
		encoderFront = encoder_front->Get();
		encoderBack = encoder_back->Get();

		if (xSemaphoreTake( ControllerSetpoint.semaphore, ( TickType_t ) 1) == pdTRUE) { // lock for reading
			if (ControllerSetpoint.timestamp > 0 && xTaskGetTickCount() < (ControllerSetpoint.timestamp + 500)) { // 500 ms
				ControllerSetpointLocal.setpoint = ControllerSetpoint.setpoint;
				ControllerSetpointLocal.mode = ControllerSetpoint.AUTO;
				ControllerSetpointLocal.timestamp = ControllerSetpoint.timestamp;
			}
			xSemaphoreGive( ControllerSetpoint.semaphore ); // give semaphore back
		}

		if (rc_throttle->isActive())
		{
			throttle_in = rc_throttle->Get(true);
			steering_in = rc_steering->Get(true);

			if (fabsf(throttle_in) < 0.05f && fabsf(steering_in) < 0.05f) {
				RC_Active = true;
			}

			if (RC_Active && (ControllerSetpointLocal.mode == ControllerSetpoint.MANUAL
			    || fabsf(throttle_in) > 0.05f || xTaskGetTickCount() > (ControllerSetpointLocal.timestamp + 200)))  // reference older than 200 ms or we need manual takeover
			{
				ControllerSetpointLocal.mode = ControllerSetpoint.MANUAL;
				ControllerSetpointLocal.timestamp = xTaskGetTickCount();
				ControllerSetpointLocal.setpoint.angular_velocity = 20*throttle_in;
				ControllerSetpointLocal.setpoint.steering = steering_in;
			}
		} else { // RC inactive
			RC_Active = false;
			if (xTaskGetTickCount() > (ControllerSetpointLocal.timestamp + 100))  // reference timeout
			{
				ControllerSetpointLocal.mode = ControllerSetpoint.MANUAL;
				ControllerSetpointLocal.timestamp = xTaskGetTickCount();
				ControllerSetpointLocal.setpoint.angular_velocity = 0;
				ControllerSetpointLocal.setpoint.steering = 0;
			}
		}

		if (ControllerSetpointLocal.mode == ControllerSetpoint.MANUAL) {
			// Red
			red->Set(0.0);
			green->Set(1.0);
			blue->Set(1.0);
		} else { // Auto
			// Green
			red->Set(1.0);
			green->Set(0.0);
			blue->Set(1.0);
		}

#if MOTOR_SYSID
		if (steering_in < -0.7)
			EnableTest = true;
		else if (steering_in > 0.7) {
			EnableTest = false;
			SpeedValue = 0;
		}

		if (EnableTest) {
			stepWait++;

			if ((stepWait % 40) == 0) { // 40x50 = 2000 ms
				stepWait = 0;

				if (Up_nDown) {
					SpeedValue += 0.05f;
					if (SpeedValue >= 0.79f)
						Up_nDown = false;
				}
				else
				{
					SpeedValue -= 0.05f;
					if (SpeedValue <= -0.79f)
						Up_nDown = true;
				}
			}
		} else {
			SpeedValue = throttle_in;
		}

		throttle->Set(SpeedValue);
		servo_front->Set(steering_in);
#else
		controller->SetSpeed(rateLimit(ControllerSetpointLocal.setpoint.angular_velocity));
		servo_front->Set(ControllerSetpointLocal.setpoint.steering);
#endif

		lspc::MessageTypesToPC::Sensors_t msg;
		msg.timestamp = microsTimer->GetTime();
		msg.wheel_angles.front = 2.f * M_PI * (float)encoderFront / (ENCODER_TICKS_REV*GEAR_RATIO);
		msg.wheel_angles.rear = 2.f * M_PI * (float)encoderBack / (ENCODER_TICKS_REV*GEAR_RATIO);
		msg.wheel_angular_velocities.front = controller->SpeedFiltered; // OBS! This should be fixed such that we have both front and rear velocities
		msg.wheel_angular_velocities.rear = controller->SpeedFiltered;
		msg.rc.throttle = throttle_in;
		msg.rc.steering = steering_in;
#if MOTOR_SYSID
		msg.motor_outputs.throttle = float(int16_t(1024*SpeedValue)) / 1024.f; // quantize according to how the PWM output will be quantized
#else
		msg.motor_outputs.throttle = float(int16_t(1024*controller->MotorOutput)) / 1024.f;
#endif
		msg.motor_outputs.steering = float(int16_t(1024*steering_in)) / 1024.f;
		msg.setpoints.angular_velocity = controller->SpeedSetpoint;
		msg.setpoints.steering = steering_in;
		lspcUSB->TransmitAsync(lspc::MessageTypesToPC::Sensors, (uint8_t *)&msg, sizeof(msg));

		osDelay(50);
		//vTaskSuspend(NULL); // suspend this task
	}
}

void Reboot_Callback(void * param, const std::vector<uint8_t>& payload)
{
	// ToDo: Need to check for magic key
	NVIC_SystemReset();
}

void EnterBootloader_Callback(void * param, const std::vector<uint8_t>& payload)
{
	// ToDo: Need to check for magic key
	USBD_Stop(&USBCDC::hUsbDeviceFS);
	USBD_DeInit(&USBCDC::hUsbDeviceFS);
	Enter_DFU_Bootloader();
}

void SetPID_Callback(void * param, const std::vector<uint8_t>& payload)
{
	SpeedController * controller = (SpeedController *)param;
	if (!controller) return;

	volatile lspc::MessageTypesFromPC::SetPID_t msg;
	if (payload.size() != sizeof(msg)) return;
	memcpy((uint8_t *)&msg, payload.data(), sizeof(msg));

	controller->SetPID(msg.P, msg.I, msg.D);
}

void Setpoint_Callback(void * param, const std::vector<uint8_t>& payload)
{
	ControllerSetpoint_t * ControllerSetpoint = (ControllerSetpoint_t *)param;
	if (!ControllerSetpoint) return;

	xSemaphoreTake( ControllerSetpoint->semaphore, ( TickType_t ) portMAX_DELAY); // lock for updating

	volatile lspc::MessageTypesFromPC::Setpoint_t msg;
	if (payload.size() != sizeof(msg)) return;
	memcpy((uint8_t *)&msg, payload.data(), sizeof(msg));

	ControllerSetpoint->timestamp = xTaskGetTickCount();
	ControllerSetpoint->setpoint.angular_velocity = msg.angular_velocity;
	ControllerSetpoint->setpoint.steering = msg.steering;

	xSemaphoreGive( ControllerSetpoint->semaphore ); // give semaphore back
}
