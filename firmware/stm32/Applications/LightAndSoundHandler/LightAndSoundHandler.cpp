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
 
#include "LightAndSoundHandler.h"
#include "cmsis_os.h"

LightAndSoundHandler::LightAndSoundHandler(PWM& red_, PWM& green_, PWM& blue_, IO& buzzer_) : _TaskHandle(0), _isRunning(false), _shouldStop(false),
	red(red_), green(green_), blue(blue_), buzzer(buzzer_)
{	
	Start();
}

LightAndSoundHandler::~LightAndSoundHandler()
{
	_shouldStop = true;
	while (_isRunning) osDelay(10);
}

int LightAndSoundHandler::Start()
{
	if (_isRunning) return 0; // task already running
	_shouldStop = false;
	return xTaskCreate( LightAndSoundHandler::Thread, (char *)"Light and Sound", THREAD_STACK_SIZE, (void*) this, THREAD_PRIORITY, &_TaskHandle);
}

int LightAndSoundHandler::Stop(uint32_t timeout)
{
	if (!_isRunning) return 0; // task not running

	_shouldStop = true;

	uint32_t timeout_millis = timeout;
	while (_isRunning && timeout_millis > 0) {
		osDelay(1);
		timeout_millis--;
	}
	if (_isRunning) return -1; // timeout trying to stop task

	return 1;
}

int LightAndSoundHandler::Restart(uint32_t timeout)
{
	if (!_isRunning) return 0; // task not running
	int errCode = Stop(timeout);
	if (errCode != 1) return errCode;
	return Start();
}

void LightAndSoundHandler::Thread(void * pvParameters)
{
	LightAndSoundHandler * task = (LightAndSoundHandler *)pvParameters;
	task->_isRunning = true;

	/* Load initialized objects for easier access */
	PWM& red = task->red;
	PWM& green = task->green;
	PWM& blue = task->blue;
	IO& buzzer = task->buzzer;

	/* Main loop */
	float RGB[3];
	bool up = true;
	float value = 0;

	while (!task->_shouldStop)
	{
		if (up)
			value += 0.01;
		else
			value -= 0.01;

		if (value <= 0 || value >= 1)
			up = !up;

		task->jetColor(value, 0, 1, RGB);

		red.Set(1.0 - RGB[0]);
		green.Set(1.0 - RGB[1]);
		blue.Set(1.0 - RGB[2]);

		osDelay(50);
	}

	task->_isRunning = false;
	task->_TaskHandle = 0;
	vTaskDelete(NULL); // delete/stop this current task
}

void LightAndSoundHandler::jetColor(const float v, const float vmin, const float vmax, float RGB[3])
{
	float value = v;
	float dv = vmax - vmin;

	if (value < vmin) value = vmin;
	if (value > vmax) value = vmax;

	RGB[0] = 1.0;
	RGB[1] = 1.0;
	RGB[2] = 1.0;

	if (value < (vmin + 0.25 * dv)) {
		RGB[0] = 0;
		RGB[1] = 4 * (value - vmin) / dv;
	} else if (value < (vmin + 0.5 * dv)) {
		RGB[0] = 0;
		RGB[2] = 1 + 4 * (vmin + 0.25 * dv - value) / dv;
	} else if (value < (vmin + 0.75 * dv)) {
		RGB[0] = 4 * (value - vmin - 0.5 * dv) / dv;
		RGB[2] = 0;
	} else {
		RGB[1] = 1 + 4 * (vmin + 0.75 * dv - value) / dv;
		RGB[2] = 0;
	}
}

