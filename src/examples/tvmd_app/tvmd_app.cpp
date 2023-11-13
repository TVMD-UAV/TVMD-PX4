/****************************************************************************
 *
 *   Copyright (c) 2012-2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file tvmd_app.hpp
 * Minimal application example for PX4 autopilot
 *
 * @author Sciyen <sciyen.ycc@gmail.com>
 * Instructions: https://docs.px4.io/main/en/modules/hello_sky.html
 */

#include "tvmd_app.hpp"

// #include <gtest/gtest.h>
// #include <ControlAllocationCGI.hpp>


ActuatorTransportor::ActuatorTransportor() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{

}

bool ActuatorTransportor::init()
{
	// execute Run() on every sensor_accel publication
	if (!_actuator_motors_sub.registerCallback() || !_actuator_servos_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	// alternatively, Run on fixed interval
	// ScheduleOnInterval(5000_us); // 2000 us interval, 200 Hz rate

	return true;
}

/**
 * @brief Run the work queue task. Once new data arrive on the actuator_motors
 * topic, check if there is new data on the actuator_servos topic. If so, boardcast
 * the data through I2C.
 */
void ActuatorTransportor::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	// update armed state
	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.copy(&vehicle_status)) {
			_armed = vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
		}
	}

	if (_actuator_motors_sub.updated()) {
		if (_actuator_motors_sub.copy(&_actuator_motors)){
			// boardcasting
			_actuator_motors_updated = true;
		}
	}

	if (_actuator_servos_sub.updated()) {
		if (_actuator_servos_sub.copy(&_actuator_servos)){
			// boardcasting
			_actuator_servos_updated = true;
		}
	}

	if (_actuator_motors_updated && _actuator_servos_updated) {
		// Boardcasting motors and servos command through I2C

		PX4_INFO("Motors: %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f",
			(double)_actuator_motors.control[0], (double)_actuator_motors.control[1], (double)_actuator_motors.control[2], (double)_actuator_motors.control[3],
			(double)_actuator_motors.control[4], (double)_actuator_motors.control[5], (double)_actuator_motors.control[6], (double)_actuator_motors.control[7]);
		PX4_INFO("Servos: %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f",
			(double)_actuator_servos.control[0], (double)_actuator_servos.control[1], (double)_actuator_servos.control[2], (double)_actuator_servos.control[3],
			(double)_actuator_servos.control[4], (double)_actuator_servos.control[5], (double)_actuator_servos.control[6], (double)_actuator_servos.control[7]);
		_actuator_motors_updated = false;
		_actuator_servos_updated = false;
	}
}

int ActuatorTransportor::task_spawn(int argc, char *argv[])
{
	ActuatorTransportor *instance = new ActuatorTransportor();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int ActuatorTransportor::print_status()
{
	PX4_INFO("Running");
	return 0;
}

int ActuatorTransportor::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ActuatorTransportor::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
		### Description
		Minimal application example for PX4 autopilot
		)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("tvmd_app", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int tvmd_app_main(int argc, char *argv[])
{
	return ActuatorTransportor::main(argc, argv);
}
