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

#include "I2COut.hpp"

#include <px4_platform_common/sem.hpp>

I2COUT::I2COUT() : 
	px4::ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default), 
	_mixing_interface_esc(),
	_mixing_interface_servo()
{
	// Getting initial parameter values
	ScheduleOnInterval(100_ms);
	update_params();
}

I2COUT::~I2COUT()
{
	perf_free(_cycle_perf);
	perf_free(_interval_perf);
}

void I2COUT::Run()
{
	_count++;
	if (should_exit()) {
		ScheduleClear();
		_mixing_interface_esc.stop();
		_mixing_interface_servo.stop();

		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);
	perf_count(_interval_perf);

	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		update_params();
		
		_mixing_interface_esc.updateParams();
		_mixing_interface_servo.updateParams();
	}

	perf_end(_cycle_perf);
	_first_update_cycle = false;
}

int I2COUT::task_spawn(int argc, char *argv[])
{
	I2COUT *instance = new I2COUT();

	if (!instance) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	instance->_mixing_interface_esc.init();
	instance->_mixing_interface_servo.init();

	_object.store(instance);
	_task_id = task_id_is_work_queue;
	instance->ScheduleNow();
	return PX4_OK;
}

void I2COUT::update_params()
{
	_mixing_interface_esc.update_params();
	_mixing_interface_servo.update_params();
}

int I2COUT::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int I2COUT::print_status()
{
	perf_print_counter(_cycle_perf);
	perf_print_counter(_interval_perf);

	PX4_INFO_RAW("ESC outputs:\n");
	_mixing_interface_esc.print_status();

	PX4_INFO_RAW("Servo outputs:\n");
	_mixing_interface_servo.print_status();

	PX4_INFO_RAW("Count: %d\n", _count);

	return 0;
}

int I2COUT::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module is responsible for driving the output pins. For boards without a separate IO chip
(eg. Pixracer), it uses the main channels. On boards with an IO chip (eg. Pixhawk), it uses the AUX channels, and the
px4io driver is used for main ones.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("i2c_out", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}


extern "C" __EXPORT int i2c_out_main(int argc, char *argv[])
{
	return I2COUT::main(argc, argv);
}