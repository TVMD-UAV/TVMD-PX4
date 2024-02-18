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

#pragma once

#include <float.h>
#include <math.h>

#include <board_config.h>


#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <lib/perf/perf_counter.h>
// #include <px4_arch/io_timer.h>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>

#include <drivers/device/i2c.h>
#include <lib/parameters/param.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>

#define I2C_NUM_OUTPUT_INSTANCE 16

using namespace time_literals;

class I2CMixingInterfaceBase: public OutputModuleInterface
{
public:
	static constexpr int MAX_ACTUATORS = MixingOutput::MAX_ACTUATORS;

	I2CMixingInterfaceBase(const char *module_name, const char *param_prefix):
		OutputModuleInterface(module_name, px4::wq_configurations::hp_default),
		_mixing_output{param_prefix, MAX_ACTUATORS, *this, MixingOutput::SchedulingPolicy::Auto, true}
		{};

	~I2CMixingInterfaceBase()
	{
		perf_free(_cycle_perf);
		perf_free(_interval_perf);
	}

	MixingOutput &mixingOutput() { return _mixing_output; }

	bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated) override
	{
		return true;
	};

	bool init()
	{
		ScheduleNow();
		return true;
	};

	void stop()
	{
		_mixing_output.unregister();
		ScheduleClear();
	}

	void update()
	{
		_mixing_output.update();
		_mixing_output.updateSubscriptions(false);
	}

	void update_params()
	{
		uint32_t previously_set_functions = 0;

		for (size_t i = 0; i < MAX_ACTUATORS; i++) {
			previously_set_functions |= (uint32_t)_mixing_output.isFunctionSet(i) << i;
		}

		updateParams();

		// Automatically set PWM configuration when a channel is first assigned
		if (!_first_update_cycle) {
			for (size_t i = 0; i < MAX_ACTUATORS; i++) {
				if ((previously_set_functions & (1u << i)) == 0 && _mixing_output.functionParamHandle(i) != PARAM_INVALID) {
					int32_t output_function;

					if (param_get(_mixing_output.functionParamHandle(i), &output_function) == 0) {
						// Servos need PWM rate 50Hz and disramed value 1500us
						if (output_function >= (int)OutputFunction::Servo1
							&& output_function <= (int)OutputFunction::ServoMax) { // Function got set to a servo
							int32_t val = 1500;
							PX4_INFO("Setting channel %i disarmed to %i", (int) val, (int)i);
							param_set(_mixing_output.disarmedParamHandle(i), &val);
						}

						// Motors need a minimum value that idles the motor and have a deadzone at the top of the range
						if (output_function >= (int)OutputFunction::Motor1
							&& output_function <= (int)OutputFunction::MotorMax) { // Function got set to a motor
							int32_t val = 1100;
							PX4_INFO("Setting channel %i minimum to %i", (int) val, (int)i);
							param_set(_mixing_output.minParamHandle(i), &val);
							val = 1900;
							PX4_INFO("Setting channel %i maximum to %i", (int) val, (int)i);
							param_set(_mixing_output.maxParamHandle(i), &val);
						}
					}
				}
			}
		}
	}

	void print_status()
	{
		perf_print_counter(_cycle_perf);
		perf_print_counter(_interval_perf);
		_mixing_output.printStatus();
	}

private:
	friend class I2COUT;

	bool _first_update_cycle{true};

	void Run() override
	{
		perf_begin(_cycle_perf);
		perf_count(_interval_perf);
		_mixing_output.update();
		_mixing_output.updateSubscriptions(false);
		perf_end(_cycle_perf);
	};

	MixingOutput _mixing_output;

	perf_counter_t	_cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};
};

class I2CMixingInterfaceESC: public I2CMixingInterfaceBase
{
public:
	I2CMixingInterfaceESC():
		I2CMixingInterfaceBase(MODULE_NAME"_ESC", "I2C_EC")
		{};
};

class I2CMixingInterfaceServo: public I2CMixingInterfaceBase
{
public:
	I2CMixingInterfaceServo():
		I2CMixingInterfaceBase(MODULE_NAME"_SERVO", "I2C_SV")
		{};
};

class I2COUT final : public ModuleBase<I2COUT>, public px4::ScheduledWorkItem
{
public:
	I2COUT();
	~I2COUT() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:
	int _count{0};
	bool _first_update_cycle{true};
	void Run() override;

	void update_params();

	I2CMixingInterfaceESC _mixing_interface_esc;
	I2CMixingInterfaceServo _mixing_interface_servo;

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	perf_counter_t	_cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};
};
