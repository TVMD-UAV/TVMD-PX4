/****************************************************************************
 *
 *   Copyright (c) 2013-2021 PX4 Development Team. All rights reserved.
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
 * @file actuator_transporter.cpp
 *
 * @author Yen-Cheng Cue <sciyen.ycc@gmail.com>
 *
 * Driver for the Lightware lidar range finder series.
 * Default I2C address 0x66 is used.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>

#include <px4_platform_common/module.h>
#include <drivers/device/i2c.h>
#include <lib/parameters/param.h>
#include <drivers/drv_hrt.h>

// uORB
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/actuator_outputs.h>

using namespace time_literals;

// The address of agent flight controllers
#define TRANSPORTER_BASEADDR 0x66
#define TRANSPORTER_ACTUATOR_OUTPUTS

class ActuatorTransporter :
public device::I2C, public I2CSPIDriver<ActuatorTransporter>
{
public:
	static const uint8_t context_num{2};
	typedef float ControlSignal_t;

	union Instruction {
		enum ControlTypes {UNKNOWN=0, MOTORS, SERVOS};
		struct {
			uint8_t armed;
			uint8_t type;
			ControlSignal_t control[8];
		} data;
		uint8_t raw[sizeof(data)];
	};


	ActuatorTransporter(const I2CSPIDriverConfig &config);

	~ActuatorTransporter()
	{
		for (int i=0; i<context_num; i++) {
			perf_free(_output_context[i].perf);
		}
		perf_free(_interval_perf_i2c);
	};

	int init() override;

	static void print_usage();

	void print_status() override;

	void RunImpl();

private:
	bool _armed{false};

	// regular subscription for additional data
	uORB::Subscription                 _vehicle_status_sub{ORB_ID(vehicle_status)};

	// subscription that schedules WorkItemExample when updated
	struct output_context_s {
		uORB::Subscription sub;
		actuator_outputs_s data;
		hrt_abstime update_time;
		Instruction::ControlTypes type;
		perf_counter_t perf;
		bool updated;
	} _output_context[context_num];

	output_context_s * _context_ptr{&_output_context[0]};

	// TODO: check the instance number of actuator_outputs
	uint32_t _minimum_update_interval_us{1000000 / 50}; // 50 Hz

	int probe() override;

	void start();

	bool check_subscription();

	uint8_t _current_context_idx{0};
	void context_switch();

	void broadcast_data(Instruction::ControlTypes type, const ControlSignal_t * const control, size_t size);

	perf_counter_t	_interval_perf_i2c{perf_alloc(PC_INTERVAL, MODULE_NAME": I2C interval")};
};

ActuatorTransporter::ActuatorTransporter(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config)
{
	// TODO: check the instance number of actuator_outputs
	// The instance number 0 and 1 are published by pwm_main and pwm_aux
	// The instance number 2 and 3 are occupied by I2COut (the order is
	// determined by the order of the mixing interface initialization, please
	// refer to the constructor of I2COut in src/drivers/i2c_out/I2COut.cpp)
	_output_context[0].sub = uORB::Subscription(ORB_ID(actuator_outputs), 2);
	_output_context[0].update_time = 0;
	_output_context[0].type = Instruction::MOTORS;
	_output_context[0].perf = perf_alloc(PC_INTERVAL, MODULE_NAME": MOTORS");

	_output_context[1].sub = uORB::Subscription(ORB_ID(actuator_outputs), 3);
	_output_context[1].update_time = 0;
	_output_context[1].type = Instruction::SERVOS;
	_output_context[1].perf = perf_alloc(PC_INTERVAL, MODULE_NAME": SERVOS");
}

int ActuatorTransporter::init()
{
	int ret = PX4_OK;

	ret = I2C::init();

	if (ret != PX4_OK) {
		return ret;
	}

	start();

	return PX4_OK;
}

/**
 * @brief probe the I2C bus to look for the device
 *
 * @return 0 if the device is found, -errno otherwise
 */
int ActuatorTransporter::probe()
{
	// TODO: check if devices are connected
	return 0;
}

/**
 * @brief This function will be called in init()
 */
void ActuatorTransporter::start()
{
	ScheduleOnInterval(10_ms); // 100 Hz
}

/**
 * @brief This function will be called periodically from Run(),
 * which is scheduled by start(). The interface is provided by
 * the I2CSPIDriver class, which is a subclass of px4::ScheduleWorkItem.
 */
void ActuatorTransporter::RunImpl(){
	// update armed state
	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.copy(&vehicle_status)) {
			_armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
		}
	}


	// To avoid the update rate of the same topic is too high, we use two contexts
	// to store the data. Once the first context is updated, the priority of the
	// two contexts will be switched.
	if (!check_subscription()) {
		// try the second priority topic
		context_switch();
		if (!check_subscription()) {
			// Both contexts are not updated, switch back to the original one
			context_switch();
		}
	}
}

/**
 * @brief Switch to next context
 *
 */
void ActuatorTransporter::context_switch()
{
	_current_context_idx = (_current_context_idx + 1) % context_num;
	_context_ptr = &_output_context[_current_context_idx];
}

/**
 * @brief This checks if there is new data on the current context.
 * If there is new data, it will be broadcasted through I2C, and the context will
 * be switched to the next one.
 * @return true if the data is updated
 */
bool ActuatorTransporter::check_subscription()
{
	if(_context_ptr->sub.updated() && _context_ptr->sub.copy(&_context_ptr->data)) {
		const hrt_abstime update_time = _context_ptr->data.timestamp;
		if (update_time >= _context_ptr->update_time + _minimum_update_interval_us) {
			// boardcasting
			broadcast_data(_context_ptr->type, _context_ptr->data.output, sizeof(_context_ptr->data.output));
			perf_count(_context_ptr->perf);
			_context_ptr->updated = true;
			_context_ptr->update_time = update_time;

			// swticth to next context, the next context become the first priority
			context_switch();
			return true;
		}
	}
	return false;
}

void ActuatorTransporter::broadcast_data(Instruction::ControlTypes type, const ControlSignal_t * const control, size_t size)
{
	// Boardcasting motors and servos command through I2C
	Instruction inst;
	inst.data.armed = _armed;
	inst.data.type = type;
	const size_t copy_size = sizeof(inst.data.control) < size ? sizeof(inst.data.control) : size;
	memcpy(inst.data.control, control, copy_size);
	int ret = transfer((uint8_t*)&inst.raw, sizeof(inst), nullptr, 0);
	if (ret == PX4_OK) {
		perf_count(_interval_perf_i2c);
	}
	else if (ret != PX4_OK) {
		PX4_ERR("I2C transfer failed");
	}
}

void ActuatorTransporter::print_status()
{
	I2CSPIDriverBase::print_status();

	for (int i=0; i<context_num; i++) {
		perf_print_counter(_output_context[i].perf);
	}

	perf_print_counter(_interval_perf_i2c);

	PX4_INFO_RAW("\nTopic status:");
	for (int i=0; i<context_num; i++) {
		PX4_INFO_RAW("context[%d]: %s\n", i, _output_context[i].sub.valid() ? "updated" : "no data");
	}

	for (int i=0; i<10; i++) {
		if (orb_exists(ORB_ID(actuator_outputs), i) == PX4_OK) {
			PX4_INFO_RAW("actuator_outputs[%d] exists\n", i);
		}
	}
}

void ActuatorTransporter::print_usage()
{
	PRINT_MODULE_USAGE_NAME("actuator_transporter", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("actuator_transporter");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(TRANSPORTER_BASEADDR);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" __EXPORT int actuator_transporter_i2c_main(int argc, char *argv[])
{
	using ThisDriver = ActuatorTransporter;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 400000;
	cli.i2c_address = TRANSPORTER_BASEADDR;
	cli.bus_option = I2CSPIBusOption::I2CExternal;
	cli.keep_running = true;

	PX4_INFO("transporter main running");

	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_TRANS_DEVTYPE_TRANSPORTER);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
