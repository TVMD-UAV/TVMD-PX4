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

using namespace time_literals;

#define TRANSPORTER_BASEADDR 0x66

class ActuatorTransporter :
#if defined(__PX4_NUTTX)
public device::I2C, public I2CSPIDriver<ActuatorTransporter>
#else
public ModuleBase<ActuatorTransporter>, public px4::ScheduledWorkItem
#endif
{
public:
#if defined(__PX4_NUTTX)
	ActuatorTransporter(const I2CSPIDriverConfig &config);
#else
	ActuatorTransporter();
	~ActuatorTransporter() override = default;
#endif

#if defined(__PX4_NUTTX)
	int init() override;
	static void print_usage();
	void print_status() override;
	void RunImpl();
#else
	int init();
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	int print_status() override;
#endif


private:
	bool _armed{false};

	// regular subscription for additional data
	uORB::Subscription                 _vehicle_status_sub{ORB_ID(vehicle_status)};

	// subscription that schedules WorkItemExample when updated
	uORB::Subscription _actuator_motors_sub{ORB_ID(actuator_motors)};
	uORB::Subscription _actuator_servos_sub{ORB_ID(actuator_servos)};

	actuator_motors_s _actuator_motors;
	actuator_servos_s _actuator_servos;

	bool _actuator_motors_updated{false};
	bool _actuator_servos_updated{false};

#if defined(__PX4_NUTTX)
	int probe() override;
#else
	void Run() override;

	int probe();
#endif

	void start();

	bool check_subscription();

	void broadcast_data();
};

#if defined(__PX4_NUTTX)
ActuatorTransporter::ActuatorTransporter(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config)
#else
ActuatorTransporter::ActuatorTransporter() : ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
#endif
{
}

int ActuatorTransporter::init()
{
	int ret = PX4_OK;

#if defined(__PX4_NUTTX)
	ret = I2C::init();
#endif

	if (ret != PX4_OK) {
		return ret;
	}

	start();

	return PX4_OK;
}

int ActuatorTransporter::probe()
{
	return 0;
}

void ActuatorTransporter::start()
{
	ScheduleOnInterval(100_ms); // 100 Hz
}

#if defined(__PX4_NUTTX)
void ActuatorTransporter::RunImpl(){
#else
void ActuatorTransporter::Run() {
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}
#endif
	if (check_subscription()) {
		broadcast_data();
	}
}


bool ActuatorTransporter::check_subscription()
{
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
	return _actuator_motors_updated && _actuator_servos_updated;
}

void ActuatorTransporter::broadcast_data()
{
	// Boardcasting motors and servos command through I2C
#if defined(__PX4_NUTTX)
#else
	PX4_INFO("Motors: %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f",
		(double)_actuator_motors.control[0], (double)_actuator_motors.control[1], (double)_actuator_motors.control[2], (double)_actuator_motors.control[3],
		(double)_actuator_motors.control[4], (double)_actuator_motors.control[5], (double)_actuator_motors.control[6], (double)_actuator_motors.control[7]);
	PX4_INFO("Servos: %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f",
		(double)_actuator_servos.control[0], (double)_actuator_servos.control[1], (double)_actuator_servos.control[2], (double)_actuator_servos.control[3],
		(double)_actuator_servos.control[4], (double)_actuator_servos.control[5], (double)_actuator_servos.control[6], (double)_actuator_servos.control[7]);
#endif
	_actuator_motors_updated = false;
	_actuator_servos_updated = false;
}

#if defined(__PX4_NUTTX)
void ActuatorTransporter::print_status()
#else
int ActuatorTransporter::print_status()
#endif
{
#if defined(__PX4_NUTTX)
	I2CSPIDriverBase::print_status();
#else
	PX4_INFO("Running");
	return 0;
#endif
}

#if defined(__PX4_NUTTX)
void ActuatorTransporter::print_usage()
#else
int ActuatorTransporter::print_usage(const char *reason)
#endif
{
	PRINT_MODULE_USAGE_NAME("actuator_transporter", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("actuator_transporter");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
#if not defined(__PX4_NUTTX)
	return 0;
#endif
}

#if not defined(__PX4_NUTTX)
int ActuatorTransporter::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ActuatorTransporter::task_spawn(int argc, char *argv[])
{
	ActuatorTransporter *instance = new ActuatorTransporter();

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
#endif

extern "C" __EXPORT int actuator_transporter_i2c_main(int argc, char *argv[])
{
	#if defined(__PX4_NUTTX)
	using ThisDriver = ActuatorTransporter;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 400000;
	cli.i2c_address = TRANSPORTER_BASEADDR;

	const char *verb = cli.optArg();

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_DIST_DEVTYPE_LIGHTWARE_LASER);

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
	#else
	return ActuatorTransporter::main(argc, argv);
	#endif
}
