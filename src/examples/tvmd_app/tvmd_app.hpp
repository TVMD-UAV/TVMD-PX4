#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
// #include <unistd.h>
// #include <stdio.h>
// #include <poll.h>
// #include <string.h>
// #include <math.h>

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

// uORB
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_servos.h>

// I2C
#include <drivers/device/i2c.h>

class ActuatorTransportor : public ModuleBase<ActuatorTransportor>, public px4::ScheduledWorkItem
{
public:
	ActuatorTransportor();
	~ActuatorTransportor() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase */
	int print_status() override;

	bool init();

private:
	void Run() override;

	bool _armed{false};

	// regular subscription for additional data
	uORB::Subscription                 _vehicle_status_sub{ORB_ID(vehicle_status)};

	// subscription that schedules WorkItemExample when updated
	uORB::SubscriptionCallbackWorkItem _actuator_motors_sub{this, ORB_ID(actuator_motors)};
	uORB::SubscriptionCallbackWorkItem _actuator_servos_sub{this, ORB_ID(actuator_servos)};

	actuator_motors_s _actuator_motors;
	actuator_servos_s _actuator_servos;

	bool _actuator_motors_updated{false};
	bool _actuator_servos_updated{false};
};
