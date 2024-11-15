/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 *
 * This module is a modification of the uuv attitide control module and is designed for
 * partially fully actuated systems.
 *
 * All the acknowledgments and credits for the fw wing app are reported in those files.
 *
 * @author Yen-Cheng Chu <sciyen.ycc@gmail.com>
 */

#include <float.h>

#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>
#include <lib/pid/pid.h>
#include <matrix/math.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/attitude_planner_meta_data.h>
#include <uORB/uORB.h>

#include "pfa_att_planner.hpp"

using matrix::Eulerf;
using matrix::Quatf;
using matrix::Matrix3f;
using matrix::Vector3f;
using matrix::Dcmf;

using uORB::SubscriptionData;

using namespace time_literals;

class PFAAttitudeControl: public ModuleBase<PFAAttitudeControl>, public ModuleParams, public px4::WorkItem
{
public:
	PFAAttitudeControl();
	~PFAAttitudeControl();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

protected:
	PFAAttPlanner planner{this};

private:
	void publishTorqueSetpoint(const hrt_abstime &timestamp_sample);
	void publishThrustSetpoint(const hrt_abstime &timestamp_sample);

	uORB::Publication<vehicle_thrust_setpoint_s>	_vehicle_thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Publication<vehicle_torque_setpoint_s>	_vehicle_torque_setpoint_pub{ORB_ID(vehicle_torque_setpoint)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};	/**< vehicle attitude setpoint */
	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
	uORB::Subscription _vehicle_rates_setpoint_sub{ORB_ID(vehicle_rates_setpoint)}; 	/**< vehicle bodyrates setpoint subscriber */
	uORB::Subscription _angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};		/**< vehicle angular velocity subscription */
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};	/**< notification of manual control updates */
	uORB::Subscription _vcontrol_mode_sub{ORB_ID(vehicle_control_mode)};			/**< vehicle status subscription */

	uORB::SubscriptionCallbackWorkItem _vehicle_attitude_sub{this, ORB_ID(vehicle_attitude)};

	uORB::Publication<attitude_planner_meta_data_s> _attitude_planner_meta_data_pub{ORB_ID(attitude_planner_meta_data)};	/**< vehicle attitude setpoint */

	vehicle_thrust_setpoint_s _vehicle_thrust_setpoint{};
	vehicle_torque_setpoint_s _vehicle_torque_setpoint{};
	manual_control_setpoint_s _manual_control_setpoint{};
	trajectory_setpoint_s _trajectory_setpoint{};
	vehicle_attitude_setpoint_s _attitude_setpoint{};
	vehicle_rates_setpoint_s _rates_setpoint{};
	vehicle_control_mode_s _vcontrol_mode{};

	perf_counter_t	_loop_perf;

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::PFA_ROLL_P>) _param_roll_p,
		(ParamFloat<px4::params::PFA_ROLL_D>) _param_roll_d,
		(ParamFloat<px4::params::PFA_PITCH_P>) _param_pitch_p,
		(ParamFloat<px4::params::PFA_PITCH_D>) _param_pitch_d,
		(ParamFloat<px4::params::PFA_YAW_P>) _param_yaw_p,
		(ParamFloat<px4::params::PFA_YAW_D>) _param_yaw_d,
		(ParamFloat<px4::params::PFA_MAX_TOR>) _param_vehicle_max_torque,
		(ParamInt<px4::params::PFA_EN_ATT_PLAN>) _param_enable_attitude_planner,

		// module inertia
		(ParamFloat<px4::params::VEH_AGENT_IXX>) _param_vehicle_agent_ixx,
		(ParamFloat<px4::params::VEH_AGENT_IYY>) _param_vehicle_agent_iyy,
		(ParamFloat<px4::params::VEH_AGENT_IZZ>) _param_vehicle_agent_izz,

		(ParamFloat<px4::params::VEH_NAV_IXX>) _param_vehicle_nav_ixx,
		(ParamFloat<px4::params::VEH_NAV_IYY>) _param_vehicle_nav_iyy,
		(ParamFloat<px4::params::VEH_NAV_IZZ>) _param_vehicle_nav_izz,

		(ParamFloat<px4::params::VEH_AGENT_MASS>) _param_vehicle_agent_mass,

		(ParamFloat<px4::params::PFA_MAX_THR>)  _param_vehicle_max_thrust
	)

	void Run() override;

	void parameters_update(bool force = false);

	void control_attitude_geo(const vehicle_attitude_s &attitude, const vehicle_attitude_setpoint_s &attitude_setpoint,
				  const vehicle_angular_velocity_s &angular_velocity, const vehicle_rates_setpoint_s &rates_setpoint);
	void constrain_actuator_commands(const Vector3f & torques, const Vector3f & thrusts);

	static inline const Vector3f _checkAllFinite(const Vector3f & vec) {
		if (!vec.isAllFinite()) return Vector3f(0.0f, 0.0f, 0.0f);
		else 			return vec;
	}

	static const uint8_t NUM_AGENTS_MAX{4};
	struct ParamModuleGeometry {
		param_t pos_x;
		param_t pos_y;
		param_t pos_z;
		param_t ax_psi;
	};

	struct ParamHandles {
		param_t num_agents;
		ParamModuleGeometry module_geometry[NUM_AGENTS_MAX];
	} _param_handles{};

	uint8_t _num_agents{0};

	Matrix3f _team_inertia{};
	void calc_team_inertia(Matrix3f &output);
};
