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
 * This module is a modification of the uuv position control module and is designed for
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
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_constraints.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/uORB.h>

// #include "MulticopterPositionControl.hpp"
#include "mc_pos_control/Takeoff/Takeoff.hpp"

using matrix::Eulerf;
using matrix::Quatf;
using matrix::Matrix3f;
using matrix::Vector3f;
using matrix::Dcmf;

using uORB::SubscriptionData;

using namespace time_literals;

class PFAPOSControl: public ModuleBase<PFAPOSControl>, public ModuleParams, public px4::WorkItem
{
public:
	PFAPOSControl();
	~PFAPOSControl();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

protected:
	/** state machine and ramp to bring the vehicle off the
	 * ground without jumps
	 */
	TakeoffHandling _takeoff;

	hrt_abstime _time_stamp_last_loop{0};		/**< time stamp of last loop iteration */
	hrt_abstime _time_position_control_enabled{0};
	void _update_position_control_enable_time();

	float _speed_up_max{-5.0f};
	float _speed_dn_max{2.0f};
	float _speed_xy_max{1.0f};

	const float _thrust_max{-1.0f};
	const float _thrust_min{-0.01f};
	const float _takeoff_hover_height{-2.0f};
	float _thrust_xy_max{0.3f};
	float _thrust_up_max{0.0f};

	uORB::Subscription _vehicle_constraints_sub{ORB_ID(vehicle_constraints)};
	vehicle_constraints_s _vehicle_constraints {
		.timestamp = 0,
		.speed_up = NAN,
		.speed_down = NAN,
		.want_takeoff = false,
	};

private:
	uORB::PublicationData<takeoff_status_s>              _takeoff_status_pub{ORB_ID(takeoff_status)};
	uORB::Publication<vehicle_attitude_setpoint_s> _att_sp_pub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Publication<vehicle_local_position_setpoint_s> _local_pos_sp_pub{ORB_ID(vehicle_local_position_setpoint)};	/**< vehicle local position setpoint publication */

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _vcontrol_mode_sub{ORB_ID(vehicle_control_mode)};

	uORB::SubscriptionCallbackWorkItem _vehicle_local_position_sub{this, ORB_ID(vehicle_local_position)};

	vehicle_attitude_s _vehicle_attitude{};
	trajectory_setpoint_s _trajectory_setpoint{};
	manual_control_setpoint_s _manual_control_setpoint{};
	vehicle_control_mode_s _vcontrol_mode{};

	perf_counter_t	_loop_perf;

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::PFA_GAIN_X_P>) _param_pose_gain_x,
		(ParamFloat<px4::params::PFA_GAIN_Y_P>) _param_pose_gain_y,
		(ParamFloat<px4::params::PFA_GAIN_Z_P>) _param_pose_gain_z,
		(ParamFloat<px4::params::PFA_GAIN_X_D>) _param_pose_gain_d_x,
		(ParamFloat<px4::params::PFA_GAIN_Y_D>) _param_pose_gain_d_y,
		(ParamFloat<px4::params::PFA_GAIN_Z_D>) _param_pose_gain_d_z,

		(ParamFloat<px4::params::PFA_VEH_MASS>) _param_vehicle_mass,
		(ParamFloat<px4::params::PFA_MAX_THR>)  _param_vehicle_max_thrust,

		(ParamFloat<px4::params::COM_SPOOLUP_TIME>) _param_com_spoolup_time, /**< time to let motors spool up after arming */
		(ParamFloat<px4::params::MPC_TKO_RAMP_T>)   _param_mpc_tko_ramp_t,   /**< time constant for smooth takeoff ramp */
		(ParamFloat<px4::params::MPC_Z_VEL_P_ACC>)  _param_mpc_z_vel_p_acc,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_UP>) _param_mpc_z_vel_max_up,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_DN>) _param_mpc_z_vel_max_dn
	)

	// DEFINE_PARAMETERS_CUSTOM_PARENT(MulticopterPositionControl,
	// DEFINE_PARAMETERS(
	// )


	void Run() override;
	/**
	 * Update our local parameter cache.
	 */
	void parameters_update(bool force = false);

	/**
	 * Control Attitude
	 */
	void publish_attitude_setpoint(const Vector3f &thrust_sp,
			const Vector3f &euler_attitude_des);

	void pose_controller_6dof(const trajectory_setpoint_s &traj_des,
			const Vector3f &euler_attitude_des,
			const vehicle_attitude_s &vehicle_attitude,
			const vehicle_local_position_s &vlocal_pos);

	void stabilization_controller_6dof(const trajectory_setpoint_s &traj_des,
			const Vector3f &euler_attitude_des,
			const vehicle_attitude_s &vehicle_attitude,
			const vehicle_local_position_s &vlocal_pos);

	static inline const Vector3f _checkAllFinite(const Vector3f & vec) {
		if (!vec.isAllFinite()) return Vector3f(0.0f, 0.0f, 0.0f);
		else 			return vec;
	}
};
