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

#include "pfa_pos_control.hpp"



/**
 * PFA pos_controller app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int pfa_pos_control_main(int argc, char *argv[]);


PFAPOSControl::PFAPOSControl():
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_takeoff_status_pub.advertise();
}

PFAPOSControl::~PFAPOSControl()
{
	perf_free(_loop_perf);
}

bool PFAPOSControl::init()
{
	if (!_vehicle_local_position_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void PFAPOSControl::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();

		_takeoff.setSpoolupTime(_param_com_spoolup_time.get());
		_takeoff.setTakeoffRampTime(_param_mpc_tko_ramp_t.get());
		// The original definition: initial_v = -g / velociry_p_gain,
		// Here, we let the initial thrust be 0.1f
		_takeoff.generateInitialRampValue(10.0f * CONSTANTS_ONE_G);
	}
}

/**
 * Record the time when position control is enabled, which is used to
 * schedule the takeoff ramp.
 */
void PFAPOSControl::_update_position_control_enable_time()
{
	if (_vcontrol_mode_sub.updated()) {
		const bool previous_vcontrol_mode_enabled = _vcontrol_mode.flag_multicopter_position_control_enabled;
		if (_vcontrol_mode_sub.update(&_vcontrol_mode)) {
			if (!previous_vcontrol_mode_enabled && _vcontrol_mode.flag_multicopter_position_control_enabled) {
				_time_position_control_enabled = _vcontrol_mode.timestamp;
			}
		}
	}
}


/**
 * Publish attitude setpoint.
 */
void PFAPOSControl::publish_attitude_setpoint(
	const Vector3f &thrust_sp, const Vector3f &euler_attitude_des)
{
	//watch if inputs are not to high
	vehicle_attitude_setpoint_s vehicle_attitude_setpoint = {};
	vehicle_attitude_setpoint.timestamp = hrt_absolute_time();

	vehicle_attitude_setpoint.roll_body = euler_attitude_des(0);
	vehicle_attitude_setpoint.pitch_body = euler_attitude_des(1);
	vehicle_attitude_setpoint.yaw_body = euler_attitude_des(2);

	const Vector3f output_thrust = _checkAllFinite(thrust_sp);
	output_thrust.copyTo(vehicle_attitude_setpoint.thrust_body);
	_att_sp_pub.publish(vehicle_attitude_setpoint);
}


/**
 * Calculate the desired thrust from the given trajectory setpoint.
 * If any entry of the setpoint is NaN, the corresponding control input
 * will be neglected.
 */
void PFAPOSControl::pose_controller_6dof(
		const trajectory_setpoint_s &traj_des, const Vector3f &euler_attitude_des,
		const vehicle_attitude_s &vehicle_attitude, const vehicle_local_position_s &vlocal_pos)
{
	// Retrieve gains
	const Vector3f Kp = Vector3f(_param_pose_gain_x.get(), _param_pose_gain_y.get(), _param_pose_gain_z.get());
	const Vector3f Kv = Vector3f(_param_pose_gain_d_x.get(), _param_pose_gain_d_y.get(), _param_pose_gain_d_z.get());

	// Retrieve current states
	// Quaternion rotation from the FRD body frame to the NED earth frame
	const Quatf q_att(vehicle_attitude.q);
	const Vector3f pos = Vector3f(vlocal_pos.x, vlocal_pos.y, vlocal_pos.z);
	const Vector3f vel = Vector3f(vlocal_pos.vx, vlocal_pos.vy, vlocal_pos.vz);

	// Calculate constrained velocity setpoint
	// TODO: the position setpoint should be constrained as the velocity setpoint
	// is saturated.
	const Vector3f vel_des = _checkAllFinite(Vector3f(traj_des.velocity));
	const Vector3f limited_vel_des = matrix::constrain(vel_des,
		Vector3f(-_speed_xy_max, -_speed_xy_max, _speed_up_max),
		Vector3f(_speed_xy_max, _speed_xy_max, _speed_dn_max));

	// Calculate errors
	const Vector3f err_pos = _checkAllFinite(Vector3f(traj_des.position) - pos);
	const Vector3f err_vel = _checkAllFinite(limited_vel_des - vel);
	const Vector3f err_acc = _checkAllFinite(Vector3f(traj_des.acceleration));

	// NED frame (north east down)
	const Vector3f resist_gravity = Vector3f(0, 0, -CONSTANTS_ONE_G);

	// Calcuate the control input
	const Vector3f control_acc = err_acc + resist_gravity + Kv.emult(err_vel) + Kp.emult(err_pos);
	const Vector3f control_force = control_acc * _param_vehicle_mass.get();
	const Vector3f control_normalized_force = control_force / _param_vehicle_max_thrust.get();

	// Rotate the thrust from global to body frame
	const Vector3f rotated_input = q_att.rotateVectorInverse(control_normalized_force);
	// const Vector3f rotated_input = control_normalized_force;

	// Limit thrust vector and check for saturation
	// NED frame (north east down)
	const Vector3f limited_thrust_des = matrix::constrain(rotated_input,
		Vector3f(-_thrust_xy_max, -_thrust_xy_max, _thrust_up_max),
		Vector3f(_thrust_xy_max, _thrust_xy_max, _thrust_min));

	publish_attitude_setpoint(limited_thrust_des, euler_attitude_des);

	vehicle_local_position_setpoint_s local_pos_sp{};
	local_pos_sp.timestamp = hrt_absolute_time();
	local_pos_sp.x = err_pos(0);
	local_pos_sp.y = err_pos(1);
	local_pos_sp.z = err_pos(2);
	local_pos_sp.vx = limited_vel_des(0);
	local_pos_sp.vy = limited_vel_des(1);
	local_pos_sp.vz = limited_vel_des(2);
	control_acc.copyTo(local_pos_sp.acceleration);
	control_normalized_force.copyTo(local_pos_sp.thrust);

	_local_pos_sp_pub.publish(local_pos_sp);
}


/**
 * Calculate the desired thrust that hovers the vehicle at the current
 * position.
 */
void PFAPOSControl::stabilization_controller_6dof(
		const trajectory_setpoint_s &traj_des, const Vector3f &euler_attitude_des,
		const vehicle_attitude_s &vehicle_attitude, const vehicle_local_position_s &vlocal_pos)
{
	trajectory_setpoint_s stab_traj_des = traj_des;
	stab_traj_des.position[0] = 0;
	stab_traj_des.position[1] = 0;
	stab_traj_des.position[2] = vlocal_pos.z;
	stab_traj_des.velocity[0] = 0;
	stab_traj_des.velocity[1] = 0;
	stab_traj_des.velocity[2] = 0;
	stab_traj_des.acceleration[0] = 0;
	stab_traj_des.acceleration[1] = 0;
	stab_traj_des.acceleration[2] = 2.5f;	// -g + 4.5 ~ -5.0

	pose_controller_6dof(stab_traj_des, euler_attitude_des, vehicle_attitude, vlocal_pos);
}

void PFAPOSControl::Run()
{
	if (should_exit()) {
		_vehicle_local_position_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	/* check vehicle control mode for changes to publication state */
	_vcontrol_mode_sub.update(&_vcontrol_mode);


	/* update parameters from storage */
	parameters_update();

	//vehicle_attitude_s attitude;
	vehicle_local_position_s vlocal_pos;

	/* only run controller if local_pos changed */
	if (_vehicle_local_position_sub.update(&vlocal_pos)) {
		// calculate dt
		const float dt =
			math::constrain(((vlocal_pos.timestamp_sample - _time_stamp_last_loop) * 1e-6f), 0.002f, 0.04f);
		_time_stamp_last_loop = vlocal_pos.timestamp_sample;

		_vehicle_attitude_sub.update(&_vehicle_attitude);
		_trajectory_setpoint_sub.update(&_trajectory_setpoint);
		_manual_control_setpoint_sub.update(&_manual_control_setpoint);

		_update_position_control_enable_time();

		if (_vcontrol_mode.flag_control_manual_enabled) {
			// manual mode: direct velocity control in z-axis
			// if (_manual_control_setpoint_sub.update(&_manual_control_setpoint)) {
			const float roll = _manual_control_setpoint.roll * M_DEG_TO_RAD_F * 90.0f;
			const float pitch = _manual_control_setpoint.pitch * M_DEG_TO_RAD_F * 90.0f;
			const float yaw = _manual_control_setpoint.yaw;

			// throttle normalize from -1~1 to 0~1
			const float level = (_manual_control_setpoint.throttle - (-1.0f)) / (1.0f - (-1.0f));

			// if throttle > 0.5, control is dominated by velocity command
			const float vel_z = - ((level < 0.5f) ? 0.0f : 2 * (level - 0.5f));

			// if throttle < 0.5, control is dominated by acceleration command (to resist gravity)
			// to avoid immediate rise of thrust setpoints
			const float acc_z = - CONSTANTS_ONE_G * ((level < 0.5f) ? (_manual_control_setpoint.throttle) : 0.0f);

			trajectory_setpoint_s traj_des{};
			traj_des.position[0] = NAN;
			traj_des.position[1] = NAN;
			traj_des.position[2] = NAN;
			traj_des.velocity[0] = 0;
			traj_des.velocity[1] = 0;
			traj_des.velocity[2] = vel_z;
			traj_des.acceleration[0] = 0;
			traj_des.acceleration[1] = 0;
			traj_des.acceleration[2] = acc_z;

			_thrust_up_max = -1.0f;
			_thrust_xy_max = -0.3f * _thrust_up_max;

			const Vector3f attitude_des = Vector3f(roll, pitch, yaw);

			pose_controller_6dof(
				traj_des, attitude_des, _vehicle_attitude, vlocal_pos);
			// }
		} else if ((_vcontrol_mode.flag_multicopter_position_control_enabled
				|| _vcontrol_mode.flag_control_offboard_enabled)
			&& (_trajectory_setpoint.timestamp >= _time_position_control_enabled)) {
			// position mode / altitude mode

			_vehicle_constraints_sub.update(&_vehicle_constraints);

			// TODO: the land process is not handled yet
			// Update takeoff state machine
			_takeoff.updateTakeoffState(
				_vcontrol_mode.flag_armed, false,
				((_param_takeoff_bypass.get() > 0) ? true : _vehicle_constraints.want_takeoff),
				_thrust_max, false, vlocal_pos.timestamp_sample);

			// Return the maximum thrust according to the takeoff state machine
			_thrust_up_max = _takeoff.updateRamp(dt, _thrust_max);
			// const float speed_down = PX4_ISFINITE(_vehicle_constraints.speed_down) ? _vehicle_constraints.speed_down :

			const bool flying     = (_takeoff.getTakeoffState() >= TakeoffState::rampup);
			const bool ramping_up = (_takeoff.getTakeoffState() == TakeoffState::rampup);
			// const bool flying_but_ground_contact = (flying && _vehicle_land_detected.ground_contact);

			// The flying state including the rampup phase
			if (flying) {
				// const Vector3f attitude_des = Vector3f(0.0f, 0.0f, _trajectory_setpoint.yaw);
				const Vector3f attitude_des = Vector3f(_manual_control_setpoint.roll, _manual_control_setpoint.pitch, _manual_control_setpoint.yaw);

				if (ramping_up) {
					// Reset position setpoint to current xy-position at the hovering height
					stabilization_controller_6dof(
						_trajectory_setpoint, attitude_des, _vehicle_attitude, vlocal_pos);
				} else {
					pose_controller_6dof(
						_trajectory_setpoint, attitude_des, _vehicle_attitude, vlocal_pos);
				}
			}
		} else {
			// An update is necessary here because otherwise the takeoff state
			// doesn't get skipped with non-altitude-controlled modes
			_takeoff.updateTakeoffState(_vcontrol_mode.flag_armed, false, false,
				_thrust_max, true, vlocal_pos.timestamp_sample);
		}
	}

	// Publish takeoff status
	const uint8_t takeoff_state = static_cast<uint8_t>(_takeoff.getTakeoffState());

	if (takeoff_state != _takeoff_status_pub.get().takeoff_state) {
		_takeoff_status_pub.get().takeoff_state = takeoff_state;
		// _takeoff_status_pub.get().tilt_limit = _tilt_limit_slew_rate.getState();
		_takeoff_status_pub.get().timestamp = hrt_absolute_time();
		_takeoff_status_pub.update();
	}

	/* Only publish if any of the proper modes are enabled */
	if (_vcontrol_mode.flag_control_manual_enabled ||
	    _vcontrol_mode.flag_control_attitude_enabled) {
	}

	perf_end(_loop_perf);
}

int PFAPOSControl::task_spawn(int argc, char *argv[])
{
	PFAPOSControl *instance = new PFAPOSControl();

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

int PFAPOSControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}


int PFAPOSControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Controls the attitude of an partially fully actuated (PFA).
Publishes `attitude_setpoint` messages.
### Implementation
Currently, this implementation supports only a few modes:
 * Position mode with takeoff
 * Manual mode
### Examples
CLI usage example:
$ pfa_pos_control start
$ pfa_pos_control status
$ pfa_pos_control stop
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("pfa_pos_control", "controller");
    PRINT_MODULE_USAGE_COMMAND("start")
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

int pfa_pos_control_main(int argc, char *argv[])
{
    return PFAPOSControl::main(argc, argv);
}
