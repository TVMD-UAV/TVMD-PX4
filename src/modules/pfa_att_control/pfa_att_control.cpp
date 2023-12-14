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

#include "pfa_att_control.hpp"


/**
 * PFA attitude control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int pfa_att_control_main(int argc, char *argv[]);


PFAAttitudeControl::PFAAttitudeControl():
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_param_handles.num_agents = param_find("CA_MD_COUNT");

	for (int i = 0; i < NUM_AGENTS_MAX; ++i) {
		char buffer[17];

		// Positions
		snprintf(buffer, sizeof(buffer), "CA_MD%u_PX", i);
		_param_handles.module_geometry[i].pos_x = param_find(buffer);

		snprintf(buffer, sizeof(buffer), "CA_MD%u_PY", i);
		_param_handles.module_geometry[i].pos_y = param_find(buffer);

		snprintf(buffer, sizeof(buffer), "CA_MD%u_PZ", i);
		_param_handles.module_geometry[i].pos_z = param_find(buffer);

		snprintf(buffer, sizeof(buffer), "CA_MD%u_AZ", i);
		_param_handles.module_geometry[i].ax_psi = param_find(buffer);
	}
}

PFAAttitudeControl::~PFAAttitudeControl()
{
	perf_free(_loop_perf);
}

bool PFAAttitudeControl::init()
{
	if (!_vehicle_attitude_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void PFAAttitudeControl::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();

		int32_t num_agents = 0;
		if (param_get(_param_handles.num_agents, &num_agents) == 0) {
			// _num_agents = math::constrain((uint8_t)num_agents, 3, 4);
			_num_agents = num_agents;
			calc_team_inertia(_team_inertia);
		} else {
			PX4_ERR("Failed to get param: num_agents");
		}
	}
}

/**
 * Calculate the inertia matrix of the whole team.
 */
void PFAAttitudeControl::calc_team_inertia(Matrix3f &output)
{
	const Matrix3f agent_inertia = matrix::diag(
		Vector3f(_param_vehicle_agent_ixx.get(), _param_vehicle_agent_iyy.get(), _param_vehicle_agent_izz.get()));

	const Matrix3f nav_inertia = matrix::diag(
		Vector3f(_param_vehicle_nav_ixx.get(), _param_vehicle_nav_iyy.get(), _param_vehicle_nav_izz.get()));

	// agent mass
	const float agent_mass = _param_vehicle_agent_mass.get();

	output = nav_inertia;

	// Calculate the inertia matrix of the whole team

	for (int i = 0; i < _num_agents; i++) {
		matrix::Vector3f::Matrix31 d; //
		if (param_get(_param_handles.module_geometry[i].pos_x, &d(0, 0)) != 0 ||
				param_get(_param_handles.module_geometry[i].pos_y, &d(1, 0)) != 0 ||
				param_get(_param_handles.module_geometry[i].pos_z, &d(2, 0)) != 0) {
				PX4_ERR("Failed to get %d-agent position", i);
		}

		// Calculate the inertia matrix of the agent according to the parallel axis theorem
		// I_S = I_R - M [d][d], where d \in R^{3x1} is the position vector between the agent and the CoM of the team
		//     = I_R + M (||d||^2 * I_3 - d * d^T)
		Matrix3f E3;
		E3.identity();
		const float d_norm_square = (d.transpose() * d)(0, 0);
		const Matrix3f agent_inertia_parallel =
			agent_inertia + agent_mass * (
				d_norm_square * E3 - d * d.transpose());

		output += agent_inertia_parallel;
	}

	PX4_INFO("Team inertia matrix:");
	for(int i=0; i<3; i++){
		for(int j=0; j<3; j++){
			printf("%7.4f\t", (double)output(i, j));
		}
		printf("\n");
	}
}

/**
 * Ensure the setpoints are valid and then publish them to the
 * corresponding topic.
 */
void PFAAttitudeControl::constrain_actuator_commands(
	const Vector3f & torques, const Vector3f & thrusts)
{
	const Vector3f constrained_torques = matrix::constrain(torques, -1.0f, 1.0f);
	const Vector3f constrained_thrusts = matrix::constrain(thrusts, -1.0f, 1.0f);
	constrained_torques.copyTo(_vehicle_torque_setpoint.xyz);
	constrained_thrusts.copyTo(_vehicle_thrust_setpoint.xyz);

	_vehicle_thrust_setpoint.timestamp = hrt_absolute_time();
	_vehicle_thrust_setpoint.timestamp_sample = 0.f;
	_vehicle_thrust_setpoint_pub.publish(_vehicle_thrust_setpoint);

	_vehicle_torque_setpoint.timestamp = hrt_absolute_time();
	_vehicle_torque_setpoint.timestamp_sample = 0.f;
	_vehicle_torque_setpoint_pub.publish(_vehicle_torque_setpoint);
}


/** Partially Fully Actuated Geometric Controller
 */
void PFAAttitudeControl::control_attitude_geo(const vehicle_attitude_s &attitude,
		const vehicle_attitude_setpoint_s &attitude_setpoint, const vehicle_angular_velocity_s &angular_velocity,
		const vehicle_rates_setpoint_s &rates_setpoint)
{
	Eulerf euler_angles(matrix::Quatf(attitude.q));

	// Retrieving state variables
	const Quatf q_att(attitude.q);
	const Matrix3f rot_att =  matrix::Dcm<float>(q_att);
	const Vector3f omega = _checkAllFinite(Vector3f(angular_velocity.xyz));

	// Retrieving setpoints
	// const Vector3f euler_offset = Vector3f(-0.3f, 0.3f, 0.0f);
	const Vector3f euler_offset = Vector3f(0.0f, 0.0f, 0.0f);
	const Vector3f att_euler_des = _checkAllFinite(Vector3f(attitude_setpoint.roll_body, attitude_setpoint.pitch_body, attitude_setpoint.yaw_body)) + euler_offset;
	const Dcmf rot_des = Eulerf(att_euler_des);

	const Vector3f omega_offset = Vector3f(-0.1f, 0.1f, -2.5f);
	// const Vector3f omega_offset = Vector3f(0.0f, 0.0f, 0.0f);
	const Vector3f omega_des = _checkAllFinite(Vector3f(rates_setpoint.roll, rates_setpoint.pitch, rates_setpoint.yaw)) + omega_offset;

	// Retrieve gains
	const Vector3f Kr = Vector3f(_param_roll_p.get(), _param_pitch_p.get(), _param_yaw_p.get());
	const Vector3f Kw = Vector3f(_param_roll_d.get(), _param_pitch_d.get(), _param_yaw_d.get());


	// Compute attitude error
	// e_R = 0.5 * (R_d.T * R - R.T * R_d)
	const Dcmf e_R = 0.5f * (rot_des.transpose() * rot_att - rot_att.transpose() * rot_des);
	const Vector3f e_R_vec = e_R.vee();
	const Vector3f err_omega = omega - omega_des;


	// Compute torque control
	// max torque should be set to the same value as the one in control allocation
	// TODO: add inertia matrix
	const Vector3f torques = - _team_inertia * (Kr.emult(e_R_vec) + Kw.emult(err_omega)) / _param_vehicle_max_torque.get();
	const Vector3f thrusts = Vector3f(attitude_setpoint.thrust_body);

	const Vector3f torque_offset = Vector3f(0.0f, 0.0f, 0.0f);

	constrain_actuator_commands(torques + torque_offset, thrusts);
}

void PFAAttitudeControl::Run()
{
	if (should_exit()) {
		_vehicle_attitude_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	/* check vehicle control mode for changes to publication state */
	_vcontrol_mode_sub.update(&_vcontrol_mode);

	/* update parameters from storage */
	parameters_update();

	vehicle_attitude_s attitude;

	/* only run controller if attitude changed */
	if (_vehicle_attitude_sub.update(&attitude)) {
		vehicle_angular_velocity_s angular_velocity {};
		_angular_velocity_sub.copy(&angular_velocity);

		if (_vcontrol_mode.flag_control_attitude_enabled
		    && _vcontrol_mode.flag_control_rates_enabled) {

			_vehicle_attitude_setpoint_sub.update(&_attitude_setpoint);
			_vehicle_rates_setpoint_sub.update(&_rates_setpoint);

			control_attitude_geo(attitude, _attitude_setpoint, angular_velocity, _rates_setpoint);
		}
	}

	perf_end(_loop_perf);
}

int PFAAttitudeControl::task_spawn(int argc, char *argv[])
{
	PFAAttitudeControl *instance = new PFAAttitudeControl();

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

int PFAAttitudeControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}


int PFAAttitudeControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Controls the attitude of an partially fully actuated (PFA) system.

Publishes `vehicle_thrust_setpont` and `vehicle_torque_setpoint` messages at a constant 250Hz.

### Implementation
Currently, this implementation supports only a few modes:

 * Position mode

### Examples
CLI usage example:
$ pfa_att_control start
$ pfa_att_control status
$ pfa_att_control stop

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("pfa_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start")
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int pfa_att_control_main(int argc, char *argv[])
{
	return PFAAttitudeControl::main(argc, argv);
}
