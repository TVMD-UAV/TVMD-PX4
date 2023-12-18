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
 * @file ActuatorEffectivenessVTOL_TVMD.hpp
 *
 * Actuator effectiveness for VTOL TVMD
 *
 * @author Yen-Cheng Chu <sciyen.ycc@gmail.com>
 */

#include "ActuatorEffectivenessVTOL_TVMD.hpp"

using namespace matrix;

ActuatorEffectivenessVTOL_TVMD::ActuatorEffectivenessVTOL_TVMD(ModuleParams *parent)
	: ModuleParams(parent)
{
	_param_handles.num_agents = param_find("CA_MD_COUNT");

	// Request module configuration param settings
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

		snprintf(buffer, sizeof(buffer), "CA_MD%u_X_GR", i);
		_param_handles.module_geometry[i].gear_ratio[0] = param_find(buffer);

		snprintf(buffer, sizeof(buffer), "CA_MD%u_Y_GR", i);
		_param_handles.module_geometry[i].gear_ratio[1] = param_find(buffer);

		for (int j = 0; j < 2; ++j) {
			// Aerial dynamics parameters
			snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_CT", 2*i+j);
			_param_handles.module_geometry[i].c_l[j] = param_find(buffer);

			snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_KM", 2*i+j);
			_param_handles.module_geometry[i].c_d[j] = param_find(buffer);

			// Servo
			snprintf(buffer, sizeof(buffer), "CA_SV_TL%u_MINA", 2*i+j);
			_param_handles.module_geometry[i].servo_min[j] = param_find(buffer);

			snprintf(buffer, sizeof(buffer), "CA_SV_TL%u_MAXA", 2*i+j);
			_param_handles.module_geometry[i].servo_max[j] = param_find(buffer);

			// slew
			snprintf(buffer, sizeof(buffer), "CA_R%u_SLEW", 2*i+j);
			_param_handles.module_geometry[i].motor_slew[j] = param_find(buffer);

			snprintf(buffer, sizeof(buffer), "CA_SV%u_SLEW", 2*i+j);
			_param_handles.module_geometry[i].servo_slew[j] = param_find(buffer);
		}
	}

	updateParams();
	// setFlightPhase(FlightPhase::HOVER_FLIGHT);
}

void ActuatorEffectivenessVTOL_TVMD::updateParams()
{
	// The minimum and maximum describe the corresponding angles of the actuator frame
	// no matter what gear ratio it is.
	// The slew constraint is applied directly on actuator space.
	// The gear ratio is used to calculate the actual rate limit of the actuator frame, which
	// will be used to calculate the local admissible force space of agents.
	ModuleParams::updateParams();

	int32_t num_agents = 0;

	if (param_get(_param_handles.num_agents, &num_agents) != 0) {
		PX4_ERR("param_get failed");
		return;
	}
	printf("Get MD_COUNT: %d\n", (int)num_agents);

	// Number of agent must be 3 ~ 4
	_geometry.num_agents = math::constrain((int)num_agents, 3, NUM_AGENTS_MAX);

	for (int i = 0; i < _geometry.num_agents; ++i) {
		PX4_INFO("TRYING TO GET %d", i);
		if (param_get(_param_handles.module_geometry[i].pos_x, &_geometry.module_geometry[i].position(0)) != 0) PX4_ERR("CAN NOT GET CV_MD_PX");
		if (param_get(_param_handles.module_geometry[i].pos_y, &_geometry.module_geometry[i].position(1)) != 0) PX4_ERR("CAN NOT GET CV_MD_PY");
		if (param_get(_param_handles.module_geometry[i].pos_z, &_geometry.module_geometry[i].position(2)) != 0) PX4_ERR("CAN NOT GET CV_MD_PZ");

		printf("Get module %d position: (%7.3f, %7.3f, %7.3f)\n", i,
			(double)_geometry.module_geometry[i].position(0),
			(double)_geometry.module_geometry[i].position(1),
			(double)_geometry.module_geometry[i].position(2));

		int32_t ax_psi;
		if (param_get(_param_handles.module_geometry[i].ax_psi, &ax_psi) != 0) PX4_ERR("CAN NOT GET CV_MD_AX");
		_geometry.module_geometry[i].ax_psi = ax_psi;


		for (int j = 0; j < 2; ++j) {
			if (param_get(_param_handles.module_geometry[i].gear_ratio[j], &_geometry.module_geometry[i].gear_ratio[j]) != 0) PX4_ERR("CAN NOT GET GEAR");

			if (param_get(_param_handles.module_geometry[i].motor_slew[j], &_geometry.module_geometry[i].motor_conf[j](0)) != 0) PX4_ERR("CAN NOT GET MOTOR SLEW");
			if (param_get(_param_handles.module_geometry[i].c_l[j], &_geometry.module_geometry[i].motor_conf[j](1)) != 0) PX4_ERR("CAN NOT GET CL");
			if (param_get(_param_handles.module_geometry[i].c_d[j], &_geometry.module_geometry[i].motor_conf[j](2)) != 0) PX4_ERR("CAN NOT GET CD");

			if (param_get(_param_handles.module_geometry[i].servo_slew[j], &_geometry.module_geometry[i].servo_conf[j](0)) != 0) PX4_ERR("CAN NOT GET SERVO SLEW");
			if (param_get(_param_handles.module_geometry[i].servo_min[j], &_geometry.module_geometry[i].servo_conf[j](1)) != 0) PX4_ERR("CAN NOT GET SERVO MIN");
			if (param_get(_param_handles.module_geometry[i].servo_max[j], &_geometry.module_geometry[i].servo_conf[j](2)) != 0) PX4_ERR("CAN NOT GET SERVI MAX");

			// motor rate: 1/s
			_geometry.module_geometry[i].motor_conf[j](0) = 1.0f / _geometry.module_geometry[i].motor_conf[j](0);

			// servo min
			const float servo_min = math::radians(_geometry.module_geometry[i].servo_conf[j](1));
			_geometry.module_geometry[i].servo_conf[j](1) = servo_min;
			// servo max
			const float servo_max = math::radians(_geometry.module_geometry[i].servo_conf[j](2));
			_geometry.module_geometry[i].servo_conf[j](2) = servo_max;
			// servo rate: rad/s
			// servo rate is amplified by the gear ratio
			_geometry.module_geometry[i].servo_conf[j](0) =
				_geometry.module_geometry[i].gear_ratio[j] * (servo_max - servo_min) / _geometry.module_geometry[i].servo_conf[j](0);

		}
	}
}

void ActuatorEffectivenessVTOL_TVMD::genRotationMatrixRz(matrix::Matrix3f &Rz, const float psi){
	Rz.setZero();
	Rz(1, 1) = Rz(0, 0) = std::cos(psi);
	Rz(1, 0) = std::sin(psi);
	Rz(0, 1) = -std::sin(psi);
	Rz(2, 2) = 1.0f;
}

bool
ActuatorEffectivenessVTOL_TVMD::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	// static_assert(_geometry.num_agents >= 3, "Parameters are not updated yet or the number of agent is invalid.");
	assert(_geometry.num_agents >= 3);

	configuration.selected_matrix = 0;
	// configuration.trim?
	// configuration.linearization_point?

	// Allocate actuators for motors and servos
	// As the allocation is non-linear, we use updateSetpoint() instead of the matrix
	for (int i = 0; i < 2 * _geometry.num_agents; ++i) {
		configuration.addActuator(ActuatorType::MOTORS, Vector3f{}, Vector3f{});
	}

	for (int i = 0; i < 2 * _geometry.num_agents; ++i) {
		configuration.addActuator(ActuatorType::SERVOS, Vector3f{}, Vector3f{});
	}


	// Calculate the effectiveness matrix manually according to module positions
	// For TVMD, the effectiveness matrix has size 6 x (3 x #agents)
	// i.e., if the number of agents is 4, the effectiveness matrix has size 6x12
	// After the matrix inverse is solved, a nonlinear transform will be invoked,
	// and all the 16 actuators will be infilled in allocate() in the control_allocation.
	{
		matrix::Matrix3f Rz;
		for (int i = 0; i < _geometry.num_agents; ++i ) {
			genRotationMatrixRz(Rz, M_PI_2_F * _geometry.module_geometry[i].ax_psi);

			// torques
			configuration.effectiveness_matrices[0].slice<3, 3>(0, 3*i) =
				_geometry.module_geometry[i].position.hat() * Rz;

			// thrusts
			configuration.effectiveness_matrices[0].slice<3, 3>(3, 3*i) = Rz;
		}
	}

	// calculate scales
	{
		// Setup scaling factor from a real one to a normalized one
		// motors: maximum thrust (N) -> 0 ~ 1
		// TODO: minimum thrust
		// matrix::Vector2f tftd;
		// const matrix::Vector2f u_prop(1, 1);
		// tf_mapping(i, tftd, u_prop);
		// _actuator_scale(get_motor_idx(i, j)) = (1.0f - ( 0.0f)) / (tftd(0) - 0.0f);
		// TODO: record maximum thrust
		// The scaling is dealt in tf_mapping and tf_inverse_mapping
		for (int i = 0; i < _geometry.num_agents; ++i) {
			for (int j = 0; j < 2; ++j) {
				const float servo_max = _geometry.module_geometry[i].servo_conf[j](2);
				const float servo_min = _geometry.module_geometry[i].servo_conf[j](1);
				const float gear_ratio = _geometry.module_geometry[i].gear_ratio[j];

				_actuator_scale(get_motor_idx(i, j)) = 1.0f;

				// servos: angle (rad) -> -1 ~ 1
				// servo_max is the angle of the servo measured at the maximum pwm value
				// servo_min is the angle of the servo measured at the minimum pwm value
				// The tilting angle limits are dealt in control allocation
				_actuator_scale(get_servo_idx(i, j)) = (1.0f - (-1.0f)) * gear_ratio / (servo_max - servo_min);
			}
		}

		// TODO: battery level compensation
	}
	// print scales
	#ifdef ACTUATOR_EFFECTIVENESS_DEBUGGER
	printf("Scales for actuators in ActEffectiveness: \n");
	for (size_t m=0; m<16; ++m) {
		printf("\t%7.4f", (double)_actuator_scale(m));
	}
	printf("\n");

	for (size_t m=0; m<6; ++m) {
		for (size_t n=0; n<16; ++n) {
			printf("\t%7.4f", (double)configuration.effectiveness_matrices[0](m, n));
		}
		printf("\n");
	}
	#endif // ACTUATOR_EFFECTIVENESS_DEBUGGER

	return true;
}

void ActuatorEffectivenessVTOL_TVMD::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
		int matrix_index, ActuatorVector &actuator_sp, const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
		const matrix::Vector<float, NUM_ACTUATORS> &actuator_max)
{

	#ifdef ACTUATOR_EFFECTIVENESS_DEBUGGER
	printf("Control:   (%7.3f, %7.3f, %7.3f, %7.3f, %7.3f, %7.3f)\n",
		(double)control_sp(0), (double)control_sp(1), (double)control_sp(2),
		(double)control_sp(3), (double)control_sp(4), (double)control_sp(5));
	#endif // ACTUATOR_EFFECTIVENESS_DEBUGGER

	// Nonlinear inverse transform
	for (int i = 0; i < _geometry.num_agents; ++i) {
		const matrix::Vector2f tftd = actuator_sp.slice<2, 1>(get_motor_idx(i, 0), 0);
		matrix::Vector2f u_prop;
		// tf_inverse_mapping(i, tftd, actuator_sp.slice<2, 1>(get_motor_idx(i, 0), 0));
		tf_inverse_mapping(i, tftd, u_prop);
		actuator_sp.slice<2, 1>(get_motor_idx(i, 0), 0) = u_prop;
		#ifdef ACTUATOR_EFFECTIVENESS_DEBUGGER
		printf("Agent %d: \n", i);
		printf("%7.3f, \t%7.3f -> %7.3f, \t%7.3f\n", (double)tftd(0), (double)tftd(1), (double)u_prop(0), (double)u_prop(1));
		#endif // ACTUATOR_EFFECTIVENESS_DEBUGGER
	}

	actuator_sp = actuator_sp.emult(_actuator_scale);
}

void ActuatorEffectivenessVTOL_TVMD::tf_inverse_mapping(const uint8_t module_id, const matrix::Vector2f &tftd, matrix::Vector2f &u_prop) const {

	const matrix::Vector2f tftd0(Tf0, Td0);

	const float m[2][2] = {{c_l, c_l}, {-c_d, c_d}};
	const matrix::SquareMatrix<float, 2> W(m);

	u_prop = inv(W) * (tftd - tftd0);
}

void ActuatorEffectivenessVTOL_TVMD::tf_mapping(const uint8_t module_id, matrix::Vector2f &tftd, const matrix::Vector2f &u_prop) const {
	// const auto c_l = _geometry.module_geometry[module_id].motor_conf[0](1);
	// const auto c_d = _geometry.module_geometry[module_id].motor_conf[0](2);

	const matrix::Vector2f tftd0(Tf0, Td0);

	const float m[2][2] = {{c_l, c_l}, {-c_d, c_d}};
	const matrix::SquareMatrix<float, 2> W(m);

	tftd = tftd0 + W * u_prop;
}


bool ActuatorEffectivenessVTOL_TVMD::throttleSpoolupFinished()
{
	vehicle_status_s vehicle_status;

	if (_vehicle_status_sub.update(&vehicle_status)) {
		_armed = vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
		_armed_time = vehicle_status.armed_time;
	}

	return _armed && hrt_elapsed_time(&_armed_time) > _param_spoolup_time * 1_s;
}
