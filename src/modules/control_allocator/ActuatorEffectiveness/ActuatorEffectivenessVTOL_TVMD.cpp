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
	_param_handles.rotor_count = param_find("CA_ROTOR_COUNT");

	// Request module configuration param settings
	for (int i = 0; i < NUM_AGENTS_MAX; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_MD%u_PX", i);
		_param_handles.module_geometry[i].pos_x = param_find(buffer);

		snprintf(buffer, sizeof(buffer), "CA_MD%u_PY", i);
		_param_handles.module_geometry[i].pos_y = param_find(buffer);

		snprintf(buffer, sizeof(buffer), "CA_MD%u_PZ", i);
		_param_handles.module_geometry[i].pos_z = param_find(buffer);

		snprintf(buffer, sizeof(buffer), "CA_MD%u_AZ", i);
		_param_handles.module_geometry[i].ax_psi = param_find(buffer);
	}

	updateParams();
	// setFlightPhase(FlightPhase::HOVER_FLIGHT);
}

void ActuatorEffectivenessVTOL_TVMD::updateParams()
{
	ModuleParams::updateParams();

	int32_t num_agents = 0;
	int32_t rotor_count = 0;

	if (param_get(_param_handles.rotor_count, &num_agents) != 0) {
		PX4_ERR("param_get failed");
		return;
	}

	if (param_get(_param_handles.num_agents, &rotor_count) != 0) {
		PX4_ERR("param_get failed");
		return;
	}

	// Number of agent must be 3 ~ 6
	_geometry.num_agents = math::constrain((int)num_agents, 3, NUM_AGENTS_MAX);

	for (int i = 0; i < _geometry.num_agents; ++i) {
		PX4_INFO("TRYING TO GET %d", i);
		if (param_get(_param_handles.module_geometry[i].pos_x, &_geometry.module_geometry[i].pos_x) != 0) PX4_ERR("CAN NOT GET CV_MD_PX");
		if (param_get(_param_handles.module_geometry[i].pos_y, &_geometry.module_geometry[i].pos_y) != 0) PX4_ERR("CAN NOT GET CV_MD_PY");
		if (param_get(_param_handles.module_geometry[i].pos_z, &_geometry.module_geometry[i].pos_z) != 0) PX4_ERR("CAN NOT GET CV_MD_PZ");
		if (param_get(_param_handles.module_geometry[i].ax_psi, &_geometry.module_geometry[i].ax_psi) != 0) PX4_ERR("CAN NOT GET CV_MD_AX");
	}
}

bool
ActuatorEffectivenessVTOL_TVMD::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	// // As the allocation is non-linear, we use updateSetpoint() instead of the matrix
	for (int i = 0; i < 2; ++i) {
		configuration.addActuator(ActuatorType::MOTORS, Vector3f{}, Vector3f{});
	}


	for (int i = 0; i < 2; ++i) {
		configuration.addActuator(ActuatorType::SERVOS, Vector3f{}, Vector3f{});
	}

	return true;

}


void ActuatorEffectivenessVTOL_TVMD::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
		int matrix_index, ActuatorVector &actuator_sp, const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
		const matrix::Vector<float, NUM_ACTUATORS> &actuator_max)
{
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
