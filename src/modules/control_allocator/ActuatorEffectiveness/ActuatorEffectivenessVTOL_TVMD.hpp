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

#pragma once

#include "ActuatorEffectiveness.hpp"
#include "ActuatorEffectivenessRotors.hpp"
#include "ActuatorEffectivenessTilts.hpp"

#include <px4_platform_common/module_params.h>

#include <uORB/topics/normalized_unsigned_setpoint.h>
#include <uORB/topics/tiltrotor_extra_controls.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/Subscription.hpp>

class ActuatorEffectivenessVTOL_TVMD : public ModuleParams, public ActuatorEffectiveness
{
public:
	static constexpr int NUM_AGENTS_MAX = 6;

	// region [Geometry Struct Definitions]
	struct ModuleGeometry {
		float pos_x;
		float pos_y;
		float pos_z;
		int32_t ax_psi;
	};

	struct Geometry {
		int num_agents;
		int rotor_count;
		ModuleGeometry module_geometry[NUM_AGENTS_MAX];
	};
	// endregion [Geometry Struct Definitions]

	ActuatorEffectivenessVTOL_TVMD(ModuleParams *parent);
	virtual ~ActuatorEffectivenessVTOL_TVMD() = default;

	bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) override;


	void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index,
			    ActuatorVector &actuator_sp, const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
			    const matrix::Vector<float, NUM_ACTUATORS> &actuator_max) override;

	const char *name() const override { return "VTOL TVMD"; }


protected:

private:

	void updateParams() override;

	// region [Geometry Parameter Handles Struct Definitions]
	struct ParamModuleGeometry {
		param_t pos_x;
		param_t pos_y;
		param_t pos_z;
		param_t ax_psi;
	};

	struct ParamHandles {
		param_t num_agents;
		param_t rotor_count;
		ParamModuleGeometry module_geometry[NUM_AGENTS_MAX];
	};
	// endregion [Geometry Parameter Handles Struct Definitions]

	ParamHandles _param_handles{};

	Geometry _geometry{};

	float _param_spoolup_time{1.f};

	// Tilt handling during motor spoolup: leave the tilts in their disarmed position unitil 1s after arming
	bool throttleSpoolupFinished();
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	bool _armed{false};
	uint64_t _armed_time{0};
};
