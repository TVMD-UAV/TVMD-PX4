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

#include <px4_platform_common/module_params.h>

#include <uORB/topics/normalized_unsigned_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/Subscription.hpp>

using namespace time_literals;

#define ACTUATOR_EFFECTIVENESS_DEBUGGER

class ActuatorEffectivenessVTOL_TVMD : public ModuleParams, public ActuatorEffectiveness
{
public:
	static constexpr int NUM_AGENTS_MAX = 4;

	// TODO: The bias terms are needed to be determined
	// static constexpr float Tf0 = -2.9717;
	// static constexpr float Td0 =  0.0040;
	static constexpr float c_l = 10.2645f;
	static constexpr float c_d =  0.2132f;
	static constexpr float Tf0 =  0.0000f;
	static constexpr float Td0 =  0.0000f;

	// region [Geometry Struct Definitions]
	struct ModuleGeometry {
		matrix::Vector3f position;
		matrix::Vector3f motor_conf[2];
		matrix::Vector3f servo_conf[2];
		uint8_t ax_psi;
		float gear_ratio[2];
	};

	struct Geometry {
		uint8_t num_agents;
		ModuleGeometry module_geometry[NUM_AGENTS_MAX];
	};
	// endregion [Geometry Struct Definitions]

	ActuatorEffectivenessVTOL_TVMD(ModuleParams *parent);
	virtual ~ActuatorEffectivenessVTOL_TVMD() = default;

	bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) override;

	void getDesiredAllocationMethod(AllocationMethod allocation_method_out[MAX_NUM_MATRICES]) const override {
		allocation_method_out[0] = AllocationMethod::CASCADED_PSEUDO_INVERSE;
	}


	void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index,
			    ActuatorVector &actuator_sp, const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
			    const matrix::Vector<float, NUM_ACTUATORS> &actuator_max) override;

	const char *name() const override { return "VTOL TVMD"; }

	// For unit test only
	void setGeometry(const Geometry geo) { _geometry = geo; };

	// void getUnallocatedControl(int matrix_index, control_allocator_status_s &status) override;

	void tf_inverse_mapping(const uint8_t module_id, const matrix::Vector2f &tftd, matrix::Vector2f &u_prop) const;

	void tf_mapping(const uint8_t module_id, matrix::Vector2f &tftd, const matrix::Vector2f &u_prop) const;

protected:

	void tf_mapping(const uint8_t module_id, const matrix::Vector2f tftd, matrix::Vector2f u_prop) const;

private:

	void updateParams() override;

	// region [Geometry Parameter Handles Struct Definitions]
	struct ParamModuleGeometry {
		param_t pos_x;
		param_t pos_y;
		param_t pos_z;
		param_t ax_psi;

		param_t c_l[2];
		param_t c_d[2];
		param_t motor_slew[2];   // propeller rate limit

		param_t servo_min[2];  // eta_x servo angle min
		param_t servo_max[2];  // eta_x servo angle max
		param_t servo_slew[2]; // eta_x rate limit
		param_t gear_ratio[2];
	};

	struct ParamHandles {
		param_t num_agents;
		ParamModuleGeometry module_geometry[NUM_AGENTS_MAX];
	};
	// endregion [Geometry Parameter Handles Struct Definitions]

	ParamHandles _param_handles{};

	Geometry _geometry{};

	ActuatorVector _actuator_scale;

	void genRotationMatrixRz(matrix::Matrix3f &Rz, const float psi);

	float _param_spoolup_time{1.f};

	// Tilt handling during motor spoolup: leave the tilts in their disarmed position unitil 1s after arming
	bool throttleSpoolupFinished();
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	bool _armed{false};
	uint64_t _armed_time{0};

	inline uint8_t get_motor_idx(uint8_t module_id, uint8_t offset) { return 2 * module_id + offset; };
	inline uint8_t get_servo_idx(uint8_t module_id, uint8_t offset) { return 2 * module_id + offset + 2 * _geometry.num_agents; };
};
