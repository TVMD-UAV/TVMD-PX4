/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file ControlAllocationCGI.hpp
 *
 * Cascaded Generalized Inverse
 *
 *
 * @author Yen-Cheng Chu <sciyen.ycc@gmail.com>
 */

#pragma once

#include "ControlAllocation.hpp"

class ControlAllocationCGI: public ControlAllocation
{
public:
	static constexpr uint8_t NUM_MODULES = 4;
	static constexpr float d = 0.0254 * 9; // meter
	static constexpr float rho = 1.225;    // kg/m3
	static constexpr float c_l = 0.020231; // propeller thrust coefficient

	static constexpr float sigma_x = M_PI / 2.0f;
	static constexpr float sigma_y = M_PI / 6.0f;
	static constexpr float r_sigma_x = M_PI / 10.0f;
	static constexpr float r_sigma_y = M_PI / 10.0f;
	static constexpr float f_max = M_PI / 10.0f;

	typedef uint8_t ActiveAgent;
	typedef matrix::Vector<float, NUM_AXES> ControlVector;
	typedef matrix::Vector<float, NUM_MODULES * 4> PseudoForceVector;

	// static constexpr uint8_t NUM_MODULES = ActuatorEffectiveness::NUM_MODULES;

	ControlAllocationCGI() = default;
	virtual ~ControlAllocationCGI() = default;

	void allocate() override;
	void setEffectivenessMatrix(const matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &effectiveness,
				    const ActuatorVector &actuator_trim, const ActuatorVector &linearization_point, int num_actuators,
				    bool update_normalization_scale) override;

protected:
	const uint8_t _actuator_idx_offset{NUM_MODULES * 2};

	// bitmap, maximum number of agents: 8
	ActiveAgent active_agents;

	// matrix::Matrix<float, NUM_ACTUATORS, NUM_AXES> _mix;
	matrix::SquareMatrix<float, NUM_AXES> _mwmt;
	matrix::SquareMatrix<float, NUM_AXES> _mwmt0;

	matrix::SquareMatrix<float, NUM_AXES> _L;
	matrix::SquareMatrix<float, NUM_AXES> _L0;

	// TODO: using built-in variable instead to save memory
	// Allocation meta data
	PseudoForceVector _f;

	// Saturated pseudo forces
	PseudoForceVector _f_c;

	// Local Admissible Constraints, only servos are considered
	matrix::Matrix<float, NUM_MODULES, 2> _upper;
	matrix::Matrix<float, NUM_MODULES, 2> _lower;

	bool _mix_update_needed{false};
	bool _rate_constraints_considered{true};

	/**
	 * Recalculate pseudo inverse if required.
	 * 1. active_agent set changed
	 */
	virtual matrix::Vector<float, NUM_AXES> getAllocatedControl() const override;

	inline int calc_num_active_agents() {return __builtin_popcount(active_agents); };

	inline void set_inactive_agent(const ActiveAgent agent_index) {active_agents &= ~((ActiveAgent)1 << agent_index); };


	void truncated_allocation(const ControlVector &u, PseudoForceVector &f, const matrix::SquareMatrix<float, NUM_AXES> &L);

	bool update_mwmt(int downdating_idx, matrix::SquareMatrix<float, NUM_AXES> &L);

	/**
	 * Update local lower & upper bound
	 */
	void calc_local_admissible();

	bool calc_saturated_agent_id(ActiveAgent &agent_idx);

	void inverse_transform(matrix::Vector3f &raw, const matrix::Vector3f &f_i);

	// inverse transform: Tf -> w_p1, w_p2
	void tf_inverse_transform(const float &tf, float &w_p1, float &w_p2);

	void forward_transform(const matrix::Vector3f &raw, matrix::Vector3f &f_i);

private:
	// void normalizeControlAllocationMatrix();
	// void updateControlAllocationMatrixScale();
	// bool _normalization_needs_update{false};
};
