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
 * @file ControlAllocationModularBundled.hpp
 *
 * Cascaded Generalized Inverse
 *
 *
 * @author Yen-Cheng Chu <sciyen.ycc@gmail.com>
 */

#pragma once
#include <stdio.h>

#include "ControlAllocation.hpp"

// #define CA_MB_DEBUGGER

class ControlAllocationModularBundled: public ControlAllocation
{
public:
	static constexpr uint8_t NUM_MODULES {4};
	static constexpr uint8_t NUM_F {12};

	typedef uint8_t ActiveAgent;
	typedef matrix::Vector<float, NUM_AXES> ControlVector;
	typedef matrix::Vector<float, NUM_F> PseudoForceVector;

	// TODO: These parameters should be loaded using param_get()
	static constexpr float prop_d {0.0254 * 9}; 	// propeller diameter in meters
	static constexpr float rho {1.225};    			// air density in kg/m3
	static constexpr float c_l {0.020231}; 			// propeller thrust coefficient

	static constexpr float sigma_eta[2] {M_PI_F / 6.0f, M_PI_F / 2.0f};
	static constexpr float r_sigma_eta[2] {M_PI_F / 10.0f, M_PI_F / 10.0f};
	static constexpr float f_max {9.818f * 1.5f};	// maximum thrust of a single agent
	static constexpr float f_min {0.01f};			// minimum thrust of a single agent

	// Coordinate transformation
	const float team_t_max {15.0f};					// maximum torque of the team
	const float team_f_max {NUM_MODULES * f_max};	// maximum thrust of the team
	const float coord_trans_f[6] {					// coordinate transformation coefficients
		team_t_max, -team_t_max, -team_t_max,
		team_f_max, -team_f_max, -team_f_max};
	const ControlVector coord_trans {coord_trans_f};



	ControlAllocationModularBundled() = default;
	virtual ~ControlAllocationModularBundled() = default;

	void setEffectivenessMatrix(const matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &effectiveness,
				    const ActuatorVector &actuator_trim, const ActuatorVector &linearization_point, int num_actuators,
				    bool update_normalization_scale) override;

	/**
	 * The function that will be invoked by the control allocator
	 * when a new control input is available. It is responsible for
	 * updating the actuator setpoint (_actuator_sp).
	 */
	void allocate() override;

	matrix::Vector<float, NUM_AXES> getAllocatedControl() const override {
		// transform back to the original coordinate
		return (_eff * _f).edivide(coord_trans); 
	};

	static void forward_transform(const matrix::Vector3f &raw, matrix::Vector3f &f_i);

protected:

	// bitmap, maximum number of agents: 8
	ActiveAgent active_agents;

	matrix::SquareMatrix<float, NUM_AXES> _mwmt;
	matrix::SquareMatrix<float, NUM_AXES> _mwmt0;

	matrix::Matrix<float, NUM_AXES, NUM_F> _eff;
	matrix::SquareMatrix<float, NUM_AXES> _L;
	matrix::SquareMatrix<float, NUM_AXES> _L0;

	// TODO: using built-in variable instead to save memory
	// Allocation meta data
	PseudoForceVector _f;

	// Saturated pseudo forces
	PseudoForceVector _f_c;

	// Local Admissible Constraints, including x-servo, y-servo angle limit, and maximum thrust.
	matrix::Matrix<float, NUM_MODULES, 3> _upper;
	matrix::Matrix<float, NUM_MODULES, 3> _lower;

	bool _mix_update_needed{false};
	bool _rate_constraints_considered{false};

	// ============================================================================
	// Recalculate pseudo inverse if required.
	// 1. active_agent set changed

	// ============================================================================
	// Operations related to active_agent_set.

	inline int calc_num_active_agents() {return __builtin_popcount(active_agents); };

	inline void set_inactive_agent(const ActiveAgent agent_index) {active_agents &= ~((ActiveAgent)1 << agent_index); };


	// ============================================================================
	// Operations related to pseudo inverse.

	void truncated_allocation(const ControlVector &u, PseudoForceVector &f, const matrix::SquareMatrix<float, NUM_AXES> &L);

	bool update_mwmt(int downdating_idx, matrix::SquareMatrix<float, NUM_AXES> &L);

	virtual void calcualte_bundled_pseudo_inverse(ControlVector &u_in) = 0;

	/**
	 * Calculate the saturated agent index and the saturated pseudo force in
	 * a bundled manner (saturating a module for each call).
	 *
	 * @param agent_idx The index of the saturated agent. Return -1 if no agent is saturated.
	 * @return True if an agent is saturated.
	 */
	// virtual bool calc_saturated_agent_id(ActiveAgent &agent_idx, matrix::Vector3f &f_ci) = 0;


	// ============================================================================
	// Vehicle related operations
	void inverse_transform(matrix::Vector3f &raw, const matrix::Vector3f &f_i) const;

#ifdef CA_MB_DEBUGGER
	template<size_t M, size_t N>
	void print_vector(const matrix::Matrix<float, M, N> &m) const;
#endif // CA_MB_DEBUGGER

private:
	const uint8_t _actuator_idx_offset{NUM_MODULES * 2};

	// void normalizeControlAllocationMatrix();
	// void updateControlAllocationMatrixScale();
	// bool _normalization_needs_update{false};

	/**
	 * Operations related to local admissible space.
	 * Update local lower & upper bound.
	 */
	void calc_local_admissible();

	void control_input_remapping(ControlVector u_in, ControlVector &u_out);

	/**
	 * Update the actuator setpoint (_actuator_sp).
	 */
	void generate_actuator_sp(const PseudoForceVector &pseudo_force);
};
