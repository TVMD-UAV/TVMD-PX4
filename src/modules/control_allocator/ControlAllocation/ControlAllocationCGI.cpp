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

#include "ControlAllocationCGI.hpp"


void
ControlAllocationCGI::setEffectivenessMatrix(
	const matrix::Matrix<float, ControlAllocation::NUM_AXES, ControlAllocation::NUM_ACTUATORS> &effectiveness,
	const ActuatorVector &actuator_trim, const ActuatorVector &linearization_point, int num_actuators,
	bool update_normalization_scale)
{
	ControlAllocation::setEffectivenessMatrix(effectiveness, actuator_trim, linearization_point, num_actuators,
			update_normalization_scale);
	_mix_update_needed = true;

	// Initialize mwmt
	_mwmt0 = _effectiveness * _effectiveness.transpose();

	// Initialize Choleskey decomposition
	_L0 = matrix::cholesky(_mwmt0);

	// Setup initial local admissible bounds
	for (int i = 0; i < NUM_MODULES; i++) {
		// Calculate lower and upper bound
		const uint8_t eta_idx = _actuator_idx_offset + 2*i;
		for (int k = 0; k < 2; k++){
			const auto vmin = _actuator_min(eta_idx + k);
			const auto vmax = _actuator_max(eta_idx + k);

			_lower(i, k) = vmin;
			_upper(i, k) = vmax;
		}
	}

	// TODO: update bounds
}

void
ControlAllocationCGI::allocate(){
	// Initialization
	_prev_actuator_sp = _actuator_sp;
	_f_c.setZero();
	if (_rate_constraints_considered)
		calc_local_admissible();

	ControlVector remaining_control = _control_sp;
	// The decomposed mwmt
	_mwmt = _mwmt0;
	_L = _L0;

	while (calc_num_active_agents() >= 3) {
		// mwmt should be guaranteed to be full rank

		// override previous allocation _f
		truncated_allocation(remaining_control, _f, _L);

		_f = _f + _f_c;

		ActiveAgent violation_idx;

		// calculate the agent has the largest norm violation
		// The saturated force vector will be directly updated to _f
		if (!calc_saturated_agent_id(violation_idx)) {
			// break if the requested control is satisfied
			// mark that the control can be satisfied
			break;
		}

		// downdating and check remaining rank
		bool full_rank = update_mwmt(violation_idx, _L);
		if (full_rank) {
			// mark that it fails to fulfill the requested control
			break;
		}

		// update f_c
		const matrix::Vector3f f_ci(_f.slice<3, 1>(4*violation_idx, 0));
		set_inactive_agent(violation_idx);
		_f_c.slice<3, 1>(4*violation_idx, 0) = f_ci;

		// Only change the components that is need to be updated
		const matrix::Matrix<float, NUM_AXES, 3> eff_slice(_effectiveness.slice<NUM_AXES, 3>(4*violation_idx, 0));
		remaining_control = remaining_control - eff_slice * f_ci;
	}

	// transform f to _actuator_sp
	for (ActiveAgent i = 0; i < NUM_MODULES; i++) {
		matrix::Vector3f raw;
		// float w_p1, w_p2;

		// only check for active agents
		if (active_agents & (1 << i)) {
			const matrix::Vector3f f_i( _f.slice<3, 1>(4*i, 0) );

			// inverse_transform: f_i -> raw
			inverse_transform(raw, f_i);

			// Propeller speed inverse mapping
			// tf -> w_p1, w_p2
			// tf_inverse_transform(raw(2), w_p1, w_p2);
		}
		else {
			raw.setZero();
			// w_p1 = 0;
			// w_p2 = 0;
		}

		const uint8_t motor_idx = 2*i;
		const uint8_t eta_idx = _actuator_idx_offset + 2*i;
		_actuator_sp(motor_idx  ) = raw(2);  // Tf    (Nm)
		_actuator_sp(motor_idx+1) = 0;       // Td    (Nm)
		_actuator_sp(eta_idx  )   = raw(0);  // eta_x (rad)
		_actuator_sp(eta_idx+1)   = raw(1);  // eta_y (rad)
		// _actuator_sp(motor_idx  ) = _actuator_trim(motor_idx  ) + w_p1;
		// _actuator_sp(motor_idx+1) = _actuator_trim(motor_idx+1) + w_p2;
		// _actuator_sp(eta_idx  ) = _actuator_trim(eta_idx  ) + raw(0);
		// _actuator_sp(eta_idx+1) = _actuator_trim(eta_idx+1) + raw(1);
	}
}

/**
 * Calculate f = M_esp^+ u
 * L should be ensured to be full rank
 * @param u control wrench input 6 DoF column vector
 * @param out_f output truncated allocation
 * @param L input the decomposed mwmt(k)
 */
void
ControlAllocationCGI::truncated_allocation(
	const ControlVector &u,
	PseudoForceVector &out_f,
	const matrix::SquareMatrix<float, NUM_AXES> &in_L)
{
	// L should be guaranteed to be full rank
	matrix::SquareMatrix<float, NUM_AXES> L_inv = inv(in_L);

	// meta control vector
	const ControlVector u_M = L_inv.T() * L_inv * u;
	out_f = _effectiveness.T() * u_M;
}


/**
 * This function can be accelerated by using a downdating Cholesky decomposition.
 * @param downdating_idx  The axis to be downdated, i.e., the saturated agent index
 * @param out_L The decomposed lower triangular matrix
 *
 * @return The decomposed L is still full rank
 */
bool
ControlAllocationCGI::update_mwmt(int downdating_idx, matrix::SquareMatrix<float, NUM_AXES> &out_L){
	matrix::Matrix<float, ControlAllocation::NUM_AXES, 3> v(_effectiveness.slice<NUM_AXES, 3>(3*downdating_idx, 0));
	_mwmt = _mwmt - v * v.T();

	// check rank
	size_t rank;
	out_L = fullRankCholesky(_mwmt, rank);
	return (rank == NUM_AXES);
}


/**
 * The local admissible force space is defined on the admissible control space,
 * and only the servo angles are explicitly considered in control allocation.
 *
 * lower and upper bound of the admissible control space will be updated.
 * This function is only needed to be called once at the begining of an allocation.
 */
void
ControlAllocationCGI::calc_local_admissible(){
	for (int i = 0; i < NUM_MODULES; i++) {
		// Calculate lower and upper bound
		const uint8_t eta_idx = _actuator_idx_offset + 2*i;
		for (int k = 0; k < 2; k++){
			const auto sp = _prev_actuator_sp(eta_idx + k);
			const auto vrate = _actuator_slew_rate_limit(eta_idx + k);
			const auto vmin = _actuator_min(eta_idx + k);
			const auto vmax = _actuator_max(eta_idx + k);

			_lower(i, k) = (sp - vrate < vmin) ? vmin : (sp - vrate);
			_upper(i, k) = (sp + vrate > vmax) ? vmax : (sp + vrate);
		}
	}

	// TODO: the maximum thrust (f_max) must be considered
}


/**
 * Calculate the agent id with the largest violation norm
 * @param agent_idx output, the agent that is saturated
 * @param f_ci output, the saturated force of the output agent
 *
 * @return if there is an agent saturated
 */
bool
ControlAllocationCGI::calc_saturated_agent_id(ActiveAgent &agent_idx)
{
	float largest_err = -1;

	for (ActiveAgent i = 0; i < NUM_MODULES; i++) {
		// only check for active agents
		if (active_agents & (1 << i)) {
			const matrix::Vector3f f_i( _f.slice<3, 1>(4*i, 0) );
			matrix::Vector3f raw;
			matrix::Vector3f f_i_sat;

			// inverse_transform: f_i -> raw
			inverse_transform(raw, f_i);

			// check if saturated
			// TODO: the maximum thrust (f_max) must be considered
			bool legal = true;
			for (int k=0; k<2 && legal; k++) {
				if ( _lower(i, k) <= raw(k) ) {
					legal = false;
					raw(k) = _lower(i, k);
				}
				else if ( raw(k) <= _upper(i, k) ) {
					legal = false;
					raw(k) = _upper(i, k);
				}
			}
			// skip if this agent is not saturated
			if (legal) continue;

			// forward_transform: raw -> f_i
			forward_transform(raw, f_i_sat);

			const float err_norm = (f_i - f_i_sat).norm_squared();
			if (err_norm > largest_err){
				largest_err = err_norm;
				agent_idx = i;
			}
			_f.slice<3, 1>(4*i, 0) = f_i_sat;
		}
	}

	return largest_err > 0;
}

/**
 * @param raw: output, eta_x, eta_y, Tf
 * @param f_i: input, force vector of a single agent
 */
void ControlAllocationCGI::inverse_transform(matrix::Vector3f &raw, const matrix::Vector3f &f_i){
	// float &eta_x, float &eta_y, float &T_f,

	// T_f = || f_i ||_2
	raw(2) = f_i.norm();

	// eta_x = asin(-y, T_f)
	raw(0) = std::asin(-f_i(1) / raw(2));

	// eta_y = atan2(x, z)
	raw(1) = std::atan2(f_i(0), f_i(2));
}

/**
 * @param raw: input, eta_x, eta_y, Tf
 * @param f_i: output, input, force vector of a single agent
 */
void ControlAllocationCGI::forward_transform(const matrix::Vector3f &raw, matrix::Vector3f &f_i){
	// const float &eta_x, const float &eta_y, const float &T_f,
	f_i(0) = std::cos(raw(0)) * std::sin(raw(1)) * raw(2);
	f_i(1) = -std::sin(raw(0)) * raw(2);
	f_i(2) = std::cos(raw(0)) * std::cos(raw(1)) * raw(2);
}

/**
 * Inverse transform from Tf to w_p1 and w_p2
 *     +-   -+           +-          -+     +-      -+
 *     | T_f |           |  c_l  c_l  |     | w_p1^2 |
 *     |     | = rho d^4 |            |  =  |        |
 *     | T_d |           | -dc_d dc_d |     | w_p2^2 |
 *     +-   -+           +-          -+     +-      -+
 *
 *                        ____________
 *                    1   |    T_f
 *    w_p1 = w_p2 = ----- | ----------
 *                   d^2  | 2 rho c_l
 *
 */
void ControlAllocationCGI::tf_inverse_transform(const float &tf, float &w_p1, float &w_p2){
	constexpr float c = 1.0f / (d * d * sqrtf(2 * rho * c_l));
	w_p1 = sqrtf(tf) * c;
	w_p2 = w_p1;
}
