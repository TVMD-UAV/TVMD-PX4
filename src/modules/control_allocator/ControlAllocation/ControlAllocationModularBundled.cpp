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

#include "ControlAllocationModularBundled.hpp"

// https://stackoverflow.com/questions/14395967/proper-initialization-of-static-constexpr-array-in-class-template
constexpr float ControlAllocationModularBundled::sigma_eta[2];
constexpr float ControlAllocationModularBundled::r_sigma_eta[2];

/**
 * This function sets up the effectiveness matrix and the trim value, which should be called after the
 * vehicle configuration is changed.
 * @param effectiveness The effectiveness matrix (typically from a ActuatorEffectiveness module, store in Configuration)
 * @param actuator_trim The trim value of the actuators (typically from a ActuatorEffectiveness module)
 * @param linearization_point The linearization point of the actuators (typically from a ControlAllocation module)
 * @param num_actuators The number of actuators
 * @param update_normalization_scale Whether to update the normalization scale
 */
void
ControlAllocationModularBundled::setEffectivenessMatrix(
	const matrix::Matrix<float, ControlAllocation::NUM_AXES,
	ControlAllocation::NUM_ACTUATORS> &effectiveness,
	const ActuatorVector &actuator_trim,
	const ActuatorVector &linearization_point, int num_actuators,
	bool update_normalization_scale)
{
	ControlAllocation::setEffectivenessMatrix(effectiveness, actuator_trim, linearization_point, num_actuators,
			update_normalization_scale);
	_mix_update_needed = true;

	// Initialize mwmt
	_eff = _effectiveness.slice<NUM_AXES, NUM_F>(0, 0);
	_mwmt0 = _eff * _eff.transpose();
	_f.setZero();
	_f_c.setZero();

	// Initialize Choleskey decomposition
	_L0 = matrix::cholesky(_mwmt0);

	// Setup initial local admissible bounds
	for (int i = 0; i < NUM_MODULES; i++) {
		// Calculate lower and upper bound
		// Initial local bounds are set to the entire Admissible Force Space
		_lower(i, 0) = -sigma_eta[0];
		_upper(i, 0) =  sigma_eta[0];

		_lower(i, 1) = -sigma_eta[1];
		_upper(i, 1) =  sigma_eta[1];

		_lower(i, 2) = f_min;
		_upper(i, 2) = f_max;
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
ControlAllocationModularBundled::truncated_allocation(
	const ControlVector &u,
	PseudoForceVector &out_f,
	const matrix::SquareMatrix<float, NUM_AXES> &in_L)
{
	// L should be guaranteed to be full rank
	matrix::SquareMatrix<float, NUM_AXES> L_inv = inv(in_L);

	// meta control vector
	const ControlVector u_M = L_inv.T() * L_inv * u;

	// truncated allocation
	for (int k=0; k<3*NUM_MODULES; ++k) {
		if (active_agents & (1 << (k / 3))) {
			out_f(k) = matrix::Vector<float, NUM_AXES>(_eff.slice<NUM_AXES, 1>(0, k)).dot(u_M);
		}
		else {
			out_f(k) = 0;
		}
	}
	// out_f = _eff.T() * u_M;
	// for (int k=0; k<3*NUM_MODULES; ++k) {
	// 	if (active_agents & (1 << (k / 3))) {}
	// 	else {
	// 		out_f(k) = 0;
	// 	}
	// }
}


/**
 * Allocate new controls. Call this function to inverse the effectiveness matrix
 * using CGI, and then calculate _actuator_sp from _control_sp within the local
 * admissible force space.
 *
 * The output, i.e. _actuator_sp is in the format described as follows
 * index       |       2 x #module     |       2 x #module     |
 * module id   |---0---|---1---|-------|---0---|---1---|-------|
 * upper/lower |-0-|-1-|-0-|-1-|-------|-0-|-1-|-0-|-1-|-------|
 * parameter   |Tf0|Td0|Tf1|Td1|-------|etx|ety|etx|ety|-------|
 * unit        |N  |Nm |N  |Nm |-------|rad|rad|rad|rad|-------|
 */
void ControlAllocationModularBundled::allocate()
{
	// Initialization
	active_agents = ~0;
	_f_c.setZero();

	if (_rate_constraints_considered)
		calc_local_admissible();

	ControlVector remaining_control;
	control_input_remapping(_control_sp, remaining_control);
	// matrix::constrain(remaining_control)

	// The decomposed mwmt
	_mwmt = _mwmt0;
	_L = _L0;

	calcualte_bundled_pseudo_inverse(remaining_control);

	// transform f to _actuator_sp
	generate_actuator_sp(_f);
}

/**
 * Using downdataing Cholesky decomposition to update the decomposed lower triangular
 * matrix. This function removes components of the `downdating_idx`-th agent from
 * _mwmt, which is a 6x3 matrix.
 * This function can be accelerated by using a downdating Cholesky decomposition.
 * @param downdating_idx  The axis to be downdated, i.e., the saturated agent index
 * @param out_L The decomposed lower triangular matrix
 *
 * @return The decomposed L is still full rank
 */
bool
ControlAllocationModularBundled::update_mwmt(int downdating_idx, matrix::SquareMatrix<float, NUM_AXES> &out_L){
	const matrix::Matrix<float, ControlAllocation::NUM_AXES, 3> v = _eff.slice<NUM_AXES, 3>(0, 3*downdating_idx);
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
ControlAllocationModularBundled::calc_local_admissible(){
	for (int i = 0; i < NUM_MODULES; i++) {
		// Calculate lower and upper bound
		const uint8_t eta_idx = _actuator_idx_offset + 2*i;
		for (int k = 0; k < 2; k++){
			// TODO: Check the domain of actuator setpoints
			const float sp = _prev_actuator_sp(eta_idx + k);
			const float vrate = r_sigma_eta[k];
			const float vmin = -sigma_eta[k];
			const float vmax =  sigma_eta[k];

			_lower(i, k) = (sp - vrate < vmin) ? vmin : (sp - vrate);
			_upper(i, k) = (sp + vrate > vmax) ? vmax : (sp + vrate);
			_lower(i, k) = f_min;
			_upper(i, k) = f_max;
		}
	}

	// TODO: the maximum thrust (f_max) must be considered
	// The maximum thrust is fixed to f_max currently.
}

/**
 * This function is used to transform the force vector to the raw control vector.
 * @param raw: output, eta_x, eta_y, Tf
 * @param f_i: input, force vector of a single agent
 */
void ControlAllocationModularBundled::inverse_transform(
	matrix::Vector3f &raw, const matrix::Vector3f &f_i) const
{
	// TODO: To tackle negative z-axis forces
	const float minimum_z_thrust = f_min;
	if (f_i(2) < minimum_z_thrust) {
		raw.setZero();
	}
	else {
		// T_f = || f_i ||_2
		raw(2) = f_i.norm();

		// eta_x = asin(-y, T_f)
		raw(0) = std::asin(-f_i(1) / raw(2));

		// eta_y = atan2(x, z)
		raw(1) = std::atan2(f_i(0), f_i(2));

		if (raw(2) < 0.1f) {
			raw(0) *= 0;
			raw(1) *= 0;
		}
	}
}

void ControlAllocationModularBundled::control_input_remapping(
	ControlVector u_in, ControlVector &u_out)
{

	// Check z-axis input before remapping
	if (-u_in(5) < -0.05f)
		u_in.slice<3, 1>(0, 0) = u_in.slice<3, 1>(0, 0) * 0.0f;

	// Coordinate transformation
	// const float team_t_max = NUM_MODULES * 0.01f;
	const float team_t_max = 15.0f;
	const float team_f_max = NUM_MODULES * f_max;
	const float coord_trans_f[6] = {team_t_max, -team_t_max, -team_t_max,
					team_f_max, -team_f_max, -team_f_max};
	// const float coord_trans_f[6] = {team_t_max, -team_t_max, -team_t_max, team_f_max, -team_f_max, -team_f_max};
	const ControlVector coord_trans(coord_trans_f);

	u_out = coord_trans.emult(u_in);
}

/**
 * This function is used to transform the raw control vector to the force vector.
 * @param raw: input, eta_x, eta_y, Tf
 * @param f_i: output, input, force vector of a single agent
 */
void ControlAllocationModularBundled::forward_transform(const matrix::Vector3f &raw, matrix::Vector3f &f_i) {
	// const float &eta_x, const float &eta_y, const float &T_f,
	f_i(0) = std::cos(raw(0)) * std::sin(raw(1)) * raw(2);
	f_i(1) = -std::sin(raw(0)) * raw(2);
	f_i(2) = std::cos(raw(0)) * std::cos(raw(1)) * raw(2);
}

void ControlAllocationModularBundled::generate_actuator_sp(const PseudoForceVector &pseudo_force)
{
	// transform f to _actuator_sp
	for (ActiveAgent i = 0; i < NUM_MODULES; i++) {
		matrix::Vector3f raw;

		// only check for active agents
		if (active_agents & (1 << i)) {
			const matrix::Vector3f f_i( pseudo_force.slice<3, 1>(3*i, 0) );

			// inverse_transform: f_i -> raw
			inverse_transform(raw, f_i);
		}
		else {
			raw.setZero();
		}

		const uint8_t motor_idx = 2*i;
		const uint8_t eta_idx = _actuator_idx_offset + 2*i;
		_actuator_sp(motor_idx  ) = raw(2);  // Tf    (N)
		_actuator_sp(motor_idx+1) = 0;       // Td    (Nm)
		_actuator_sp(eta_idx  )   = raw(0);  // eta_x (rad)
		_actuator_sp(eta_idx+1)   = raw(1);  // eta_y (rad)
	}

	// Warning: The post-modification in actuator effectiveness will not affect the inner state.
	// TODO: This should be replaced using internal state feedback
	// All the control allocation codes may require to be moved to ActuatorEffectiveness.
	_prev_actuator_sp = _actuator_sp;
}

#ifdef CA_MB_DEBUGGER
template<size_t M, size_t N>
void ControlAllocationModularBundled::print_vector(const matrix::Matrix<float, M, N> &mat) const {
	if (N != 1) {
		for (size_t m=0; m<M; ++m) {
			for (size_t n=0; n<N; ++n) {
				printf("\t%7.4f", (double)mat(m, n));
			}
			printf("\n");
		}
	}
	else {
		for (size_t n=0; n<N; ++n) {
			for (size_t m=0; m<M; ++m) {
				printf("\t%7.4f", (double)mat(m, n));
			}
			printf("\n");
		}
	}
}
#endif // CA_MB_DEBUGGER
