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
 * @file ControlAllocationEBRCA.hpp
 *
 * Cascaded Generalized Inverse
 *
 *
 * @author Yen-Cheng Chu <sciyen.ycc@gmail.com>
 */

#include "ControlAllocationEBRCA.hpp"

ControlAllocationEBRCA::ControlAllocationEBRCA()
{
	printf("It's EBRCA running\n");
	_control_allocation_meta_data_pub.advertise();
}

void
ControlAllocationEBRCA::calcualte_bundled_pseudo_inverse(ControlVector &u_in)
{
	_iter = 0;
	_meta_data.timestamp = hrt_absolute_time();
	u_in.copyTo(_meta_data.control_sp);
	#ifdef CA_EBRCA_DEBUGGER
	printf("===================\n");
	printf("Unallocated: (%7.3f, %7.3f, %7.3f, %7.3f, %7.3f, %7.3f)\n",
		(double)u_in(0), (double)u_in(1), (double)u_in(2),
		(double)u_in(3), (double)u_in(4), (double)u_in(5));
	#endif

	for (uint8_t i = 0; i < NUM_MODULES; i++) {
		_f(i*3 + 0) = 0;
		_f(i*3 + 1) = 0;
		// _f(i*3 + 2) = f_min + 0.01f;
		if ( _f(i*3 + 2) < f_min )
			_f(i*3 + 2) = f_min + 0.01f;
		if ( _f(i*3 + 2) >= f_max )
			_f(i*3 + 2) = f_max - 0.01f;

		_meta_data.f_x[_idx(i)] = _f(i*3 + 0);
		_meta_data.f_y[_idx(i)] = _f(i*3 + 1);
		_meta_data.f_z[_idx(i)] = _f(i*3 + 2);
	}

	// Pseudo Boundary Protection (PBP)

	// previous state => using linearization point
	ControlVector u_prev = _eff * _f;
	// Redistributed Control Allocation

	ControlVector u_delta = u_in - u_prev;

	float c = 0;
	bool full_rank = true;
	while (full_rank && (c < 1) && (calc_num_active_agents() >= 3)) {
		// Calculate _f_c = M_esp^+ u
		// let _f_c be the allocation without constraints
		truncated_allocation(u_delta, _f_c, _L);

		// solve maximum increment
		int8_t saturated_agent = -1;
		float d = calc_saturated_agent_id(saturated_agent, _f, _f + _f_c);
		#ifdef CA_EBRCA_DEBUGGER
		printf("saturated_agent: %d, d=%f\n", saturated_agent, (double)d);
		#endif // CA_EBRCA_DEBUGGER

		_meta_data.saturated_idx[_iter] = saturated_agent;

		// invalid allocation (all agents are saturated)
		if ( (saturated_agent < 0) )
			break;

		// TODO: deal with multiple saturated agents
		set_inactive_agent(saturated_agent);

		// if no agent is saturated, then the allocation is complete
		// update the allocation
		if (d > 0) {
			// the increment must not be larger than the requested control
			d = (d > 1 - c) ? (1 - c) : d;
			c += d;
			_f = _f + d * _f_c;
		}

		for (uint8_t i = 0; i < NUM_MODULES; i++) {
			_meta_data.f_x[_idx(i)] = _f(i*3 + 0);
			_meta_data.f_y[_idx(i)] = _f(i*3 + 1);
			_meta_data.f_z[_idx(i)] = _f(i*3 + 2);
		}
		_meta_data.increment[_iter] = d;

		#ifdef CA_EBRCA_DEBUGGER
		printf("updated c: %f, d=%f\n", (double)c, (double)d);
		#endif // CA_EBRCA_DEBUGGER

		// downdating and check remaining rank
		full_rank = update_mwmt(saturated_agent, _L);

		_iter += 1;
	}
	_meta_data.increment[_iter] = -1;

	const matrix::Vector<float, NUM_AXES>  allocated_raw = _eff * _f;
	allocated_raw.copyTo(_meta_data.allocated_control);
	
	#ifdef CA_EBRCA_DEBUGGER
	const matrix::Vector<float, NUM_AXES>  allocated = getAllocatedControl();
	printf("allocation complete\n");
	printf("Allocated: (%7.3f, %7.3f, %7.3f, %7.3f, %7.3f, %7.3f)\n",
		(double)allocated(0), (double)allocated(1), (double)allocated(2),
		(double)allocated(3), (double)allocated(4), (double)allocated(5));
	#endif // CA_EBRCA_DEBUGGER

	// Post Torque Enhancement
	#ifdef CA_EBRCA_ENABLE_PBP
	if (full_rank) {
		if (c < 1) {
		}
	}
	#endif // CA_EBRCA_ENABLE_PBP

	_control_allocation_meta_data_pub.publish(_meta_data);
}

/**
 * Solve the maximum increment from f0 to the local admissible boundary
 * along the vector f1 - f0.
 * @param sat_idx output, the index of the saturated agent
 * @param f0 the current allocation
 * @param f1 the allocation without constraints
 * @return the maximum increment
 */
float
ControlAllocationEBRCA::calc_saturated_agent_id(int8_t &sat_idx, PseudoForceVector f0, PseudoForceVector f1)
{
	float smallest_increment = INFINITY;
	sat_idx = -1;

	for (ActiveAgent i = 0; i < NUM_MODULES; i++) {

		// only check for active agents
		if (active_agents & (1 << i)) {
			#ifdef CA_EBRCA_DEBUGGER
			printf("Agent: %d\n", i);
			#endif // CA_EBRCA_DEBUGGER

			const matrix::Vector3f f0_i( f0.slice<3, 1>(3*i, 0) );
			const matrix::Vector3f f1_i( f1.slice<3, 1>(3*i, 0) );
			const matrix::Vector3f f_delta = f1_i - f0_i;

			// if (f_delta.norm() < 1e-10f) {
			// 	continue;
			// }

			// inverse_transform: f_i -> raw0
			matrix::Vector3f raw0, raw1;
			matrix::Vector3f raw01;
			inverse_transform(raw0, f0_i);
			inverse_transform(raw01, f1_i);
			inverse_transform_on_tangent_plane(raw1, f0_i, f_delta);

			#ifdef CA_EBRCA_DEBUGGER
			printf("f0: %7.3f, %7.3f, %7.3f (", (double)f0_i(0), (double)f0_i(1), (double)f0_i(2));
			printf("raw0: %7.3f, %7.3f, %7.3f)\n", (double)raw0(0), (double)raw0(1), (double)raw0(2));
			printf("f1: %7.3f, %7.3f, %7.3f (", (double)f1_i(0), (double)f1_i(1), (double)f1_i(2));
			printf("raw1: %7.3f, %7.3f, %7.3f)\n", (double)raw01(0), (double)raw01(1), (double)raw01(2));
			printf("tangent: %7.3f, %7.3f, %7.3f\n", (double)raw1(0), (double)raw1(1), (double)raw1(2));
			#endif // CA_EBRCA_DEBUGGER

			// determine local boundary
			const float x_bound = (raw1(0) < 0) ? _lower(i,0) : _upper(i,0);
			const float y_bound = (raw1(1) < 0) ? _lower(i,1) : _upper(i,1);
			const float z_bound = (raw1(2) < 0) ? _lower(i,2) : _upper(i,2);

			// check boundary violations, a violation should be handled right away
			const bool eta_x_violation = ( signbit(raw0(0) - x_bound) == signbit(raw1(0)) );
			const bool eta_y_violation = ( signbit(raw0(1) - y_bound) == signbit(raw1(1)) );
			const bool tf_violation = ( signbit(raw0(2) - z_bound) == signbit(raw1(2)) );

			#ifdef CA_EBRCA_DEBUGGER
			printf("bounds: %7.3f, %7.3f, %7.3f\n", (double)x_bound, (double)y_bound, (double)z_bound);
			printf("violation: %d, %d, %d\n", eta_x_violation, eta_y_violation, tf_violation);
			#endif // CA_EBRCA_DEBUGGER

			// TODO: for any active agent, there should not be any negative increment
			if (eta_x_violation || eta_y_violation || tf_violation) {
				// any violation => mark as saturated, and return 0 (smallest increment)
				sat_idx = i;
				return 0;
			}

			// solve the intersections
			float tx = -1, ty = -1, tz = -1;
			tx = check_negative(solve_intersections_x_axis(f0_i, f_delta, y_bound));
			ty = check_negative(solve_intersections_y_axis(f0_i, f_delta, x_bound));
			tz = check_negative(solve_intersections_z_axis(f0_i, f_delta, z_bound));

			// check the minimum violation
			const float t_min0 = (tx < ty) ? tx : ty;
			const float t_min = (tz < t_min0) ? tz : t_min0;

			_meta_data.t_x[_idx(i)] = _f(i*3 + 0);
			_meta_data.t_y[_idx(i)] = _f(i*3 + 1);
			_meta_data.t_z[_idx(i)] = _f(i*3 + 2);
			_meta_data.t_min[_idx(i)] = t_min;

			#ifdef CA_EBRCA_DEBUGGER
			printf("intersections: %f, %f, %f, %f\n", (double)tx, (double)ty, (double)tz, (double)t_min);
			#endif // CA_EBRCA_DEBUGGER

			if ( t_min < 0 ) {
				sat_idx = i;
				return 0;
			}

			if ((t_min >= 0) && (t_min <= smallest_increment)) {
				smallest_increment = t_min;
				sat_idx = i;
			}
		}
	}

	if (smallest_increment >= INFINITY)
		smallest_increment = 1;

	return smallest_increment;
}

void ControlAllocationEBRCA::inverse_transform_on_tangent_plane(
	matrix::Vector3f &raw, const matrix::Vector3f &f0_i, const matrix::Vector3f &f_delta_i) const
{
	const float prob_len = 0.1;

	matrix::Vector3f raw0;
	inverse_transform(raw0, f0_i);

	matrix::Vector3f ff = prob_len * f_delta_i / f_delta_i.norm();

	const float jac_fa[3][3] = {
		{-raw0(2)*sinf(raw0(0))*sinf(raw0(1)), raw0(2)*cosf(raw0(0))*cosf(raw0(1)), cosf(raw0(0))*sinf(raw0(1))},
		{-raw0(2)*cosf(raw0(0)), 0, -sinf(raw0(0))},
		{-raw0(2)*sinf(raw0(0))*cosf(raw0(1)), -raw0(2)*sinf(raw0(1))*cosf(raw0(0)), cosf(raw0(0))*cosf(raw0(1))}
	};
	const matrix::SquareMatrix3f jac_f = matrix::SquareMatrix3f(jac_fa);

	raw = inv(jac_f) * ff;
}

float
ControlAllocationEBRCA::solve_intersections_x_axis(
	const matrix::Vector3f &f0_i, const matrix::Vector3f &f_delta_i,
	const float & y_bound)
{
	if (tanf(y_bound) > 1e10f) {
		// if sigma_y = pi/2, then the intersection is on the x-y plane
		return - f0_i(2) / f_delta_i(2);
	}
	else {
		const float v_z = f0_i(2) * tanf(y_bound);
		const float v_delta_z = f_delta_i(2) * tanf(y_bound);

		const float tx = - (f0_i(0) - v_z) / (f_delta_i(0) - v_delta_z);

		// if it the starting point is on the boundary and it is not violated,
		// then the intersection must be at infinity
		if ( abs(f0_i(0) - v_z) < _epsilon) return INFINITY;

		// there is no increment along the x-axis
		else if ( abs(f_delta_i(0) - v_delta_z) < _epsilon) return INFINITY;
		else return tx;
	}
}

float
ControlAllocationEBRCA::solve_intersections_y_axis(
	const matrix::Vector3f &f0_i, const matrix::Vector3f &f_delta_i,
	const float & x_bound)
{
	const float v_delta_y = f_delta_i(1) / tanf(x_bound);
	const float v_y = f0_i(1) / tanf(x_bound);

	const float tya = f_delta_i(0) * f_delta_i(0) + f_delta_i(2) * f_delta_i(2) - v_delta_y * v_delta_y;
	const float tyb = f0_i(0) * f_delta_i(0) + f0_i(2) * f_delta_i(2) - v_y * v_delta_y;
	const float tyc = f0_i(0) * f0_i(0) + f0_i(2) * f0_i(2) - v_y * v_y;

	// check existence of the solution
	if (tyb * tyb - tya * tyc < 0) {
		return INFINITY;
	}

	// check if there is only one solution
	else if (abs(tya) < _epsilon) {
		if (abs(tyb) < _epsilon) return INFINITY;
		else if (abs(tyc) < _epsilon) return INFINITY;
		else return -tyc / (2 * tyb);
	}

	// solve the quadratic equation, and select the smaller one
	else {
		const float t1 = (-tyb - sqrtf(tyb * tyb - tya * tyc)) / tya;
		const float t2 = (-tyb + sqrtf(tyb * tyb - tya * tyc)) / tya;
		// printf("t1: %f, t2: %f\n", (double)t1, (double)t2);

		// choose the smaller (nearest) one but not negative one
		if ( t1 < t2 ) {
			if (t1 < 0) {
				if (t2 <= 0) return INFINITY;
				else return t2;
			}
			else return t1;
		}
		else {
			if (t2 < 0) {
				if (t1 <= 0) return INFINITY;
				else return t1;
			}
			else return t2;
		}
	}
}

float
ControlAllocationEBRCA::solve_intersections_z_axis(
	const matrix::Vector3f &f0_i, const matrix::Vector3f &f_delta_i,
	const float & z_bound)
{
	const float kkb = f0_i.dot(f_delta_i);
	const float kkc = f0_i.norm_squared() - z_bound * z_bound;
	const float f_delta_i_norm = f_delta_i.norm_squared();

	// only one solution
	if (f_delta_i_norm < _epsilon) {
		if (abs(kkc) < _epsilon) return INFINITY;
		else if (abs(kkb) < _epsilon) return INFINITY;
		else return - kkc / (2 * kkb);
	}
	// no solution
	else if (kkb * kkb - f_delta_i_norm * kkc < 0) {
		return INFINITY;
	}
	else{
		return (-kkb + sqrtf(kkb * kkb - f_delta_i_norm * kkc)) / f_delta_i_norm;
	}
}
