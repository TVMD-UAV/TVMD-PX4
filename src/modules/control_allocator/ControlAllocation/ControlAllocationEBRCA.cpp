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


void
ControlAllocationEBRCA::calcualte_bundled_pseudo_inverse(ControlVector &u_in){
	#ifdef CA_CGI_DEBUGGER
	uint8_t iter = 0;
	#endif // CA_CGI_DEBUGGER

	while (calc_num_active_agents() >= 3) {
		// mwmt should be guaranteed to be full rank
		#ifdef CA_CGI_DEBUGGER
		printf("\n===================================================\nIter: %d \n", iter++);
		printf("remaining control: \n");
		print_vector(u_in);
		printf("\n");
		#endif // CA_CGI_DEBUGGER

		// override previous allocation _f
		truncated_allocation(u_in, _f, _L);

		#ifdef CA_CGI_DEBUGGER
		printf("Pre-allocated results _f: \n");
		print_vector(_f);
		printf("Current saturated controls: _f_c \n");
		print_vector(_f_c);
		printf("Evaluate truncated allocation: u \n");
		print_vector(getAllocatedControl());
		printf("\n");
		#endif // CA_CGI_DEBUGGER

		_f = _f + _f_c;
		#ifdef CA_CGI_DEBUGGER
		printf("Updated allocation _f: \n");
		print_vector(_f);
		#endif // CA_CGI_DEBUGGER

		ActiveAgent violation_idx;
		matrix::Vector3f f_ci;

		// calculate the agent has the largest norm violation
		// The saturated force vector will be directly updated to _f
		if (!calc_saturated_agent_id(violation_idx, f_ci)) {
			// break if the requested control is satisfied
			// mark that the control can be satisfied
			#ifdef CA_CGI_DEBUGGER
			printf("\n===================================================\n");
			printf("f: \n");
			// _f.slice<3, 1>(3*violation_idx, 0) = f_ci;
			print_vector(_f);
			printf("\n");

			printf("u: \n");
			print_vector(getAllocatedControl());
			printf("\n");
			printf("Allocation satisfied \n");
			#endif // CA_CGI_DEBUGGER
			break;
		}
		#ifdef CA_CGI_DEBUGGER
		printf("The saturated agent: %d \n", violation_idx);
		#endif // CA_CGI_DEBUGGER

		// downdating and check remaining rank
		bool full_rank = update_mwmt(violation_idx, _L);
		if (!full_rank) {
			// mark that it fails to fulfill the requested control
			#ifdef CA_CGI_DEBUGGER
			printf("\n===================================================\n");
			printf("Rank lose \n");
			#endif // CA_CGI_DEBUGGER
			break;
		}

		// update f_c
		set_inactive_agent(violation_idx);
		_f_c.slice<3, 1>(3*violation_idx, 0) = f_ci;

		// Only change the components that is needed to be updated
		const matrix::Matrix<float, NUM_AXES, 3> eff_slice = _eff.slice<NUM_AXES, 3>(0, 3*violation_idx);
		u_in = u_in - eff_slice * f_ci;

		#ifdef CA_CGI_DEBUGGER
		printf("f_c: \n");
		print_vector(_f_c);

		printf("f: \n");
		_f.slice<3, 1>(3*violation_idx, 0) = f_ci;
		print_vector(_f);

		printf("u: \n");
		print_vector(getAllocatedControl());
		printf("\n");

		printf("Allocation: \n");
		printf("Single: ");
		print_vector(_eff * _f_c);
		printf("Total : ");
		print_vector(eff_slice * f_ci);
		printf("\n");
		#endif //CA_CGI_DEBUGGER
	}
}

/**
 * Calculate the agent id with the largest violation norm
 * @param agent_idx output, the agent that is saturated
 *
 * @return if there is an agent saturated
 */
bool
ControlAllocationEBRCA::calc_saturated_agent_id(ActiveAgent &agent_idx, matrix::Vector3f &f_ci)
{
	float largest_err = -1;

	for (ActiveAgent i = 0; i < NUM_MODULES; i++) {
		// only check for active agents
		if (active_agents & (1 << i)) {
			const matrix::Vector3f f_i( _f.slice<3, 1>(3*i, 0) );
			matrix::Vector3f raw;
			matrix::Vector3f f_i_sat;

			// inverse_transform: f_i -> raw
			inverse_transform(raw, f_i);
			#ifdef CA_CGI_DEBUGGER
			printf("Agent %d, inverse transform: \n", i);
			printf("From force(%7.3f, %7.3f, %7.3f) to raw(%7.3f, %7.3f, %7.3f)\n",
				(double)f_i(0), (double)f_i(1), (double)f_i(2),
				(double)raw(0), (double)raw(1), (double)raw(2));
			#endif // CA_CGI_DEBUGGER

			// check if saturated, raw <= saturated controls
			// TODO: the maximum thrust (f_max) must be considered
			bool legal = true;
			for (int k=0; k<3 && legal; k++) {
				if ( raw(k) < _lower(i, k) ) {
					#ifdef CA_CGI_DEBUGGER
					printf("- Agent %d, entry %d violate lower constraints: \n", i, k);
					printf("- Raw ( %7.3f ) < Lower ( %7.3f ) \n", (double)raw(k), (double)_lower(i, k));
					#endif // CA_CGI_DEBUGGER
					legal = false;
					raw(k) = _lower(i, k);
				}
				else if ( _upper(i, k) < raw(k) ) {
					#ifdef CA_CGI_DEBUGGER
					printf("- Agent %d, entry %d violate upper constraints: \n", i, k);
					printf("- Raw ( %7.3f ) > Upper ( %7.3f ) \n", (double)raw(k), (double)_upper(i, k));
					#endif // CA_CGI_DEBUGGER
					legal = false;
					raw(k) = _upper(i, k);
				}
			}
			// skip if this agent is not saturated
			if (legal) continue;

			// forward_transform: raw -> f_i_sat
			forward_transform(raw, f_i_sat);

			const float err_norm = (f_i - f_i_sat).norm_squared();
			#ifdef CA_CGI_DEBUGGER
			printf("- Error norm: %7.3f\n", (double)err_norm);
			#endif // CA_CGI_DEBUGGER
			if (err_norm > largest_err){
				largest_err = err_norm;
				agent_idx = i;
			}
			// _f.slice<3, 1>(3*i, 0) = f_i_sat;
			f_ci = f_i_sat;
		}
	}

	return largest_err > 0;
}
