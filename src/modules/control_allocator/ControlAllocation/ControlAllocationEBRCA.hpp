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
 * Exact Bundle Redistributed Control Allocation
 *
 *
 * @author Yen-Cheng Chu <sciyen.ycc@gmail.com>
 */

#pragma once
#include <stdio.h>
#include <math.h>

#include "ControlAllocation.hpp"
#include "ControlAllocationModularBundled.hpp"
#include <uORB/Publication.hpp>
#include <uORB/topics/control_allocation_meta_data.h>

// #define CA_EBRCA_DEBUGGER
// #define CA_EBRCA_ENABLE_PBP

class ControlAllocationEBRCA: public ControlAllocationModularBundled
{
public:
	ControlAllocationEBRCA();

	virtual ~ControlAllocationEBRCA() = default;

	void calcualte_bundled_pseudo_inverse(ControlVector &u_in) override;

protected:
	float calc_saturated_agent_id(
		int8_t &sat_idx, PseudoForceVector f0, PseudoForceVector f1);

private:
	uORB::Publication<control_allocation_meta_data_s> _control_allocation_meta_data_pub{ORB_ID(control_allocation_meta_data)};

	const float _epsilon = 1e-20f;
	void inverse_transform_on_tangent_plane(
		matrix::Vector3f &raw, const matrix::Vector3f &f0_i, const matrix::Vector3f &f_delta_i) const;

	float solve_intersections_x_axis(const matrix::Vector3f &f0_i, const matrix::Vector3f &f_delta_i, const float & y_bound);

	float solve_intersections_y_axis(const matrix::Vector3f &f0_i, const matrix::Vector3f &f_delta_i, const float & x_bound);

	float solve_intersections_z_axis(const matrix::Vector3f &f0_i, const matrix::Vector3f &f_delta_i, const float & z_bound);

	inline float check_negative(const float &x) const {
		return (x < 0.0f) ? INFINITY : x;
	}

	control_allocation_meta_data_s _meta_data;
	uint8_t _iter{0};
	uint8_t inline _idx(uint8_t aid) const {
		return _iter * NUM_MODULES + aid;
	}
};
