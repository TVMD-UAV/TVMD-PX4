/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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
 * @file ControlAllocationTest.cpp
 *
 * Tests for Control Allocation Algorithms
 *
 * @author Yen-Cheng Chu <sciyen.ycc@gmail.com>
 */
#include <stdio.h>

#include <gtest/gtest.h>
#include <ControlAllocationCGI.hpp>

using namespace matrix;

TEST(ControlAllocationTest, AllZeroCase)
{
	ControlAllocationCGI method;

	matrix::Vector<float, 6> control_sp;
	matrix::Vector<float, 6> control_allocated;
	matrix::Vector<float, 6> control_allocated_expected;
	// matrix::Matrix<float, 6, 16> effectiveness;
	matrix::Vector<float, 16> actuator_sp;
	matrix::Vector<float, 16> actuator_trim;
	matrix::Vector<float, 16> linearization_point;
	matrix::Vector<float, 16> actuator_sp_expected;

	control_sp(0) = 1.1f;
	control_sp(1) = 1.1f;
	control_sp(2) = -1.1f;
	control_sp(3) = 1.1f;
	control_sp(4) = 1.1f;
	control_sp(5) = -30.0f;

	const float expected[ActuatorEffectiveness::NUM_AXES][ActuatorEffectiveness::NUM_ACTUATORS] = {
		{ 0.0000f, 	 0.0000f, 	 0.3536f, 	 0.0000f, 	 0.0000f, 	-0.3536f, 	 0.0000f, 	 0.0000f, 	 0.3536f, 	 0.0000f, 	 0.0000f, 	-0.3536f, 	 0.0000f, 	 0.0000f, 	 0.0000f, 	 0.0000f },
		{ 0.0000f, 	 0.0000f, 	-0.3536f, 	 0.0000f, 	 0.0000f, 	-0.3536f, 	 0.0000f, 	 0.0000f, 	 0.3536f, 	 0.0000f, 	 0.0000f, 	 0.3536f, 	 0.0000f, 	 0.0000f, 	 0.0000f, 	 0.0000f },
		{ 0.3536f, 	 0.3536f, 	 0.0000f, 	 0.3536f, 	-0.3536f, 	 0.0000f, 	-0.3536f, 	-0.3536f, 	 0.0000f, 	 0.3536f, 	-0.3536f, 	 0.0000f, 	 0.0000f, 	 0.0000f, 	 0.0000f, 	 0.0000f },
		{ 0.0000f, 	-1.0000f, 	 0.0000f, 	 0.0000f, 	-1.0000f, 	 0.0000f, 	 1.0000f, 	-0.0000f, 	 0.0000f, 	 1.0000f, 	-0.0000f, 	 0.0000f, 	 0.0000f, 	 0.0000f, 	 0.0000f, 	 0.0000f },
		{ 1.0000f, 	 0.0000f, 	 0.0000f, 	 1.0000f, 	 0.0000f, 	 0.0000f, 	 0.0000f, 	 1.0000f, 	 0.0000f, 	 0.0000f, 	 1.0000f, 	 0.0000f, 	 0.0000f, 	 0.0000f, 	 0.0000f, 	 0.0000f },
		{ 0.0000f, 	 0.0000f, 	 1.0000f, 	 0.0000f, 	 0.0000f, 	 1.0000f, 	 0.0000f, 	 0.0000f, 	 1.0000f, 	 0.0000f, 	 0.0000f, 	 1.0000f, 	 0.0000f, 	 0.0000f, 	 0.0000f, 	 0.0000f },
	};
	ActuatorEffectiveness::EffectivenessMatrix effectiveness(expected);

	control_allocated = control_sp;

	method.setEffectivenessMatrix(effectiveness, actuator_trim, linearization_point, 16, false);
	method.setControlSetpoint(control_sp);
	method.allocate();
	// method.clipActuatorSetpoint();
	actuator_sp = method.getActuatorSetpoint();
	control_allocated_expected = method.getAllocatedControl();

	const float coord_trans_f[6] = {1.0f, -1.0f, -1.0f, 1.0f, -1.0f, -1.0f};
	const ControlAllocationCGI::ControlVector coord_trans(coord_trans_f);
	control_allocated_expected = control_allocated_expected.emult(coord_trans);

	std::cout << control_sp << std::endl;
	std::cout << control_allocated_expected << std::endl;
	std::cout << actuator_sp << std::endl;

	// EXPECT_EQ(actuator_sp, actuator_sp_expected);
	EXPECT_EQ(control_allocated, control_allocated_expected);
}
