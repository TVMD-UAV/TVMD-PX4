/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
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
 * @file ActuatorEffectivenessVTOL_TVMD.cpp
 *
 * Tests for ActuatorEffectivenessVTOL_TVMD
 *
 * @author Yen-Cheng Chu <sciyen.ycc@gmail.com>
 */

#include <stdio.h>

#include <gtest/gtest.h>
#include <ActuatorEffectivenessVTOL_TVMD.hpp>
#include "../ControlAllocation/ControlAllocationCGI.hpp"

using namespace matrix;

TEST(ActuatorEffectivenessVTOL_TVMD, AllZeroCase)
{
	float grid_size = 0.5f * std::sqrt(2.0f) / 2.0f;
	// const float grid_size = 0.215f;

	ActuatorEffectivenessVTOL_TVMD::Geometry geometry = {};
	// Manually update geometry configurations
	{
		geometry.num_agents = 4;

		geometry.module_geometry[0].position(0) = 1.0f * grid_size;
		geometry.module_geometry[0].position(1) = 1.0f * grid_size;
		geometry.module_geometry[0].position(2) = 0.0f;
		geometry.module_geometry[0].ax_psi = 1;

		geometry.module_geometry[1].position(0) = 1.0f * grid_size;
		geometry.module_geometry[1].position(1) =-1.0f * grid_size;
		geometry.module_geometry[1].position(2) = 0.0f;
		geometry.module_geometry[1].ax_psi = 1;

		geometry.module_geometry[2].position(0) =-1.0f * grid_size;
		geometry.module_geometry[2].position(1) = 1.0f * grid_size;
		geometry.module_geometry[2].position(2) = 0.0f;
		geometry.module_geometry[2].ax_psi = 0;

		geometry.module_geometry[3].position(0) =-1.0f * grid_size;
		geometry.module_geometry[3].position(1) =-1.0f * grid_size;
		geometry.module_geometry[3].position(2) = 0.0f;
		geometry.module_geometry[3].ax_psi = 0;

		for (int k=0; k<4; ++k) {
			geometry.module_geometry[k].gear_ratio[0] = 1.0f;
			geometry.module_geometry[k].gear_ratio[1] = 1.0f;
			geometry.module_geometry[k].motor_conf[0](0) =  50.0f;         // rate
			geometry.module_geometry[k].motor_conf[0](1) =  0.020231f;     // Cl
			geometry.module_geometry[k].motor_conf[0](2) =  0.0188f;       // Cd
			geometry.module_geometry[k].motor_conf[1](0) =  50.0f;         // rate
			geometry.module_geometry[k].motor_conf[1](1) =  0.020231f;     // Cl
			geometry.module_geometry[k].motor_conf[1](2) =  0.0188f;       // Cd

			// unit in radian (note, the conversion from degree to radian only happen when loading data from ROM)
			geometry.module_geometry[k].servo_conf[0](0) =  50 * M_PI_F / 6.0f;  // rate
			geometry.module_geometry[k].servo_conf[0](1) = -M_PI_F / 6.0f;       // min
			geometry.module_geometry[k].servo_conf[0](2) =  M_PI_F / 6.0f;       // max
			geometry.module_geometry[k].servo_conf[1](0) =  50 * M_PI_F / 2.0f;  // rate
			geometry.module_geometry[k].servo_conf[1](1) = -M_PI_F / 2.0f;       // min
			geometry.module_geometry[k].servo_conf[1](2) =  M_PI_F / 2.0f;       // max
		}
	}

	ControlAllocationCGI ca_method;
	ActuatorEffectivenessVTOL_TVMD act_eff(NULL);
	act_eff.setGeometry(geometry);

	// Configuring
	ActuatorEffectiveness::Configuration conf{};
	act_eff.getEffectivenessMatrix(conf, EffectivenessUpdateReason::CONFIGURATION_UPDATE);

	// Setup effectiveness matrix in CA
	ControlAllocation::ActuatorVector trim, linearization_point;
	trim.setZero();
	linearization_point.setZero();
	ca_method.setEffectivenessMatrix(conf.effectiveness_matrices[0], trim, linearization_point, 16, false);

	const float control_sp_f[6] = {0.5f, 0.5f, 0.5f, -2.0f, 2.0f, -40.0f};
	matrix::Vector<float, 6> control_sp(control_sp_f);

	ca_method.setControlSetpoint(control_sp);
	ca_method.allocate();

	ControlAllocation::ActuatorVector actuator_sp = ca_method.getActuatorSetpoint();

	std::cout << "Allocation actuator setpoints (force):" << std::endl;
	std::cout << actuator_sp << std::endl;

	act_eff.updateSetpoint(control_sp, 0, actuator_sp,
		ca_method.getActuatorMin(), ca_method.getActuatorMax());

	// ca_method.applySlewRateLimit();
	ca_method.clipActuatorSetpoint();



	// Check effectiveness
	const float expected[ActuatorEffectiveness::NUM_AXES][ActuatorEffectiveness::NUM_ACTUATORS] = {
		{ 0.0000f, 	 0.0000f, 	 0.3536f, 	 0.0000f, 	 0.0000f, 	-0.3536f, 	 0.0000f, 	 0.0000f, 	 0.3536f, 	 0.0000f, 	 0.0000f, 	-0.3536f, 	 0.0000f, 	 0.0000f, 	 0.0000f, 	 0.0000f },
		{ 0.0000f, 	 0.0000f, 	-0.3536f, 	 0.0000f, 	 0.0000f, 	-0.3536f, 	 0.0000f, 	 0.0000f, 	 0.3536f, 	 0.0000f, 	 0.0000f, 	 0.3536f, 	 0.0000f, 	 0.0000f, 	 0.0000f, 	 0.0000f },
		{ 0.3536f, 	 0.3536f, 	 0.0000f, 	 0.3536f, 	-0.3536f, 	 0.0000f, 	-0.3536f, 	-0.3536f, 	 0.0000f, 	 0.3536f, 	-0.3536f, 	 0.0000f, 	 0.0000f, 	 0.0000f, 	 0.0000f, 	 0.0000f },
		{ 0.0000f, 	-1.0000f, 	 0.0000f, 	 0.0000f, 	-1.0000f, 	 0.0000f, 	 1.0000f, 	-0.0000f, 	 0.0000f, 	 1.0000f, 	-0.0000f, 	 0.0000f, 	 0.0000f, 	 0.0000f, 	 0.0000f, 	 0.0000f },
		{ 1.0000f, 	 0.0000f, 	 0.0000f, 	 1.0000f, 	 0.0000f, 	 0.0000f, 	 0.0000f, 	 1.0000f, 	 0.0000f, 	 0.0000f, 	 1.0000f, 	 0.0000f, 	 0.0000f, 	 0.0000f, 	 0.0000f, 	 0.0000f },
		{ 0.0000f, 	 0.0000f, 	 1.0000f, 	 0.0000f, 	 0.0000f, 	 1.0000f, 	 0.0000f, 	 0.0000f, 	 1.0000f, 	 0.0000f, 	 0.0000f, 	 1.0000f, 	 0.0000f, 	 0.0000f, 	 0.0000f, 	 0.0000f },
	};
	ActuatorEffectiveness::EffectivenessMatrix effectivenss = ca_method.getEffectivenessMatrix();
	ActuatorEffectiveness::EffectivenessMatrix expected_effectiveness(expected);
	EXPECT_EQ(effectivenss, expected_effectiveness);
	std::cout << "Effectiveness:" << std::endl;
	std::cout << effectivenss << std::endl;


	// From actuator setpoints to body force
	std::cout << "Allocation actuator setpoints (raw):" << std::endl;
	std::cout << actuator_sp << std::endl;

	// calculate scales
	matrix::Vector2f max_tftd;
	const matrix::Vector2f max_u_prop(1, 1);
	act_eff.tf_mapping(0, max_tftd, max_u_prop);
	std::cout << "Maximum Thrust Force (N) and Torque (Nm):" << std::endl;
	std::cout << max_tftd << std::endl;

	matrix::Vector2f motor_scale;
	matrix::Vector2f servo_scale;
	for (int j=0; j<2; ++j) {
		const float servo_max = geometry.module_geometry[0].servo_conf[j](2);
		const float servo_min = geometry.module_geometry[0].servo_conf[j](1);
		motor_scale(j) = 1.0f;
		servo_scale(j) = (1.0f - (-1.0f)) / (servo_max - servo_min);
	}

	std::cout << "Calculated Scales: \n";
	std::cout << "Motor scales: " << motor_scale << std::endl;
	std::cout << "Servo scales: " << servo_scale << std::endl;

	ControlAllocationCGI::ControlVector body_wrench;
	body_wrench.setZero();
	for (int i=0; i<4; ++i) {
		const matrix::Matrix<float, 6, 3> eff_slice = effectivenss.slice<6, 3>(0, 3*i);
		matrix::Vector2f motor_sp = actuator_sp.slice<2, 1>(2*i, 0);
		matrix::Vector2f servo_sp = actuator_sp.slice<2, 1>(2*i+8, 0);

		// re-scaling: from normalized to actual
		motor_sp = motor_sp.edivide(motor_scale);
		servo_sp = servo_sp.edivide(servo_scale);
		printf("Scaled controls for agent %d:", i);
		printf("%7.3f\t %7.3f\t %7.3f\t %7.3f\n", (double)motor_sp(0), (double)motor_sp(1), (double)servo_sp(0), (double)servo_sp(1));

		// mapping to tf td
		matrix::Vector2f tftd;
		act_eff.tf_mapping(i, tftd, motor_sp);

		// mapping to individual control
		const matrix::Vector3f raw(servo_sp(0), servo_sp(1), tftd(0));

		// mapping to force vector
		matrix::Vector3f f_i;
		ControlAllocationCGI::forward_transform(raw, f_i);

		body_wrench += eff_slice * f_i;
		std::cout << ", force: \t" << f_i << std::endl;
	}

	const float coord_trans_f[6] = {1.0f, -1.0f, -1.0f, 1.0f, -1.0f, -1.0f};
	const ControlAllocationCGI::ControlVector coord_trans(coord_trans_f);
	body_wrench = body_wrench.emult(coord_trans);

	// forward transformation
	EXPECT_EQ(body_wrench, control_sp);
}
