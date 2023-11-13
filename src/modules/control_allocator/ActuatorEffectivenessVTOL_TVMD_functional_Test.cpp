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
 * @file ActuatorEffectivenessVTOL_TVMD_functional_Test.cpp
 *
 * Tests for ActuatorEffectivenessVTOL_TVMD
 *
 * @author Yen-Cheng Chu <sciyen.ycc@gmail.com>
 */

#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <ActuatorEffectivenessVTOL_TVMD.hpp>
// #include <uORB/topics/obstacle_distance.h>
// #include <uORB/uORBManager.hpp>

#include <gtest/gtest.h>

class ActuatorEffectivenessTVMDParameterTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		param_control_autosave(false);
		param_reset_all();
	}
};


TEST_F(ActuatorEffectivenessTVMDParameterTest, testParamReadWrite)
{
	ActuatorEffectiveness *act_eff = nullptr;
	act_eff = new ActuatorEffectivenessVTOL_TVMD(NULL);

	ActuatorEffectiveness::Configuration conf{};
	act_eff->getEffectivenessMatrix(conf, EffectivenessUpdateReason::CONFIGURATION_UPDATE);

	// Check effectiveness
	const float expected[ActuatorEffectiveness::NUM_AXES][ActuatorEffectiveness::NUM_ACTUATORS] = {
		{ 0.0000f, 	 0.0000f, 	 0.3536f, 	 0.0000f, 	 0.0000f, 	-0.3536f, 	 0.0000f, 	 0.0000f, 	 0.3536f, 	 0.0000f, 	 0.0000f, 	-0.3536f, 	 0.0000f, 	 0.0000f, 	 0.0000f, 	 0.0000f },
		{ 0.0000f, 	 0.0000f, 	-0.3536f, 	 0.0000f, 	 0.0000f, 	-0.3536f, 	 0.0000f, 	 0.0000f, 	 0.3536f, 	 0.0000f, 	 0.0000f, 	 0.3536f, 	 0.0000f, 	 0.0000f, 	 0.0000f, 	 0.0000f },
		{ 0.3536f, 	 0.3536f, 	 0.0000f, 	 0.3536f, 	-0.3536f, 	 0.0000f, 	-0.3536f, 	-0.3536f, 	 0.0000f, 	 0.3536f, 	-0.3536f, 	 0.0000f, 	 0.0000f, 	 0.0000f, 	 0.0000f, 	 0.0000f },
		{ 0.0000f, 	-1.0000f, 	 0.0000f, 	 0.0000f, 	-1.0000f, 	 0.0000f, 	 1.0000f, 	-0.0000f, 	 0.0000f, 	 1.0000f, 	-0.0000f, 	 0.0000f, 	 0.0000f, 	 0.0000f, 	 0.0000f, 	 0.0000f },
		{ 1.0000f, 	 0.0000f, 	 0.0000f, 	 1.0000f, 	 0.0000f, 	 0.0000f, 	 0.0000f, 	 1.0000f, 	 0.0000f, 	 0.0000f, 	 1.0000f, 	 0.0000f, 	 0.0000f, 	 0.0000f, 	 0.0000f, 	 0.0000f },
		{ 0.0000f, 	 0.0000f, 	 1.0000f, 	 0.0000f, 	 0.0000f, 	 1.0000f, 	 0.0000f, 	 0.0000f, 	 1.0000f, 	 0.0000f, 	 0.0000f, 	 1.0000f, 	 0.0000f, 	 0.0000f, 	 0.0000f, 	 0.0000f },
	};
	ActuatorEffectiveness::EffectivenessMatrix expected_effectiveness(expected);
	EXPECT_EQ(conf.effectiveness_matrices[0],
		expected_effectiveness);

	// // GIVEN a parameter handle
	// param_t param = param_handle(px4::params::CP_DIST);

	// // WHEN: we get the parameter
	// float value = -999.f;
	// int status = param_get(param, &value);

	// // THEN it should be successful and have the default value
	// EXPECT_EQ(0, status);
	// EXPECT_FLOAT_EQ(-1.f, value);

	// // WHEN: we set the parameter
	// value = 42.f;
	// status = param_set(param, &value);

	// // THEN: it should be successful
	// EXPECT_EQ(0, status);

	// // WHEN: we get the parameter again
	// float value2 = -1999.f;
	// status = param_get(param, &value2);

	// // THEN: it should be exactly the value we set
	// EXPECT_EQ(0, status);
	// EXPECT_FLOAT_EQ(42.f, value2);
}

