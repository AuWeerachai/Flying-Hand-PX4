/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#include <gtest/gtest.h>
#include "ActuatorEffectivenessMultirotor6Dof.hpp"

TEST(ActuatorEffectivenessMultirotor6Dof, SmallCommandsKeepArmedIdle)
{
	// Set geometry before constructing the ModuleParams subscriber.
	param_control_autosave(false);
	const int32_t count = 6;
	ASSERT_EQ(param_set(param_find("CA_ROTOR_COUNT"), &count), 0);
	ActuatorEffectivenessMultirotor6Dof aerial(nullptr);
	ActuatorEffectiveness::Configuration configuration{};
	ASSERT_TRUE(aerial.getEffectivenessMatrix(configuration, EffectivenessUpdateReason::CONFIGURATION_UPDATE));
	ASSERT_EQ(configuration.num_actuators[0], 6);
	ActuatorEffectiveness::ActuatorVector actuator_sp, minimum, maximum;
	actuator_sp.setAll(.5f);
	minimum.setAll(0.f);
	maximum.setAll(1.f);

	for (const float command : {-.019f, 0.f, .005f, .019f, .021f}) {
		actuator_sp(5) = command;
		aerial.updateSetpoint({}, 0, actuator_sp, minimum, maximum);
		EXPECT_EQ(aerial.getStoppedMotors(), 0u);
		EXPECT_FLOAT_EQ(actuator_sp(5), command);
		EXPECT_TRUE(actuator_sp.isAllFinite());
	}
}

TEST(ActuatorEffectivenessMultirotor6Dof, UnderwaterStopBehaviorUnchanged)
{
	param_control_autosave(false);
	const int32_t count = 6;
	ASSERT_EQ(param_set(param_find("CA_ROTOR_COUNT"), &count), 0);
	ActuatorEffectivenessUUV underwater(nullptr);
	ActuatorEffectiveness::Configuration configuration{};
	ASSERT_TRUE(underwater.getEffectivenessMatrix(configuration, EffectivenessUpdateReason::CONFIGURATION_UPDATE));
	ActuatorEffectiveness::ActuatorVector actuator_sp, minimum, maximum;
	actuator_sp.setAll(.5f);
	minimum.setAll(0.f);
	maximum.setAll(1.f);
	actuator_sp(5) = .019f;
	underwater.updateSetpoint({}, 0, actuator_sp, minimum, maximum);
	EXPECT_EQ(underwater.getStoppedMotors(), 1u << 5);
	actuator_sp(5) = .021f;
	underwater.updateSetpoint({}, 0, actuator_sp, minimum, maximum);
	EXPECT_EQ(underwater.getStoppedMotors(), 0u);
}
