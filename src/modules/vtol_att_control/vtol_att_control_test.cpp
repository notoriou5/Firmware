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

#include <gtest/gtest.h>
#include <vtol_att_control/tiltrotor.h>
#include <vtol_att_control/vtol_att_control_main.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>

class VtolAttitudeControlTest : public ::testing::Test
{
public:

	VtolAttitudeControl _vtol_att_control;

	vehicle_attitude_setpoint_s att_sp{};
	vehicle_local_position_s local_pos{};

	void SetUp() override
	{
		param_reset_all();
	}
};


class TestTiltrotor : public Tiltrotor
{
public:

	TestTiltrotor(VtolAttitudeControl *att_control) :
		Tiltrotor(att_control) {};

	float get_thrust_transition() { return this->_thrust_transition; };

	vehicle_attitude_setpoint_s get_att_sp() { return *this->_v_att_sp; };
	vehicle_local_position_s get_local_pos() { return *this->_local_pos; };
};

TEST_F(VtolAttitudeControlTest, setupTest)
{
	TestTiltrotor tiltrotor(_vtol_att_control);

	std::cout << "DEBUG ------------" << std::endl;

	// this data can be accessed
	float thrust = tiltrotor.get_thrust_transition();
	std::cout << "thrust: " << thrust << std::endl;

	// this results in a Seg. Fault!
	local_pos = tiltrotor.get_local_pos();
	std::cout << "v_x: " << local_pos.vx << std::endl;

	EXPECT_FLOAT_EQ(0.0f, 1.0f);
}
