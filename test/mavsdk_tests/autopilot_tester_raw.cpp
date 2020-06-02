/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include "autopilot_tester_raw.h"
#include <iostream>
#include <future>

std::string connection_url {"udp://"};

void AutopilotTester::connect(const std::string uri)
{
	std::cout << "______SITL DEBUG_____: connect ---------" << std::endl;
	ConnectionResult ret = _mavsdk.add_any_connection(uri);
	REQUIRE(ret == ConnectionResult::Success);

	std::cout << "Waiting for system connect" << std::endl;
	REQUIRE(poll_condition_with_timeout(
	[this]() { return _mavsdk.is_connected(); }, std::chrono::seconds(25)));

	auto &system = _mavsdk.system();

	_mission_raw.reset(new MissionRaw(system));

}

void AutopilotTester::upload_mission()
{
	std::cout << "______SITL DEBUG_____: upload_mission ---------" << std::endl;
	std::promise<void> prom;
	auto fut = prom.get_future();

	_mission_raw->upload_mission_async(_mission_raw_items, [&prom](MissionRaw::Result result) {
	    REQUIRE(MissionRaw::Result::Success == result);
            prom.set_value();
        });

	// TODO: Adapt time limit based on mission size, flight speed, sim speed factor, etc.

	// REQUIRE(poll_condition_with_timeout(
	// [this]() { return _mission->mission_finished(); }, std::chrono::seconds(180)));

	REQUIRE(fut.wait_for(std::chrono::seconds(1)) == std::future_status::ready);
}

void AutopilotTester::execute_mission()
{
	std::cout << "______SITL DEBUG_____: execute_mission ---------" << std::endl;
	std::promise<void> prom;
	auto fut = prom.get_future();

	_mission_raw->start_mission_async([&prom](MissionRaw::Result result) {
	    REQUIRE(MissionRaw::Result::Success == result);
            prom.set_value();
        });

	// TODO: Adapt time limit based on mission size, flight speed, sim speed factor, etc.

	// REQUIRE(poll_condition_with_timeout(
	// [this]() { return _mission->mission_finished(); }, std::chrono::seconds(180)));

	REQUIRE(fut.wait_for(std::chrono::seconds(1)) == std::future_status::ready);
}

void AutopilotTester::prepare_simple_mission_raw()
{
	std::cout << "______SITL DEBUG_____: prepare_simple_mission_raw ---------" << std::endl;

	MissionRaw::MissionItem new_raw_item{};
	new_raw_item.seq = 0;
	new_raw_item.frame = 6; // MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
	new_raw_item.command = 16; // MAV_CMD_NAV_WAYPOINT
	new_raw_item.current = 0;
	new_raw_item.autocontinue = 1;
	new_raw_item.param1 = 1.0; // Hold
	new_raw_item.param2 = 1.0; // Accept Radius
	new_raw_item.param3 = 1.0; // Pass Radius
	new_raw_item.param4 = NAN; // Yaw
	new_raw_item.x = int32_t(std::round(47.398170 * 1e7));
	new_raw_item.y = int32_t(std::round(8.545649 * 1e7));
	new_raw_item.z = 5.0f;
	new_raw_item.mission_type = 0; // MAV_MISSION_TYPE_MISSION

        _mission_raw_items.push_back(new_raw_item);

	new_raw_item.seq = 1;
	new_raw_item.frame = 6; // MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
	new_raw_item.command = 16; // MAV_CMD_NAV_WAYPOINT
	new_raw_item.current = 0;
	new_raw_item.autocontinue = 1;
	new_raw_item.param1 = 1.0; // Hold
	new_raw_item.param2 = 1.0; // Accept Radius
	new_raw_item.param3 = 1.0; // Pass Radius
	new_raw_item.param4 = NAN; // Yaw
	new_raw_item.x = int32_t(std::round(47.398170 * 1e7));
	new_raw_item.y = int32_t(std::round(8.545649 * 1e7));
	new_raw_item.z = 15.0f;
	new_raw_item.mission_type = 0; // MAV_MISSION_TYPE_MISSION

        _mission_raw_items.push_back(new_raw_item);

    	_mission_raw_items[0].current = 1;
}
