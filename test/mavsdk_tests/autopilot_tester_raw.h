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

#pragma once

#include <mavsdk/mavsdk.h>
#include <mavsdk/geometry.h>
#include <mavsdk/plugins/mission_raw/mission_raw.h>
#include "catch2/catch.hpp"
#include <chrono>
#include <memory>
#include <thread>

extern std::string connection_url;

using namespace mavsdk;
using namespace mavsdk::geometry;

class AutopilotTester
{
public:

	void connect(const std::string uri);
	void upload_mission();
	void execute_mission();
	void prepare_simple_mission_raw();


private:

	mavsdk::Mavsdk _mavsdk{};
	std::unique_ptr<mavsdk::MissionRaw> _mission_raw{};

	std::vector<MissionRaw::MissionItem> _mission_raw_items;


};

template<typename Rep, typename Period>
bool poll_condition_with_timeout(
	std::function<bool()> fun, std::chrono::duration<Rep, Period> duration)
{
	// We need millisecond resolution for sleeping.
	const std::chrono::milliseconds duration_ms(duration);

	unsigned iteration = 0;

	while (!fun()) {
		std::this_thread::sleep_for(duration_ms / 10);

		if (iteration++ >= 10) {
			return false;
		}
	}

	return true;
}

inline float sq(float x) { return x * x; };
