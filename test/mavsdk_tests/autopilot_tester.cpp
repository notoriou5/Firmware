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

#include "autopilot_tester.h"
#include <iostream>
#include <future>

std::string connection_url {"udp://"};

void AutopilotTester::connect(const std::string uri)
{
	std::cout << "______SITL DEBUG_____: connect ---------" << std::endl;
	ConnectionResult ret = _mavsdk.add_any_connection(uri);
	REQUIRE(ret == ConnectionResult::SUCCESS);

	std::cout << "Waiting for system connect" << std::endl;
	REQUIRE(poll_condition_with_timeout(
	[this]() { return _mavsdk.is_connected(); }, std::chrono::seconds(25)));

	auto &system = _mavsdk.system();

	_telemetry.reset(new Telemetry(system));
	_action.reset(new Action(system));
	_mission.reset(new Mission(system));
	_offboard.reset(new Offboard(system));
}

void AutopilotTester::wait_until_ready()
{
	std::cout << "Waiting for system to be ready" << std::endl;
	CHECK(poll_condition_with_timeout(
	[this]() { return _telemetry->health_all_ok(); }, std::chrono::seconds(20)));
}

void AutopilotTester::wait_until_ready_local_position_only()
{
	std::cout << "Waiting for system to be ready" << std::endl;
	CHECK(poll_condition_with_timeout(
	[this]() {
		return
			(_telemetry->health().gyrometer_calibration_ok &&
			 _telemetry->health().accelerometer_calibration_ok &&
			 _telemetry->health().magnetometer_calibration_ok &&
			 _telemetry->health().level_calibration_ok &&
			 _telemetry->health().local_position_ok);
	}, std::chrono::seconds(20)));
}

void AutopilotTester::store_home()
{
	std::cout << "______SITL DEBUG_____: store_home ---------" << std::endl;
	request_ground_truth();
	_home = get_ground_truth_position();
}

void AutopilotTester::check_home_within(float acceptance_radius_m)
{
	std::cout << "______SITL DEBUG_____: check_home_within ---------" << std::endl;
	CHECK(ground_truth_horizontal_position_close_to(_home, acceptance_radius_m));
}

void AutopilotTester::set_takeoff_altitude(const float altitude_m)
{
	std::cout << "______SITL DEBUG_____: set_takeoff_altitude ---------" << std::endl;
	CHECK(Action::Result::SUCCESS == _action->set_takeoff_altitude(altitude_m));
	const auto result = _action->get_takeoff_altitude();
	CHECK(result.first == Action::Result::SUCCESS);
	CHECK(result.second == Approx(altitude_m));
}

void AutopilotTester::arm()
{
	std::cout << "______SITL DEBUG_____: arm ---------" << std::endl;
	const auto result = _action->arm();
	REQUIRE(result == Action::Result::SUCCESS);
}

void AutopilotTester::takeoff()
{
	std::cout << "______SITL DEBUG_____: takeoff ---------" << std::endl;
	const auto result = _action->takeoff();
	REQUIRE(result == Action::Result::SUCCESS);
}

void AutopilotTester::land()
{
	std::cout << "______SITL DEBUG_____: land ---------" << std::endl;
	const auto result = _action->land();
	REQUIRE(result == Action::Result::SUCCESS);
}

void AutopilotTester::transition_to_fixedwing()
{
	std::cout << "______SITL DEBUG_____: transition_to_fixedwing ---------" << std::endl;
	const auto result = _action->transition_to_fixedwing();
	REQUIRE(result == Action::Result::SUCCESS);
}

void AutopilotTester::transition_to_multicopter()
{
	std::cout << "______SITL DEBUG_____: transition_to_multicopter ---------" << std::endl;
	const auto result = _action->transition_to_multicopter();
	REQUIRE(result == Action::Result::SUCCESS);
}

void AutopilotTester::wait_until_disarmed()
{
	std::cout << "______SITL DEBUG_____: wait_until_disarmed ---------" << std::endl;
	REQUIRE(poll_condition_with_timeout(
	[this]() { return !_telemetry->armed(); }, std::chrono::seconds(90)));
}

void AutopilotTester::wait_until_hovering()
{
	std::cout << "______SITL DEBUG_____: wait_until_hovering ---------" << std::endl;
	REQUIRE(poll_condition_with_timeout(
	[this]() { return _telemetry->landed_state() == Telemetry::LandedState::IN_AIR; }, std::chrono::seconds(60)));
}

void AutopilotTester::prepare_square_mission(MissionOptions mission_options)
{
	std::cout << "______SITL DEBUG_____: prepare_square_mission ---------" << std::endl;
	const auto ct = get_coordinate_transformation();

	std::vector<std::shared_ptr<MissionItem>> mission_items {};
	mission_items.push_back(create_mission_item({mission_options.leg_length_m, 0.}, mission_options, ct));
	mission_items.push_back(create_mission_item({mission_options.leg_length_m, mission_options.leg_length_m},
				mission_options, ct));
	mission_items.push_back(create_mission_item({0., mission_options.leg_length_m}, mission_options, ct));

	_mission->set_return_to_launch_after_mission(mission_options.rtl_at_end);

	std::promise<void> prom;
	auto fut = prom.get_future();

	_mission->upload_mission_async(mission_items, [&prom](Mission::Result result) {
		REQUIRE(Mission::Result::SUCCESS == result);
		prom.set_value();
	});

	REQUIRE(fut.wait_for(std::chrono::seconds(2)) == std::future_status::ready);
}

void AutopilotTester::prepare_square_vtol_mission(MissionOptions mission_options)
{
	std::cout << "______SITL DEBUG_____: prepare_square_vtol_mission ---------" << std::endl;
	const auto ct = get_coordinate_transformation();

	std::vector<std::shared_ptr<MissionItem>> mission_items {};
	mission_items.push_back(create_mission_item({mission_options.leg_length_m, 0.}, mission_options, ct));
	mission_items.push_back(create_mission_item({mission_options.leg_length_m, mission_options.leg_length_m},
				mission_options, ct));
	mission_items.push_back(create_mission_item({0., mission_options.leg_length_m}, mission_options, ct));
	mission_items.push_back(create_mission_item({0., 50.0f}, mission_options, ct));


	_mission->set_return_to_launch_after_mission(mission_options.rtl_at_end);

	std::promise<void> prom;
	auto fut = prom.get_future();

	_mission->upload_mission_async(mission_items, [&prom](Mission::Result result) {
		REQUIRE(Mission::Result::SUCCESS == result);
		prom.set_value();
	});

	REQUIRE(fut.wait_for(std::chrono::seconds(2)) == std::future_status::ready);
}

void AutopilotTester::prepare_N_shaped_mission(MissionOptions mission_options)
{
	std::cout << "______SITL DEBUG_____: prepare_simple_vtol_mission ---------" << std::endl;
	const auto ct = get_coordinate_transformation();

	std::vector<std::shared_ptr<MissionItem>> mission_items {};
	mission_items.push_back(create_mission_item({mission_options.leg_length_m, 0.f}, mission_options, ct));
	mission_items.push_back(create_mission_item({0.f, mission_options.leg_length_m}, mission_options, ct));
	mission_items.push_back(create_mission_item({mission_options.leg_length_m, mission_options.leg_length_m}, mission_options, ct));
	_mission->set_return_to_launch_after_mission(mission_options.rtl_at_end);

	std::promise<void> prom;
	auto fut = prom.get_future();

	_mission->upload_mission_async(mission_items, [&prom](Mission::Result result) {
		REQUIRE(Mission::Result::SUCCESS == result);
		prom.set_value();
	});

	REQUIRE(fut.wait_for(std::chrono::seconds(2)) == std::future_status::ready);
}

void AutopilotTester::execute_mission()
{
	std::cout << "______SITL DEBUG_____: execute_mission ---------" << std::endl;
	std::promise<void> prom;
	auto fut = prom.get_future();

	_mission->start_mission_async([&prom](Mission::Result result) {
		REQUIRE(Mission::Result::SUCCESS == result);
		prom.set_value();
	});

	// TODO: Adapt time limit based on mission size, flight speed, sim speed factor, etc.

	REQUIRE(poll_condition_with_timeout(
	[this]() { return _mission->mission_finished(); }, std::chrono::seconds(180)));

	REQUIRE(fut.wait_for(std::chrono::seconds(1)) == std::future_status::ready);
}

void AutopilotTester::pause_mission()
{
	std::cout << "______SITL DEBUG_____: pause_mission ---------" << std::endl;
	std::promise<void> prom;
	auto fut = prom.get_future();


	_mission->pause_mission_async([&prom](Mission::Result result) {
		REQUIRE(Mission::Result::SUCCESS == result);
		prom.set_value();
	});

	REQUIRE(fut.wait_for(std::chrono::seconds(1)) == std::future_status::ready);
}

void AutopilotTester::set_mission_item()
{
	std::cout << "______SITL DEBUG_____: set_mission_item ---------" << std::endl;
	std::promise<void> prom;
	auto fut = prom.get_future();

	_mission->set_current_mission_item_async(3, [&prom](Mission::Result result) {
		REQUIRE(Mission::Result::SUCCESS == result);
		prom.set_value();
	});

	REQUIRE(fut.wait_for(std::chrono::seconds(1)) == std::future_status::ready);
}

void AutopilotTester::execute_specific_mission_item()
{
	std::cout << "______SITL DEBUG_____: execute_specific_mission_item ---------" << std::endl;
	std::promise<void> prom;
	auto fut = prom.get_future();

	_mission->start_mission_async([&prom](Mission::Result result) {
		REQUIRE(Mission::Result::SUCCESS == result);
		prom.set_value();
	});

	// TODO: Adapt time limit based on mission size, flight speed, sim speed factor, etc.

	REQUIRE(poll_condition_with_timeout(
	[this]() { return (_mission->current_mission_item() == 2); }, std::chrono::seconds(20)));

	REQUIRE(fut.wait_for(std::chrono::seconds(1)) == std::future_status::ready);
}

CoordinateTransformation AutopilotTester::get_coordinate_transformation()
{
	const auto home = _telemetry->home_position();
	CHECK(std::isfinite(home.latitude_deg));
	CHECK(std::isfinite(home.longitude_deg));
	return CoordinateTransformation({home.latitude_deg, home.longitude_deg});
}

std::shared_ptr<MissionItem>  AutopilotTester::create_mission_item(
	const CoordinateTransformation::LocalCoordinate &local_coordinate,
	const MissionOptions &mission_options,
	const CoordinateTransformation &ct)
{
	std::cout << "______SITL DEBUG_____: create_mission_item ---------" << std::endl;
	auto mission_item = std::make_shared<MissionItem>();
	const auto pos_north = ct.global_from_local(local_coordinate);
	mission_item->set_position(pos_north.latitude_deg, pos_north.longitude_deg);
	mission_item->set_relative_altitude(mission_options.relative_altitude_m);
	std::cout << "mission item:" << pos_north.latitude_deg << ", "<< pos_north.longitude_deg << ", " << mission_options.relative_altitude_m << std::endl;
	return mission_item;
}

void AutopilotTester::execute_rtl()
{
	std::cout << "______SITL DEBUG_____: execute_rtl ---------" << std::endl;
	REQUIRE(Action::Result::SUCCESS == _action->return_to_launch());
}

void AutopilotTester::offboard_goto(const Offboard::PositionNEDYaw &target, float acceptance_radius_m,
				    std::chrono::seconds timeout_duration)
{
	_offboard->set_position_ned(target);
	REQUIRE(_offboard->start() == Offboard::Result::SUCCESS);
	CHECK(poll_condition_with_timeout(
	[ = ]() { return estimated_position_close_to(target, acceptance_radius_m); }, timeout_duration));
	std::cout << "Target position reached" << std::endl;
}

void AutopilotTester::offboard_land()
{
	Offboard::VelocityNEDYaw land_velocity;
	land_velocity.north_m_s = 0.0f;
	land_velocity.east_m_s = 0.0f;
	land_velocity.down_m_s = 1.0f;
	land_velocity.yaw_deg = 0.0f;
	_offboard->set_velocity_ned(land_velocity);
}

bool AutopilotTester::estimated_position_close_to(const Offboard::PositionNEDYaw &target_pos, float acceptance_radius_m)
{
	Telemetry::PositionNED est_pos = _telemetry->position_velocity_ned().position;
	return sq(est_pos.north_m - target_pos.north_m) +
	       sq(est_pos.east_m - target_pos.east_m) +
	       sq(est_pos.down_m - target_pos.down_m)  < sq(acceptance_radius_m);
}

bool AutopilotTester::estimated_horizontal_position_close_to(const Offboard::PositionNEDYaw &target_pos,
		float acceptance_radius_m)
{
	Telemetry::PositionNED est_pos = _telemetry->position_velocity_ned().position;
	return sq(est_pos.north_m - target_pos.north_m) +
	       sq(est_pos.east_m - target_pos.east_m) < sq(acceptance_radius_m);
}

void AutopilotTester::request_ground_truth()
{
	CHECK(_telemetry->set_rate_ground_truth(15) == Telemetry::Result::SUCCESS);
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

Telemetry::GroundTruth AutopilotTester::get_ground_truth_position()
{
	return _telemetry->ground_truth();
}

bool AutopilotTester::ground_truth_horizontal_position_close_to(const Telemetry::GroundTruth &target_pos,
		float acceptance_radius_m)
{
	CHECK(std::isfinite(target_pos.latitude_deg));
	CHECK(std::isfinite(target_pos.longitude_deg));
	using GlobalCoordinate = CoordinateTransformation::GlobalCoordinate;
	using LocalCoordinate = CoordinateTransformation::LocalCoordinate;
	CoordinateTransformation ct(GlobalCoordinate{target_pos.latitude_deg, target_pos.longitude_deg});

	Telemetry::GroundTruth current_pos = _telemetry->ground_truth();
	CHECK(std::isfinite(current_pos.latitude_deg));
	CHECK(std::isfinite(current_pos.longitude_deg));
	LocalCoordinate local_pos = ct.local_from_global(GlobalCoordinate{current_pos.latitude_deg, current_pos.longitude_deg});

	return sq(local_pos.north_m) + sq(local_pos.east_m) < sq(acceptance_radius_m);
}
