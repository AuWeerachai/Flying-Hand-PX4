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

/**
 * Test the mag fusion
 */

#include <gtest/gtest.h>
#include "EKF/ekf.h"
#include "sensor_simulator/sensor_simulator.h"
#include "sensor_simulator/ekf_wrapper.h"
#include "test_helper/reset_logging_checker.h"
#include <lib/world_magnetic_model/geo_mag_declination.h>

class EkfMagTest : public ::testing::Test
{
public:

	EkfMagTest(): ::testing::Test(),
		_ekf{std::make_shared<Ekf>()},
		_sensor_simulator(_ekf),
		_ekf_wrapper(_ekf) {};

	std::shared_ptr<Ekf> _ekf;
	SensorSimulator _sensor_simulator;
	EkfWrapper _ekf_wrapper;

	// Setup the Ekf with synthetic measurements
	void SetUp() override
	{
		// Init, then manually set in air and at rest (default for a real vehicle)
		_ekf->init(0);
		_ekf->set_in_air_status(false);
		_ekf->set_vehicle_at_rest(true);
	}

	const uint32_t _init_duration_s{6};
};

TEST_F(EkfMagTest, fusionStartWithReset)
{
	_ekf->set_min_required_gps_health_time(5e6);
	// GIVEN: some meaningful mag data
	const float mag_heading = M_PI_F / 3.f;
	const float incl = 63.1f;
	const Vector3f mag_data(0.2f * cosf(mag_heading), -0.2f * sinf(mag_heading),
				0.4f * sinf(incl) * sqrtf(0.2f * 0.2f + 0.4f * 0.4f));
	_sensor_simulator._mag.setData(mag_data);

	const int initial_quat_reset_counter = _ekf_wrapper.getQuaternionResetCounter();
	_sensor_simulator.runSeconds(_init_duration_s);

	// THEN: the fusion initializes using the mag data and runs normally
	EXPECT_NEAR(_ekf_wrapper.getYawAngle(), mag_heading, radians(1.f));
	EXPECT_TRUE(_ekf_wrapper.isIntendingMagHeadingFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingMag3DFusion());

	EXPECT_EQ(_ekf_wrapper.getQuaternionResetCounter(), initial_quat_reset_counter + 1);
	// AND WHEN: GNSS fusion starts
	_ekf_wrapper.enableGpsFusion();
	_sensor_simulator.startGps();
	_sensor_simulator.runSeconds(6);

	// THEN: the earth mag field is reset to the WMM
	EXPECT_EQ(_ekf_wrapper.getQuaternionResetCounter(), initial_quat_reset_counter + 2);

	Vector3f mag_earth = _ekf->getMagEarthField();
	float mag_decl = atan2f(mag_earth(1), mag_earth(0));
	float mag_decl_wmm_deg = 0.f;
	_ekf->get_mag_decl_deg(mag_decl_wmm_deg);
	EXPECT_NEAR(degrees(mag_decl), mag_decl_wmm_deg, 1e-5f);

	float mag_incl = asinf(mag_earth(2) / fmaxf(mag_earth.norm(), 1e-4f));
	float mag_incl_wmm_deg = 0.f;
	_ekf->get_mag_inc_deg(mag_incl_wmm_deg);
	EXPECT_NEAR(degrees(mag_incl), mag_incl_wmm_deg, 1e-5f);
}

TEST_F(EkfMagTest, noInitLargeStrength)
{
	// GIVEN: a really large magnetic field
	_ekf_wrapper.enableMagStrengthCheck();
	const Vector3f mag_data(1.f, 1.f, 1.f);
	_sensor_simulator._mag.setData(mag_data);

	const int initial_quat_reset_counter = _ekf_wrapper.getQuaternionResetCounter();
	_sensor_simulator.runSeconds(_init_duration_s);

	// THEN: the fusion shouldn't start
	EXPECT_FALSE(_ekf_wrapper.isIntendingMagHeadingFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingMag3DFusion());
	EXPECT_EQ(0, (int) _ekf->control_status_flags().yaw_align);
	EXPECT_EQ(_ekf_wrapper.getQuaternionResetCounter(), initial_quat_reset_counter);
}

TEST_F(EkfMagTest, suddenLargeStrength)
{
	_ekf_wrapper.enableMagStrengthCheck();

	// GIVEN: some meaningful mag data
	const float mag_heading = -M_PI_F / 7.f;
	Vector3f mag_data(0.2f * cosf(mag_heading), -0.2f * sinf(mag_heading), 0.4f);
	_sensor_simulator._mag.setData(mag_data);

	_sensor_simulator.runSeconds(_init_duration_s);

	// THEN: the fusion initializes using the mag data and runs normally
	EXPECT_NEAR(_ekf_wrapper.getYawAngle(), mag_heading, radians(1.f));
	EXPECT_TRUE(_ekf_wrapper.isIntendingMagHeadingFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingMag3DFusion());

	// BUT WHEN: the mag field norm is suddenly too large
	mag_data *= 5.f;
	_sensor_simulator._mag.setData(mag_data);
	_sensor_simulator.runSeconds(6.f);

	// THEN: the mag fusion should stop after some time
	EXPECT_FALSE(_ekf_wrapper.isIntendingMagHeadingFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingMag3DFusion());
}

TEST_F(EkfMagTest, noInitLargeInclination)
{
	// GIVEN: a really large magnetic field
	_ekf_wrapper.enableMagInclinationCheck();
	// To prevent an early pass of the inclination check, "force WMM" must be set
	_ekf_wrapper.enableMagCheckForceWMM();
	_sensor_simulator.startGps();
	Vector3f mag_data(0.4f, 0.f, 0.f);
	_sensor_simulator._mag.setData(mag_data);

	const int initial_quat_reset_counter = _ekf_wrapper.getQuaternionResetCounter();
	_sensor_simulator.runSeconds(_init_duration_s + 10.f); // live some extra time fo GNSS checks to pass

	// THEN: the fusion shouldn't start
	EXPECT_FALSE(_ekf_wrapper.isIntendingMagHeadingFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingMag3DFusion());
	EXPECT_EQ(0, (int) _ekf->control_status_flags().yaw_align);
	EXPECT_EQ(_ekf_wrapper.getQuaternionResetCounter(), initial_quat_reset_counter);

	// BUT then: as soon as there is some meaningful data
	const float mag_heading = -M_PI_F / 7.f;
	mag_data = Vector3f(0.2f * cosf(mag_heading), -0.2f * sinf(mag_heading), 0.4f);
	_sensor_simulator._mag.setData(mag_data);

	_sensor_simulator.runSeconds(2.f);

	float decl_deg = 0.f;
	_ekf->get_mag_decl_deg(decl_deg);

	// THEN: the fusion initializes using the mag data and runs normally
	EXPECT_NEAR(_ekf_wrapper.getYawAngle(), mag_heading + radians(decl_deg), radians(1.f));
	EXPECT_TRUE(_ekf_wrapper.isIntendingMagHeadingFusion());
	EXPECT_EQ(1, (int) _ekf->control_status_flags().yaw_align);
	EXPECT_EQ(_ekf_wrapper.getQuaternionResetCounter(), initial_quat_reset_counter + 1);
}

TEST_F(EkfMagTest, suddenInclinationChange)
{
	_ekf_wrapper.enableMagInclinationCheck();
	_ekf_wrapper.enableMagCheckForceWMM();
	_sensor_simulator.startGps();

	// GIVEN: some meaningful mag data
	const float mag_heading = -M_PI_F / 7.f;
	Vector3f mag_data(0.2f * cosf(mag_heading), -0.2f * sinf(mag_heading), 0.4f);
	_sensor_simulator._mag.setData(mag_data);

	_sensor_simulator.runSeconds(_init_duration_s + 10.f);

	float decl_deg = 0.f;
	_ekf->get_mag_decl_deg(decl_deg);

	// THEN: the fusion initializes using the mag data and runs normally
	EXPECT_NEAR(_ekf_wrapper.getYawAngle(), mag_heading + radians(decl_deg), radians(1.f));
	EXPECT_TRUE(_ekf_wrapper.isIntendingMagHeadingFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingMag3DFusion());

	// BUT WHEN: the mag field inclination suddenly changes
	mag_data(2) = -mag_data(2);
	_sensor_simulator._mag.setData(mag_data);
	_sensor_simulator.runSeconds(6.f);

	// THEN: the mag fusion should stop after some time
	EXPECT_FALSE(_ekf_wrapper.isIntendingMagHeadingFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingMag3DFusion());
}

// Test the estimator itself, including magnetic-state resets, not a duplicate
// of the declination selection logic. Angles at the wrap boundary are included.
class EkfManualDeclinationTest : public EkfMagTest, public ::testing::WithParamInterface<float>
{
public:
	void SetUp() override
	{
		EkfMagTest::SetUp();
		_ekf->getParamHandle()->mag_declination_source = 0;
		_ekf->getParamHandle()->mag_declination_deg = GetParam();
		_ekf->getParamHandle()->mag_fusion_type = MagFuseType::INIT;
		_sensor_simulator._mag.setData(magData());
	}

	Vector3f magData() const
	{
		// Fixed sensor observation: magnetic heading 60 degrees.
		const float strength = get_mag_strength_gauss(40.44, -79.94);
		const float inclination = radians(get_mag_inclination_degrees(40.44, -79.94));
		return Dcmf(Eulerf(0.f, -inclination, -radians(60.f))) * Vector3f(strength, 0.f, 0.f);
	}

	void setGlobalPosition(double lat, double lon)
	{
		// Advance beyond the last mag fusion before changing the origin. WMM
		// updates use the current global position, not just the origin's label.
		_sensor_simulator._mag.stop();
		_sensor_simulator.runSeconds(0.2f);
		ASSERT_TRUE(_ekf->resetGlobalPositionTo(lat, lon, 122.2f));
		ASSERT_TRUE(_ekf->setEkfGlobalOrigin(lat, lon, 122.2f));
		_sensor_simulator._mag.start();
	}

	void expectManualHeading() const
	{
		EXPECT_TRUE(_ekf->control_status_flags().yaw_align);
		EXPECT_NEAR(wrap_pi(_ekf_wrapper.getYawAngle() - radians(60.f + GetParam())), 0.f, radians(1.f));
		EXPECT_FALSE(_ekf_wrapper.isIntendingMagHeadingFusion());
		EXPECT_FALSE(_ekf_wrapper.isIntendingMag3DFusion());
		float saved_decl = 123.f;
		EXPECT_FALSE(_ekf->get_mag_decl_deg(saved_decl));
		EXPECT_FLOAT_EQ(saved_decl, 123.f);
		EXPECT_FLOAT_EQ(_ekf->getParamHandle()->mag_declination_deg, GetParam());
	}

	void expectManualField(double lat, double lon) const
	{
		const Vector3f field = _ekf->getMagEarthField();
		EXPECT_NEAR(wrap_pi(atan2f(field(1), field(0)) - radians(GetParam())), 0.f, radians(0.1f));
		EXPECT_NEAR(field.norm(), get_mag_strength_gauss(lat, lon), 0.001f);
		EXPECT_NEAR(degrees(asinf(field(2) / field.norm())), get_mag_inclination_degrees(lat, lon), 0.1f);
	}
};

TEST_P(EkfManualDeclinationTest, startupWithoutGlobalPosition)
{
	_sensor_simulator.runSeconds(_init_duration_s);
	expectManualHeading();
}

TEST_P(EkfManualDeclinationTest, startupWithGlobalPosition)
{
	_sensor_simulator._mag.stop();
	_sensor_simulator.runSeconds(1.f);
	setGlobalPosition(40.44, -79.94);
	_sensor_simulator.runSeconds(_init_duration_s);
	expectManualHeading();
	expectManualField(40.44, -79.94);
}

TEST_P(EkfManualDeclinationTest, gpsArrivalDoesNotReplaceManualDeclination)
{
	_ekf->set_min_required_gps_health_time(5e6);
	_sensor_simulator.runSeconds(_init_duration_s);
	expectManualHeading();
	const auto resets = _ekf_wrapper.getQuaternionResetCounter();
	_sensor_simulator.setGpsLatitude(40.44);
	_sensor_simulator.setGpsLongitude(-79.94);
	_ekf_wrapper.enableGpsFusion();
	_sensor_simulator.startGps();
	_sensor_simulator.runSeconds(16.f);
	ASSERT_TRUE(_ekf_wrapper.isIntendingGpsFusion());
	expectManualHeading();
	expectManualField(40.44, -79.94);
	EXPECT_EQ(_ekf_wrapper.getQuaternionResetCounter(), resets);
}

TEST_P(EkfManualDeclinationTest, worldModelRefreshAndGroundCalibrationReset)
{
	_sensor_simulator.runSeconds(_init_duration_s);

	for (const Vector2f &location : {Vector2f(40.44f, -79.94f), Vector2f(47.f, 8.f)}) {
		setGlobalPosition(location(0), location(1));
		_sensor_simulator.runSeconds(2.f);
		expectManualHeading();
		expectManualField(location(0), location(1));
	}

	// A real calibration-change sample must still initialize with the manual
	// reference, not the last WMM direction or a learned field direction.
	const auto resets = _ekf_wrapper.getQuaternionResetCounter();
	_sensor_simulator._mag.stop();
	_sensor_simulator.runSeconds(0.02f);
	_ekf->setMagData(magSample{_sensor_simulator.getTime(), magData(), true});
	_sensor_simulator.runSeconds(0.2f);
	_sensor_simulator._mag.start();
	_sensor_simulator.runSeconds(_init_duration_s);
	expectManualHeading();
	EXPECT_EQ(_ekf_wrapper.getQuaternionResetCounter(), resets + 1);
}

TEST_P(EkfManualDeclinationTest, initOnlyTracksGyroNotContinuousCompass)
{
	_sensor_simulator._mag.stop();
	_sensor_simulator.runSeconds(1.f);
	setGlobalPosition(40.44, -79.94);
	_sensor_simulator.runSeconds(_init_duration_s);
	expectManualHeading();
	const auto resets = _ekf_wrapper.getQuaternionResetCounter();
	const float initial_yaw = _ekf_wrapper.getYawAngle();

	// Rotate 90 degrees using the gyro, with deliberately unchanged mag data.
	_ekf->set_vehicle_at_rest(false);
	_sensor_simulator._imu.setGyroData(Vector3f(0.f, 0.f, radians(45.f)));
	_sensor_simulator.runSeconds(2.f);
	_sensor_simulator._imu.setGyroData(Vector3f());
	_sensor_simulator.runSeconds(2.f);
	EXPECT_NEAR(wrap_pi(_ekf_wrapper.getYawAngle() - initial_yaw - radians(90.f)), 0.f, radians(1.f));
	EXPECT_FALSE(_ekf_wrapper.isIntendingMagHeadingFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingMag3DFusion());
	EXPECT_EQ(_ekf_wrapper.getQuaternionResetCounter(), resets);

	const float turned_yaw = _ekf_wrapper.getYawAngle();
	_sensor_simulator._mag.setData(Vector3f(-0.2f, 0.f, 0.4f));
	_sensor_simulator.runSeconds(10.f);
	EXPECT_NEAR(wrap_pi(_ekf_wrapper.getYawAngle() - turned_yaw), 0.f, radians(1.f));
	EXPECT_EQ(_ekf_wrapper.getQuaternionResetCounter(), resets);

	_sensor_simulator._imu.setGyroData(Vector3f(0.f, 0.f, -radians(45.f)));
	_sensor_simulator.runSeconds(2.f);
	_sensor_simulator._imu.setGyroData(Vector3f());
	_sensor_simulator.runSeconds(2.f);
	EXPECT_NEAR(wrap_pi(_ekf_wrapper.getYawAngle() - initial_yaw), 0.f, radians(1.f));
}

TEST_P(EkfManualDeclinationTest, takeoffKeepsManualReferenceAndInitOnly)
{
	_sensor_simulator._mag.stop();
	_sensor_simulator.runSeconds(1.f);
	setGlobalPosition(40.44, -79.94);
	_sensor_simulator.runSeconds(_init_duration_s);
	const auto resets = _ekf_wrapper.getQuaternionResetCounter();
	_ekf_wrapper.enableRangeHeightFusion();
	_sensor_simulator._rng.setLimits(0.1f, 20.f);
	_sensor_simulator._rng.setData(3.f, 100);
	_sensor_simulator.startRangeFinder();
	_ekf->set_in_air_status(true);
	_ekf->set_vehicle_at_rest(false);
	_sensor_simulator.runSeconds(10.f);
	ASSERT_TRUE(_ekf->control_status_flags().mag_aligned_in_flight);
	expectManualHeading();
	expectManualField(40.44, -79.94);
	EXPECT_EQ(_ekf_wrapper.getQuaternionResetCounter(), resets);
}

TEST_P(EkfManualDeclinationTest, continuousFusionUsesManualDeclinationObservation)
{
	_ekf->getParamHandle()->mag_fusion_type = MagFuseType::HEADING;
	_sensor_simulator._mag.stop();
	_sensor_simulator.runSeconds(1.f);
	setGlobalPosition(40.44, -79.94);
	_sensor_simulator.runSeconds(30.f);
	ASSERT_TRUE(_ekf_wrapper.isIntendingMagHeadingFusion());
	EXPECT_NEAR(wrap_pi(_ekf_wrapper.getYawAngle() - radians(60.f + GetParam())), 0.f, radians(1.f));
	const auto field = _ekf->getMagEarthField();
	EXPECT_NEAR(wrap_pi(atan2f(field(1), field(0)) - radians(GetParam())), 0.f, radians(0.1f));
}

TEST_P(EkfManualDeclinationTest, autoFusionKeepsManualReferenceAfterTakeoff)
{
	_ekf->getParamHandle()->mag_fusion_type = MagFuseType::AUTO;
	_sensor_simulator._mag.stop();
	_sensor_simulator.runSeconds(1.f);
	setGlobalPosition(40.44, -79.94);
	_sensor_simulator.runSeconds(_init_duration_s);
	_ekf_wrapper.enableRangeHeightFusion();
	_sensor_simulator._rng.setLimits(0.1f, 20.f);
	_sensor_simulator._rng.setData(3.f, 100);
	_sensor_simulator.startRangeFinder();
	_ekf->set_in_air_status(true);
	_ekf->set_vehicle_at_rest(false);
	_sensor_simulator.runSeconds(30.f);
	ASSERT_TRUE(_ekf->control_status_flags().mag_aligned_in_flight);
	ASSERT_TRUE(_ekf_wrapper.isIntendingMag3DFusion());
	EXPECT_NEAR(wrap_pi(_ekf_wrapper.getYawAngle() - radians(60.f + GetParam())), 0.f, radians(1.f));
	expectManualField(40.44, -79.94);
}

INSTANTIATE_TEST_SUITE_P(ManualDeclination, EkfManualDeclinationTest,
			 ::testing::Values(0.f, 35.f, -35.f, 179.f, -179.f, 180.f, -180.f));

TEST_F(EkfMagTest, invalidManualDeclinationDoesNotInjectNonFiniteState)
{
	_ekf->getParamHandle()->mag_declination_source = 0;
	_ekf->getParamHandle()->mag_declination_deg = NAN;
	_sensor_simulator.runSeconds(_init_duration_s);
	EXPECT_TRUE(_ekf->control_status_flags().yaw_align);
	EXPECT_TRUE(PX4_ISFINITE(_ekf_wrapper.getYawAngle()));
	_sensor_simulator._mag.stop();
	_sensor_simulator.runSeconds(0.2f);
	ASSERT_TRUE(_ekf->resetGlobalPositionTo(40.44, -79.94, 122.2f));
	_sensor_simulator._mag.start();
	_sensor_simulator.runSeconds(10.f);
	EXPECT_TRUE(PX4_ISFINITE(_ekf_wrapper.getYawAngle()));
	EXPECT_TRUE(_ekf->getMagEarthField().isAllFinite());
	EXPECT_TRUE(_ekf->getMagBias().isAllFinite());
}

TEST_F(EkfMagTest, manualDeclinationDoesNotBypassMagStrengthCheck)
{
	_ekf->getParamHandle()->mag_declination_source = 0;
	_ekf->getParamHandle()->mag_declination_deg = 35.f;
	_ekf->getParamHandle()->mag_fusion_type = MagFuseType::INIT;
	_ekf_wrapper.enableMagStrengthCheck();
	_sensor_simulator._mag.setData(Vector3f(1.f, 1.f, 1.f));
	_sensor_simulator.runSeconds(_init_duration_s);
	EXPECT_FALSE(_ekf->control_status_flags().yaw_align);
}

TEST_F(EkfMagTest, manualDeclinationDoesNotBypassMagInclinationCheck)
{
	_ekf->getParamHandle()->mag_declination_source = 0;
	_ekf->getParamHandle()->mag_declination_deg = 35.f;
	_ekf->getParamHandle()->mag_fusion_type = MagFuseType::INIT;
	_ekf_wrapper.enableMagInclinationCheck();
	_ekf_wrapper.enableMagCheckForceWMM();
	_sensor_simulator._mag.stop();
	_sensor_simulator.runSeconds(1.f);
	ASSERT_TRUE(_ekf->resetGlobalPositionTo(40.44, -79.94, 122.2f));
	_sensor_simulator._mag.setData(Vector3f(0.4f, 0.f, 0.f));
	_sensor_simulator._mag.start();
	_sensor_simulator.runSeconds(_init_duration_s);
	float inclination = NAN;
	ASSERT_TRUE(_ekf->get_mag_inc_deg(inclination));
	EXPECT_FALSE(_ekf->control_status_flags().yaw_align);
}

class EkfAutomaticDeclinationTest : public EkfMagTest, public ::testing::WithParamInterface<int> {};

TEST_P(EkfAutomaticDeclinationTest, automaticModesKeepExistingSelectionAndSaving)
{
	_ekf->getParamHandle()->mag_declination_source = GetParam();
	_ekf->getParamHandle()->mag_declination_deg = 35.f;
	_sensor_simulator.runSeconds(_init_duration_s);
	const float startup_decl = (GetParam() & GeoDeclinationMask::SAVE_GEO_DECL) ? 35.f : 0.f;
	EXPECT_NEAR(_ekf_wrapper.getYawAngle(), radians(startup_decl), radians(1.f));
	_sensor_simulator._mag.stop();
	_sensor_simulator.runSeconds(0.2f);
	const auto resets = _ekf_wrapper.getQuaternionResetCounter();
	ASSERT_TRUE(_ekf->resetGlobalPositionTo(40.44, -79.94, 122.2f));
	_sensor_simulator._mag.start();

	// Inspect the reset itself, before normal magnetic fusion can adjust it.
	for (int i = 0; i < 500 && _ekf_wrapper.getQuaternionResetCounter() == resets; ++i) {
		_sensor_simulator.runMicroseconds(1000);
	}

	ASSERT_EQ(_ekf_wrapper.getQuaternionResetCounter(), resets + 1);
	float decl = 123.f;
	EXPECT_EQ(_ekf->get_mag_decl_deg(decl), bool(GetParam() & GeoDeclinationMask::SAVE_GEO_DECL));

	if (GetParam() & GeoDeclinationMask::SAVE_GEO_DECL) {
		EXPECT_NEAR(decl, get_mag_declination_degrees(40.44, -79.94), 0.001f);
	}

	const float expected_decl = (GetParam() & GeoDeclinationMask::USE_GEO_DECL)
				    ? get_mag_declination_degrees(40.44, -79.94) : 35.f;
	EXPECT_NEAR(_ekf_wrapper.getYawAngle(), radians(expected_decl), radians(1.f));
}

INSTANTIATE_TEST_SUITE_P(AutomaticDeclination, EkfAutomaticDeclinationTest, ::testing::Values(1, 2, 3));
