/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * @file multicopter_omni_params.c
 *
 * Parameters for omnidirectional (fully-actuated) multicopter attitude control.
 */

/**
 * Maximum direct-force (horizontal) scaled thrust for omnidirectional vehicles
 *
 * Limits the horizontal NED thrust vector before body-frame projection,
 * relative to the maximum possible thrust of an omnidirectional multirotor.
 * The total thrust and tilt envelopes can further reduce this limit. The
 * value of this parameter does not affect the behavior if the attitude mode
 * is not set to one of omni-directional modes (if OMNI_ATT_MODE is 0).
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @group Multicopter Omnidirectional
 */
PARAM_DEFINE_FLOAT(OMNI_DFC_MAX_THR, 0.15f);

/**
 * Omni-directional attitude setpoint mode
 *
 * Specifies the type of attitude setpoint sent to the attitude controller.
 * The parameter can be set to a normal attitude setpoint, where the tilt
 * of the vehicle (roll and pitch) are calculated from the desired thrust
 * vector. This should be the behavior for the non-omnidirectional vehicles,
 * such as quadrotors and other multirotors with coplanar rotors.
 * The "constant zero tilt" mode enforces zero roll and pitch and can only be
 * used for omnidirectional vehicles. The min-tilt mode enforces zero tilt
 * up to a maximum set acceleration (thrust) and tilts the vehicle as small
 * as possible if the thrust vector is larger than the maximum. The "constant
 * tilt" and "constant roll/pitch" modes enforce a given tilt or roll/pitch
 * for the vehicle. The estimation modes estimate the optimal tilt or roll/pitch
 * to counteract with the external force (e.g., wind).
 *
 * @min 0
 * @max 6
 * @value 0 tilted attitude
 * @value 1 min-tilt attitude
 * @value 2 constant zero tilt
 * @value 3 constant tilt
 * @value 4 constant roll/pitch
 * @value 5 estimate tilt/roll/pitch
 * @value 6 slow attitude change
 * @group Multicopter Omnidirectional
 */
PARAM_DEFINE_INT32(OMNI_ATT_MODE, 0);

/**
 * Omni-directional attitude setpoint tilt angle
 *
 * Specifies the tilt angle in degrees for the constant-tilt attitude
 * setpoint generation mode (OMNI_ATT_MODE=3).
 *
 * @unit deg
 * @min 0
 * @max 90
 * @decimal 2
 * @group Multicopter Omnidirectional
 */
PARAM_DEFINE_FLOAT(OMNI_ATT_TLT_ANG, 0.0f);

/**
 * Omni-directional attitude setpoint tilt direction angle
 *
 * Specifies the direction of the tilt in degrees for the constant-tilt
 * attitude setpoint generation mode (OMNI_ATT_MODE=3). The direction is
 * measured from North.
 *
 * @unit deg
 * @min -360
 * @max 360
 * @decimal 2
 * @group Multicopter Omnidirectional
 */
PARAM_DEFINE_FLOAT(OMNI_ATT_TLT_DIR, 0.0f);

/**
 * Omni-directional attitude setpoint roll angle
 *
 * Specifies the roll angle in degrees for the constant-roll/pitch attitude
 * setpoint generation mode (OMNI_ATT_MODE=4).
 *
 * @unit deg
 * @min -90
 * @max 90
 * @decimal 2
 * @group Multicopter Omnidirectional
 */
PARAM_DEFINE_FLOAT(OMNI_ATT_ROLL, 0.0f);

/**
 * Omni-directional attitude setpoint pitch angle
 *
 * Specifies the pitch angle in degrees for the constant-roll/pitch attitude
 * setpoint generation mode (OMNI_ATT_MODE=4).
 *
 * @unit deg
 * @min -90
 * @max 90
 * @decimal 2
 * @group Multicopter Omnidirectional
 */
PARAM_DEFINE_FLOAT(OMNI_ATT_PITCH, 0.0f);

/**
 * Axes for thrust projection in omni-directional attitude modes
 *
 * Specifies the axis used for projecting inertial thrust setpoint
 * to the body-fixed frame in omni-directional modes (OMNI_ATT_MODE > 0).
 *
 * @min 0
 * @max 1
 * @value 0 commanded attitude
 * @value 1 current attitude
 * @group Multicopter Omnidirectional
 */
PARAM_DEFINE_INT32(OMNI_PROJ_AXES, 1);

/**
 * Rate for change of attitude
 *
 * Specifies the rate in which the attitude can change in slow attitude mode
 * (OMNI_ATT_MODE = 6), rotating the Z axis from the current attitude to the
 * optimal attitude. The value is in degrees per position control loop.
 *
 * @unit deg
 * @min 0
 * @decimal 3
 * @group Multicopter Omnidirectional
 */
PARAM_DEFINE_FLOAT(OMNI_ATT_RATE, 0.5f);
