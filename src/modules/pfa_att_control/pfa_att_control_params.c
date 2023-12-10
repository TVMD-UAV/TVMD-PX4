/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
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
 * AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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
 * @file pfa_att_control_params.c
 *
 * Parameters defined by the attitude control task for partially fully actuated (PFA) systems.
 *
 * This module is a modification of the uuv attitide control module and is designed for
 * partially fully actuated systems.
 *
 * @author Yen-Cheng Chu <sciyen.ycc@gmail.com>
 */

/*
 * Controller parameters, accessible via MAVLink
 *
 */

// Roll gains
/**
 * Roll proportional gain
 *
 * @group UUV Attitude Control
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(PFA_ROLL_P, 0.70f);

/**
 * Roll differential gain
 *
 * @group UUV Attitude Control
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(PFA_ROLL_D, 1.20f);


// Pitch gains
/**
 * Pitch proportional gain
 *
 * @group UUV Attitude Control
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(PFA_PITCH_P, 0.70f);

/**
 * Pitch differential gain
 *
 * @group UUV Attitude Control
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(PFA_PITCH_D, 1.20f);


// Yaw gains
/**
 * Yawh proportional gain
 *
 * @group UUV Attitude Control
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(PFA_YAW_P, 0.30f);

/**
 * Yaw differential gain
 *
 * @group UUV Attitude Control
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(PFA_YAW_D, 1.00f);

/**
 * Maximum torque of the vechicle in Nm
 *
 * @group PFA Attitude Control
 */
PARAM_DEFINE_FLOAT(PFA_MAX_TOR, 15.0f);
