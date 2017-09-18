/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "fc/rc_modes.h"
#include "drivers/camera_control.h"
#include "drivers/opentco.h"

typedef struct {
    uint8_t boxId;
    bool isActivated;
} opentco_cam_switch_state_t;

typedef struct opentcoCameraProfile_s {
    uint16_t supportedFeatures;
} opentcoCameraProfile_t;

PG_DECLARE(opentcoCameraProfile_t, opentcoCameraProfile);

extern opentcoDevice_t *camDevice;

bool is_fpv_cam_osd_open;

typedef enum {
    OPENTCO_CAM_KEY_NONE,
    OPENTCO_CAM_KEY_ENTER,
    OPENTCO_CAM_KEY_LEFT,
    OPENTCO_CAM_KEY_UP,
    OPENTCO_CAM_KEY_RIGHT,
    OPENTCO_CAM_KEY_DOWN,
    OPENTCO_CAM_KEY_LEFT_LONG,
    OPENTCO_CAM_KEY_RIGHT_AND_TOP,
    OPENTCO_CAM_KEY_RELEASE,
} opentcoCamSimulationKeyEvent_e;

void opentcoCamProcess(timeUs_t currentTimeUs);
bool opentcoCamInit(void);

// used for unit test
opentco_cam_switch_state_t switchStates[BOXCAMERA3 - BOXCAMERA1 + 1];
