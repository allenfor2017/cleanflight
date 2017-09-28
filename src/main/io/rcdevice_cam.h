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

#include <stdbool.h>
#include <stdint.h>

#include "common/time.h"
#include "drivers/rcdevice.h"
#include "fc/rc_modes.h"

typedef struct {
    uint8_t boxId;
    bool isActivated;
} rcdevice_cam_switch_state_t;

extern runcamDevice_t *camDevice;
bool rcdeviceInMenu;

bool rcdeviceInit(void);
void rcdeviceProcess(timeUs_t currentTimeUs);

bool isRcDeviceCamReady();
bool isFeatureSupported(uint8_t feature);
void rcdeviceCamProcessMode();
void rcdeviceCamSimulate5KeyCablePressProcessMode(timeUs_t currentTimeUs);

// used for unit test
rcdevice_cam_switch_state_t switchStates[BOXCAMERA3 - BOXCAMERA1 + 1];
