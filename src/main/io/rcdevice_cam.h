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
#include "drivers/rcdevice.h"

typedef struct {
    uint8_t boxId;
    bool isActivated;
} rcdevice_cam_switch_state_t;

extern runcamDevice_t *camDevice;

bool is_fpv_cam_osd_open;

// packet header and tail

#define RCSPLIT_PACKET_HEADER           0x55
#define RCSPLIT_PACKET_CMD_CTRL  0x01
#define RCSPLIT_PACKET_TAIL     0xaa

typedef enum {
    RCSPLIT_CTRL_ARGU_INVALID = 0x0,
    RCSPLIT_CTRL_ARGU_WIFI_BTN = 0x1,
    RCSPLIT_CTRL_ARGU_POWER_BTN = 0x2,  
    RCSPLIT_CTRL_ARGU_CHANGE_MODE = 0x3,
    RCSPLIT_CTRL_ARGU_WHO_ARE_YOU = 0xFF,
} rcsplit_ctrl_argument_e;

typedef enum {
    RCDEVICE_CAM_KEY_NONE,
    RCDEVICE_CAM_KEY_ENTER,
    RCDEVICE_CAM_KEY_LEFT,
    RCDEVICE_CAM_KEY_UP,
    RCDEVICE_CAM_KEY_RIGHT,
    RCDEVICE_CAM_KEY_DOWN,
    RCDEVICE_CAM_KEY_LEFT_LONG,
    RCDEVICE_CAM_KEY_RIGHT_AND_TOP,
    RCDEVICE_CAM_KEY_RELEASE,
} rcdeviceCamSimulationKeyEvent_e;

bool rcdeviceCamInit(void);
void rcdeviceCamProcess(timeUs_t currentTimeUs);

// used for unit test
rcdevice_cam_switch_state_t switchStates[BOXCAMERA3 - BOXCAMERA1 + 1];
