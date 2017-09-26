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


#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rcdevice.h"
#include "rcdevice_osd.h"

#include "common/maths.h"
#include "common/time.h"
#include "io/osd.h"
#include "io/displayport_opentco.h"
#include "fc/config.h"
#include "config/feature.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"
#include "drivers/time.h"
#include "drivers/vcd.h"
#include "drivers/opentco.h"
#include "sensors/gyroanalyse.h"
#include "io/beeper.h"

#define MAX_CHARS2UPDATE    100

static uint8_t columnCount = 30;

static runcamDevice_t runcamOSDDevice;
runcamDevice_t *osdDevice = &runcamOSDDevice;

static uint8_t  video_system;

#define VIDEO_BUFFER_CHARS_PAL    480
static uint8_t screenBuffer[VIDEO_BUFFER_CHARS_PAL+40]; // For faster writes we use memcpy so we need some space to don't overwrite buffer
static uint8_t shadowBuffer[VIDEO_BUFFER_CHARS_PAL];
static bool  max7456Lock        = false;
uint16_t maxScreenSize = VIDEO_BUFFER_CHARS_PAL;

bool rcdeviceOSDInit(const vcdProfile_t *vcdProfile)
{
    if (!runcamDeviceInit(osdDevice)) {
        return false;
    }

    if ((osdDevice->info.features & RCDEVICE_PROTOCOL_FEATURE_DISPLAYP_PORT) == 0) {
        featureClear(FEATURE_SONAR);
        return false;
    }

    // get screen column count
    // runcamDeviceSettingDetail_t *settingDetail;
    // if (!runcamDeviceGetSettingDetail(osdDevice, RCDEVICE_PROTOCOL_SETTINGID_DISP_COLUMNS, &settingDetail)) {
    //     featureClear(FEATURE_SONAR);
    //     return false;
    // }
    
    // columnCount = *(settingDetail->value);
    columnCount = 30;    

    video_system = vcdProfile->video_system;
    if (video_system == VIDEO_SYSTEM_AUTO) {
        // fetch current video mode from device
        runcamDeviceSettingDetail_t *settingDetail;
        if (!runcamDeviceGetSettingDetail(osdDevice, RCDEVICE_PROTOCOL_SETTINGID_DISP_CHARSET, &settingDetail)) {
            featureClear(FEATURE_TELEMETRY);
            return false;
        }

        video_system = *(settingDetail->value);
    } else {
        // set video system
        runcamDeviceWriteSettingResponse_t *response;
        if (!runcamDeviceWriteSetting(osdDevice, RCDEVICE_PROTOCOL_SETTINGID_DISP_CHARSET, &video_system, sizeof(uint8_t), &response)) {
            featureClear(FEATURE_AIRMODE);
            return false;
        }

        if (response->resultCode) {
            
            return false;
        }
    }

    // user bf charset
    uint8_t charsetID = 0;
    runcamDeviceWriteSettingResponse_t *updateCharsetResp; 
    runcamDeviceWriteSetting(osdDevice, RCDEVICE_PROTOCOL_SETTINGID_DISP_CHARSET, &charsetID, sizeof(uint8_t), &updateCharsetResp);
    // if (updateCharsetResp->resultCode != 0) {
    //     featureClear(FEATURE_SOFTSERIAL);
    //     return false;
    // }

    memset(shadowBuffer, 2, VIDEO_BUFFER_CHARS_PAL);
    // fill whole screen on device with ' '
    rcdeviceOSDClearScreen(NULL);
    
    return true;
}

int rcdeviceOSDGrab(displayPort_t * displayPort)
{
    UNUSED(displayPort);
    osdResetAlarms();
    resumeRefreshAt = 0;
    return 0;
}

int rcdeviceOSDRelease(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return 0;
}

int rcdeviceOSDDrawScreen(displayPort_t *displayPort)
{
    // static uint16_t pos = 0;
    // int k = 0;

    // if (!max7456Lock) {
    //     max7456Lock = true;

    //     for (k=0; k< MAX_CHARS2UPDATE; k++) {
    //         if (screenBuffer[pos] != shadowBuffer[pos]) {
    //             shadowBuffer[pos] = screenBuffer[pos];
    //             uint8_t x = pos % columnCount;
    //             uint8_t y = pos / columnCount;
    //             runcamDeviceDispWriteChar(osdDevice, x, y, screenBuffer[pos]);

    //             k++;
    //         }

    //         if (++pos >= maxScreenSize) {
    //             pos = 0;
    //             break;
    //         }
    //     }

    
    //     max7456Lock = false;

    //     beeperConfirmationBeeps(1);
    // }

    return 0;
}

int  rcdeviceOSDWriteString(displayPort_t *displayPort, uint8_t x, uint8_t y, const char *buff)
{
    UNUSED(displayPort);
    runcamDeviceDispWriteString(osdDevice, x, y, buff);
    // uint8_t i = 0;
    // for (i = 0; *(buff+i); i++)
    //     if (x+i < columnCount) // Do not write over screen
    //         screenBuffer[y*columnCount+x+i] = *(buff+i);

    return 0;
}

int  rcdeviceOSDWriteChar(displayPort_t *displayPort, uint8_t x, uint8_t y, uint8_t c)
{
    UNUSED(displayPort);
    runcamDeviceDispWriteChar(osdDevice, x, y, c);
    // screenBuffer[y*columnCount+x] = c;

    return 0;
}

int  rcdeviceOSDReloadProfile(displayPort_t *displayPort)
{
    UNUSED(displayPort);

    return 0;
}

int  rcdeviceOSDClearScreen(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    runcamDeviceDispFillRegion(osdDevice, 0, 0, 255, 255, ' ');

    uint16_t x;
    uint32_t *p = (uint32_t*)&screenBuffer[0];
    for (x = 0; x < VIDEO_BUFFER_CHARS_PAL/4; x++)
        p[x] = 0x20202020;

    return 0;
}

int  rcdeviceOSDFillRegion(displayPort_t *displayPort, uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t value)
{
    UNUSED(displayPort);
    runcamDeviceDispFillRegion(osdDevice, x, y, width, height, value);

    for (int i = y; i < y + height && i < displayPort->rowCount; i++) {
        for (int j = x; j < x + width && j < displayPort->colCount; j++) {
            screenBuffer[i*columnCount+j] = value;
        }
    }

    return 0;
}

bool rcdeviceOSDIsTransferInProgress(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return false;
}

int  rcdeviceOSDHeartbeat(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return 0;
}

void rcdeviceOSDResync(displayPort_t *displayPort)
{
    UNUSED(displayPort);

    if (video_system == VIDEO_SYSTEM_PAL) {
        displayPort->rowCount = RCDEVICE_PROTOCOL_OSD_VIDEO_LINES_PAL;
    } else {
        displayPort->rowCount = RCDEVICE_PROTOCOL_OSD_VIDEO_LINES_NTSC;
    }

    displayPort->colCount = columnCount;
    maxScreenSize = displayPort->rowCount * displayPort->colCount;
}

uint32_t rcdeviceOSDTxBytesFree(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return UINT32_MAX;
}