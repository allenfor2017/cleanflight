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
#include <stdlib.h>
#include <string.h>

#include "drivers/display.h"

#include "io/beeper.h"
#include "io/serial.h"
#include "io/rcsplit.h"
#include "io/rcsplit_types.h"
#include "io/rcsplit_packet_helper.h"

#ifdef USE_RCCAMERA_DISPLAYPORT

displayPort_t rccameraDisplayPort;

static uint8_t screenBuffer[RCCAMERA_SCREEN_CHARACTER_COLUMN_COUNT * RCCAMERA_SCREEN_CHARACTER_ROW_COUNT];

static int grab(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return 0;
}

static int release(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return 0;
}

static int clearScreen(displayPort_t *displayPort)
{
    UNUSED(displayPort);

    sbuf_t buf;
    uint16_t expectedPacketSize = 0;
    expectedPacketSize = rcCamOSDGenerateClearPacket(NULL);
    buf.ptr = (uint8_t*)malloc(expectedPacketSize);
    rcCamOSDGenerateClearPacket(&buf);
    serialWriteBuf(rcSplitSerialPort, buf.ptr, sbufBytesRemaining(&buf));
    free(buf.ptr);

    uint16_t bufferLen = RCCAMERA_SCREEN_CHARACTER_COLUMN_COUNT * RCCAMERA_SCREEN_CHARACTER_ROW_COUNT;
    uint16_t x;
    uint32_t *p = (uint32_t*)&screenBuffer[0];
    for (x = 0; x < bufferLen/4; x++)
        p[x] = 0x20202020;

    return 0;
}

static int drawScreen(displayPort_t *displayPort)
{
    UNUSED(displayPort);

    sbuf_t buf;
    uint16_t expectedPacketSize = 0;
    expectedPacketSize = rcCamOSDGenerateDrawScreenPacket(NULL, screenBuffer);
    buf.ptr = (uint8_t*)malloc(expectedPacketSize);
    rcCamOSDGenerateDrawScreenPacket(&buf, screenBuffer);
    serialWriteBuf(rcSplitSerialPort, buf.ptr, sbufBytesRemaining(&buf));
    free(buf.ptr);
    
    return 0;
}

static int screenSize(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return displayPort->rows * displayPort->cols;
}

static int _writeString(displayPort_t *displayPort, uint8_t x, uint8_t y, const uint8_t *s, uint16_t len)
{
    UNUSED(displayPort);

    uint16_t adjustedXPos = x * RCCAMERA_CHARACTER_WIDTH_TOTAL;
    uint16_t adjustedYPos = y * RCCAMERA_CHARACTER_HEIGHT_TOTAL;
    uint8_t i = 0;
    for (i = 0; i < len; i++)
        if (adjustedXPos + i < RCCAMERA_SCREEN_CHARACTER_COLUMN_COUNT) // Do not write over screen
            screenBuffer[adjustedYPos * RCCAMERA_SCREEN_CHARACTER_COLUMN_COUNT + adjustedXPos + i] = *(s + i);

    return 0;
}

static int writeString(displayPort_t *displayPort, uint8_t x, uint8_t y, const char *s)
{
    return _writeString(displayPort, x, y, (const uint8_t*)s, strlen(s));
}

static int writeChar(displayPort_t *displayPort, uint8_t x, uint8_t y, uint8_t c)
{
    return _writeString(displayPort, x, y, &c, 1);
}

static bool isTransferInProgress(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return false;
}

static void resync(displayPort_t *displayPort)
{
    UNUSED(displayPort);
}

static int heartbeat(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return 0;
}

static uint32_t txBytesFree(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return UINT32_MAX;
}

static const displayPortVTable_t rccameraDisplayPortVTable = {
    .grab = grab,
    .release = release,
    .clearScreen = clearScreen,
    .drawScreen = drawScreen,
    .screenSize = screenSize,
    .write = writeString,
    .writeChar = writeChar,
    .isTransferInProgress = isTransferInProgress,
    .heartbeat = heartbeat,
    .resync = resync,
    .txBytesFree = txBytesFree,
};

displayPort_t *rccameraDisplayPortInit(serialPort_t *cameraSerialPort)
{
    if (!cameraSerialPort) {
        return NULL;
    }

    // rccameraDisplayPort.device = cameraSerialPort;
    displayInit(&rccameraDisplayPort, &rccameraDisplayPortVTable);
    rccameraDisplayPort.rows = RCCAMERA_SCREEN_CHARACTER_ROW_COUNT;
    rccameraDisplayPort.cols = RCCAMERA_SCREEN_CHARACTER_COLUMN_COUNT;    

    return &rccameraDisplayPort;
}

#endif
