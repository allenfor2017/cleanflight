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
#include <ctype.h>
#include <string.h>

#include "io/beeper.h"
#include "io/serial.h"

#include "scheduler/scheduler.h"

#include "drivers/serial.h"

#include "common/streambuf.h"
#include "common/maths.h"

#include "io/rcsplit.h"

uint8_t crc8_ccitt(uint8_t crc, unsigned char a)
{
    crc ^= a;
    for (uint8_t i = 8; i > 0; --i) { 
        if (crc & 0x80)
            crc = (crc << 1) ^ 0x31;
        else
            crc = (crc << 1);
    }
    return crc;
}

uint8_t rcCamCalcPacketCRC(sbuf_t *buf, uint8_t *base, uint8_t skipDataLocation, uint8_t skipDataLength)
{
    uint8_t offset = 0;
    uint8_t crc = 0x00;
    sbufSwitchToReader(buf, base);
    int len = sbufBytesRemaining(buf);
    while (len) {
        if (offset == skipDataLocation) { // ingore the crc field
            sbufAdvance(buf, skipDataLength);
            offset++;
            len = sbufBytesRemaining(buf);
            continue;
        }

        uint8_t c = sbufReadU8(buf);
        crc = crc8_ccitt(crc, c);

        offset++;
        len = sbufBytesRemaining(buf);
    }

    return crc;
}

uint16_t rcCamOSDGeneratePacket(sbuf_t *src, uint8_t command, const uint8_t *data, uint16_t len)
{
    uint16_t pakcetLen = sizeof(rcsplit_packet_v2_t) - sizeof(uint8_t*) + len;

    if (src == NULL) {
        return pakcetLen;
    }

    uint8_t *base = src->ptr;
    uint8_t crcFieldOffset = 0;
    uint8_t combinedCommand = RCSPLIT_OPENCTO_CAMERA_DEVICE << 4 & 0xF0;
    combinedCommand |= command & 0x0F;
 
    sbufWriteU8(src, RCSPLIT_PACKET_HEADER);
    sbufWriteU8(src, combinedCommand);
    sbufWriteU8(src, len);
    if (len > 0) {
        sbufWriteData(src, data, len);
    }
    crcFieldOffset = sbufConstPtr(src) - base;
    sbufWriteU8(src, 0);
    
    // calc the crc of the packet, and skip the crc field
    uint8_t crc = rcCamCalcPacketCRC(src, base, crcFieldOffset, sizeof(uint8_t));
    src->ptr = base;
    // write crc value to the position that it should be there
    sbufAdvance(src, crcFieldOffset);
    // sbufWriteU16(src, crc);
    sbufWriteU8(src, crc);

    sbufSwitchToReader(src, base);

    return sbufBytesRemaining(src);
}

uint16_t rcCamOSDGenerateControlPacket(sbuf_t *buf, uint8_t subcommand)
{
    uint16_t packetSize = rcCamOSDGeneratePacket(buf, RCSPLIT_PACKET_CMD_CTRL, &subcommand, sizeof(subcommand));
    return packetSize;
}

uint16_t rcCamOSDGenerateClearPacket(sbuf_t *buf)
{
    uint16_t packetSize = rcCamOSDGeneratePacket(buf, RCSPLIT_PACKET_CMD_OSD_CLEAR, NULL, 0);
    return packetSize;
}

uint16_t rcCamOSDGenerateDrawParticleScreenPacket(sbuf_t *buf, uint8_t *dataBuf, uint16_t dataLen)
{
    uint16_t packetSize = rcCamOSDGeneratePacket(buf, 
                                                RCSPLIT_PACKET_CMD_OSD_DRAW_PARTICLE_SCREEN_DATA, 
                                                dataBuf, 
                                                dataLen);
    return packetSize;
}

uint16_t rcCamOSDGenerateGetCameraInfoPacket(sbuf_t *buf)
{
    uint8_t data = 1;
    uint16_t packetSize = rcCamOSDGeneratePacket(buf, 
                                                RCSPLIT_PACKET_CMD_GET_CAMERA_INFO, 
                                                &data, 
                                                1);
    return packetSize;
}

uint16_t rcCamOSDGenerateGetMainMenuConfigurationsPacket(sbuf_t *buf)
{
    //menutype：
    // 1://OME_String:只显示值
    // 2://OME_TAB:TAB选项(包括开关)
    // 3://OME_UINT8:UINT8选择值范围
    // 4://OME_FLOAT:FLOAT选择值范围
    // 5://OME_Submenu:下级菜单

    //data封包为：<camModelId><counts> <m1-menuid><m1-menutype>
    uint8_t data[] = {2,9,
                      1,2,
                      2,4,
                      3,5,
                      4,5,
                      5,3,
                      6,3,
                      7,2,
                      8,4,
                      9,5
        };
    uint16_t packetSize = rcCamOSDGeneratePacket(buf,
                                                RCSPLIT_PACKET_CMD_GET_CONFIGURATIONS, 
                                                data,
                                                sizeof(data));
    return packetSize;
}

uint16_t rcCamOSDGenerateGetItemMenuConfigurationsPacket(sbuf_t *buf)
{
    //data封包为：<upMenuId><counts> <m1-menuid><m1-menutype>
    uint8_t data[] = {5,9,
                      10,2,
                      11,4,
                      12,5,
                      13,5,
                      14,3,
                      15,3,
                      16,2,
                      17,4,
                      18,5
    };
    uint16_t packetSize = rcCamOSDGeneratePacket(buf,
                                                RCSPLIT_PACKET_CMD_GET_CONFIGURATION_ITEMS, 
                                                data,
                                                sizeof(data));
    return packetSize;
}

uint16_t rcCamOSDGenerateGetConfigurationsValuesPacket(sbuf_t *buf)
{
     //data封包为：<strCounts><tabCounts><unitCounts><floatCounts> + 1/2/3/4
                //1. <m1-menuValType><m1-menuId><m1-menuValLength><m1-*menuVal>
                //2. <m1-menuValType><m1-menuId><m1-maxNums><m1-menuVal>
                //3. <m1-menuValType><m1-menuId><m1-valMin><m1-valMax><m1-valStep><m1-menuVal>
                //4. <m1-menuValType><m1-menuId><m1-valMin><m1-valMax><m1-valStep><m1-valMultipler><m1-menuVal>

    uint8_t data[] = {1,3,3,2, 
                      1,1,5,'H','E','L','L','O', 
                      2,2,3, 
                      2,3,1, 
                      2,4,3,3, 
                      3,5,1,90,1,10, 
                      3,6,1,50,1,10,
                      3,7,1,255,1,1,
                      4,8,1,100,5,100,1, 
                      4,9,1,100,1,10,1 
    };

    uint16_t packetSize = rcCamOSDGeneratePacket(buf, 
                                                RCSPLIT_PACKET_CMD_GET_CONFIGURATIONS_VALUES, 
                                                data, 
                                                sizeof(data));
    return packetSize;
}