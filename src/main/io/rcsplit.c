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
#include <ctype.h>

#include <platform.h>

#include "common/utils.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "fc/rc_controls.h"

#include "io/beeper.h"
#include "io/serial.h"

#include "scheduler/scheduler.h"

#include "drivers/serial.h"
#include "drivers/display.h"

#include "cms/cms.h"
#include "cms/cms_types.h"

#include "io/rcsplit.h"
#include "io/rcsplit_packet_helper.h"


// communicate with camera device variables
serialPort_t *rcSplitSerialPort = NULL;
rcsplit_switch_state_t switchStates[BOXCAMERA3 - BOXCAMERA1 + 1];
rcsplit_state_e cameraState = RCSPLIT_STATE_UNKNOWN;
rcsplit_osd_camera_info_t cameraInfo = { RCSPLIT_VIDEOFMT_UNKNOWN  };

static uint8_t rcsplitReqBuffer[1024];
static uint8_t rcsplitRespBuffer[1024];
static uint8_t resplitRespDataBlockLength = 0;

// 存放当前有多少个菜单项的cms根菜单里
uint8_t camModelId = 0;
static uint8_t menuItemCount = 0;
static rcsplit_cms_menu_data_t *menuItems = NULL;

uint8_t cmsc_CamTitle = 0;
uint8_t cmsc_DWDR = 0;
uint8_t cmsc_AreaStateA = 0;
uint8_t cmsc_AreaStateB = 0;
uint8_t cmsc_Burst = 0;
uint8_t cmsc_IRSmart = 0;
uint8_t cmsc_LensShad = 0;
uint8_t cmsc_TWODNR = 0;
uint8_t cmsc_Mirror = 0;
uint8_t cmsc_NegImage = 0;

uint8_t cmsc_Shutter = 0;
uint8_t cmsc_AGC = 0;
uint8_t cmsc_Whitebal = 0;
uint8_t cmsc_Colortemp = 0;
uint8_t cmsc_BackLight = 0;
uint8_t cmsc_AreaSel = 0;
uint8_t cmsc_HLCMode = 0;
uint8_t cmsc_DayNight = 0;
uint8_t cmsc_IRLevel = 0;
uint8_t cmsc_Display = 0;
uint8_t cmsc_DNDelay = 0;
uint8_t cmsc_NDDelay = 0;

static const char * const cmsc_SwitchNames[] = {
    "---", "OFF", "ON "
};
static const char * const cmsc_ShutterNames[] = {
    "---", "AUTO", "1/60", "FLK", "1/250", "1/500", "1/1000", "1/2000", "1/4000", "1/5000", "1/10000", "1/100000"
};
static const char * const cmsc_AGCNames[] = {
    "---", "OFF", "LOW", "MIDDLE", "HIGH"
};
static const char * const cmsc_WhitebalNames[] = {
    "---", "ATW1", "ATW2", "AWC->SET", "MANUAL"
};
static const char * const cmsc_ColortempNames[] = {
    "---", "MANUAL", "INDOOR", "OUTDOOR"
};
static const char * const cmsc_BackLightNames[] = {
    "---", "OFF", "BLC", "HLC"
};
static const char * const cmsc_AreaSelNames[] = {
    "---", "AREA1", "AREA2"
};
static const char * const cmsc_HLCModeNames[] = {
    "---", "ALL DAY", "NIGHT ONLY"
};
static const char * const cmsc_DayNightNames[] = {
    "---", "AUTO", "COLOR", "B/W", "EXT"
};
static const char * const cmsc_IRLevelNames[] = {
    "---", "HIGH", "LOW"
};
static const char * const cmsc_DisplayNames[] = {
    "---", "LCD", "USER", "CRT"
};
static const char * const cmsc_DelayNames[] = {
    "---", "1 SEC" , "3 SEC" , "5 SEC" , "10 SEC" , "15 SEC" , "20 SEC" , "25 SEC" , "30 SEC"
};

OSD_TAB_t cmsc_mCamTitleTab = { &cmsc_CamTitle, 2, cmsc_SwitchNames};
OSD_TAB_t cmsc_mDWDRTab = { &cmsc_DWDR, 2, cmsc_SwitchNames};
OSD_TAB_t cmsc_mAreaStateATab = { &cmsc_AreaStateA, 2, cmsc_SwitchNames};
OSD_TAB_t cmsc_mAreaStateBTab = { &cmsc_AreaStateB, 2, cmsc_SwitchNames};
OSD_TAB_t cmsc_mBurstTab = { &cmsc_Burst, 2, cmsc_SwitchNames};
OSD_TAB_t cmsc_mIRSmartTab = { &cmsc_IRSmart, 2, cmsc_SwitchNames};
OSD_TAB_t cmsc_mLensShadTab = { &cmsc_LensShad, 2, cmsc_SwitchNames};
OSD_TAB_t cmsc_mTWODNRTab = { &cmsc_TWODNR, 2, cmsc_SwitchNames};
OSD_TAB_t cmsc_mMirrorTab = { &cmsc_Mirror, 2, cmsc_SwitchNames};
OSD_TAB_t cmsc_mNegImageTab = { &cmsc_NegImage, 2, cmsc_SwitchNames};

OSD_TAB_t cmsc_mShutterTab = { &cmsc_Shutter, 11, cmsc_ShutterNames};
OSD_TAB_t cmsc_mAGCTab = { &cmsc_AGC, 4, cmsc_AGCNames};
OSD_TAB_t cmsc_mWhitebalTab = { &cmsc_Whitebal, 4, cmsc_WhitebalNames};
OSD_TAB_t cmsc_mColortempTab = { &cmsc_Colortemp, 3, cmsc_ColortempNames};
OSD_TAB_t cmsc_mBackLightTab = { &cmsc_BackLight, 3, cmsc_BackLightNames};
OSD_TAB_t cmsc_mAreaSelTab = { &cmsc_AreaSel, 2, cmsc_AreaSelNames};
OSD_TAB_t cmsc_mHLCModeTab = { &cmsc_HLCMode, 2, cmsc_HLCModeNames};
OSD_TAB_t cmsc_mDayNightTab = { &cmsc_DayNight, 4, cmsc_DayNightNames};
OSD_TAB_t cmsc_mIRLevelTab = { &cmsc_IRLevel, 2, cmsc_IRLevelNames};
OSD_TAB_t cmsc_mDisplayTab = { &cmsc_Display, 3, cmsc_DisplayNames};
OSD_TAB_t cmsc_mDNDelayTab = { &cmsc_DNDelay, 8, cmsc_DelayNames};
OSD_TAB_t cmsc_mNDDelayTab = { &cmsc_NDDelay, 8, cmsc_DelayNames};

//OSD 2
uint8_t cmsc_VideoStandard = 0;
uint8_t cmsc_Detail = 0;
uint8_t cmsc_Edge = 0;
uint8_t cmsc_Saturation = 0;
uint8_t cmsc_THREEDNR = 0;

static const char * const cmsc_VideoStandardNames[] = {
    "---", "NTSC-60HZ","PAL-50HZ"
};
static const char * const cmsc_ImageEnhanceNames[] = {
    "---", "AUTO","MANUAL"
};

OSD_TAB_t cmsc_mVideoStandardTab = { &cmsc_VideoStandard, 2, cmsc_VideoStandardNames};
OSD_TAB_t cmsc_mDetailTab = { &cmsc_Detail, 2, cmsc_ImageEnhanceNames};
OSD_TAB_t cmsc_mEdgeTab = { &cmsc_Edge, 2, cmsc_ImageEnhanceNames};
OSD_TAB_t cmsc_mSaturationTab = { &cmsc_Saturation, 2, cmsc_ImageEnhanceNames};
OSD_TAB_t cmsc_mTHREEDNRTab = { &cmsc_THREEDNR, 2, cmsc_ImageEnhanceNames};


static long runcamCmsSetTabConfigValue(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);
    
    // if (trampCmsPitMode == 0) {
    //     // Bouce back
    //     trampCmsPitMode = 1;
    // } else {
    //     trampSetPitMode(trampCmsPitMode - 1);
    // }
    return 0;
}

static long runcamCmsSetUintConfigValue(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    return 0;
}

static long runcamCmsSetFloatConfigValue(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    return 0;
}


static const char * const CamModeName[] = {
    "Swift"         , "Swift 2"   , "Swift Mini" , "Swift RR" , "OWL"          , 
    "OWL Plus"      , "OWL 2"     , "Eagle"      , "Eagle 2"  , "Micro Swift"  , 
    "Micro Swift 2" , "RunCam HD" , "RunCam 2"   , "RunCam 3" , "RunCam Split"
};

char * const MenuNameOSDA[] = {
    "CAM TITLE"  , "EXPOSURE"   , "WHITE BAL." , "BACKLIGHT"  , "DAY&NIGHT"  ,
    "IMAGE ADJ." , "DPC"        , "LANGUAGE"   , "RESET"      , "SHUTTER"    ,
    "BRIGHTNESS" , "AGC"        , "DWDR"       , "LEVEL"      , "COLOR TEMP" ,
    "BLUE"       , "RED"        , "LEVEL"      , "MODE"       , "AREA SEL."  ,
    "AREA STATE" , "GAIN"       , "HEIGHT"     , "WIDTH"      , "LEFT/RIGHT" ,
    "TOP/BOTTOM" , "AREA SEL."  , "AREA STATE" , "GAIN"       , "HEIGHT"     ,
    "WIDTH"      , "LEFT/RIGHT" , "TOP/BOTTOM" , "D->N LEVEL" , "D->N DELAY" ,
    "N->D LEVEL" , "N->D DELAY" , "BURST"      , "IR SMART"   , "IR LEVEL"   ,
    "IR GAIN"    , "HEIGHT"     , "WIDTH"      , "LEFT/RIGHT" , "TOP/BOTTOM" ,
    "LENS SHAD." , "2DNR"       , "MIRROR"     , "FONT COLOR" , "CONTRAST"   ,
    "SHARPNESS"  , "DISPLAY"    , "NEG. IMAGE" , "FONT"       , "ID&TITLE"   ,
    "GAMMA"      , "PED LEVEL"  , "COLOR GAIN" , "GAMMA"      , "PED LEVEL"  ,
    "COLOR GAIN" , "PED LEVEL"  , "COLOR GAIN" , "FACTORY"
};

char * const MenuNameOSDB[] = {
    "IMAGE"        , "DAY&NIGHT" , "VIDEO STANDARD" , "LANGUAGE"      , "LOAD DEFAULT" ,  
    "SOFT VERSION" , "SAVE&EXIT" , "WDR"            , "IMAGE ENHANCE" , "MIRROR"       , 
    "BRIGHTNESS"   , "ZOOM IN"   , "SHARPNESS"      , "DETAIL"        , "VALUE"        , 
    "EDGE"         , "VALUE"     , "SATURATION"     , "VALUE"         , "MAXGAIN"      , 
    "3D-NR"        , "NR-MODE"   , "VALUE" 
};


char * getMenuNameById(uint8_t menuId){
    if( camModelId == MicroSwift 
        || camModelId == MicroSwift2 
        || camModelId == OWL 
        || camModelId == OWLPlus 
        || camModelId == OWL2 
        || camModelId == Swift 
        || camModelId == Swift2 
        || camModelId == SwiftMini 
        || camModelId == SwiftRR 
    ){
        return MenuNameOSDA[menuId];
    }else if(camModelId == Eagle || camModelId == Eagle2){
        return MenuNameOSDB[menuId];
    }
    return "Unknown";
}


OSD_TAB_t getOSDTabById(uint8_t menuId,uint8_t menuVal){
    if( camModelId == MicroSwift 
        || camModelId == MicroSwift2 
        || camModelId == OWL 
        || camModelId == OWLPlus 
        || camModelId == OWL2 
        || camModelId == Swift 
        || camModelId == Swift2 
        || camModelId == SwiftMini 
        || camModelId == SwiftRR 
    ){
        switch(menuId){
            case CAMTITLE:
                cmsc_CamTitle = menuVal;
                return cmsc_mCamTitleTab;
            break;
            case DWDR:
                cmsc_DWDR = menuVal;
                return cmsc_mDWDRTab;
            break;
            case AREASTATEA:
                cmsc_AreaStateA = menuVal;
                return cmsc_mAreaStateATab;
            break;
            case AREASTATEB:
                cmsc_AreaStateB = menuVal;
                return cmsc_mAreaStateBTab;
            break;
            case BURST:
                cmsc_Burst = menuVal;
                return cmsc_mBurstTab;
            break;
            case IRSMART:
                cmsc_IRSmart = menuVal;
                return cmsc_mIRSmartTab;
            break;
            case LENSSHAD:
                cmsc_LensShad = menuVal;
                return cmsc_mLensShadTab;
            break;
            case TWODNR:
                cmsc_TWODNR = menuVal;
                return cmsc_mTWODNRTab;
            break;
            case MIRROR:
                cmsc_Mirror = menuVal;
                return cmsc_mMirrorTab;
            break;
            case NEGIMAGE:
                cmsc_NegImage = menuVal;
                return cmsc_mNegImageTab;
            break;
            case SHUTTER:
                cmsc_Shutter = menuVal;
                return cmsc_mShutterTab;
            break;
            case AGC:
                cmsc_AGC = menuVal;
                return cmsc_mAGCTab;
            break;
            case WHITEBAL:
                cmsc_Whitebal = menuVal;
                return cmsc_mWhitebalTab;
            break;
            case COLORTEMP:
                cmsc_Colortemp = menuVal;
                return cmsc_mColortempTab;
            break;
            case BACKLIGHT:
                cmsc_BackLight = menuVal;
                return cmsc_mBackLightTab;
            break;
            case AREASELA:
            case AREASELB:
                cmsc_AreaSel = menuVal;
                return cmsc_mAreaSelTab;
            break;
            case HLCMODE:
                cmsc_HLCMode = menuVal;
                return cmsc_mHLCModeTab;
            break;
            case DAYNIGHT:
                cmsc_DayNight = menuVal;
                return cmsc_mDayNightTab;
            break;
            case IRLEVEL:
                cmsc_IRLevel = menuVal;
                return cmsc_mIRLevelTab;
            break;
            case DISPLAY:
                cmsc_Display = menuVal;
                return cmsc_mDisplayTab;
            break;
            case DNDELAY:
                cmsc_DNDelay = menuVal;
                return cmsc_mDNDelayTab;
            break;
            case NDDELAY:
                cmsc_NDDelay = menuVal;
                return cmsc_mNDDelayTab;
            break;
        }
    }else if(camModelId == Eagle || camModelId == Eagle2){
        switch(menuId){
            case TDAYNIGHT:
             cmsc_DayNight = menuVal;
             return cmsc_mDayNightTab;
        break;
        case WDR:
             cmsc_DWDR = menuVal;
             return cmsc_mDWDRTab;
        break;
        case TMIRROR:
            cmsc_Mirror = menuVal;
            return cmsc_mMirrorTab;
        break;
        case VIDEOSTANDARD:
            cmsc_VideoStandard = menuVal;
            return cmsc_mVideoStandardTab;
        break;
        case DETAIL:
            cmsc_Detail = menuVal;
            return cmsc_mDetailTab;
        break;
        case EDGE:
            cmsc_Edge = menuVal;
            return cmsc_mEdgeTab;
        break;
        case SATURATION:
            cmsc_Saturation = menuVal;
            return cmsc_mSaturationTab;
        break;
        case THREEDNR:
            cmsc_THREEDNR = menuVal;
            return cmsc_mTHREEDNRTab;
        break;
        }
    }
}

typedef enum {
    RCSPLIT_RECV_STATUS_WAIT_HEADER = 0,
    RCSPLIT_RECV_STATUS_WAIT_COMMAND,
    RCSPLIT_RECV_STATUS_WAIT_LENGTH,
    RCSPLIT_RECV_STATUS_WAIT_DATA,
    RCSPLIT_RECV_STATUS_WAIT_CRC
} rcsplitRecvStatus_e;

static int rcsplitReceivePos = 0;
rcsplitRecvStatus_e rcsplitReceiveState = RCSPLIT_RECV_STATUS_WAIT_HEADER;
void rcCameraCmsUpdateMenu(void);
OSD_Entry * findOSDEntryById(uint8_t menuId,CMS_Menu mCMS_Menu);

uint8_t crc_high_first(uint8_t *ptr, uint8_t len)
{
    uint8_t i; 
    uint8_t crc=0x00;
    while (len--) {
        crc ^= *ptr++;  
        for (i=8; i>0; --i) { 
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x31;
            else
                crc = (crc << 1);
        }
    }
    return (crc); 
}

static void sendCtrlCommand(rcsplit_ctrl_argument_e argument)
{
    if (!rcSplitSerialPort)
        return ;

    if (argument == 1) {
        // uint8_t screenBuffer[RCCAMERA_SCREEN_CHARACTER_COLUMN_COUNT * RCCAMERA_SCREEN_CHARACTER_ROW_COUNT] = {
        // 'A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A',
        // 'A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A',
        // 'A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A',
        // 'A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A',
        // 'A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A',
        // 'A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A',
        // 'A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A',
        // 'A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A',
        // 'A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A',
        // 'A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A',
        // 'A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A',
        // };

        // sbuf_t buf;
        // uint16_t expectedPacketSize = rcCamOSDGenerateDrawScreenPacket(NULL, screenBuffer);
        // uint8_t *base = (uint8_t*)malloc(expectedPacketSize);
        // buf.ptr = base;
        // uint16_t actualPacketSize = rcCamOSDGenerateDrawScreenPacket(&buf, screenBuffer);
        // serialWriteBuf(rcSplitSerialPort, base, actualPacketSize);
        // return ;

        // for (int i = 0; i < 16; i++) {
        //     sbuf_t buf;
        //     uint8_t *base = NULL;
        //     uint16_t expectedPacketSize = 0, actualPacketSize = 0;
        //     expectedPacketSize = rcCamOSDGenerateDrawStringPacket(NULL, 0, i, "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAA", 30);
        //     base = (uint8_t*)malloc(expectedPacketSize);
        //     buf.ptr = base;
        //     actualPacketSize = rcCamOSDGenerateDrawStringPacket(&buf, 0, i, "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAA", 30);
        //     serialWriteBuf(rcSplitSerialPort, base, actualPacketSize);
        //     free(base);
        // }

        // display logo and help
        // int x = 3;
        // int y = 1;
        // char fontOffset = 160;
        // for (int row = 0; row < 4; row++) {
        //     for (int column = 0; column < 24; column++) {
        //         if (fontOffset != 255)  {// FIXME magic number
        //             sbuf_t buf;
        //             uint16_t expectedPacketSize = 0;
        //             uint16_t actualPacketSize = 0;
        //             uint8_t *base = NULL;

        //             fontOffset++;
        //             expectedPacketSize = rcCamOSDGenerateDrawStringPacket(NULL, x + column, y + row, "?", 1);
        //             base = (uint8_t*)malloc(expectedPacketSize);
        //             buf.ptr = base;
        //             actualPacketSize = rcCamOSDGenerateDrawStringPacket(&buf, x + column, y + row, "?", 1);
        //             serialWriteBuf(rcSplitSerialPort, base, actualPacketSize);
        //             free(base);
        //         }
        //     }
        // }

        // return ;
    }
    // beeperConfirmationBeeps(3);
    uint8_t uart_buffer[5] = {0};
    uint8_t crc = 0;

    uart_buffer[0] = RCSPLIT_PACKET_HEADER;
    uart_buffer[1] = RCSPLIT_PACKET_CMD_CTRL;
    uart_buffer[2] = argument;
    uart_buffer[3] = RCSPLIT_PACKET_TAIL;
    crc = crc_high_first(uart_buffer, 4);

    // build up a full request [header]+[command]+[argument]+[crc]+[tail]
    uart_buffer[3] = crc;
    uart_buffer[4] = RCSPLIT_PACKET_TAIL;

    // write to device
    serialWriteBuf(rcSplitSerialPort, uart_buffer, 5);
}

static void rcSplitProcessMode() 
{
    // if the device not ready, do not handle any mode change event
    if (RCSPLIT_STATE_IS_READY != cameraState) 
        return ;

    for (boxId_e i = BOXCAMERA1; i <= BOXCAMERA3; i++) {
        uint8_t switchIndex = i - BOXCAMERA1;
        if (IS_RC_MODE_ACTIVE(i)) {
            // check last state of this mode, if it's true, then ignore it. 
            // Here is a logic to make a toggle control for this mode
            if (switchStates[switchIndex].isActivated) {
                continue;
            }

            uint8_t argument = RCSPLIT_CTRL_ARGU_INVALID;
            switch (i) {
            case BOXCAMERA1:
                argument = RCSPLIT_CTRL_ARGU_WIFI_BTN;
                break;
            case BOXCAMERA2:
                argument = RCSPLIT_CTRL_ARGU_POWER_BTN;
                break;
            case BOXCAMERA3:
                argument = RCSPLIT_CTRL_ARGU_CHANGE_MODE;
                break;
            default:
                argument = RCSPLIT_CTRL_ARGU_INVALID;
                break;
            }
            
            if (argument != RCSPLIT_CTRL_ARGU_INVALID) {
                sendCtrlCommand(argument);
                switchStates[switchIndex].isActivated = true;
            }
        } else {
            switchStates[switchIndex].isActivated = false;
        }
    }
}

static void retriveCameraInfo()
{
    sbuf_t buf;
    uint16_t expectedPacketSize = 0;
    uint8_t *base = NULL;
    expectedPacketSize = rcCamOSDGenerateGetCameraInfoPacket(NULL);
    base = (uint8_t*)malloc(expectedPacketSize);
    buf.ptr = base;
    
    rcCamOSDGenerateGetCameraInfoPacket(&buf);
    serialWriteBuf(rcSplitSerialPort, base, expectedPacketSize);
    free(base);
}

bool rcSplitInit(void)
{
    // found the port config with FUNCTION_RUNCAM_SPLIT_CONTROL
    // User must set some UART inteface with RunCam Split at peripherals column in Ports tab
    if (rcSplitSerialPort == NULL) {
        serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RCSPLIT);
        if (portConfig) {
            rcSplitSerialPort = openSerialPort(portConfig->identifier, FUNCTION_RCSPLIT, NULL, 115200, MODE_RXTX, 0);
        }
    }

    if (!rcSplitSerialPort) {
        return false;
    }

    cameraState = RCSPLIT_STATE_INITIALIZING;

    // set init value to true, to avoid the action auto run when the flight board start and the switch is on.
    for (boxId_e i = BOXCAMERA1; i <= BOXCAMERA3; i++) {
        uint8_t switchIndex = i - BOXCAMERA1;
        switchStates[switchIndex].boxId = 1 << i;
        switchStates[switchIndex].isActivated = true; 
    }
    
    retriveCameraInfo();

#ifdef USE_RCSPLIT
    setTaskEnabled(TASK_RCSPLIT, true);
#endif

    return true;
}

static void rcsplitResetReceiver()
{
    rcsplitReceiveState = RCSPLIT_RECV_STATUS_WAIT_HEADER;
    rcsplitReceivePos = 0;
}

static void rcsplitHandleResponse(void)
{
    uint8_t commandID = rcsplitRespBuffer[1] & 0x0F;
    uint8_t dataLen = rcsplitRespBuffer[2];
    switch (commandID) {
        case RCSPLIT_PACKET_CMD_GET_CAMERA_INFO:
            {
                if (dataLen < sizeof(rcsplit_osd_camera_info_t))
                    return ;
                    
                memcpy(&cameraInfo, rcsplitRespBuffer + 3, sizeof(rcsplit_osd_camera_info_t));

                cameraState = RCSPLIT_STATE_IS_READY;
            }
            break;
        case RCSPLIT_PACKET_CMD_GET_CONFIGURATIONS:
            {
                //data封包为：<camModelId><counts> <m1-menuid><m1-menutype>
                camModelId = rcsplitRespBuffer[3];//camera Type
                printf("CONFIGURATIONS camModelId   : %d\n",camModelId);
                uint8_t counts = rcsplitRespBuffer[4];//menuItem counts
                uint8_t *dataBuffer = rcsplitRespBuffer + 5;
                uint8_t pos = 0;
                
                if (menuItems == NULL) {
                    menuItems = malloc(sizeof(rcsplit_cms_menu_data_t) * counts);
                }

                printf("CONFIGURATIONS camModelName : %s\n",CamModeName[camModelId]);
                printf("CONFIGURATIONS counts       : %d\n",counts);

                while (pos < (dataLen-2)){
                    menuItems[menuItemCount].menuId = dataBuffer[pos++];
                    printf("CONFIGURATIONS menuId   : %d\n",menuItems[menuItemCount].menuId);
                    printf("CONFIGURATIONS menuName : %s\n",getMenuNameById(menuItems[menuItemCount].menuId));
                    menuItems[menuItemCount].menuType = dataBuffer[pos++];
                    printf("CONFIGURATIONS menuType : %d\n",menuItems[menuItemCount].menuType);
                    menuItemCount++;
                }

                // create OSD_Entry
                char menuTitleName[25] = "SETUP - ";
                strcat(menuTitleName, CamModeName[camModelId]);
                OSD_Entry *menuEntries = (OSD_Entry *)malloc(sizeof(OSD_Entry) * 3 + menuItemCount);
                OSD_Entry headerEntry = {menuTitleName, OME_Label, NULL, NULL, 0 };
                memcpy(&(menuEntries[0]), &headerEntry, sizeof(OSD_Entry));

                for (int i = 0; i < menuItemCount;i++) {
                    rcsplit_cms_menu_data_t menuData = menuItems[i];
                    rcsplit_cms_menu_entry_data_t *entryData = malloc(sizeof(rcsplit_cms_menu_entry_data_t));
                    entryData->menuId = menuItems[menuItemCount].menuId;
                    entryData->data = NULL;

                    OSD_MenuElement type;
                    CMSEntryFuncPtr func;
                    switch(menuData.menuType){
                        case MENUSTRING://OME_String:只显示值
                            type = OME_String;
                            func = NULL;
                        break;
                        case MENUTAB://OME_TAB:TAB选项(包括开关)
                            type = OME_TAB;
                            func = runcamCmsSetTabConfigValue;
                        break;
                        case MENUUINT8://OME_UINT8:整型选择值范围
                            type = OME_UINT8;
                            func = runcamCmsSetUintConfigValue;
                        break;
                        case MENUFLOAT://OME_FLOAT:浮点型选择值范围
                            type = OME_FLOAT;
                            func = runcamCmsSetFloatConfigValue;
                        break;
                        case MENUSUBMENU://OME_Submenu:下级菜单
                            type = OME_Submenu;
                            func = cmsMenuChange;
                            //有下级菜单，可以去请求该下级菜单
                            //请求下一级菜单的接口请求FUNCTION

                        break;
                    }

                    OSD_Entry menuEntry = { getMenuNameById(menuData.menuId), type, func, entryData, 0 };
                    memcpy(&(menuEntries[1 + i]), &menuEntry, sizeof(OSD_Entry));
                }
                OSD_Entry backEntry = { "EXIT", OME_Back, NULL, NULL, 0 };
                memcpy(&(menuEntries[1 + menuItemCount]), &backEntry, sizeof(OSD_Entry));
                OSD_Entry lastEntry = { NULL, OME_END, NULL, NULL, 0 };
                memcpy(&(menuEntries[2 + menuItemCount]), &lastEntry, sizeof(OSD_Entry));

                cmsx_menuCameraRuncam.entries = menuEntries;
            }
            break;
            case RCSPLIT_PACKET_CMD_GET_CONFIGURATION_ITEMS:
            {
                //data封包为：<upMenuId><counts> <m1-menuid><m1-menutype>
                uint8_t upMenuId = rcsplitRespBuffer[3];//上一级MENU_ID
                uint8_t counts = rcsplitRespBuffer[4];//数据条目总数
                uint8_t *dataBuffer = rcsplitRespBuffer + 5;
                uint8_t pos = 0;

                //存放所有子菜单项
                uint8_t nextMenuItemCount = 0;
                rcsplit_cms_menu_data_t *nextMenuItems = malloc(sizeof(rcsplit_cms_menu_data_t) * counts);

                while (pos < (dataLen-2)){
                    nextMenuItems[nextMenuItemCount].menuId = dataBuffer[pos++];
                    printf("CONFIGURATIONS item menuId   : %d\n",nextMenuItems[nextMenuItemCount].menuId);
                    printf("CONFIGURATIONS menuName : %s\n",getMenuNameById(nextMenuItems[nextMenuItemCount].menuId));
                    nextMenuItems[nextMenuItemCount].menuType = dataBuffer[pos++];
                    printf("CONFIGURATIONS item menuType : %d\n",nextMenuItems[nextMenuItemCount].menuType);
                    nextMenuItemCount++;
                }

                printf("CONFIGURATIONS upMenuName   : %s\n",getMenuNameById(upMenuId));

                //create OSD_Entry
                OSD_Entry *nextMenuItemEntries = (OSD_Entry *)malloc(sizeof(OSD_Entry) * 3 + nextMenuItemCount);
                OSD_Entry headerEntry = {getMenuNameById(upMenuId), OME_Label, NULL, NULL, 0 };
                memcpy(&(nextMenuItemEntries[0]), &headerEntry, sizeof(OSD_Entry)); 

                for (int i = 0; i < nextMenuItemCount;i++) {
                    rcsplit_cms_menu_data_t menuData = nextMenuItems[i];
                    rcsplit_cms_menu_entry_data_t *entryData = malloc(sizeof(rcsplit_cms_menu_entry_data_t));
                    entryData->menuId = nextMenuItems[nextMenuItemCount].menuId;
                    entryData->data = NULL;

                    OSD_MenuElement type;
                    CMSEntryFuncPtr func;
                    switch(menuData.menuType){
                        case MENUSTRING://OME_String:只显示值
                            type = OME_String;
                            func = NULL;
                        break;
                        case MENUTAB://OME_TAB:选项(包括开关)
                            type = OME_TAB;
                            func = runcamCmsSetTabConfigValue;
                        break;
                        case MENUUINT8://OME_UINT8:选择值范围
                            type = OME_UINT8;
                            func = runcamCmsSetUintConfigValue;
                        break;
                        case MENUFLOAT://OME_FLOAT:浮点型选择值范围
                            type = OME_FLOAT;
                            func = runcamCmsSetFloatConfigValue;
                        break;
                        case MENUSUBMENU://OME_Submenu:下级菜单
                            type = OME_Submenu;
                            func = cmsMenuChange;
                            //有下级菜单，可以去请求该下级菜单
                            //请求下一级菜单的接口请求FUNCTION
                            
                        break;
                    }
                    
                    OSD_Entry menuEntry = { getMenuNameById(menuData.menuId), type, func, entryData, 0 };
                    memcpy(&(nextMenuItemEntries[1 + i]), &menuEntry, sizeof(OSD_Entry));
                }

                OSD_Entry backEntry = { "RETURN", OME_Back, NULL, NULL, 0 };
                memcpy(&(nextMenuItemEntries[1 + nextMenuItemCount]), &backEntry, sizeof(OSD_Entry));
                OSD_Entry lastEntry = { NULL, OME_END, NULL, NULL, 0 };
                memcpy(&(nextMenuItemEntries[2 + nextMenuItemCount]), &lastEntry, sizeof(OSD_Entry));
                //create CMS_Menu
                CMS_Menu cmsx_nextMenuCameraRuncam = {"CAMERARUNCAM",OME_MENU,NULL,NULL,NULL,nextMenuItemEntries};
                
                printf("------------------------------------------------0\n");

                OSD_Entry* upMenuOSDEntry = findOSDEntryById(upMenuId,cmsx_menuCameraRuncam);
                upMenuOSDEntry->data = &cmsx_nextMenuCameraRuncam;
            }
            break;
        case RCSPLIT_PACKET_CMD_GET_CONFIGURATIONS_VALUES:
            {
                uint8_t strCounts = rcsplitRespBuffer[3];//string datas size
                uint8_t tabCounts = rcsplitRespBuffer[4];//tab datas size
                uint8_t unitCounts = rcsplitRespBuffer[5];//unit datas size
                uint8_t floatCounts = rcsplitRespBuffer[6];//unit datas size
                uint8_t *dataBuffer = rcsplitRespBuffer + 7;

                printf("CONFIGURATIONS strCounts  : %d\n",strCounts);
                printf("CONFIGURATIONS tabCounts  : %d\n",tabCounts);
                printf("CONFIGURATIONS unitCounts : %d\n",unitCounts);
                printf("CONFIGURATIONS floatCounts  : %d\n",floatCounts);

                uint8_t pos = 0;
                uint8_t menuValueItemCount = 0;
                runcam_OSD_String_t *menuStrValueItems = malloc(sizeof(runcam_OSD_String_t) * strCounts);
                runcam_OSD_TAB_t *menuTabValueItems = malloc(sizeof(runcam_OSD_TAB_t) * tabCounts);
                runcam_OSD_UINT8_t *menuUnitValueItems = malloc(sizeof(runcam_OSD_UINT8_t) * unitCounts);
                runcam_OSD_FLOAT_t *menuFloatValueItems = malloc(sizeof(runcam_OSD_FLOAT_t) * floatCounts);

                //data封包为：<strCounts><tabCounts><unitCounts><floatCounts> + 1/2/3/4
                //1. <m1-menuValType><m1-menuId><m1-menuValLength><m1-*menuVal>
                //2. <m1-menuValType><m1-menuId><m1-menuVal>
                //3. <m1-menuValType><m1-menuId><m1-valMin><m1-valMax><m1-valStep><m1-menuVal>
                //4. <m1-menuValType><m1-menuId><m1-valMin><m1-valMax><m1-valStep><m1-valMultipler><m1-menuVal>

                while (pos < (dataLen-4)){
                    uint8_t menuValType = dataBuffer[pos++];

                    switch(menuValType){
                        case MENUSTRING://OME_String
                            {
                                menuStrValueItems[menuValueItemCount].menuId = dataBuffer[pos++];
                                printf("CONFIGURATIONS VALUE menuId  : %d\n",menuStrValueItems[menuValueItemCount].menuId);
                                menuStrValueItems[menuValueItemCount].menuValLength = dataBuffer[pos++];
                                char *menuVal = malloc(sizeof(char) * menuStrValueItems[menuValueItemCount].menuValLength);
                                memcpy(menuVal, &(dataBuffer[pos]), menuStrValueItems[menuValueItemCount].menuValLength);
                                pos += menuStrValueItems[menuValueItemCount].menuValLength;
                                menuStrValueItems[menuValueItemCount].menuVal = menuVal;
                                printf("CONFIGURATIONS item menuVal : %s\n",menuStrValueItems[menuValueItemCount].menuVal);
                                //fill value
                                // OSD_Entry* upMenuOSDEntry = findOSDEntryById(menuStrValueItems[menuValueItemCount].menuId,cmsx_menuCameraRuncam);
                                // upMenuOSDEntry->data = menuVal;
                            }
                            break;
                        case MENUTAB://OME_TAB
                            {
                                menuTabValueItems[menuValueItemCount].menuId = dataBuffer[pos++];
                                printf("CONFIGURATIONS VALUE menuId  : %d\n",menuTabValueItems[menuValueItemCount].menuId);
                                menuTabValueItems[menuValueItemCount].menuVal = dataBuffer[pos++];
                                printf("CONFIGURATIONS VALUE menuVal : %d\n",menuTabValueItems[menuValueItemCount].menuVal);

                                // OSD_TAB_t cmsx_mTable = getOSDTabById(menuTabValueItems[menuValueItemCount].menuId,menuTabValueItems[menuValueItemCount].menuVal);
                                // OSD_Entry* upMenuOSDEntry = findOSDEntryById(menuUnitValueItems[menuValueItemCount].menuId,cmsx_menuCameraRuncam);
                                // upMenuOSDEntry->data = &cmsx_mTable;
                            }
                            break;
                        case MENUUINT8://OME_UINT8
                            {
                                menuUnitValueItems[menuValueItemCount].menuId = dataBuffer[pos++];
                                printf("CONFIGURATIONS VALUE menuId  : %d\n",menuUnitValueItems[menuValueItemCount].menuId);
                                menuUnitValueItems[menuValueItemCount].valMin = dataBuffer[pos++];
                                printf("CONFIGURATIONS VALUE valMin  : %d\n",menuUnitValueItems[menuValueItemCount].valMin);
                                menuUnitValueItems[menuValueItemCount].valMax = dataBuffer[pos++];
                                printf("CONFIGURATIONS VALUE valMax  : %d\n",menuUnitValueItems[menuValueItemCount].valMax);
                                menuUnitValueItems[menuValueItemCount].valStep = dataBuffer[pos++];
                                printf("CONFIGURATIONS VALUE valStep : %d\n",menuUnitValueItems[menuValueItemCount].valStep);
                                menuUnitValueItems[menuValueItemCount].menuVal = dataBuffer[pos++];
                                printf("CONFIGURATIONS VALUE menuVal : %d\n",menuUnitValueItems[menuValueItemCount].menuVal);
                                //当前值，范围最小值，范围最大值，一步值
                                OSD_UINT8_t saCmsEntFreqRef = {
                                    &menuUnitValueItems[menuValueItemCount].menuVal, 
                                    menuUnitValueItems[menuValueItemCount].valMin, 
                                    menuUnitValueItems[menuValueItemCount].valMax, 
                                    menuUnitValueItems[menuValueItemCount].valStep 
                                };

                                // OSD_Entry* upMenuOSDEntry = findOSDEntryById(menuUnitValueItems[menuValueItemCount].menuId,cmsx_menuCameraRuncam);
                                // upMenuOSDEntry->data = &saCmsEntFreqRef;
                            }
                            break;
                        case MENUFLOAT://OME_FLOAT
                            {
                                menuFloatValueItems[menuValueItemCount].menuId = dataBuffer[pos++];
                                printf("CONFIGURATIONS VALUE menuId  : %d\n",menuFloatValueItems[menuValueItemCount].menuId);
                                menuFloatValueItems[menuValueItemCount].valMin = dataBuffer[pos++];
                                printf("CONFIGURATIONS VALUE valMin  : %d\n",menuFloatValueItems[menuValueItemCount].valMin);
                                menuFloatValueItems[menuValueItemCount].valMax = dataBuffer[pos++];
                                printf("CONFIGURATIONS VALUE valMax  : %d\n",menuFloatValueItems[menuValueItemCount].valMax);
                                menuFloatValueItems[menuValueItemCount].valStep = dataBuffer[pos++];
                                printf("CONFIGURATIONS VALUE valStep : %d\n",menuFloatValueItems[menuValueItemCount].valStep);
                                menuFloatValueItems[menuValueItemCount].valMultipler = dataBuffer[pos++];
                                printf("CONFIGURATIONS VALUE valMultipler : %d\n",menuFloatValueItems[menuValueItemCount].valMultipler);
                                menuFloatValueItems[menuValueItemCount].menuVal = dataBuffer[pos++];
                                printf("CONFIGURATIONS VALUE menuVal : %d\n",menuFloatValueItems[menuValueItemCount].menuVal);
                                
                                OSD_FLOAT_t saCmsEntFreqRef = {
                                    &menuFloatValueItems[menuValueItemCount].menuVal,
                                    menuFloatValueItems[menuValueItemCount].valMin,
                                    menuFloatValueItems[menuValueItemCount].valMax,
                                    menuFloatValueItems[menuValueItemCount].valStep,
                                    menuFloatValueItems[menuValueItemCount].valMultipler
                                };

                                // OSD_Entry* upMenuOSDEntry = findOSDEntryById(menuFloatValueItems[menuValueItemCount].menuId,cmsx_menuCameraRuncam);
                                // upMenuOSDEntry->data = &saCmsEntFreqRef;
                            }
                        break;
                    }
                    menuValueItemCount++;
                }
            }
            break;
        case RCSPLIT_PACKET_CMD_SET_CONFIGURATIONS_VALUE://设置参数后返回结果
            {
                
            }
            break;
    }

    return ;
}

OSD_Entry * findOSDEntryById(uint8_t findMenuId,CMS_Menu findCMSMenu)
{
    printf("---------------------------------99");
    OSD_Entry *menuEntries = findCMSMenu.entries;
    if(menuEntries == NULL){
        return NULL;
    }
    int i = 0;
    bool flag = true;
    while(flag){
        printf("menuEntries[i].text：%s",menuEntries[i].text);
        if(menuEntries[i].text == NULL){
            flag = false;
            return NULL;
            printf("menuEntries[i].text == NULL\n");
        }
        if(menuEntries[i].type == OME_Submenu){
            if(findMenuId == (((rcsplit_cms_menu_entry_data_t *) menuEntries[i].data)->menuId)){
                flag = false;
                printf("findOSDEntryById() find a menuEntries\n");
                return &menuEntries[i];
            }else{
                CMS_Menu *nextCMSMenu = ((CMS_Menu *) ((rcsplit_cms_menu_entry_data_t *) menuEntries[i].data)->data);
                if(nextCMSMenu != NULL){
                    findOSDEntryById(findMenuId,*nextCMSMenu);
                }
            }
        }
        i++;
    }
}


void rcSplitReceive(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    if (!rcSplitSerialPort)
        return ;

    while (serialRxBytesWaiting(rcSplitSerialPort)) {
        uint8_t c = serialRead(rcSplitSerialPort);
        rcsplitRespBuffer[rcsplitReceivePos++] = c;
        switch(rcsplitReceiveState) {
            case RCSPLIT_RECV_STATUS_WAIT_HEADER:
                if (c == RCSPLIT_PACKET_HEADER) {
                    rcsplitReceiveState = RCSPLIT_RECV_STATUS_WAIT_COMMAND;
                    printf("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA\n");
                } else {
                    rcsplitReceivePos = 0;
                }
                break;

            case RCSPLIT_RECV_STATUS_WAIT_COMMAND:
                {
                    uint8_t deviceID = (c & 0xF0) >> 4;
                    if (deviceID != 0x2) { // not camera device id
                        rcsplitResetReceiver();
                    } else {
                        rcsplitReceiveState = RCSPLIT_RECV_STATUS_WAIT_LENGTH; 
                        printf("BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB\n");
                    }
                }
                break;

            case RCSPLIT_RECV_STATUS_WAIT_LENGTH:
                rcsplitReceiveState = RCSPLIT_RECV_STATUS_WAIT_DATA;
                printf("CCCCCCCCCCCCCCCCCCCCCCCCCCCC\n");
                resplitRespDataBlockLength = c;
                break;

            case RCSPLIT_RECV_STATUS_WAIT_DATA:
                if ((rcsplitReceivePos - resplitRespDataBlockLength) == 3) {
                    rcsplitReceiveState = RCSPLIT_RECV_STATUS_WAIT_CRC;
                    printf("DDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD\n");
                } 
                break;
            
            case RCSPLIT_RECV_STATUS_WAIT_CRC:
            {
                uint8_t crc = crc_high_first(rcsplitRespBuffer, rcsplitReceivePos - 1);
                printf("---------------------------->c   ：%d\n", c);
                printf("---------------------------->crc ：%d\n", crc);
                if (crc != c) {
                    printf("EEEEEEEEEEEEEEEEEEEEEEEEEEEEEE\n");
                } else {
                    printf("FFFFFFFFFFFFFFFFFFFFFFFFFFFFF\n");
                    rcsplitHandleResponse();
                }
                rcsplitResetReceiver();
            }
                break;

            default:
                rcsplitResetReceiver();
            }
    }

    return ;
}

void rcSplitProcess(timeUs_t currentTimeUs)
{

    UNUSED(currentTimeUs);

    if (rcSplitSerialPort == NULL)
        return ;

    rcSplitReceive(currentTimeUs);

    // process rcsplit custom mode if has any changed
    rcSplitProcessMode();
}

static OSD_Entry runcamMenuEntries[] =
{
    { "- SETUP -", OME_Label, NULL, NULL, 0 },
    { "EXIT",   OME_Back, NULL, NULL, 0 },
    { NULL,     OME_END, NULL, NULL, 0 }
};

CMS_Menu cmsx_menuCameraRuncam = {
    .GUARD_text = "CAMERARUNCAM",
    .GUARD_type = OME_MENU,
    .onEnter = NULL,
    .onExit = NULL,
    .onGlobalExit = NULL,
    .entries = runcamMenuEntries,
};