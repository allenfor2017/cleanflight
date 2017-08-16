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
#include "common/time.h"
#include "fc/fc_msp.h"
#include "io/rcsplit_types.h"

typedef enum {
    MENUSTRING = 1,
    MENUTAB,
    MENUUINT8,
    MENUFLOAT,
    MENUSUBMENU
} camMenuType_e;

typedef enum {
    Swift = 0,
    Swift2,
    SwiftMini,
    SwiftRR,
    OWL,
    OWLPlus,
    OWL2,
    Eagle,
    Eagle2,
    MicroSwift,
    MicroSwift2,
    RunCamHD,
    RunCam2,
    RunCam3,
    RunCamSplit
} camModel_e;

enum OSDTypeA{
//MAIN
    CAMTITLE = 0,
    EXPOSURE,
    WHITEBAL,
    BACKLIGHT,
    DAYNIGHT,
    IMAGEADJ,
    DPC,
    LANGUAGE,
    MENURESET,
//EXPOSURE
    SHUTTER,
    BRIGHTNESS,
    AGC,
    DWDR,
//EXPOSURE - DWDR
    DWDRLEVEL,
//WHITE-BAL - MANUAL
    COLORTEMP,
    BLUE,
    RED,
//BACKLIGHT - HLC
    HLCLEVEL,
    HLCMODE,
//BACKLIGHT - BLC - A
    AREASELA,
    AREASTATEA,
    GAINA,
    HEIGHTA,
    WIDTHA,
    LEFTRIGHTA,
    TOPBOTTOMA,
//BACKLIGHT - BLC - B
    AREASELB,
    AREASTATEB,
    GAINB,
    HEIGHTB,
    WIDTHB,
    LEFTRIGHTB,
    TOPBOTTOMB,
//DAY-NIGHT - AUTO
    DNLEVEL,
    DNDELAY,
    NDLEVEL,
    NDDELAY,
//DAY-NIGHT - B/W
    BURST,
    IRSMART,
    IRLEVEL,
//DAY-NIGHT - B/W - IR SMART
    IRGAIN,
    IRHEIGHT,
    IRWIDTH,
    IRLEFTRIGHT,
    IRTOPBOTTOM,
//IMAGE-ADJ
    LENSSHAD,
    TWODNR,
    MIRROR,
    FONTCOLOR,
    CONTRAST,
    SHARPNESS,
    DISPLAY,
    NEGIMAGE,
//IMAGE-ADJ - FONT-COLOR
    FONT,
    IDTITLE,
//DISPLAY - LCD
    LCDGAMMA,
    LCDPEDLEVEL,
    LCDCOLORGAIN,
//DISPLAY - USER
    USERGAMMA,
    USERPEDLEVEL,
    USERCOLORGAIN,
//DISPLAY - CRT
    CRTPEDLEVEL,
    CRTCOLORGAIN,
//RESET
    FACTORY,
};

enum OSDTypeB{
//MAIN
    IMAGE = 0,
    TDAYNIGHT,
    VIDEOSTANDARD,
    TLANGUAGE,
    LOADDEFAULT,
    SOFTVERSION,
    SAVEEXIT,
//IMAGE
    WDR,
    IMAGEENHANCE,
    TMIRROR,
    TBRIGHTNESS,
    ZOOMIN,
//IMAGEENHANCE
    TSHARPNESS,
    DETAIL,
    DETAILVALUE,
    EDGE,
    EDGEVALUE,
    SATURATION,
    SATURATIONVALUE,
//eagle 2
    MAXGAIN,
//eagle
    THREEDNR,
    NRMODE,
    NRMODEVALUE
};

bool rcSplitInit(void);
void rcSplitProcess(timeUs_t currentTimeUs);

// only for unit test
extern rcsplit_state_e cameraState;
extern serialPort_t *rcSplitSerialPort;
extern rcsplit_switch_state_t switchStates[BOXCAMERA3 - BOXCAMERA1 + 1];

#ifdef CMS
#include "cms/cms.h"
#include "cms/cms_types.h"
extern CMS_Menu cmsx_menuCameraRuncam;
#endif