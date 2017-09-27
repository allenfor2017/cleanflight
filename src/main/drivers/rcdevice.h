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
 
 #include "common/streambuf.h"
 #include "drivers/serial.h"
 
 #define RCDEVICE_PROTOCOL_HEADER                        0xCC
 
 
 #define RCDEVICE_PROTOCOL_VERSION_STRING_LENGTH         10
 #define RCDEVICE_PROTOCOL_MAX_DATA_SIZE                 62
 
 // Commands
 #define RCDEVICE_PROTOCOL_COMMAND_GET_DEVICE_INFO                   0x00
 // camera control
 #define RCDEVICE_PROTOCOL_COMMAND_CAMERA_CONTROL                    0x01
 // 5 key osd cable simulation
 #define RCDEVICE_PROTOCOL_COMMAND_5KEY_SIMULATION_PRESS             0x02
 #define RCDEVICE_PROTOCOL_COMMAND_5KEY_SIMULATION_RELEASE           0x03
 #define RCDEVICE_PROTOCOL_COMMAND_5KEY_CONNECTION                   0x04
 // device setting access
 #define RCDEVICE_PROTOCOL_COMMAND_GET_SETTINGS                      0x10
 #define RCDEVICE_PROTOCOL_COMMAND_READ_SETTING_DETAIL               0x11
 #define RCDEVICE_PROTOCOL_COMMAND_READ_SETTING                      0x12
 #define RCDEVICE_PROTOCOL_COMMAND_WRITE_SETTING                     0x13
 // display port support
 #define RCDEVICE_PROTOCOL_COMMAND_DISP_FILL_REGION                  0x20
 #define RCDEVICE_PROTOCOL_COMMAND_DISP_WRITE_CHAR                   0x21
 #define RCDEVICE_PROTOCOL_COMMAND_DISP_WRITE_HORT_STRING            0x22
 #define RCDEVICE_PROTOCOL_COMMAND_DISP_WRITE_VERT_STRING            0x24
 #define RCDEVICE_PROTOCOL_COMMAND_DISP_WRITE_CHARS                  0x25
 
 
 // Feature Flag sets, it's a uint16_t flag
 #define RCDEVICE_PROTOCOL_FEATURE_SIMULATE_POWER_BUTTON             (1 << 0)
 #define RCDEVICE_PROTOCOL_FEATURE_SIMULATE_WIFI_BUTTON              (1 << 1)
 #define RCDEVICE_PROTOCOL_FEATURE_CHANGE_MODE                       (1 << 2)
 #define RCDEVICE_PROTOCOL_FEATURE_SIMULATE_5_KEY_OSD_CABLE          (1 << 3)
 #define RCDEVICE_PROTOCOL_FEATURE_DEVICE_SETTINGS_ACCESS            (1 << 4)
 #define RCDEVICE_PROTOCOL_FEATURE_DISPLAYP_PORT                     (1 << 5)
 #define RCDEVICE_PROTOCOL_FEATURE_START_RECORDING                   (1 << 6)
 #define RCDEVICE_PROTOCOL_FEATURE_STOP_RECORDING                    (1 << 7)
 
 // Operation of Camera Button Simulation
 #define RCDEVICE_PROTOCOL_SIMULATE_WIFI_BTN         0x00
 #define RCDEVICE_PROTOCOL_SIMULATE_POWER_BTN        0x01
 #define RCDEVICE_PROTOCOL_CHANGE_MODE               0x02
 #define RCDEVICE_PROTOCOL_START_RECORDING           0x03
 #define RCDEVICE_PROTOCOL_STOP_RECORDING            0x04
 #define RCDEVICE_PROTOCOL_UNKNOWN_CAMERA_OPERATION  0xFF
 
 
 // Operation Of 5 Key OSD Cable Simulation
 #define RCDEVICE_PROTOCOL_5KEY_SIMULATION_SET       0x01
 #define RCDEVICE_PROTOCOL_5KEY_SIMULATION_LEFT      0x02
 #define RCDEVICE_PROTOCOL_5KEY_SIMULATION_RIGHT     0x03
 #define RCDEVICE_PROTOCOL_5KEY_SIMULATION_UP        0x04
 #define RCDEVICE_PROTOCOL_5KEY_SIMULATION_DOWN      0x05
 
 // Operation of RCDEVICE_PROTOCOL_COMMAND_5KEY_CONNECTION
 #define RCDEVICE_PROTOCOL_5KEY_FUNCTION_OPEN        0x01
 #define RCDEVICE_PROTOCOL_5KEY_FUNCTION_CLOSE       0x02
 
 // packet header and tail

#define RCSPLIT_PACKET_HEADER    0x55
#define RCSPLIT_PACKET_CMD_CTRL  0x01
#define RCSPLIT_PACKET_TAIL      0xaa

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

 typedef enum {
     RCDEVICE_PROTOCOL_RCSPLIT_VERSION       = 0x00, // this is used to indicate the device that using rcsplit firmware version that <= 1.1.0
     RCDEVICE_PROTOCOL_VERSION_1_0           = 0x01,
     RCDEVICE_PROTOCOL_UNKNOWN
 } rcdevice_protocol_version_e;
 
 // Reserved setting ids
 typedef enum {
     RCDEVICE_PROTOCOL_SETTINGID_DISP_CHARSET        = 0,
     RCDEVICE_PROTOCOL_SETTINGID_DISP_COLUMNS        = 1,
     RCDEVICE_PROTOCOL_SETTINGID_DISP_TV_MODE        = 2,
     RCDEVICE_PROTOCOL_SETTINGID_RESERVED3           = 3,
     RCDEVICE_PROTOCOL_SETTINGID_RESERVED4           = 4,
     RCDEVICE_PROTOCOL_SETTINGID_RESERVED5           = 5,
     RCDEVICE_PROTOCOL_SETTINGID_RESERVED6           = 6,
     RCDEVICE_PROTOCOL_SETTINGID_RESERVED7           = 7,
     RCDEVICE_PROTOCOL_SETTINGID_RESERVED8           = 8,
     RCDEVICE_PROTOCOL_SETTINGID_RESERVED9           = 9,
     RCDEVICE_PROTOCOL_SETTINGID_RESERVED10          = 10,
     RCDEVICE_PROTOCOL_SETTINGID_RESERVED11          = 11,
     RCDEVICE_PROTOCOL_SETTINGID_RESERVED12          = 12,
     RCDEVICE_PROTOCOL_SETTINGID_RESERVED13          = 13,
     RCDEVICE_PROTOCOL_SETTINGID_RESERVED14          = 14,
     RCDEVICE_PROTOCOL_SETTINGID_RESERVED15          = 15,
     RCDEVICE_PROTOCOL_SETTINGID_RESERVED16          = 16,
     RCDEVICE_PROTOCOL_SETTINGID_RESERVED17          = 17,
     RCDEVICE_PROTOCOL_SETTINGID_RESERVED18          = 18,
     RCDEVICE_PROTOCOL_SETTINGID_RESERVED19          = 19,
 } rcdeviceReservedSettingID_e;
 
 typedef enum {
     RCDEVICE_PROTOCOL_SETTINGTYPE_UINT8             = 0,
     RCDEVICE_PROTOCOL_SETTINGTYPE_INT8              = 1,
     RCDEVICE_PROTOCOL_SETTINGTYPE_UINT16            = 2,
     RCDEVICE_PROTOCOL_SETTINGTYPE_INT16             = 3,
     RCDEVICE_PROTOCOL_SETTINGTYPE_FLOAT             = 8,
     RCDEVICE_PROTOCOL_SETTINGTYPE_TEXT_SELECTION    = 9,
     RCDEVICE_PROTOCOL_SETTINGTYPE_STRING            = 10,
     RCDEVICE_PROTOCOL_SETTINGTYPE_FOLDER            = 11,
     RCDEVICE_PROTOCOL_SETTINGTYPE_INFO              = 12,
     RCDEVICE_PROTOCOL_SETTINGTYPE_UNKNOWN
 } rcdeviceSettingType_e;
 
 typedef struct {
     char firmwareVersion[RCDEVICE_PROTOCOL_VERSION_STRING_LENGTH];
     rcdevice_protocol_version_e protocolVersion;
     uint16_t features;
 } runcamDeviceInfo_t;
 
 typedef struct _runcamDeviceSetting {
     uint8_t id;
     char *name;
     char *value;
     struct _runcamDeviceSetting *next;
 } runcamDeviceSetting_t;
 
 typedef struct _runcamDeviceSettingTextSelection {
     char *text;
     struct _runcamDeviceSettingTextSelection *next;
 } runcamDeviceSettingTextSelection_t;
 
 typedef struct {
     uint8_t type;
     uint8_t *value;
     uint8_t *minValue;
     uint8_t *maxValue;
     uint16_t decimalPoint;
     uint8_t *stepSize;
     uint8_t maxStringSize;
     runcamDeviceSettingTextSelection_t *textSelections;
 } runcamDeviceSettingDetail_t;
 
 typedef struct {
     uint8_t resultCode;
     uint8_t needUpdateMenuItems;
 } runcamDeviceWriteSettingResponse_t;
 
 typedef struct {
     serialPort_t *serialPort;
     uint8_t buffer[RCDEVICE_PROTOCOL_MAX_DATA_SIZE];
     sbuf_t streamBuffer;
     sbuf_t *sbuf;
     runcamDeviceInfo_t info;
 } runcamDevice_t;
 
 bool runcamDeviceInit(runcamDevice_t *device);
 
 // camera button simulation
 bool runcamDeviceSimulateCameraButton(runcamDevice_t *device, uint8_t operation);
 
 // 5 key osd cable simulation
 bool runcamDeviceOpen5KeyOSDCableConnection(runcamDevice_t *device);
 bool runcamDeviceClose5KeyOSDCableConnection(runcamDevice_t *device);
 bool runcamDeviceSimulate5KeyOSDCableButtonPress(runcamDevice_t *device, uint8_t operation);
 bool runcamDeviceSimulate5KeyOSDCableButtonRelease(runcamDevice_t *device);
 
 // DisplayPort feature support
 void runcamDeviceDispFillRegion(runcamDevice_t *device, uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t c);
 void runcamDeviceDispWriteChar(runcamDevice_t *device, uint8_t x, uint8_t y, uint8_t c);
 void runcamDeviceDispWriteString(runcamDevice_t *device, uint8_t x, uint8_t y, const char *text);
 void runcamDeviceDispWriteChars(runcamDevice_t *device, uint8_t *data, uint8_t datalen);
 
 // Device Setting Access
 bool runcamDeviceGetSettings(runcamDevice_t *device, uint8_t parentSettingID, runcamDeviceSetting_t **outSettingList);
 void runcamDeviceReleaseSetting(runcamDeviceSetting_t *settingList);
 bool runcamDeviceGetSettingDetail(runcamDevice_t *device, uint8_t settingID, runcamDeviceSettingDetail_t **outSettingDetail);
 void runcamDeviceReleaseSettingDetail(runcamDeviceSettingDetail_t *settingDetail);
 bool runcamDeviceWriteSetting(runcamDevice_t *device, uint8_t settingID, uint8_t *data, uint8_t dataLen, runcamDeviceWriteSettingResponse_t **response);
