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
 
 #include "common/time.h"
 
 #include "config/parameter_group_ids.h"
 #include "config/parameter_group.h"
 
 #include "drivers/opentco.h"
 #include "drivers/opentco_cam.h"
 
 #include "fc/rc_controls.h"
 #include "fc/runtime_config.h"
 
 #include "rx/rx.h"
 #include "io/rcsplit.h"
 
 #define IS_HI(X)  (rcData[X] > 1750)
 #define IS_LO(X)  (rcData[X] < 1250)
 #define IS_MID(X) (rcData[X] > 1250 && rcData[X] < 1750)
 
 static opentcoDevice_t openTCOCamDevice;
 opentcoDevice_t *camDevice = &openTCOCamDevice;
 opentco_cam_switch_state_t switchStates[BOXCAMERA3 - BOXCAMERA1 + 1];
 bool fpvCameraOSDMenuIsHandShake = false;
 timeUs_t lastTimeUs = 0;
 bool needRelease = false;
 
 PG_REGISTER(opentcoCameraProfile_t, opentcoCameraProfile, PG_FPV_CAMERA_CONFIG, 0);
 
 static void opentcoCamQuerySupportedFeatures()
 {
     uint16_t opentcoCamFeatures = 0;
 
     // fetch available and acitvated features
     bool isSuccess = opentcoReadRegisterUint16(camDevice, OPENTCO_CAM_REGISTER_SUPPORTED_FEATURES,  &opentcoCamFeatures);
     if(isSuccess){
        beeperConfirmationBeeps(3);
     }else{
        beeperConfirmationBeeps(5);
     }
     // store
     opentcoCameraProfileMutable()->supportedFeatures = opentcoCamFeatures;
 }
 
 static bool opentcoCamControl(opentcoDevice_t *camDevice, uint8_t controlbehavior)
 {
     opentcoInitializeFrame(camDevice, OPENTCO_CAM_COMMAND_CAMERA_CONTROL);
     sbufWriteU8(camDevice->sbuf, controlbehavior);
     opentcoSendFrame(camDevice);
     return true;
 }
 
 static bool opentcoSimulationPress(opentcoDevice_t *camDevice, uint8_t controlbehavior)
 {
     opentcoInitializeFrame(camDevice, OPENTCO_CAM_COMMAND_5KEY_SIMULATION_PRESS);
     sbufWriteU8(camDevice->sbuf, controlbehavior);
     opentcoSendFrame(camDevice);
     return true;
 }
 static bool opentcoSimulationRelease(opentcoDevice_t *camDevice)
 {
     opentcoInitializeFrame(camDevice, OPENTCO_CAM_COMMAND_5KEY_SIMULATION_RELEASE);
     opentcoSendFrame(camDevice);
     return true;
 }
 static bool opentcoHandshakeDisconnect(opentcoDevice_t *camDevice, uint8_t controlbehavior)
 {
     opentcoInitializeFrame(camDevice, OPENTCO_CAM_COMMAND_HANDSHAKE_DISCONNECT);
     sbufWriteU8(camDevice->sbuf, controlbehavior);
     opentcoSendFrame(camDevice);
     return true;
 }

 static bool opentcoGetSubSettings(opentcoDevice_t *camDevice, uint8_t settingID,uint8_t chunkIndex)
 {
    opentcoInitializeFrame(camDevice, OPENTCO_CAM_COMMAND_GET_SETTINGS);
    sbufWriteU8(camDevice->sbuf, settingID);
    sbufWriteU8(camDevice->sbuf, chunkIndex);
    opentcoSendFrame(camDevice);
    return true;
 }

 static bool opentcoReadSettingDetail(opentcoDevice_t *camDevice, uint8_t settingID)
 {
    opentcoInitializeFrame(camDevice, OPENTCO_CAM_COMMAND_READ_SETTING_DETAIL);
    sbufWriteU8(camDevice->sbuf, settingID);
    opentcoSendFrame(camDevice);
    return true;
 }

 static bool opentcoWriteSetting(opentcoDevice_t *camDevice, uint8_t settingID, char *value)
 {
    opentcoInitializeFrame(camDevice, OPENTCO_CAM_COMMAND_WRITE_SETTING);
    sbufWriteU8(camDevice->sbuf, settingID);
    sbufWriteString(camDevice->sbuf, value);
    opentcoSendFrame(camDevice);
    return true;
 }
 
 static bool isFeatureSupported(uint8_t feature)
 {
     if (opentcoCameraProfile()->supportedFeatures & feature)
         return true;
 
     return false;
 }
 
 static void opentcoCamProcessMode()
 {
     if (camDevice->serialPort == NULL)
         return ;
 
     for (boxId_e i = BOXCAMERA1; i <= BOXCAMERA3; i++) {
         uint8_t switchIndex = i - BOXCAMERA1;
         
         if (IS_RC_MODE_ACTIVE(i)) {
             // check last state of this mode, if it's true, then ignore it.
             // Here is a logic to make a toggle control for this mode
             if (switchStates[switchIndex].isActivated) {
                 continue;
             }
 
             uint8_t behavior = 0;
             switch (i) {
             case BOXCAMERA1:
                 if (isFeatureSupported(OPENTCO_CAM_FEATURE_SIMULATE_WIFI_BTN))
                     behavior = OPENTCO_CAM_CONTROL_SIMULATE_WIFI_BTN;
                 break;
             case BOXCAMERA2:
                 if (isFeatureSupported(OPENTCO_CAM_FEATURE_SIMULATE_POWER_BTN))
                     behavior = OPENTCO_CAM_CONTROL_SIMULATE_POWER_BTN;
                 break;
             case BOXCAMERA3:
                 if (isFeatureSupported(OPENTCO_CAM_FEATURE_CHANGE_MODE))
                     behavior = OPENTCO_CAM_CONTROL_SIMULATE_CHANGE_MODE;
                 break;
             default:
                 behavior = 0;
                 break;
             }
             if (behavior != 0) {
                 opentcoCamControl(camDevice, behavior);
                 switchStates[switchIndex].isActivated = true;
             }
         } else {
             switchStates[switchIndex].isActivated = false;
         }
     }
 }
 
 void opentcoCamSimulate5KeyCablePress(opentcoCamSimulationKeyEvent_e key)
 {
     UNUSED(key);

     opentcoCam5KeySimulationKey_e behavior;

     if(key == OPENTCO_CAM_KEY_LEFT){
        behavior = OPENTCO_CAM_5KEY_SIMULATION_LEFT;
     }
     else if(key == OPENTCO_CAM_KEY_UP){
        behavior = OPENTCO_CAM_5KEY_SIMULATION_UP;
     }
     else if(key == OPENTCO_CAM_KEY_RIGHT){
        behavior = OPENTCO_CAM_5KEY_SIMULATION_RIGHT;
     }
     else if(key == OPENTCO_CAM_KEY_DOWN){
        behavior = OPENTCO_CAM_5KEY_SIMULATION_DOWN;
     }
     else if(key == OPENTCO_CAM_KEY_ENTER){
        behavior = OPENTCO_CAM_5KEY_SIMULATION_SET;
     }

     opentcoSimulationPress(camDevice,behavior);
 }

 void opentcoCamHandshakeAndDisconnect(opentcoCamSimulationKeyEvent_e key)
 {
     UNUSED(key);

     opentcoCamHandShakeAndDisconnectSimulationKey_e behavior;
     if(key == OPENTCO_CAM_KEY_LEFT_LONG){
        behavior = OPENTCO_CAM_DISCONNECT_PROTOCOL;
     }
     else if(key == OPENTCO_CAM_KEY_RIGHT_AND_TOP){
        behavior = OPENTCO_CAM_HANDSHAKE_PROTOCOL;
     }

     opentcoHandshakeDisconnect(camDevice,behavior);
 }

 bool opentcoCamHandShakePackageParser(opentcoDevice_t *device){
    // ... [DEVICE:4|CMD:4] [OPEID:4|RESULT:4] [CRC:8]
    // prepare crc calc
    uint8_t crc = crc8_dvb_s2(0, OPENTCO_PROTOCOL_HEADER);
    // fetch data (serial buffer already contains enough bytes)
    uint8_t data[3];
    for(int i = 0; i < 3; i++) {
        uint8_t rx = serialRead(device->serialPort);
        data[i] = rx;
        crc = crc8_dvb_s2(crc, rx);
    }
    // check crc
    if (crc != 0) return false;
    
    // check device and command
    uint8_t valid_devcmd = ((OPENTCO_DEVICE_CAM_RESPONSE | device->id) << 4) | OPENTCO_CAM_COMMAND_HANDSHAKE_DISCONNECT;
    if (data[0] != valid_devcmd) return false;
   
    //check result
    uint8_t operationID = (data[1] & 0xF0) >> 4;
    uint8_t resResult = (data[1] & 0x0F);
    if(resResult == 1){
        if(operationID == OPENTCO_CAM_HANDSHAKE_PROTOCOL){
            fpvCameraOSDMenuIsHandShake = true;
            is_fpv_cam_osd_open = true;
        }
        else if(operationID == OPENTCO_CAM_DISCONNECT_PROTOCOL){
            fpvCameraOSDMenuIsHandShake = false;
            is_fpv_cam_osd_open = false;
        }
    }
    return true;
 }

 bool opentcoCam5KeySimulationPackageParser(opentcoDevice_t *device , opentcoCamSimulationKeyEvent_e key){
    // prepare crc calc
    uint8_t crc = crc8_dvb_s2(0, OPENTCO_PROTOCOL_HEADER);
    // fetch data (serial buffer already contains enough bytes)
    uint8_t data[2];
    for(int i = 0; i < 2; i++) {
        uint8_t rx = serialRead(device->serialPort);
        data[i] = rx;
        crc = crc8_dvb_s2(crc, rx);
    }
    // check crc
    if (crc != 0) return false;
    // check device and command
    uint8_t valid_devcmd;
    if(key == OPENTCO_CAM_KEY_RELEASE){
        valid_devcmd = ((OPENTCO_DEVICE_CAM_RESPONSE | device->id) << 4) | OPENTCO_CAM_COMMAND_5KEY_SIMULATION_RELEASE;
    }else{
        valid_devcmd = ((OPENTCO_DEVICE_CAM_RESPONSE | device->id) << 4) | OPENTCO_CAM_COMMAND_5KEY_SIMULATION_PRESS;
    }
    if (data[0] != valid_devcmd) return false;

    return true;
 }

 bool opentcoCamReadPackage(opentcoDevice_t *device , opentcoCamSimulationKeyEvent_e key)
 {
    uint32_t max_retries = 3;
    while (max_retries--) {
         // send read request
         if(key == OPENTCO_CAM_KEY_LEFT_LONG || key == OPENTCO_CAM_KEY_RIGHT_AND_TOP){
             opentcoCamHandshakeAndDisconnect(key);
         }else if(key == OPENTCO_CAM_KEY_RELEASE){
            opentcoSimulationRelease(device);
         }else{
            opentcoCamSimulate5KeyCablePress(key);
         }
         // wait 1000ms for reply
         timeMs_t timeout = millis() + 1000;

         bool header_received = false;
         while (millis() < timeout) {
             // register request replies will contain 3/4 bytes:
             if (!header_received) {
                 // read serial bytes until we find a header:
                 if (serialRxBytesWaiting(device->serialPort) > 0) {
                     uint8_t rx = serialRead(device->serialPort);
                     if (rx == OPENTCO_PROTOCOL_HEADER) {
                         header_received = true;
                     }
                 }
             } else {
                 // header found, now wait for the remaining bytes to arrive
                 if(key == OPENTCO_CAM_KEY_LEFT_LONG || key == OPENTCO_CAM_KEY_RIGHT_AND_TOP){

                     //[HEADER:8] [DEVICE:4|CMD:4] [OPEID:4|RESULT:4] [CRC:8]
                    if (serialRxBytesWaiting(device->serialPort) >= 3) {
                        if (!opentcoCamHandShakePackageParser(device)) {
                            // received broken / bad response
                            break;
                        }
                        
                        // received valid data
                        return true;
                    }
                }else{
                    //[HEADER:8] [DEVICE:4|CMD:4] [CRC:8]
                    if (serialRxBytesWaiting(device->serialPort) >= 2) {
                        
                        if (!opentcoCam5KeySimulationPackageParser(device,key)) {
                            // received broken / bad response
                            break;
                        }

                        // received valid data
                        return true;
                    }
                }
             }
         }
    }

    return false;
 }

 static void opentcoCamSimulate5KeyCablePressProcessMode(timeUs_t currentTimeUs)
 {

    if (camDevice->serialPort == NULL) {
        return ;
    }

    opentcoCamSimulationKeyEvent_e key = OPENTCO_CAM_KEY_NONE;

    if(needRelease){
        if(IS_MID(YAW) && IS_MID(PITCH) && IS_MID(ROLL)){
            key = OPENTCO_CAM_KEY_RELEASE;
            if(!opentcoCamReadPackage(camDevice,key)){
                beeperConfirmationBeeps(3);
                fpvCameraOSDMenuIsHandShake = false;
                is_fpv_cam_osd_open = false;
            }else{
                needRelease = false;
            }
            return ;
        }else{
            return ;
        }
    }else{
        if(IS_MID(ROLL) && IS_MID(PITCH) && IS_LO(YAW)){//Disconnect HI YAW
            if(fpvCameraOSDMenuIsHandShake){
                if(lastTimeUs == 0){
                    lastTimeUs = currentTimeUs;
                }
                else if((currentTimeUs - lastTimeUs) >= 2000 * 1000){
                    lastTimeUs = 0;
                    key = OPENTCO_CAM_KEY_LEFT_LONG;
                }
            }
        }else{
             lastTimeUs = 0;
    
            if (IS_MID(THROTTLE) && IS_HI(YAW) && IS_HI(PITCH) && !ARMING_FLAG(ARMED)) {//HandShake HI YAW + HI PITCH
                 if(!fpvCameraOSDMenuIsHandShake){
                    key = OPENTCO_CAM_KEY_RIGHT_AND_TOP;
                 }
            }else{
                if(fpvCameraOSDMenuIsHandShake){
                    if(IS_LO(ROLL)){//Left LO ROLL
                        key = OPENTCO_CAM_KEY_LEFT;
                    }
                    else if(IS_HI(PITCH)){//Up HI PITCH
                        key = OPENTCO_CAM_KEY_UP;
                    }
                    else if(IS_HI(ROLL)){//Right HI ROLL
                        key = OPENTCO_CAM_KEY_RIGHT;
                    }
                    else if(IS_LO(PITCH)){//Down LO PITCH
                        key = OPENTCO_CAM_KEY_DOWN;
                    }
                    else if(IS_MID(THROTTLE) && IS_MID(ROLL) && IS_MID(PITCH) && IS_HI(YAW)){//Enter HI YAW
                        key = OPENTCO_CAM_KEY_ENTER;
                    }
                }
            }
        }
    }
    
    if(key != OPENTCO_CAM_KEY_NONE){
        if(key != OPENTCO_CAM_KEY_RELEASE) beeperConfirmationBeeps(1);
        if(!opentcoCamReadPackage(camDevice,key)){
            beeperConfirmationBeeps(3);
            fpvCameraOSDMenuIsHandShake = false;
            is_fpv_cam_osd_open = false;
        }else{
            needRelease = true;
        }
    }
 
 }


 
 void opentcoCamProcess(timeUs_t currentTimeUs)
 {
     UNUSED(currentTimeUs);
 
     // process camera custom mode if has any changed
     if (isFeatureSupported(OPENTCO_CAM_FEATURE_SIMULATE_POWER_BTN) ||
         isFeatureSupported(OPENTCO_CAM_FEATURE_SIMULATE_WIFI_BTN) ||
         isFeatureSupported(OPENTCO_CAM_FEATURE_CHANGE_MODE)) {
         opentcoCamProcessMode();
     }
     
     if (isFeatureSupported(OPENTCO_CAM_FEATURE_SIMULATE_5KEY_OSD_CABLE)) {
         opentcoCamSimulate5KeyCablePressProcessMode(currentTimeUs);
     }

     if(isFeatureSupported(OPENTCO_CAM_FEATURE_SETTING_ACCESS)){
         
     }
 }
 
 bool opentcoCamInit(void)
 {
     // open serial port
     camDevice->id = OPENTCO_DEVICE_CAM;
     if (!opentcoInit(camDevice)) {
         return false;
     }
     
     opentcoCamQuerySupportedFeatures();
 
     for (boxId_e i = BOXCAMERA1; i <= BOXCAMERA3; i++) {
         uint8_t switchIndex = i - BOXCAMERA1;
         switchStates[switchIndex].boxId = 1 << i;
         switchStates[switchIndex].isActivated = true; 
     }
     
     return true;
 }