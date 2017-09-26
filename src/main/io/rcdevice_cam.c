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
 
 #include "common/crc.h"
 #include "cms/cms.h"
 
 #include "config/parameter_group_ids.h"
 #include "config/parameter_group.h"
 #include "io/rcdevice_cam.h"
 
 #include "fc/rc_controls.h"
 #include "fc/runtime_config.h"
 #include "fc/config.h"
 #include "config/feature.h"

 #include "rx/rx.h"
 
 #define IS_HI(X)  (rcData[X] > 1750)
 #define IS_LO(X)  (rcData[X] < 1250)
 #define IS_MID(X) (rcData[X] > 1250 && rcData[X] < 1750)
 static runcamDevice_t runcamDevice;
 runcamDevice_t *camDevice = &runcamDevice;
 
 rcdevice_cam_switch_state_t switchStates[BOXCAMERA3 - BOXCAMERA1 + 1];

 timeUs_t lastTimeUs = 0;
 bool needRelease = false;
 
 
 bool isFeatureSupported(uint8_t feature){
     if (camDevice->info.features & feature)
         return true;
 
     return false;
 }

 bool isRcdeviceCamReady(){
    if(camDevice->serialPort != NULL && isFeatureSupported(RCDEVICE_PROTOCOL_FEATURE_SIMULATE_5_KEY_OSD_CABLE)){
        return true;
     }
     return false;
 }
 
 void rcdeviceCamProcessMode(){
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
 
             uint8_t behavior = RCDEVICE_PROTOCOL_SIMULATE_UNKNOWN;
             switch (i) {
             case BOXCAMERA1:
                 if (isFeatureSupported(RCDEVICE_PROTOCOL_FEATURE_SIMULATE_WIFI_BUTTON))
                     behavior = RCDEVICE_PROTOCOL_SIMULATE_WIFI_BTN;
                 break;
             case BOXCAMERA2:
                 if (isFeatureSupported(RCDEVICE_PROTOCOL_FEATURE_SIMULATE_POWER_BUTTON))
                     behavior = RCDEVICE_PROTOCOL_SIMULATE_POWER_BTN;
                 break;
             case BOXCAMERA3:
                 if (isFeatureSupported(RCDEVICE_PROTOCOL_FEATURE_CHANGE_MODE))
                     behavior = RCDEVICE_PROTOCOL_CHANGE_MODE;
                 break;
            default:
                break;
             }
             if (behavior != RCDEVICE_PROTOCOL_SIMULATE_UNKNOWN) {
                 runcamDeviceSimulateCameraButton(camDevice, behavior);
                 switchStates[switchIndex].isActivated = true;
             }
         } else {
             switchStates[switchIndex].isActivated = false;
         }
     }
 }
 
 bool rcdeviceCamSimulate5KeyCablePress(rcdeviceCamSimulationKeyEvent_e key){
     UNUSED(key);

     uint8_t operation;
     if(key == RCDEVICE_CAM_KEY_LEFT){
        operation = RCDEVICE_PROTOCOL_5KEY_SIMULATION_LEFT;
     }else if(key == RCDEVICE_CAM_KEY_UP){
        operation = RCDEVICE_PROTOCOL_5KEY_SIMULATION_UP;
     }else if(key == RCDEVICE_CAM_KEY_RIGHT){
        operation = RCDEVICE_PROTOCOL_5KEY_SIMULATION_RIGHT;
     }else if(key == RCDEVICE_CAM_KEY_DOWN){
        operation = RCDEVICE_PROTOCOL_5KEY_SIMULATION_DOWN;
     }else if(key == RCDEVICE_CAM_KEY_ENTER){
        operation = RCDEVICE_PROTOCOL_5KEY_SIMULATION_SET;
     }

     return runcamDeviceSimulate5KeyOSDCableButtonPress(camDevice,operation);
 }

 bool rcdeviceSend5KeyOSDCableSimualtionEvent(rcdeviceCamSimulationKeyEvent_e key){

    bool reqResult = false;
    switch(key){
    case RCDEVICE_CAM_KEY_RIGHT_AND_TOP:
        reqResult = runcamDeviceOpen5KeyOSDCableConnection(camDevice);
        break;
    case RCDEVICE_CAM_KEY_LEFT_LONG:
        reqResult = runcamDeviceClose5KeyOSDCableConnection(camDevice);
        break;
    case RCDEVICE_CAM_KEY_ENTER:
    case RCDEVICE_CAM_KEY_LEFT:
    case RCDEVICE_CAM_KEY_UP:
    case RCDEVICE_CAM_KEY_RIGHT:
    case RCDEVICE_CAM_KEY_DOWN:
        reqResult = rcdeviceCamSimulate5KeyCablePress(key);
        break;
    case RCDEVICE_CAM_KEY_RELEASE:
        reqResult = runcamDeviceSimulate5KeyOSDCableButtonRelease(camDevice);
        break;
    }

    return reqResult;
 }

 void rcdeviceCamSimulate5KeyCablePressProcessMode(timeUs_t currentTimeUs){

#ifdef CMS
    if(cmsInMenu){
        return ;
    }
#endif

    if (camDevice->serialPort == NULL) {
        return ;
    }

    rcdeviceCamSimulationKeyEvent_e key = RCDEVICE_CAM_KEY_NONE;

    if(needRelease){
        if(IS_MID(YAW) && IS_MID(PITCH) && IS_MID(ROLL)){
            key = RCDEVICE_CAM_KEY_RELEASE;
            if(!rcdeviceSend5KeyOSDCableSimualtionEvent(key)){
                beeperConfirmationBeeps(3);
                rcdeviceInMenu = false;
            }else{
                needRelease = false;
            }
            return ;
        }else{
            return ;
        }
    }else{
        if(IS_MID(ROLL) && IS_MID(PITCH) && IS_LO(YAW)){//Disconnect HI YAW
            if(rcdeviceInMenu){
                if(lastTimeUs == 0){
                    lastTimeUs = currentTimeUs;
                }
                else if((currentTimeUs - lastTimeUs) >= 2000 * 1000){
                    lastTimeUs = 0;
                    key = RCDEVICE_CAM_KEY_LEFT_LONG;
                }
            }
        }else{
             lastTimeUs = 0;
            
             if(rcdeviceInMenu){
                if(IS_LO(ROLL)){//Left LO ROLL
                    key = RCDEVICE_CAM_KEY_LEFT;
                }
                else if(IS_HI(PITCH)){//Up HI PITCH
                    key = RCDEVICE_CAM_KEY_UP;
                }
                else if(IS_HI(ROLL)){//Right HI ROLL
                    key = RCDEVICE_CAM_KEY_RIGHT;
                }
                else if(IS_LO(PITCH)){//Down LO PITCH
                    key = RCDEVICE_CAM_KEY_DOWN;
                }
                else if(IS_MID(THROTTLE) && IS_MID(ROLL) && IS_MID(PITCH) && IS_HI(YAW)){//Enter HI YAW
                    key = RCDEVICE_CAM_KEY_ENTER;
                }
             }else{
                 // if (IS_MID(THROTTLE) && IS_HI(YAW) && IS_HI(PITCH) && !ARMING_FLAG(ARMED)) {//HandShake HI YAW + HI PITCH
                if(IS_MID(THROTTLE) && IS_MID(ROLL) && IS_MID(PITCH) && IS_HI(YAW) && !ARMING_FLAG(ARMED)){//Enter HI YAW
                    key = RCDEVICE_CAM_KEY_RIGHT_AND_TOP;
                }
             }
        }
    }
    
    if(key != RCDEVICE_CAM_KEY_NONE){
        if(key != RCDEVICE_CAM_KEY_RELEASE) beeperConfirmationBeeps(1);
        if(!rcdeviceSend5KeyOSDCableSimualtionEvent(key)){
            beeperConfirmationBeeps(3);
            rcdeviceInMenu = false;
        }else{
            needRelease = true;
        }
    }
 
 }

 void rcdeviceCamProcess(timeUs_t currentTimeUs){
     UNUSED(currentTimeUs);
 
     if (isFeatureSupported(RCDEVICE_PROTOCOL_FEATURE_SIMULATE_POWER_BUTTON) ||
         isFeatureSupported(RCDEVICE_PROTOCOL_FEATURE_SIMULATE_WIFI_BUTTON) ||
         isFeatureSupported(RCDEVICE_PROTOCOL_FEATURE_CHANGE_MODE)) {
         rcdeviceCamProcessMode();
     }
    
    //  if(isFeatureSupported(RCDEVICE_PROTOCOL_FEATURE_SIMULATE_5_KEY_OSD_CABLE)){
    //     rcdeviceCamSimulate5KeyCablePressProcessMode(currentTimeUs);
    //  }

 }
 
 bool rcdeviceCamInit(void)
 {
     // open serial port
     if (!runcamDeviceInit(camDevice)) {
         return false;
     }
 
     for (boxId_e i = BOXCAMERA1; i <= BOXCAMERA3; i++) {
         uint8_t switchIndex = i - BOXCAMERA1;
         switchStates[switchIndex].boxId = 1 << i;
         switchStates[switchIndex].isActivated = true; 
     }
     
     return true;
 }