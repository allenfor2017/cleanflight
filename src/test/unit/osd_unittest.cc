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

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

extern "C" {
    #include "platform.h"

    #include "build/debug.h"

    #include "blackbox/blackbox.h"

    #include "config/parameter_group_ids.h"

    #include "drivers/max7456_symbols.h"

    #include "fc/config.h"
    #include "fc/rc_controls.h"
    #include "fc/rc_modes.h"
    #include "fc/runtime_config.h"

    #include "flight/pid.h"
    #include "flight/imu.h"

    #include "io/gps.h"
    #include "io/osd.h"

    #include "sensors/battery.h"

    #include "rx/rx.h"

    void osdRefresh(timeUs_t currentTimeUs);
    void osdFormatTime(char * buff, osd_timer_precision_e precision, timeUs_t time);
    void osdFormatTimer(char *buff, bool showSymbol, int timerIndex);

    uint16_t rssi;
    attitudeEulerAngles_t attitude;
    pidProfile_t *currentPidProfile;
    int16_t debug[DEBUG16_VALUE_COUNT];
    int16_t rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
    uint8_t GPS_numSat;
    uint16_t GPS_distanceToHome;
    uint16_t GPS_directionToHome;
    int32_t GPS_coord[2];
    gpsSolutionData_t gpsSol;

    PG_REGISTER(batteryConfig_t, batteryConfig, PG_BATTERY_CONFIG, 0);
    PG_REGISTER(blackboxConfig_t, blackboxConfig, PG_BLACKBOX_CONFIG, 0);
    PG_REGISTER(systemConfig_t, systemConfig, PG_SYSTEM_CONFIG, 0);
    PG_REGISTER(pilotConfig_t, pilotConfig, PG_PILOT_CONFIG, 0);

    timeUs_t simulationTime = 0;
    batteryState_e simulationBatteryState;
    uint8_t simulationBatteryCellCount;
    uint16_t simulationBatteryVoltage;
    uint32_t simulationBatteryAmperage;
    uint32_t simulationMahDrawn;
    int32_t simulationAltitude;
    int32_t simulationVerticalSpeed;
}

/* #define DEBUG_OSD */

#include "unittest_macros.h"
#include "unittest_displayport.h"
#include "gtest/gtest.h"

void setDefualtSimulationState()
{
    rssi = 1024;

    simulationBatteryState = BATTERY_OK;
    simulationBatteryCellCount = 4;
    simulationBatteryVoltage = 168;
    simulationBatteryAmperage = 0;
    simulationMahDrawn = 0;
    simulationAltitude = 0;
    simulationVerticalSpeed = 0;
}

/*
 * Performs a test of the OSD actions on arming.
 * (reused throughout the test suite)
 */
void doTestArm(bool testEmpty = true)
{
    // given
    // craft has been armed
    ENABLE_ARMING_FLAG(ARMED);

    // when
    // sufficient OSD updates have been called
    osdRefresh(simulationTime);

    // then
    // arming alert displayed
    displayPortTestBufferSubstring(12, 7, "ARMED");

    // given
    // armed alert times out (0.5 seconds)
    simulationTime += 0.5e6;

    // when
    // sufficient OSD updates have been called
    osdRefresh(simulationTime);

    // then
    // arming alert disappears
#ifdef DEBUG_OSD
    displayPortTestPrint();
#endif
    if (testEmpty) {
        displayPortTestBufferIsEmpty();
    }
}

/*
 * Performs a test of the OSD actions on disarming.
 * (reused throughout the test suite)
 */
void doTestDisarm()
{
    // given
    // craft is disarmed after having been armed
    DISABLE_ARMING_FLAG(ARMED);

    // when
    // sufficient OSD updates have been called
    osdRefresh(simulationTime);

    // then
    // post flight statistics displayed
    displayPortTestBufferSubstring(2, 2, "  --- STATS ---");
}

// STUBS
extern "C" {
    void beeperConfirmationBeeps(uint8_t beepCount) {
        UNUSED(beepCount);
    }

    bool IS_RC_MODE_ACTIVE(boxId_e boxId) {
        UNUSED(boxId);
        return false;
    }

    uint32_t micros() {
        return simulationTime;
    }

    bool isBeeperOn() {
        return false;
    }

    bool isAirmodeActive() {
        return false;
    }

    uint8_t getCurrentPidProfileIndex() {
        return 0;
    }

    uint8_t getCurrentControlRateProfileIndex() {
        return 0;
    }

    batteryState_e getBatteryState() {
        return simulationBatteryState;
    }

    uint8_t getBatteryCellCount() {
        return simulationBatteryCellCount;
    }

    uint16_t getBatteryVoltage() {
        return simulationBatteryVoltage;
    }

    uint16_t getBatteryAverageCellVoltage() {
        return simulationBatteryVoltage / simulationBatteryCellCount;
    }

    int32_t getAmperage() {
        return simulationBatteryAmperage;
    }

    int32_t getMAhDrawn() {
        return simulationMahDrawn;
    }

    int32_t getEstimatedAltitude() {
        return simulationAltitude;
    }

    int32_t getEstimatedVario() {
        return simulationVerticalSpeed;
    }

    unsigned int blackboxGetLogNumber() {
        return 0;
    }

    bool isSerialTransmitBufferEmpty(const serialPort_t *instance) {
        UNUSED(instance);
        return false;
    }

    void serialWrite(serialPort_t *instance, uint8_t ch) {
        UNUSED(instance);
        UNUSED(ch);
    }

    bool cmsDisplayPortRegister(displayPort_t *pDisplay) {
        UNUSED(pDisplay);
        return false;
    }
}
