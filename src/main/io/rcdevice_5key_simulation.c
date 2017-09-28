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

#include <stdint.h>

#include "io/rcdevice_5key_simulation.h"
#include "drivers/rcdevice.h"
#include "io/rcdevice_cam.h"

bool rcdeviceIs5KeySimulationReady(void)
{
    bool isReady = isRcDeviceCamReady();
    return isReady;
}

void rcdevice5KeySimulationProcessMode(timeUs_t currentTimeUs)
{
    rcdeviceCamSimulate5KeyCablePressProcessMode(currentTimeUs);
}
