/******************************************************************************
 *
 *  Project:
 *      WashTrac Core
 *
 *  Module:
 *      Input Manager
 *
 *  File:
 *      input_manager.h
 *
 *  Description:
 *      Defines the public interface for reading and processing the WashTrac
 *      hardware inputs.
 *
 *      Input 1 is permanently assigned as the Wash Busy input.
 *      Input 2 is permanently assigned as the E-Stop input.
 *      Inputs 3 through 6 are configurable spare inputs.
 *
 *      All reported input states are logical states after configuration,
 *      including the per-input inversion setting.
 *
 *  Copyright:
 *      © 2026 WashTrac
 *
 ******************************************************************************/

#pragma once

#include "system.h"

#include <cstdint>

namespace WashTrac::Inputs
{

Result Initialize();

void Update();

bool IsWashBusy();

bool IsEStopActive();

bool ReadInput(uint8_t inputNumber);

bool IsInitialized();

} // namespace WashTrac::Inputs