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
 *      Inputs 2 through 6 are reserved for future expansion.
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