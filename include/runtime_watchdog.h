/******************************************************************************
 *
 *  Project:
 *      WashTrac Core
 *
 *  Module:
 *      Runtime Watchdog
 *
 *  File:
 *      runtime_watchdog.h
 *
 *  Description:
 *      Defines the public interface for supervising the main runtime task
 *      through the ESP-IDF Task Watchdog Timer.
 *
 *  Copyright:
 *      © 2026 WashTrac
 *
 ******************************************************************************/

#pragma once

#include "system.h"

namespace WashTrac::RuntimeWatchdog
{

Result Initialize();

void Update();

bool IsInitialized();

} // namespace WashTrac::RuntimeWatchdog