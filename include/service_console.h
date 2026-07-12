/******************************************************************************
 *
 *  Project:
 *      WashTrac Core
 *
 *  Module:
 *      Service Console
 *
 *  File:
 *      service_console.h
 *
 *  Description:
 *      Defines the public interface for the read-only WashTrac Core service
 *      console.
 *
 *      The console receives fixed-length UART commands, dispatches supported
 *      diagnostic queries, and reports controller status without modifying
 *      runtime state.
 *
 *  Copyright:
 *      © 2026 WashTrac
 *
 ******************************************************************************/

#pragma once

#include "system.h"

namespace WashTrac::ServiceConsole
{

Result Initialize();

void Update();

bool IsInitialized();

} // namespace WashTrac::ServiceConsole