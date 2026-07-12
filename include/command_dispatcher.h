/******************************************************************************
 *
 *  Project:
 *      WashTrac Core
 *
 *  Module:
 *      CM5 Command Dispatcher
 *
 *  File:
 *      command_dispatcher.h
 *
 *  Description:
 *      Defines the fixed-memory command dispatcher for CM5 requests received
 *      through the dedicated UART2 transport.
 *
 *  Copyright:
 *      © 2026 WashTrac
 *
 ******************************************************************************/

#pragma once

#include "system.h"

namespace WashTrac::CommandDispatcher
{

Result Initialize();

void Update();

bool IsInitialized();

} // namespace WashTrac::CommandDispatcher