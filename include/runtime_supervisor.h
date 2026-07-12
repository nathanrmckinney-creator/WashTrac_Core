/******************************************************************************
 *
 *  Project:
 *      WashTrac Core
 *
 *  Module:
 *      Runtime Supervisor
 *
 ******************************************************************************/

#pragma once

#include "system.h"

namespace WashTrac::RuntimeSupervisor
{

Result Initialize();

void Update();

bool IsInitialized();

} // namespace WashTrac::RuntimeSupervisor