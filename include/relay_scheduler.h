/******************************************************************************
 *
 *  Project:
 *      WashTrac Core
 *
 *  Module:
 *      Relay Scheduler
 *
 *  File:
 *      relay_scheduler.h
 *
 *  Description:
 *      Defines the public interface for scheduling and controlling WashTrac
 *      relay outputs using locally stored whole-second timing configuration.
 *
 *  Copyright:
 *      © 2026 WashTrac
 *
 ******************************************************************************/

#pragma once

#include "system.h"

#include <cstdint>

namespace WashTrac::Relays
{

enum class RelayId : uint8_t
{
    WashStart = 1U,
    Relay2 = 2U,
    Relay3 = 3U,
    Relay4 = 4U,
    Relay5 = 5U,
    Relay6 = 6U
};

enum class RelayState : uint8_t
{
    Idle = 0,
    WaitingOnDelay,
    Active,
    WaitingOffDelay
};

Result Initialize();

Result Start(RelayId relay);

Result Cancel(RelayId relayId);

void Update();

RelayState GetState(RelayId relayId);

bool IsActive(RelayId relayId);

bool IsRunning(RelayId relayId);

bool IsInitialized();

} // namespace WashTrac::Relays