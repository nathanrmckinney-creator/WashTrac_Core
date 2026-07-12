/******************************************************************************
 *
 *  Project:
 *      WashTrac Core
 *
 *  Module:
 *      Diagnostics Manager
 *
 *  File:
 *      diagnostics_manager.h
 *
 *  Description:
 *      Defines the public interface for collecting centralized, read-only
 *      production runtime diagnostics.
 *
 *  Copyright:
 *      © 2026 WashTrac
 *
 ******************************************************************************/

#pragma once

#include "fault_manager.h"
#include "relay_scheduler.h"
#include "state_machine.h"
#include "system.h"
#include "system_health.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace WashTrac::Diagnostics
{

struct Snapshot
{
    std::array<bool, INPUT_COUNT> inputStates;
    std::array<uint32_t, INPUT_COUNT> inputTransitionCounts;

    std::array<Relays::RelayState, RELAY_COUNT> relayStates;
    std::array<uint32_t, RELAY_COUNT> relayActivationCounts;

    StateMachine::SystemState systemState;
    uint8_t retryCount;

    std::size_t pendingWashCount;
    bool queueEmpty;
    bool queueFull;

    Faults::FaultStatus currentFault;
    SystemHealth::Snapshot systemHealth;

    uint64_t updateCount;
};

Result Initialize();

void Update();

void ResetStatistics();

const Snapshot& GetSnapshot();

bool IsInitialized();

} // namespace WashTrac::Diagnostics