/******************************************************************************
 *
 *  Project:
 *      WashTrac Core
 *
 *  Module:
 *      Diagnostics Manager
 *
 *  File:
 *      diagnostics_manager.cpp
 *
 *  Description:
 *      Production Diagnostics Manager implementation providing centralized,
 *      read-only runtime diagnostics for service, manufacturing, UART, and
 *      future controller interfaces.
 *
 *      The subsystem records:
 *          - Current input states
 *          - Per-input transition counts
 *          - Current relay scheduler states
 *          - Per-relay activation counts
 *          - State Machine status and retry count
 *          - Wash Queue status
 *          - Current fault status
 *          - System Health snapshot
 *          - Diagnostics update count
 *
 *      No dynamic memory allocation is used.
 *
 *  Copyright:
 *      © 2026 WashTrac
 *
 ******************************************************************************/

#include "diagnostics_manager.h"

#include "input_manager.h"
#include "wash_queue.h"

#include "esp_log.h"

#include <cstddef>
#include <cstdint>
#include <limits>

namespace
{

constexpr const char* LOG_TAG = "DiagnosticsManager";

WashTrac::Diagnostics::Snapshot g_snapshot{};
std::array<bool, WashTrac::INPUT_COUNT> g_previousInputStates{};
std::array<WashTrac::Relays::RelayState, WashTrac::RELAY_COUNT>
    g_previousRelayStates{};

bool g_initialized = false;

uint32_t SaturatingIncrement(const uint32_t value)
{
    if (value == std::numeric_limits<uint32_t>::max())
    {
        return value;
    }

    return value + 1U;
}

uint64_t SaturatingIncrement(const uint64_t value)
{
    if (value == std::numeric_limits<uint64_t>::max())
    {
        return value;
    }

    return value + 1ULL;
}

WashTrac::Relays::RelayId RelayIdFromIndex(const std::size_t index)
{
    return static_cast<WashTrac::Relays::RelayId>(index + 1U);
}

void RefreshLiveData(const bool countTransitions)
{
    for (std::size_t index = 0U;
         index < WashTrac::INPUT_COUNT;
         ++index)
    {
        const bool currentState =
            WashTrac::Inputs::ReadInput(
                static_cast<uint8_t>(index + 1U));

        if (countTransitions &&
            currentState != g_previousInputStates[index])
        {
            g_snapshot.inputTransitionCounts[index] =
                SaturatingIncrement(
                    g_snapshot.inputTransitionCounts[index]);
        }

        g_snapshot.inputStates[index] = currentState;
        g_previousInputStates[index] = currentState;
    }

    for (std::size_t index = 0U;
         index < WashTrac::RELAY_COUNT;
         ++index)
    {
        const WashTrac::Relays::RelayState currentState =
            WashTrac::Relays::GetState(
                RelayIdFromIndex(index));

        if (countTransitions &&
            currentState == WashTrac::Relays::RelayState::Active &&
            g_previousRelayStates[index] !=
                WashTrac::Relays::RelayState::Active)
        {
            g_snapshot.relayActivationCounts[index] =
                SaturatingIncrement(
                    g_snapshot.relayActivationCounts[index]);
        }

        g_snapshot.relayStates[index] = currentState;
        g_previousRelayStates[index] = currentState;
    }

    g_snapshot.systemState =
        WashTrac::StateMachine::GetState();

    g_snapshot.retryCount =
        WashTrac::StateMachine::GetRetryCount();

    g_snapshot.pendingWashCount =
        WashTrac::WashQueue::Count();

    g_snapshot.queueEmpty =
        WashTrac::WashQueue::IsEmpty();

    g_snapshot.queueFull =
        WashTrac::WashQueue::IsFull();

    g_snapshot.currentFault =
        WashTrac::Faults::GetStatus();

    g_snapshot.systemHealth =
        WashTrac::SystemHealth::GetSnapshot();
}

} // namespace

namespace WashTrac::Diagnostics
{

Result Initialize()
{
    if (g_initialized)
    {
        return Result::OK;
    }

    if (!Inputs::IsInitialized() ||
        !Relays::IsInitialized() ||
        !WashQueue::IsInitialized() ||
        !Faults::IsInitialized() ||
        !SystemHealth::IsInitialized())
    {
        ESP_LOGE(
            LOG_TAG,
            "Initialization failed because a required subsystem "
            "is not initialized.");

        return Result::NOT_INITIALIZED;
    }

    g_snapshot = {};
    g_previousInputStates = {};

    for (std::size_t index = 0U;
         index < RELAY_COUNT;
         ++index)
    {
        g_previousRelayStates[index] =
            Relays::RelayState::Idle;
    }

    RefreshLiveData(false);

    g_initialized = true;

    ESP_LOGI(
        LOG_TAG,
        "Diagnostics Manager initialized.");

    return Result::OK;
}

void Update()
{
    if (!g_initialized)
    {
        return;
    }

    RefreshLiveData(true);

    g_snapshot.updateCount =
        SaturatingIncrement(
            g_snapshot.updateCount);
}

void ResetStatistics()
{
    if (!g_initialized)
    {
        return;
    }

    g_snapshot.inputTransitionCounts = {};
    g_snapshot.relayActivationCounts = {};
    g_snapshot.updateCount = 0ULL;

    RefreshLiveData(false);

    ESP_LOGI(
        LOG_TAG,
        "Diagnostics statistics reset.");
}

const Snapshot& GetSnapshot()
{
    return g_snapshot;
}

bool IsInitialized()
{
    return g_initialized;
}

} // namespace WashTrac::Diagnostics