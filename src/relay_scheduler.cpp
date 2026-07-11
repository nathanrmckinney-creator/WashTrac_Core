/******************************************************************************
 *
 *  Project:
 *      WashTrac Core
 *
 *  Module:
 *      Relay Scheduler
 *
 *  File:
 *      relay_scheduler.cpp
 *
 *  Description:
 *      Initial Relay Scheduler implementation providing:
 *          - Module initialization
 *          - Per-relay state tracking
 *          - Safe relay shutdown
 *          - Public state-query interface
 *
 *      Timing and relay execution logic will be added in the next revision.
 *
 ******************************************************************************/

#include "relay_scheduler.h"

#include "config.h"

#include "gpio_manager.h"

#include "esp_log.h"

#include "esp_timer.h"

#include <array>

namespace
{

constexpr const char* LOG_TAG = "RelayScheduler";

struct RelayRuntime
{
    WashTrac::Relays::RelayState state;
    int64_t stateStartMicroseconds;
};

struct RelaySchedulerState
{
    std::array<RelayRuntime, WashTrac::RELAY_COUNT> relays;
    bool initialized;
};

RelaySchedulerState g_state{};

void SetRelayOutput(uint8_t relayNumber, bool active)
{
    switch (relayNumber)
    {
        case 1U:
            WashTrac::GPIO::SetRelay1(active);
            break;

        case 2U:
            WashTrac::GPIO::SetRelay2(active);
            break;

        case 3U:
            WashTrac::GPIO::SetRelay3(active);
            break;

        case 4U:
            WashTrac::GPIO::SetRelay4(active);
            break;

        case 5U:
            WashTrac::GPIO::SetRelay5(active);
            break;

        case 6U:
            WashTrac::GPIO::SetRelay6(active);
            break;

        default:
            break;
    }
}

bool IsRelayNumberValid(uint8_t relayNumber)
{
    return relayNumber >= 1U &&
           relayNumber <= WashTrac::RELAY_COUNT;
}

} // namespace

namespace WashTrac::Relays
{

Result Initialize()
{
    if (g_state.initialized)
        return Result::OK;

    for (std::size_t i = 0U; i < RELAY_COUNT; ++i)
    {
        g_state.relays[i].state = RelayState::Idle;
        g_state.relays[i].stateStartMicroseconds = 0;
        SetRelayOutput(static_cast<uint8_t>(i + 1U), false);
    }

    g_state.initialized = true;

    ESP_LOGI(LOG_TAG, "Relay Scheduler initialized.");

    return Result::OK;
}

Result Start(RelayId relayId)
{
    const uint8_t relayNumber =
        static_cast<uint8_t>(relayId);

    if (!g_state.initialized)
        return Result::NOT_INITIALIZED;

    if (!IsRelayNumberValid(relayNumber))
        return Result::INVALID_PARAMETER;

    const RelayConfig& config =
        ConfigurationManager::Get().relays[relayNumber - 1U];

    if (!config.enabled)
        return Result::INVALID_PARAMETER;

    RelayRuntime& relayRuntime =
        g_state.relays[relayNumber - 1U];

    if (relayRuntime.state != RelayState::Idle)
        return Result::INVALID_PARAMETER;

    relayRuntime.stateStartMicroseconds =
        esp_timer_get_time();

    if (config.onDelaySeconds > 0U)
    {
        relayRuntime.state =
            RelayState::WaitingOnDelay;
    }
    else
    {
        SetRelayOutput(relayNumber, true);

        relayRuntime.state =
            RelayState::Active;

        relayRuntime.stateStartMicroseconds =
            esp_timer_get_time();
    }

    return Result::OK;
}

Result Cancel(RelayId relayId)
{
    const uint8_t relayNumber =
        static_cast<uint8_t>(relayId);

    if (!g_state.initialized)
        return Result::NOT_INITIALIZED;

    if (!IsRelayNumberValid(relayNumber))
        return Result::INVALID_PARAMETER;

    RelayRuntime& relayRuntime =
        g_state.relays[relayNumber - 1U];

    SetRelayOutput(relayNumber, false);

    relayRuntime.state = RelayState::Idle;
    relayRuntime.stateStartMicroseconds = 0;

    return Result::OK;
}

void Update()
{
    if (!g_state.initialized)
        return;

    const int64_t currentTimeMicroseconds =
        esp_timer_get_time();

    for (std::size_t i = 0U; i < RELAY_COUNT; ++i)
    {
        RelayRuntime& relay = g_state.relays[i];

        const RelayConfig& config =
            ConfigurationManager::Get().relays[i];

        const uint8_t relayNumber =
            static_cast<uint8_t>(i + 1U);

        const int64_t elapsedMicroseconds =
            currentTimeMicroseconds -
            relay.stateStartMicroseconds;

        switch (relay.state)
        {
            case RelayState::Idle:
                break;

            case RelayState::WaitingOnDelay:
            {
                const int64_t onDelayMicroseconds =
                    static_cast<int64_t>(
                        config.onDelaySeconds) *
                    1000000LL;

                if (elapsedMicroseconds >=
                    onDelayMicroseconds)
                {
                    SetRelayOutput(relayNumber, true);

                    relay.state = RelayState::Active;
                    relay.stateStartMicroseconds =
                        currentTimeMicroseconds;
                }

                break;
            }

            case RelayState::Active:
            {
                const int64_t durationMicroseconds =
                    static_cast<int64_t>(
                        config.durationSeconds) *
                    1000000LL;

                if (elapsedMicroseconds >=
                    durationMicroseconds)
                {
                    if (config.offDelaySeconds > 0U)
                    {
                        /*
                         * The relay remains energized during the
                         * configured off-delay.
                         */
                        relay.state =
                            RelayState::WaitingOffDelay;

                        relay.stateStartMicroseconds =
                            currentTimeMicroseconds;
                    }
                    else
                    {
                        SetRelayOutput(relayNumber, false);

                        relay.state = RelayState::Idle;
                        relay.stateStartMicroseconds = 0;
                    }
                }

                break;
            }

            case RelayState::WaitingOffDelay:
            {
                const int64_t offDelayMicroseconds =
                    static_cast<int64_t>(
                        config.offDelaySeconds) *
                    1000000LL;

                if (elapsedMicroseconds >=
                    offDelayMicroseconds)
                {
                    SetRelayOutput(relayNumber, false);

                    relay.state = RelayState::Idle;
                    relay.stateStartMicroseconds = 0;
                }

                break;
            }
        }
    }
}

RelayState GetState(RelayId relayId)
{
    const uint8_t relayNumber =
        static_cast<uint8_t>(relayId);

    if (!g_state.initialized)
        return RelayState::Idle;

    if (!IsRelayNumberValid(relayNumber))
        return RelayState::Idle;

    return g_state.relays[relayNumber - 1U].state;
}

bool IsActive(RelayId relayId)
{
    return GetState(relayId) == RelayState::Active;
}

bool IsRunning(RelayId relayId)
{
    return GetState(relayId) != RelayState::Idle;
}

bool IsInitialized()
{
    return g_state.initialized;
}

} // namespace WashTrac::Relays