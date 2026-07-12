/******************************************************************************
 *
 *  Project:
 *      WashTrac Core
 *
 *  Module:
 *      Input Manager
 *
 *  File:
 *      input_manager.cpp
 *
 *  Description:
 *      Production Input Manager implementation providing:
 *          - Reading all six physical inputs
 *          - Uniform per-input inversion handling
 *          - Disabled-input suppression
 *          - Immediate Wash Busy activation from logical Input 1
 *          - Configurable whole-second Wash Busy release delay
 *          - Fail-safe E-Stop reporting from logical Input 2
 *          - Real hardware input as the only source of wash activity
 *
 *  Copyright:
 *      © 2026 WashTrac
 *
 ******************************************************************************/

#include "input_manager.h"

#include "config.h"
#include "gpio_manager.h"

#include "esp_log.h"
#include "esp_timer.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace
{

constexpr const char* LOG_TAG = "InputManager";
constexpr int64_t MICROSECONDS_PER_SECOND = 1000000LL;

struct InputManagerState
{
    std::array<bool, WashTrac::INPUT_COUNT> inputStates;

    bool washBusy;
    bool washBusyReleasePending;

    int64_t washBusyReleaseStartMicroseconds;

    bool initialized;
};

InputManagerState g_state{};

int64_t SecondsToMicroseconds(const uint16_t seconds)
{
    return static_cast<int64_t>(seconds) *
           MICROSECONDS_PER_SECOND;
}

bool ApplyConfiguredLogic(
    const bool rawState,
    const WashTrac::InputConfig& config)
{
    if (!config.enabled)
    {
        return false;
    }

    return config.inverted
        ? !rawState
        : rawState;
}

bool ReadRawInput(const uint8_t inputNumber)
{
    switch (inputNumber)
    {
        case 1U:
            return WashTrac::GPIO::ReadInput1();

        case 2U:
            return WashTrac::GPIO::ReadInput2();

        case 3U:
            return WashTrac::GPIO::ReadInput3();

        case 4U:
            return WashTrac::GPIO::ReadInput4();

        case 5U:
            return WashTrac::GPIO::ReadInput5();

        case 6U:
            return WashTrac::GPIO::ReadInput6();

        default:
            return false;
    }
}

void RefreshLogicalInputStates()
{
    const WashTrac::CoreConfig& configuration =
        WashTrac::ConfigurationManager::Get();

    for (std::size_t index = 0U;
         index < WashTrac::INPUT_COUNT;
         ++index)
    {
        const uint8_t inputNumber =
            static_cast<uint8_t>(index + 1U);

        const bool rawState =
            ReadRawInput(inputNumber);

        g_state.inputStates[index] =
            ApplyConfiguredLogic(
                rawState,
                configuration.inputs[index]);
    }
}

void UpdateWashBusyState()
{
    const bool washBusyInputActive =
        g_state.inputStates[0];

    if (washBusyInputActive)
    {
        g_state.washBusy = true;
        g_state.washBusyReleasePending = false;
        g_state.washBusyReleaseStartMicroseconds = 0;
        return;
    }

    if (!g_state.washBusy)
    {
        g_state.washBusyReleasePending = false;
        g_state.washBusyReleaseStartMicroseconds = 0;
        return;
    }

    const int64_t currentTimeMicroseconds =
        esp_timer_get_time();

    if (!g_state.washBusyReleasePending)
    {
        g_state.washBusyReleasePending = true;
        g_state.washBusyReleaseStartMicroseconds =
            currentTimeMicroseconds;
        return;
    }

    const uint16_t releaseDelaySeconds =
        WashTrac::ConfigurationManager::Get()
            .washBusyReleaseDelaySeconds;

    const int64_t releaseDelayMicroseconds =
        SecondsToMicroseconds(releaseDelaySeconds);

    const int64_t elapsedMicroseconds =
        currentTimeMicroseconds -
        g_state.washBusyReleaseStartMicroseconds;

    if (elapsedMicroseconds >= releaseDelayMicroseconds)
    {
        g_state.washBusy = false;
        g_state.washBusyReleasePending = false;
        g_state.washBusyReleaseStartMicroseconds = 0;

        ESP_LOGI(
            LOG_TAG,
            "Wash Busy released after %u seconds.",
            static_cast<unsigned>(releaseDelaySeconds));
    }
}

} // namespace

namespace WashTrac::Inputs
{

Result Initialize()
{
    if (g_state.initialized)
    {
        return Result::OK;
    }

    if (!ConfigurationManager::IsInitialized())
    {
        return Result::NOT_INITIALIZED;
    }

    g_state.inputStates.fill(false);

    g_state.washBusy = false;
    g_state.washBusyReleasePending = false;
    g_state.washBusyReleaseStartMicroseconds = 0;

    RefreshLogicalInputStates();
    UpdateWashBusyState();

    g_state.initialized = true;

    ESP_LOGI(
        LOG_TAG,
        "Input Manager initialized with configurable inversion "
        "for all six inputs.");

    return Result::OK;
}

void Update()
{
    if (!g_state.initialized)
    {
        return;
    }

    RefreshLogicalInputStates();
    UpdateWashBusyState();
}

bool IsWashBusy()
{
    if (!g_state.initialized)
    {
        return false;
    }

    return g_state.washBusy;
}

bool IsEStopActive()
{
    if (!g_state.initialized)
    {
        return true;
    }

    return g_state.inputStates[1];
}

bool ReadInput(const uint8_t inputNumber)
{
    if (!g_state.initialized)
    {
        return false;
    }

    if (inputNumber < 1U ||
        inputNumber > INPUT_COUNT)
    {
        return false;
    }

    return g_state.inputStates[inputNumber - 1U];
}

bool IsInitialized()
{
    return g_state.initialized;
}

} // namespace WashTrac::Inputs