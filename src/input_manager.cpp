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
 *      Input Manager implementation providing:
 *          - Reading and caching all hardware inputs
 *          - Immediate Wash Busy activation from Input 1
 *          - Configurable whole-second Wash Busy release delay
 *          - Real hardware input as the only source of wash activity
 *
 ******************************************************************************/

#include "input_manager.h"

#include "config.h"
#include "gpio_manager.h"

#include "esp_log.h"
#include "esp_timer.h"

#include <array>
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

int64_t SecondsToMicroseconds(uint16_t seconds)
{
    return static_cast<int64_t>(seconds) *
           MICROSECONDS_PER_SECOND;
}

void UpdateWashBusyState()
{
    const bool washBusyInputActive =
        g_state.inputStates[0];

    /*
     * Input 1 becoming active immediately establishes Wash Busy.
     * Any pending release delay is cancelled.
     */
    if (washBusyInputActive)
    {
        g_state.washBusy = true;
        g_state.washBusyReleasePending = false;
        g_state.washBusyReleaseStartMicroseconds = 0;

        return;
    }

    /*
     * If Wash Busy was never established by the real hardware input,
     * do not start any timer and do not simulate a wash.
     */
    if (!g_state.washBusy)
    {
        g_state.washBusyReleasePending = false;
        g_state.washBusyReleaseStartMicroseconds = 0;

        return;
    }

    const int64_t currentTimeMicroseconds =
        esp_timer_get_time();

    /*
     * Input 1 has just become inactive. Begin the configured
     * whole-second release delay.
     */
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

    /*
     * Wash Busy clears only after Input 1 has remained continuously
     * inactive for the complete configured delay.
     */
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
        return Result::OK;

    if (!ConfigurationManager::IsInitialized())
        return Result::NOT_INITIALIZED;

    g_state.inputStates.fill(false);

    g_state.washBusy = false;
    g_state.washBusyReleasePending = false;
    g_state.washBusyReleaseStartMicroseconds = 0;

    g_state.initialized = true;

    ESP_LOGI(LOG_TAG, "Input Manager initialized.");

    return Result::OK;
}

void Update()
{
    if (!g_state.initialized)
        return;

    g_state.inputStates[0] = GPIO::ReadInput1();
    g_state.inputStates[1] = GPIO::ReadInput2();
    g_state.inputStates[2] = GPIO::ReadInput3();
    g_state.inputStates[3] = GPIO::ReadInput4();
    g_state.inputStates[4] = GPIO::ReadInput5();
    g_state.inputStates[5] = GPIO::ReadInput6();

    UpdateWashBusyState();
}

bool IsWashBusy()
{
    if (!g_state.initialized)
        return false;

    return g_state.washBusy;
}

bool IsEStopActive()
{
    if (!g_state.initialized)
        return true;

    const InputConfig& config =
        WashTrac::ConfigurationManager::Get().inputs[1];

    const bool rawInputActive =
        g_state.inputStates[1];

    return config.inverted
        ? !rawInputActive
        : rawInputActive;
}

bool ReadInput(uint8_t inputNumber)
{
    if (!g_state.initialized)
        return false;

    if (inputNumber < 1U || inputNumber > INPUT_COUNT)
        return false;

    return g_state.inputStates[inputNumber - 1U];
}

bool IsInitialized()
{
    return g_state.initialized;
}

} // namespace WashTrac::Inputs