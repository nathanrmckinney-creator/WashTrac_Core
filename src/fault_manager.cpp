/******************************************************************************
 *
 *  Project:
 *      WashTrac Core
 *
 *  Module:
 *      Fault Manager
 *
 *  File:
 *      fault_manager.cpp
 *
 *  Description:
 *      Production Fault Manager implementation providing:
 *          - Current controller fault tracking
 *          - Previous fault retention
 *          - Fault occurrence counting
 *          - FreeRTOS tick timestamps
 *          - Duplicate-fault suppression
 *          - Event Logger integration for fault transitions
 *
 *      Complete historical event storage is handled by the Event Logger
 *      subsystem.
 *
 ******************************************************************************/

#include "fault_manager.h"

#include "event_logger.h"

#include "esp_log.h"

#include "freertos/task.h"

namespace
{

constexpr const char* LOG_TAG = "FaultManager";

WashTrac::Faults::FaultStatus g_currentFault{};
WashTrac::Faults::FaultStatus g_previousFault{};

uint32_t g_occurrenceCount = 0U;
bool g_initialized = false;

const char* FaultToString(
    const WashTrac::Faults::FaultCode code)
{
    using WashTrac::Faults::FaultCode;

    switch (code)
    {
        case FaultCode::None:
            return "None";

        case FaultCode::WashStartTimeout:
            return "Wash Start Timeout";

        case FaultCode::RelaySchedulerFailure:
            return "Relay Scheduler Failure";

        case FaultCode::QueueFailure:
            return "Queue Failure";

        case FaultCode::ConfigurationFailure:
            return "Configuration Failure";

        case FaultCode::EStopActive:
            return "E-Stop Active";

        case FaultCode::InvalidState:
            return "Invalid State";

        case FaultCode::InternalError:
            return "Internal Error";

        default:
            return "Unknown";
    }
}

void ResetStatus(
    WashTrac::Faults::FaultStatus& status)
{
    status.code = WashTrac::Faults::FaultCode::None;
    status.active = false;
    status.occurrenceCount = 0U;
    status.timestamp = 0U;
}

} // namespace

namespace WashTrac::Faults
{

Result Initialize()
{
    if (g_initialized)
        return Result::OK;

    ResetStatus(g_currentFault);
    ResetStatus(g_previousFault);

    g_occurrenceCount = 0U;
    g_initialized = true;

    ESP_LOGI(LOG_TAG, "Fault Manager initialized.");

    return Result::OK;
}

void Raise(const FaultCode code)
{
    if (!g_initialized)
        return;

    /*
     * FaultCode::None is not a valid fault raise. Clear() must be used
     * when the current fault condition has ended.
     */
    if (code == FaultCode::None)
    {
        ESP_LOGW(
            LOG_TAG,
            "Ignored request to raise FaultCode::None.");

        return;
    }

    /*
     * Repeated reports of the currently active fault do not create a new
     * occurrence and do not alter its original timestamp.
     */
    if (g_currentFault.active &&
        g_currentFault.code == code)
    {
        return;
    }

    /*
     * Preserve the currently active fault before replacing it with a
     * different fault condition.
     */
    if (g_currentFault.active)
    {
        g_previousFault = g_currentFault;
    }

    ++g_occurrenceCount;

    g_currentFault.code = code;
    g_currentFault.active = true;
    g_currentFault.occurrenceCount = g_occurrenceCount;
    g_currentFault.timestamp = xTaskGetTickCount();

    WashTrac::Events::Log(
        WashTrac::Events::EventCode::FaultRaised,
        static_cast<uint32_t>(code));

    ESP_LOGE(
        LOG_TAG,
        "FAULT RAISED: %s. Occurrence: %lu.",
        FaultToString(code),
        static_cast<unsigned long>(g_occurrenceCount));
}

void Clear()
{
    if (!g_initialized)
        return;

    if (!g_currentFault.active)
        return;

    const FaultCode clearedCode =
        g_currentFault.code;

    ESP_LOGI(
        LOG_TAG,
        "FAULT CLEARED: %s.",
        FaultToString(clearedCode));

    /*
     * Retain the complete fault record before clearing current health.
     */
    g_previousFault = g_currentFault;

    g_currentFault.code = FaultCode::None;
    g_currentFault.active = false;
    g_currentFault.occurrenceCount = g_occurrenceCount;
    g_currentFault.timestamp = xTaskGetTickCount();

    WashTrac::Events::Log(
        WashTrac::Events::EventCode::FaultCleared,
        static_cast<uint32_t>(clearedCode));
}

bool IsActive()
{
    if (!g_initialized)
        return false;

    return g_currentFault.active;
}

FaultCode GetCode()
{
    if (!g_initialized)
        return FaultCode::None;

    return g_currentFault.code;
}

const FaultStatus& GetStatus()
{
    return g_currentFault;
}

const FaultStatus& GetPreviousStatus()
{
    return g_previousFault;
}

bool IsInitialized()
{
    return g_initialized;
}

} // namespace WashTrac::Faults
