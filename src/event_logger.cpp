/******************************************************************************
 *
 *  Project:
 *      WashTrac Core
 *
 *  Module:
 *      Event Logger
 *
 *  File:
 *      event_logger.cpp
 *
 *  Description:
 *      Production Event Logger implementation providing:
 *          - Fixed-size circular event storage
 *          - Chronological event retrieval
 *          - Automatic overwrite of the oldest event when full
 *          - FreeRTOS tick timestamps
 *          - Human-readable diagnostic logging
 *          - No dynamic memory allocation
 *
 ******************************************************************************/

#include "event_logger.h"

#include "esp_log.h"

#include "freertos/task.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace
{

constexpr const char* LOG_TAG = "EventLogger";

struct EventLoggerState
{
    std::array<
        WashTrac::Events::Event,
        WashTrac::Events::MAX_EVENTS> events;

    std::size_t head;
    std::size_t count;
    bool initialized;
};

EventLoggerState g_state{};

const char* EventToString(
    const WashTrac::Events::EventCode code)
{
    using WashTrac::Events::EventCode;

    switch (code)
    {
        case EventCode::None:
            return "None";

        case EventCode::Boot:
            return "Boot";

        case EventCode::ConfigurationLoaded:
            return "Configuration Loaded";

        case EventCode::ConfigurationSaved:
            return "Configuration Saved";

        case EventCode::FactoryReset:
            return "Factory Reset";

        case EventCode::WashQueued:
            return "Wash Queued";

        case EventCode::WashDequeued:
            return "Wash Dequeued";

        case EventCode::WashStartAttempt:
            return "Wash Start Attempt";

        case EventCode::WashStarted:
            return "Wash Started";

        case EventCode::WashCompleted:
            return "Wash Completed";

        case EventCode::RetryStarted:
            return "Retry Started";

        case EventCode::FaultRaised:
            return "Fault Raised";

        case EventCode::FaultCleared:
            return "Fault Cleared";

        case EventCode::EStopActive:
            return "E-Stop Active";

        case EventCode::EStopCleared:
            return "E-Stop Cleared";

        case EventCode::QueueCleared:
            return "Queue Cleared";

        case EventCode::SystemInitialized:
            return "System Initialized";

        default:
            return "Unknown";
    }
}

std::size_t GetOldestIndex()
{
    if (g_state.count < WashTrac::Events::MAX_EVENTS)
        return 0U;

    return g_state.head;
}

} // namespace

namespace WashTrac::Events
{

Result Initialize()
{
    if (g_state.initialized)
        return Result::OK;

    g_state.events.fill(
        Event{
            EventCode::None,
            0U,
            0U});

    g_state.head = 0U;
    g_state.count = 0U;
    g_state.initialized = true;

    ESP_LOGI(
        LOG_TAG,
        "Event Logger initialized with capacity %u.",
        static_cast<unsigned>(MAX_EVENTS));

    return Result::OK;
}

void Log(
    const EventCode code,
    const uint32_t value)
{
    if (!g_state.initialized)
        return;

    if (code == EventCode::None)
    {
        ESP_LOGW(
            LOG_TAG,
            "Ignored EventCode::None.");

        return;
    }

    Event& destination =
        g_state.events[g_state.head];

    destination.code = code;
    destination.timestamp = xTaskGetTickCount();
    destination.value = value;

    g_state.head =
        (g_state.head + 1U) % MAX_EVENTS;

    if (g_state.count < MAX_EVENTS)
    {
        ++g_state.count;
    }

    ESP_LOGD(
        LOG_TAG,
        "Event: %s. Value: %lu.",
        EventToString(code),
        static_cast<unsigned long>(value));
}

std::size_t Count()
{
    if (!g_state.initialized)
        return 0U;

    return g_state.count;
}

bool Get(
    const std::size_t index,
    Event& event)
{
    if (!g_state.initialized)
        return false;

    if (index >= g_state.count)
        return false;

    const std::size_t oldestIndex =
        GetOldestIndex();

    const std::size_t physicalIndex =
        (oldestIndex + index) % MAX_EVENTS;

    event = g_state.events[physicalIndex];

    return true;
}

void Clear()
{
    if (!g_state.initialized)
        return;

    g_state.events.fill(
        Event{
            EventCode::None,
            0U,
            0U});

    g_state.head = 0U;
    g_state.count = 0U;

    ESP_LOGI(LOG_TAG, "Event log cleared.");
}

bool IsFull()
{
    if (!g_state.initialized)
        return false;

    return g_state.count >= MAX_EVENTS;
}

bool IsInitialized()
{
    return g_state.initialized;
}

} // namespace WashTrac::Events
