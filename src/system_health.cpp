/******************************************************************************
 *
 *  Project:
 *      WashTrac Core
 *
 *  Module:
 *      System Health Manager
 *
 *  File:
 *      system_health.cpp
 *
 *  Description:
 *      Production System Health Manager implementation providing:
 *          - System uptime tracking
 *          - Current and minimum free heap monitoring
 *          - Largest available heap block monitoring
 *          - Main task stack high-water monitoring
 *          - Runtime-loop execution timing
 *          - Runtime-loop deadline overrun counting
 *
 *      No dynamic memory allocation is used.
 *
 ******************************************************************************/

#include "system_health.h"

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <cstdint>
#include <limits>

namespace
{

constexpr const char* LOG_TAG = "SystemHealth";

WashTrac::SystemHealth::Snapshot g_snapshot{};
bool g_initialized = false;

uint32_t SaturateToUint32(
    const uint64_t value)
{
    if (value > std::numeric_limits<uint32_t>::max())
    {
        return std::numeric_limits<uint32_t>::max();
    }

    return static_cast<uint32_t>(value);
}

uint32_t GetMainTaskStackHighWaterBytes()
{
    const UBaseType_t highWaterWords =
        uxTaskGetStackHighWaterMark(nullptr);

    const uint64_t highWaterBytes =
        static_cast<uint64_t>(highWaterWords) *
        static_cast<uint64_t>(sizeof(StackType_t));

    return SaturateToUint32(highWaterBytes);
}

void RefreshSnapshot()
{
    const int64_t uptimeMicroseconds =
        esp_timer_get_time();

    g_snapshot.uptimeMilliseconds =
        uptimeMicroseconds > 0
            ? static_cast<uint64_t>(uptimeMicroseconds) / 1000ULL
            : 0ULL;

    g_snapshot.freeHeapBytes =
        heap_caps_get_free_size(MALLOC_CAP_8BIT);

    g_snapshot.minimumFreeHeapBytes =
        heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);

    g_snapshot.largestFreeHeapBlockBytes =
        heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);

    g_snapshot.mainTaskStackHighWaterBytes =
        GetMainTaskStackHighWaterBytes();
}

} // namespace

namespace WashTrac::SystemHealth
{

Result Initialize()
{
    if (g_initialized)
        return Result::OK;

    g_snapshot = {};
    g_initialized = true;

    RefreshSnapshot();

    ESP_LOGI(
        LOG_TAG,
        "System Health Manager initialized. "
        "Free heap: %lu bytes. "
        "Minimum free heap: %lu bytes. "
        "Main task stack high-water: %lu bytes.",
        static_cast<unsigned long>(g_snapshot.freeHeapBytes),
        static_cast<unsigned long>(g_snapshot.minimumFreeHeapBytes),
        static_cast<unsigned long>(
            g_snapshot.mainTaskStackHighWaterBytes));

    return Result::OK;
}

void Update()
{
    if (!g_initialized)
        return;

    RefreshSnapshot();
}

void RecordLoopDuration(
    const uint32_t durationMicroseconds)
{
    if (!g_initialized)
        return;

    g_snapshot.lastLoopDurationMicroseconds =
        durationMicroseconds;

    if (durationMicroseconds >
        g_snapshot.maximumLoopDurationMicroseconds)
    {
        g_snapshot.maximumLoopDurationMicroseconds =
            durationMicroseconds;
    }

    const uint64_t loopDeadlineMicroseconds =
        static_cast<uint64_t>(SYSTEM_TICK_MS) * 1000ULL;

    if (static_cast<uint64_t>(durationMicroseconds) >
        loopDeadlineMicroseconds)
    {
        ++g_snapshot.loopDeadlineOverrunCount;

        ESP_LOGW(
            LOG_TAG,
            "Runtime loop deadline exceeded. "
            "Duration: %lu us. Deadline: %llu us. "
            "Overruns: %lu.",
            static_cast<unsigned long>(durationMicroseconds),
            static_cast<unsigned long long>(
                loopDeadlineMicroseconds),
            static_cast<unsigned long>(
                g_snapshot.loopDeadlineOverrunCount));
    }
}

const Snapshot& GetSnapshot()
{
    return g_snapshot;
}

bool IsInitialized()
{
    return g_initialized;
}

} // namespace WashTrac::SystemHealth
