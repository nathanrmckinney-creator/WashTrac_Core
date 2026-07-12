/******************************************************************************
 *
 *  Project:
 *      WashTrac Core
 *
 *  Module:
 *      Runtime Watchdog
 *
 *  File:
 *      runtime_watchdog.cpp
 *
 *  Description:
 *      Production Runtime Watchdog implementation providing:
 *          - Main runtime task subscription to the ESP-IDF Task Watchdog
 *          - Periodic watchdog feeding
 *          - Delayed runtime-loop detection after execution resumes
 *          - Fault Manager integration for abnormal supervision delays
 *
 *      No dynamic memory allocation is used.
 *
 *  Copyright:
 *      © 2026 WashTrac
 *
 ******************************************************************************/

#include "runtime_watchdog.h"

#include "fault_manager.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <cstdint>

namespace
{

constexpr const char* LOG_TAG = "RuntimeWatchdog";

/*
 * Normal runtime execution occurs every SYSTEM_TICK_MS. A supervision gap of
 * two seconds indicates that the main runtime task was delayed far beyond its
 * intended operating period, while remaining below common hardware watchdog
 * reset thresholds.
 */
constexpr int64_t MAXIMUM_UPDATE_INTERVAL_US = 2000000LL;

int64_t g_previousUpdateTimeUs = 0;
bool g_initialized = false;

} // namespace

namespace WashTrac::RuntimeWatchdog
{

Result Initialize()
{
    if (g_initialized)
    {
        return Result::OK;
    }

    if (!Faults::IsInitialized())
    {
        ESP_LOGE(
            LOG_TAG,
            "Initialization failed because Fault Manager is not initialized.");

        return Result::NOT_INITIALIZED;
    }

    /*
     * esp_task_wdt_status() returns ESP_OK when the current task is already
     * subscribed. This can occur when the project configuration subscribes
     * app_main automatically.
     */
    const esp_err_t statusResult =
        esp_task_wdt_status(nullptr);

    if (statusResult != ESP_OK)
    {
        const esp_err_t addResult =
            esp_task_wdt_add(nullptr);

        if (addResult != ESP_OK)
        {
            ESP_LOGE(
                LOG_TAG,
                "Failed to subscribe runtime task to watchdog: %s",
                esp_err_to_name(addResult));

            return Result::ERROR;
        }
    }

    g_previousUpdateTimeUs = esp_timer_get_time();
    g_initialized = true;

    ESP_LOGI(
        LOG_TAG,
        "Runtime Watchdog initialized and runtime task subscribed.");

    return Result::OK;
}

void Update()
{
    if (!g_initialized)
    {
        return;
    }

    const int64_t currentTimeUs =
        esp_timer_get_time();

    if (g_previousUpdateTimeUs > 0)
    {
        const int64_t updateIntervalUs =
            currentTimeUs - g_previousUpdateTimeUs;

        if (updateIntervalUs > MAXIMUM_UPDATE_INTERVAL_US)
        {
            ESP_LOGE(
                LOG_TAG,
                "Runtime supervision delay detected: %lld us.",
                static_cast<long long>(updateIntervalUs));

            Faults::Raise(
                Faults::FaultCode::InternalError);
        }
    }

    g_previousUpdateTimeUs = currentTimeUs;

    const esp_err_t resetResult =
        esp_task_wdt_reset();

    if (resetResult != ESP_OK)
    {
        ESP_LOGE(
            LOG_TAG,
            "Failed to feed runtime watchdog: %s",
            esp_err_to_name(resetResult));

        Faults::Raise(
            Faults::FaultCode::InternalError);
    }
}

bool IsInitialized()
{
    return g_initialized;
}

} // namespace WashTrac::RuntimeWatchdog