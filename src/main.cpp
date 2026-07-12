/******************************************************************************
 *
 *  Project:
 *      WashTrac Core
 *
 *  File:
 *      main.cpp
 *
 *  Description:
 *      Main firmware entry point, system initialization, and runtime loop.
 *
 *  Copyright:
 *      © 2026 WashTrac
 *
 ******************************************************************************/

#include "system.h"

#include "config.h"
#include "event_logger.h"
#include "fault_manager.h"
#include "gpio_manager.h"
#include "input_manager.h"
#include "manufacturing_self_test.h"
#include "relay_scheduler.h"
#include "runtime_supervisor.h"
#include "service_console.h"
#include "state_machine.h"
#include "system_health.h"
#include "wash_queue.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace
{

constexpr const char* LOG_TAG = "WashTracCore";

bool CheckResult(const WashTrac::Result result, const char* const moduleName)
{
    if (result == WashTrac::Result::OK)
    {
        ESP_LOGI(LOG_TAG, "%s initialized.", moduleName);
        return true;
    }

    ESP_LOGE(LOG_TAG, "%s initialization failed. Result code: %u",
             moduleName, static_cast<unsigned int>(result));
    return false;
}

bool InitializeSystem()
{
    if (!CheckResult(WashTrac::Events::Initialize(),"Event Logger")) return false;

    WashTrac::Events::Log(WashTrac::Events::EventCode::Boot);

    ESP_LOGI(LOG_TAG, "Project: %s", WashTrac::PROJECT_NAME);
    ESP_LOGI(LOG_TAG, "Firmware version: %s", WashTrac::FIRMWARE_VERSION);
    ESP_LOGI(LOG_TAG, "Hardware revision: %u",
             static_cast<unsigned int>(WashTrac::HARDWARE_REVISION));
    ESP_LOGI(LOG_TAG, "Configuration version: %u",
             static_cast<unsigned int>(WashTrac::CONFIG_VERSION));

    if (!CheckResult(WashTrac::ConfigurationManager::Initialize(),"Configuration Manager")) return false;

    const esp_err_t gpioResult = WashTrac::GPIO::Initialize();
    if (gpioResult != ESP_OK)
    {
        ESP_LOGE(LOG_TAG,"GPIO Manager initialization failed: %s",esp_err_to_name(gpioResult));
        return false;
    }

    ESP_LOGI(LOG_TAG,"GPIO Manager initialized.");

    if (!CheckResult(WashTrac::Inputs::Initialize(),"Input Manager")) return false;
    if (!CheckResult(WashTrac::Relays::Initialize(),"Relay Scheduler")) return false;
    if (!CheckResult(WashTrac::WashQueue::Initialize(),"Wash Queue")) return false;
    if (!CheckResult(WashTrac::Faults::Initialize(),"Fault Manager")) return false;
    if (!CheckResult(WashTrac::SystemHealth::Initialize(),"System Health Manager")) return false;

    if (!CheckResult(WashTrac::RuntimeSupervisor::Initialize(),"Runtime Supervisor")) return false;

    WashTrac::StateMachine::Initialize();
    ESP_LOGI(LOG_TAG,"State Machine initialized.");

    if (!CheckResult(WashTrac::ManufacturingSelfTest::Initialize(),"Manufacturing Self-Test")) return false;
    if (!CheckResult(WashTrac::ServiceConsole::Initialize(),"Service Console")) return false;

    WashTrac::Events::Log(WashTrac::Events::EventCode::SystemInitialized);
    ESP_LOGI(LOG_TAG,"System foundation initialized.");
    return true;
}

void RunSystem()
{
    while (true)
    {
        const int64_t loopStart = esp_timer_get_time();

        WashTrac::Inputs::Update();
        WashTrac::StateMachine::Update();
        WashTrac::Relays::Update();
        WashTrac::RuntimeSupervisor::Update();
        WashTrac::ServiceConsole::Update();

        const int64_t loopEnd = esp_timer_get_time();
        WashTrac::SystemHealth::RecordLoopDuration(
            static_cast<uint32_t>(loopEnd - loopStart));

        vTaskDelay(pdMS_TO_TICKS(WashTrac::SYSTEM_TICK_MS));
    }
}

}

extern "C" void app_main()
{
    ESP_LOGI(LOG_TAG,"WashTrac Core booting.");

    if (!InitializeSystem())
    {
        ESP_LOGE(LOG_TAG,"WashTrac Core initialization failed. Runtime operation has been inhibited.");
        while (true)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    ESP_LOGI(LOG_TAG,"WashTrac Core initialization complete.");
    RunSystem();
}