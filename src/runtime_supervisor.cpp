/******************************************************************************
 *
 *  Project:
 *      WashTrac Core
 *
 *  Module:
 *      Runtime Supervisor
 *
 ******************************************************************************/

#include "runtime_supervisor.h"

#include "fault_manager.h"
#include "system_health.h"

#include "esp_log.h"

namespace
{
constexpr const char* LOG_TAG = "RuntimeSupervisor";
bool g_initialized = false;
}

namespace WashTrac::RuntimeSupervisor
{

Result Initialize()
{
    if (g_initialized)
    {
        return Result::OK;
    }

    if (!SystemHealth::IsInitialized())
    {
        return Result::NOT_INITIALIZED;
    }

    if (!Faults::IsInitialized())
    {
        return Result::NOT_INITIALIZED;
    }

    g_initialized = true;

    ESP_LOGI(LOG_TAG, "Runtime Supervisor initialized.");

    return Result::OK;
}

void Update()
{
    if (!g_initialized)
    {
        return;
    }

    SystemHealth::Update();

    (void)Faults::IsActive();
    (void)Faults::GetStatus();
}

bool IsInitialized()
{
    return g_initialized;
}

} // namespace WashTrac::RuntimeSupervisor