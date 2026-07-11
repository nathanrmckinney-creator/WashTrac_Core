#include "state_machine.h"

#include "input_manager.h"
#include "relay_scheduler.h"
#include "wash_queue.h"

#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace WashTrac::StateMachine
{

namespace
{

constexpr const char* LOG_TAG = "StateMachine";

SystemState g_state = SystemState::Idle;
uint8_t g_retryCount = 0U;

/*
 * Tick count recorded whenever the state changes.
 * All configured timing remains in whole seconds.
 */
TickType_t g_stateEntryTick = 0;

const char* GetStateName(const SystemState state)
{
    switch (state)
    {
        case SystemState::Idle:
            return "Idle";

        case SystemState::StartingWash:
            return "StartingWash";

        case SystemState::WaitingForBusy:
            return "WaitingForBusy";

        case SystemState::WashRunning:
            return "WashRunning";

        case SystemState::ReleaseDelay:
            return "ReleaseDelay";

        case SystemState::QueueDelay:
            return "QueueDelay";

        case SystemState::RetryDelay:
            return "RetryDelay";

        case SystemState::Fault:
            return "Fault";

        case SystemState::EStop:
            return "EStop";

        default:
            return "Unknown";
    }
}

void ChangeState(const SystemState newState)
{
    if (newState != g_state)
    {
        ESP_LOGI(
            LOG_TAG,
            "State change: %s -> %s",
            GetStateName(g_state),
            GetStateName(newState));
    }

    g_state = newState;
    g_stateEntryTick = xTaskGetTickCount();
}

bool StateTimeElapsed(const uint16_t seconds)
{
    const TickType_t elapsedTicks =
        xTaskGetTickCount() - g_stateEntryTick;

    return elapsedTicks >= pdMS_TO_TICKS(
        static_cast<uint32_t>(seconds) * 1000U);
}

} // namespace

void Initialize()
{
    g_retryCount = 0U;

    ChangeState(SystemState::Idle);

    ESP_LOGI(LOG_TAG, "State Machine initialized.");
}

void Update()
{
    /*
     * E-Stop has priority over all normal state-machine behavior.
     *
     * Any relay pulse that has already started is intentionally not
     * interrupted. The relay scheduler remains responsible for completing
     * that pulse.
     */
    if (WashTrac::Inputs::IsEStopActive())
    {
        if (g_state != SystemState::EStop)
        {
            EnterEStop();
        }

        return;
    }

    /*
     * Recover automatically when the E-Stop circuit becomes healthy.
     * The queue remains empty and no previous wash attempt resumes.
     */
    if (g_state == SystemState::EStop)
    {
        g_retryCount = 0U;
        ChangeState(SystemState::Idle);
    }

    switch (g_state)
    {
        case SystemState::Idle:
        {
            if (!WashTrac::WashQueue::IsEmpty())
            {
                if (WashTrac::WashQueue::Dequeue() ==
                    WashTrac::Result::OK)
                {
                    ChangeState(SystemState::StartingWash);
                }
            }

            break;
        }

        case SystemState::StartingWash:
        {
            const WashTrac::Result result =
                WashTrac::Relays::Start(
                    WashTrac::Relays::RelayId::WashStart);

            if (result == WashTrac::Result::OK)
            {
                ChangeState(SystemState::WaitingForBusy);
            }
            else
            {
                ESP_LOGE(
                    LOG_TAG,
                    "Failed to start Wash Start relay. Result code: %u",
                    static_cast<unsigned int>(result));

                /*
                 * A relay scheduler failure means the controller cannot
                 * safely issue the requested wash start.
                 */
                WashTrac::WashQueue::Clear();
                ChangeState(SystemState::Fault);
            }

            break;
        }

        case SystemState::WaitingForBusy:
        {
            /*
             * The relay pulse alone never counts as a successful wash.
             * Only the real Wash Busy input confirms that the wash started.
             */
            if (WashTrac::Inputs::IsWashBusy())
            {
                g_retryCount = 0U;
                ChangeState(SystemState::WashRunning);
            }

            break;
        }

        case SystemState::WashRunning:
        case SystemState::ReleaseDelay:
        case SystemState::QueueDelay:
        case SystemState::RetryDelay:
        case SystemState::Fault:
        case SystemState::EStop:
        default:
            break;
    }
}

SystemState GetState()
{
    return g_state;
}

bool IsBusy()
{
    switch (g_state)
    {
        case SystemState::StartingWash:
        case SystemState::WaitingForBusy:
        case SystemState::WashRunning:
        case SystemState::ReleaseDelay:
        case SystemState::QueueDelay:
        case SystemState::RetryDelay:
            return true;

        case SystemState::Idle:
        case SystemState::Fault:
        case SystemState::EStop:
        default:
            return false;
    }
}

bool IsFaulted()
{
    return g_state == SystemState::Fault;
}

void ClearFault()
{
    if (g_state == SystemState::Fault)
    {
        ESP_LOGI(LOG_TAG, "Fault cleared.");

        g_retryCount = 0U;
        ChangeState(SystemState::Idle);
    }
}

uint8_t GetRetryCount()
{
    return g_retryCount;
}

void EnterEStop()
{
    /*
     * Clear pending washes and prevent retries. Do not cancel Relay 1,
     * because an already-fired wash-start pulse must finish normally.
     */
    ESP_LOGW(LOG_TAG, "E-Stop active. Clearing pending wash queue.");

    WashTrac::WashQueue::Clear();

    g_retryCount = 0U;
    ChangeState(SystemState::EStop);
}

} // namespace WashTrac::StateMachine