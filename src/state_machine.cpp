#include "state_machine.h"

#include "config.h"
#include "event_logger.h"
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

/*
 * Number of retry attempts already performed for the current wash.
 *
 * The initial relay activation is not a retry.
 *
 * Examples:
 *     Initial attempt active: g_retryCount = 0
 *     First retry active:     g_retryCount = 1
 *     Second retry active:    g_retryCount = 2
 */
uint8_t g_retryCount = 0U;

/*
 * Total number of Wash Start relay attempts made for the current wash,
 * including the initial attempt.
 */
uint8_t g_attemptCount = 0U;

/*
 * Tick count recorded whenever the state changes.
 * All configured timing remains in whole seconds.
 */
TickType_t g_stateEntryTick = 0U;

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

    const TickType_t requiredTicks =
        pdMS_TO_TICKS(
            static_cast<uint32_t>(seconds) * 1000U);

    return elapsedTicks >= requiredTicks;
}

void ResetCurrentWashAttempt()
{
    g_retryCount = 0U;
    g_attemptCount = 0U;
}

void EnterFault(const char* reason)
{
    WashTrac::WashQueue::Clear();

    ESP_LOGE(
        LOG_TAG,
        "Wash fault: %s. Attempts made: %u.",
        reason,
        static_cast<unsigned>(g_attemptCount));

    ChangeState(SystemState::Fault);
}

} // namespace

void Initialize()
{
    ResetCurrentWashAttempt();

    ChangeState(SystemState::Idle);

    ESP_LOGI(LOG_TAG, "State Machine initialized.");
}

void Update()
{
    /*
     * E-Stop has priority over every normal state.
     *
     * A Wash Start relay pulse that has already begun is intentionally
     * allowed to finish. The Relay Scheduler remains responsible for
     * completing that pulse.
     */
    if (WashTrac::Inputs::IsEStopActive())
    {
        if (g_state != SystemState::EStop)
        {
            EnterEStop();
        }
        else if (!WashTrac::WashQueue::IsEmpty())
        {
            /*
             * Requests received while E-Stop remains active are discarded.
             */
            WashTrac::WashQueue::Clear();
        }

        return;
    }

    /*
     * Recover automatically when the E-Stop circuit becomes healthy.
     * No previous wash attempt or queued request is resumed.
     */
    if (g_state == SystemState::EStop)
    {
        WashTrac::Events::Log(
            WashTrac::Events::EventCode::EStopCleared);

        ResetCurrentWashAttempt();
        ChangeState(SystemState::Idle);
    }

    switch (g_state)
    {
        case SystemState::Idle:
        {
            ResetCurrentWashAttempt();

            if (!WashTrac::WashQueue::IsEmpty())
            {
                const WashTrac::Result dequeueResult =
                    WashTrac::WashQueue::Dequeue();

                if (dequeueResult == WashTrac::Result::OK)
                {
                    ChangeState(SystemState::StartingWash);
                }
                else
                {
                    EnterFault(
                        "Unable to remove pending wash from queue");
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
                ++g_attemptCount;

                if (g_attemptCount > 1U)
                {
                    g_retryCount =
                        static_cast<uint8_t>(
                            g_attemptCount - 1U);
                }

                if (g_retryCount > 0U)
                {
                    WashTrac::Events::Log(
                        WashTrac::Events::EventCode::RetryStarted,
                        static_cast<uint32_t>(g_retryCount));
                }

                WashTrac::Events::Log(
                    WashTrac::Events::EventCode::WashStartAttempt,
                    static_cast<uint32_t>(g_attemptCount));

                ESP_LOGI(
                    LOG_TAG,
                    "Wash Start attempt %u fired.",
                    static_cast<unsigned>(g_attemptCount));

                ChangeState(SystemState::WaitingForBusy);
            }
            else
            {
                ESP_LOGE(
                    LOG_TAG,
                    "Failed to start Wash Start relay. Result code: %u",
                    static_cast<unsigned>(result));

                EnterFault(
                    "Wash Start relay scheduler failure");
            }

            break;
        }

        case SystemState::WaitingForBusy:
        {
            /*
             * A relay pulse never establishes a wash.
             * Only the real Wash Busy input can confirm startup.
             */
            if (WashTrac::Inputs::IsWashBusy())
            {
                WashTrac::Events::Log(
                    WashTrac::Events::EventCode::WashStarted,
                    static_cast<uint32_t>(g_attemptCount));

                ESP_LOGI(
                    LOG_TAG,
                    "Wash Busy confirmed on attempt %u.",
                    static_cast<unsigned>(g_attemptCount));

                ChangeState(SystemState::WashRunning);
                break;
            }

            /*
             * Wait for the already-fired relay sequence to finish before
             * declaring this attempt unsuccessful. This does not interrupt
             * or shorten the configured relay pulse.
             */
            if (WashTrac::Relays::IsRunning(
                    WashTrac::Relays::RelayId::WashStart))
            {
                break;
            }

            const uint8_t maximumAttempts =
                WashTrac::ConfigurationManager::Get()
                    .washStartMaxAttempts;

            if (g_attemptCount >= maximumAttempts)
            {
                EnterFault(
                    "Wash Busy did not activate");
                break;
            }

            ESP_LOGW(
                LOG_TAG,
                "Wash Busy not detected after attempt %u. "
                "Beginning retry delay.",
                static_cast<unsigned>(g_attemptCount));

            ChangeState(SystemState::RetryDelay);

            break;
        }

        case SystemState::RetryDelay:
        {
            /*
             * E-Stop is checked before the switch and therefore prevents
             * all retry activity.
             */
            const uint16_t retryDelaySeconds =
                WashTrac::ConfigurationManager::Get()
                    .washStartRetryDelaySeconds;

            if (StateTimeElapsed(retryDelaySeconds))
            {
                ChangeState(SystemState::StartingWash);
            }

            break;
        }

        case SystemState::WashRunning:
        {
            /*
             * Read the physical Input 1 state directly so the state machine
             * can enter ReleaseDelay as soon as Wash Busy drops.
             *
             * Inputs::IsWashBusy() remains true during its configured
             * continuous release-confirmation period.
             */
            if (!WashTrac::Inputs::ReadInput(1U))
            {
                ChangeState(SystemState::ReleaseDelay);
            }

            break;
        }

        case SystemState::ReleaseDelay:
        {
            /*
             * If the real Wash Busy input returns at any point during the
             * release-confirmation period, the release was not continuous.
             */
            if (WashTrac::Inputs::ReadInput(1U))
            {
                ChangeState(SystemState::WashRunning);
                break;
            }

            /*
             * The Input Manager clears IsWashBusy() only after Input 1 has
             * remained continuously inactive for the complete configured
             * release delay.
             */
            if (!WashTrac::Inputs::IsWashBusy())
            {
                WashTrac::Events::Log(
                    WashTrac::Events::EventCode::WashCompleted,
                    static_cast<uint32_t>(g_attemptCount));

                ESP_LOGI(
                    LOG_TAG,
                    "Wash completion confirmed.");

                ResetCurrentWashAttempt();

                if (WashTrac::WashQueue::IsEmpty())
                {
                    ChangeState(SystemState::Idle);
                }
                else
                {
                    ChangeState(SystemState::QueueDelay);
                }
            }

            break;
        }

        case SystemState::QueueDelay:
        {
            /*
             * A pending wash may be removed externally while this delay is
             * active. In that case there is nothing left to start.
             */
            if (WashTrac::WashQueue::IsEmpty())
            {
                ChangeState(SystemState::Idle);
                break;
            }

            const uint16_t interWashDelaySeconds =
                WashTrac::ConfigurationManager::Get()
                    .interWashDelaySeconds;

            if (!StateTimeElapsed(interWashDelaySeconds))
            {
                break;
            }

            const WashTrac::Result dequeueResult =
                WashTrac::WashQueue::Dequeue();

            if (dequeueResult != WashTrac::Result::OK)
            {
                EnterFault(
                    "Unable to launch queued wash");
                break;
            }

            ResetCurrentWashAttempt();
            ChangeState(SystemState::StartingWash);

            break;
        }

        case SystemState::Fault:
        {
            /*
             * The fault has already been recorded and the pending queue has
             * already been cleared by EnterFault().
             *
             * Return automatically to Idle without retrying or restoring any
             * failed request.
             */
            ResetCurrentWashAttempt();
            ChangeState(SystemState::Idle);

            break;
        }

        case SystemState::EStop:
        {
            /*
             * E-Stop active handling occurs before the switch.
             * E-Stop recovery also occurs before the switch.
             */
            break;
        }

        default:
        {
            EnterFault(
                "Unknown state-machine state");
            break;
        }
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
    if (g_state != SystemState::Fault)
        return;

    ESP_LOGI(LOG_TAG, "Fault cleared.");

    WashTrac::WashQueue::Clear();

    ResetCurrentWashAttempt();
    ChangeState(SystemState::Idle);
}

uint8_t GetRetryCount()
{
    return g_retryCount;
}

void EnterEStop()
{
    /*
     * Clear all pending washes and prevent retries.
     *
     * Do not cancel Relay 1 because an already-fired wash-start pulse must
     * finish normally.
     */
    ESP_LOGW(
        LOG_TAG,
        "E-Stop active. Clearing pending wash queue.");

    WashTrac::WashQueue::Clear();

    WashTrac::Events::Log(
        WashTrac::Events::EventCode::EStopActive);

    ResetCurrentWashAttempt();
    ChangeState(SystemState::EStop);
}

} // namespace WashTrac::StateMachine