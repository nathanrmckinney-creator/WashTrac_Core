/******************************************************************************
 *
 *  Project:
 *      WashTrac Core
 *
 *  Module:
 *      Manufacturing Self-Test
 *
 *  File:
 *      manufacturing_self_test.cpp
 *
 *  Description:
 *      Implements the non-destructive WashTrac Core Manufacturing Self-Test.
 *
 *      The subsystem verifies configuration integrity and runtime subsystem
 *      readiness without energizing relays, modifying configuration, clearing
 *      queues, or changing State Machine operation.
 *
 *  Copyright:
 *      © 2026 WashTrac
 *
 ******************************************************************************/

#include "manufacturing_self_test.h"

#include "config.h"
#include "event_logger.h"
#include "input_manager.h"
#include "relay_scheduler.h"
#include "state_machine.h"
#include "system_health.h"
#include "wash_queue.h"

#include <cstddef>
#include <cstdint>

namespace
{

WashTrac::ManufacturingSelfTest::Report g_report{};
bool g_initialized = false;

using WashTrac::ManufacturingSelfTest::CheckId;
using WashTrac::ManufacturingSelfTest::CheckResult;
using WashTrac::ManufacturingSelfTest::CheckStatus;

constexpr std::size_t ToIndex(CheckId check)
{
    return static_cast<std::size_t>(check);
}

void ResetResults()
{
    for (std::size_t index = 0U;
         index < WashTrac::ManufacturingSelfTest::CHECK_COUNT;
         ++index)
    {
        g_report.checks[index].check =
            static_cast<CheckId>(index);
        g_report.checks[index].status =
            CheckStatus::NotRun;
        g_report.checks[index].detail = 0U;
    }
}

void SetResult(
    CheckId check,
    bool passed,
    uint32_t detail = 0U)
{
    CheckResult& result = g_report.checks[ToIndex(check)];

    result.check = check;
    result.status =
        passed ? CheckStatus::Pass : CheckStatus::Fail;
    result.detail = detail;

    if (!passed)
        g_report.passed = false;
}

bool IsRelayStateValid(
    WashTrac::Relays::RelayState state)
{
    using WashTrac::Relays::RelayState;

    switch (state)
    {
        case RelayState::Idle:
        case RelayState::WaitingOnDelay:
        case RelayState::Active:
        case RelayState::WaitingOffDelay:
            return true;

        default:
            return false;
    }
}

bool IsSystemStateValid(
    WashTrac::StateMachine::SystemState state)
{
    using WashTrac::StateMachine::SystemState;

    switch (state)
    {
        case SystemState::Idle:
        case SystemState::StartingWash:
        case SystemState::WaitingForBusy:
        case SystemState::WashRunning:
        case SystemState::ReleaseDelay:
        case SystemState::QueueDelay:
        case SystemState::RetryDelay:
        case SystemState::Fault:
        case SystemState::EStop:
            return true;

        default:
            return false;
    }
}

} // namespace

namespace WashTrac::ManufacturingSelfTest
{

Result Initialize()
{
    if (g_initialized)
        return Result::OK;

    g_report = {};
    g_report.passed = false;
    g_report.runCount = 0U;

    ResetResults();

    g_initialized = true;

    return Result::OK;
}

Result Run()
{
    if (!g_initialized)
        return Result::NOT_INITIALIZED;

    g_report.passed = true;
    ++g_report.runCount;

    ResetResults();

    SetResult(
        CheckId::ConfigurationIntegrity,
        ConfigurationManager::IsInitialized() &&
            ConfigurationManager::IsConfigurationValid());

    const bool eventLoggerInitialized =
        Events::IsInitialized();

    bool eventLoggerPassed = eventLoggerInitialized;
    uint32_t eventCount = 0U;

    if (eventLoggerInitialized)
    {
        const std::size_t count = Events::Count();
        eventCount = static_cast<uint32_t>(count);

        if (count > Events::MAX_EVENTS)
        {
            eventLoggerPassed = false;
        }
        else if (count > 0U)
        {
            Events::Event event{};

            if (!Events::Get(count - 1U, event))
                eventLoggerPassed = false;
        }
    }

    SetResult(
        CheckId::EventLogger,
        eventLoggerPassed,
        eventCount);

    const bool systemHealthInitialized =
        SystemHealth::IsInitialized();

    bool systemHealthPassed = systemHealthInitialized;
    uint32_t freeHeapBytes = 0U;

    if (systemHealthInitialized)
    {
        const SystemHealth::Snapshot& snapshot =
            SystemHealth::GetSnapshot();

        freeHeapBytes = snapshot.freeHeapBytes;

        if (snapshot.freeHeapBytes == 0U ||
            snapshot.minimumFreeHeapBytes == 0U ||
            snapshot.largestFreeHeapBlockBytes == 0U)
        {
            systemHealthPassed = false;
        }
    }

    SetResult(
        CheckId::SystemHealth,
        systemHealthPassed,
        freeHeapBytes);

    const bool inputsInitialized =
        Inputs::IsInitialized();

    uint32_t inputStateBitmap = 0U;

    if (inputsInitialized)
    {
        for (uint8_t inputNumber = 1U;
             inputNumber <= INPUT_COUNT;
             ++inputNumber)
        {
            if (Inputs::ReadInput(inputNumber))
            {
                inputStateBitmap |=
                    (1UL << (inputNumber - 1U));
            }
        }
    }

    SetResult(
        CheckId::Inputs,
        inputsInitialized,
        inputStateBitmap);

    const bool relaysInitialized =
        Relays::IsInitialized();

    bool relaySchedulerPassed = relaysInitialized;
    uint32_t runningRelayBitmap = 0U;

    if (relaysInitialized)
    {
        for (uint8_t relayNumber = 1U;
             relayNumber <= RELAY_COUNT;
             ++relayNumber)
        {
            const Relays::RelayId relayId =
                static_cast<Relays::RelayId>(relayNumber);

            const Relays::RelayState relayState =
                Relays::GetState(relayId);

            if (!IsRelayStateValid(relayState))
                relaySchedulerPassed = false;

            if (Relays::IsRunning(relayId))
            {
                runningRelayBitmap |=
                    (1UL << (relayNumber - 1U));
            }
        }
    }

    SetResult(
        CheckId::RelayScheduler,
        relaySchedulerPassed,
        runningRelayBitmap);

    const bool queueInitialized =
        WashQueue::IsInitialized();

    bool queuePassed = queueInitialized;
    uint32_t queueCount = 0U;

    if (queueInitialized)
    {
        const std::size_t count = WashQueue::Count();
        queueCount = static_cast<uint32_t>(count);

        if (count > WashQueue::MAX_PENDING_WASHES)
            queuePassed = false;

        if (WashQueue::IsEmpty() != (count == 0U))
            queuePassed = false;

        if (WashQueue::IsFull() !=
            (count == WashQueue::MAX_PENDING_WASHES))
        {
            queuePassed = false;
        }
    }

    SetResult(
        CheckId::WashQueue,
        queuePassed,
        queueCount);

    const StateMachine::SystemState state =
        StateMachine::GetState();

    SetResult(
        CheckId::StateMachine,
        IsSystemStateValid(state),
        static_cast<uint32_t>(state));

    return Result::OK;
}

const Report& GetReport()
{
    return g_report;
}

std::size_t GetCheckCount()
{
    return CHECK_COUNT;
}

bool GetCheck(
    std::size_t index,
    CheckResult& result)
{
    if (index >= CHECK_COUNT)
        return false;

    result = g_report.checks[index];
    return true;
}

bool IsInitialized()
{
    return g_initialized;
}

const char* GetCheckName(CheckId check)
{
    switch (check)
    {
        case CheckId::ConfigurationIntegrity:
            return "Configuration Integrity";

        case CheckId::EventLogger:
            return "Event Logger";

        case CheckId::SystemHealth:
            return "System Health";

        case CheckId::Inputs:
            return "Input Visibility";

        case CheckId::RelayScheduler:
            return "Relay Scheduler";

        case CheckId::WashQueue:
            return "Wash Queue";

        case CheckId::StateMachine:
            return "State Machine";

        default:
            return "Unknown";
    }
}

const char* GetStatusName(CheckStatus status)
{
    switch (status)
    {
        case CheckStatus::NotRun:
            return "NOT RUN";

        case CheckStatus::Pass:
            return "PASS";

        case CheckStatus::Fail:
            return "FAIL";

        default:
            return "UNKNOWN";
    }
}

} // namespace WashTrac::ManufacturingSelfTest