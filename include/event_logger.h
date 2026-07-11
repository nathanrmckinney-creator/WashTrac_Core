/******************************************************************************
 *
 *  Project:
 *      WashTrac Core
 *
 *  Module:
 *      Event Logger
 *
 *  File:
 *      event_logger.h
 *
 *  Description:
 *      Defines the public interface for the WashTrac Core runtime event log.
 *
 *      Events are stored in a fixed-size in-memory circular buffer.
 *      No dynamic allocation is used.
 *
 *  Copyright:
 *      © 2026 WashTrac
 *
 ******************************************************************************/

#pragma once

#include "system.h"

#include "freertos/FreeRTOS.h"

#include <cstddef>
#include <cstdint>

namespace WashTrac::Events
{

constexpr std::size_t MAX_EVENTS = 512U;

enum class EventCode : uint16_t
{
    None = 0,

    Boot,

    ConfigurationLoaded,
    ConfigurationSaved,
    FactoryReset,

    WashQueued,
    WashDequeued,
    WashStartAttempt,
    WashStarted,
    WashCompleted,

    RetryStarted,

    FaultRaised,
    FaultCleared,

    EStopActive,
    EStopCleared,

    QueueCleared,

    SystemInitialized
};

struct Event
{
    EventCode code;
    TickType_t timestamp;
    uint32_t value;
};

Result Initialize();

void Log(
    EventCode code,
    uint32_t value = 0U);

std::size_t Count();

bool Get(
    std::size_t index,
    Event& event);

void Clear();

bool IsFull();

bool IsInitialized();

} // namespace WashTrac::Events