/******************************************************************************
 *
 *  Project:
 *      WashTrac Core
 *
 *  Module:
 *      Fault Manager
 *
 *  File:
 *      fault_manager.h
 *
 *  Description:
 *      Defines the public interface for tracking the controller's current
 *      and previously active fault conditions.
 *
 *      The Fault Manager maintains controller health state only. Complete
 *      historical event records are handled separately by the Event Logger.
 *
 *  Copyright:
 *      © 2026 WashTrac
 *
 ******************************************************************************/

#pragma once

#include "system.h"

#include "freertos/FreeRTOS.h"

#include <cstdint>

namespace WashTrac::Faults
{

enum class FaultCode : uint8_t
{
    None = 0,

    WashStartTimeout,

    RelaySchedulerFailure,

    QueueFailure,

    ConfigurationFailure,

    EStopActive,

    InvalidState,

    InternalError
};

struct FaultStatus
{
    FaultCode code;
    bool active;
    uint32_t occurrenceCount;
    TickType_t timestamp;
};

Result Initialize();

void Raise(FaultCode code);

void Clear();

bool IsActive();

FaultCode GetCode();

const FaultStatus& GetStatus();

const FaultStatus& GetPreviousStatus();

bool IsInitialized();

} // namespace WashTrac::Faults