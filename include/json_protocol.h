/******************************************************************************
 *
 *  Project:
 *      WashTrac Core
 *
 *  Module:
 *      JSON Protocol
 *
 *  File:
 *      json_protocol.h
 *
 *  Description:
 *      Defines the fixed-memory Version 1 JSON message format used between
 *      WashTrac Core and the CM5 controller.
 *
 *  Copyright:
 *      © 2026 WashTrac
 *
 ******************************************************************************/

#pragma once

#include "system.h"

#include <cstddef>
#include <cstdint>

namespace WashTrac::JsonProtocol
{

constexpr std::size_t REQUEST_ID_LENGTH = 40U;
constexpr std::size_t NAME_LENGTH = 32U;
constexpr std::size_t RESPONSE_LENGTH = 512U;

enum class Command : uint8_t
{
    Unknown = 0,
    Ping,
    Status,
    StartWash,
    QueueStatus,
    Diagnostics,
    GetConfig,
    SetRelay,
    SetInput,
    SetTiming,
    SaveConfig
};

struct Message
{
    Command command;

    char requestId[REQUEST_ID_LENGTH];
    char name[NAME_LENGTH];

    uint8_t relayNumber;
    uint8_t inputNumber;

    bool enabled;
    bool inverted;

    uint16_t onDelaySeconds;
    uint16_t durationSeconds;
    uint16_t offDelaySeconds;

    uint16_t washBusyReleaseDelaySeconds;
    uint16_t interWashDelaySeconds;

    bool hasRequestId;
    bool hasName;
    bool hasRelayNumber;
    bool hasInputNumber;
    bool hasEnabled;
    bool hasInverted;
    bool hasOnDelaySeconds;
    bool hasDurationSeconds;
    bool hasOffDelaySeconds;
    bool hasWashBusyReleaseDelaySeconds;
    bool hasInterWashDelaySeconds;
};

Result Parse(
    const char* json,
    Message& message);

Result WriteOk(
    const char* requestId,
    const char* operation,
    char* output,
    std::size_t outputSize);

Result WriteError(
    const char* requestId,
    const char* operation,
    const char* error,
    char* output,
    std::size_t outputSize);

const char* CommandToString(
    Command command);

} // namespace WashTrac::JsonProtocol