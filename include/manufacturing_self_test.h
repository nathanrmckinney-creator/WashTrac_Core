/******************************************************************************
 *
 *  Project:
 *      WashTrac Core
 *
 *  Module:
 *      Manufacturing Self-Test
 *
 *  File:
 *      manufacturing_self_test.h
 *
 *  Description:
 *      Public interface for the WashTrac Core Manufacturing Self-Test.
 *
 *      The Manufacturing Self-Test performs non-destructive verification of
 *      critical firmware subsystems before shipment or field service.
 *
 *      No relays are energized and no persistent configuration is modified.
 *
 *  Copyright:
 *      © 2026 WashTrac
 *
 ******************************************************************************/

#pragma once

#include "system.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace WashTrac::ManufacturingSelfTest
{

enum class CheckId : uint8_t
{
    ConfigurationIntegrity = 0,
    EventLogger,
    SystemHealth,
    Inputs,
    RelayScheduler,
    WashQueue,
    StateMachine
};

enum class CheckStatus : uint8_t
{
    NotRun = 0,
    Pass,
    Fail
};

struct CheckResult
{
    CheckId     check;
    CheckStatus status;
    uint32_t    detail;
};

constexpr std::size_t CHECK_COUNT = 7U;

struct Report
{
    std::array<CheckResult, CHECK_COUNT> checks;

    bool passed;
    uint32_t runCount;
};

Result Initialize();

Result Run();

const Report& GetReport();

bool IsInitialized();

const char* GetCheckName(CheckId check);

const char* GetStatusName(CheckStatus status);

} // namespace WashTrac::ManufacturingSelfTest