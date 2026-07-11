/******************************************************************************
 *
 *  Project:
 *      WashTrac Core
 *
 *  Module:
 *      System Health Manager
 *
 *  File:
 *      system_health.h
 *
 *  Description:
 *      Defines the public interface for runtime health monitoring.
 *
 *      The subsystem provides:
 *          - System uptime
 *          - Current and minimum free heap
 *          - Largest available heap block
 *          - Main task stack high-water mark
 *          - Runtime-loop execution timing
 *          - Runtime-loop deadline overrun counting
 *
 *  Copyright:
 *      © 2026 WashTrac
 *
 ******************************************************************************/

#pragma once

#include "system.h"

#include <cstdint>

namespace WashTrac::SystemHealth
{

struct Snapshot
{
    uint64_t uptimeMilliseconds;

    uint32_t freeHeapBytes;
    uint32_t minimumFreeHeapBytes;
    uint32_t largestFreeHeapBlockBytes;

    uint32_t mainTaskStackHighWaterBytes;

    uint32_t lastLoopDurationMicroseconds;
    uint32_t maximumLoopDurationMicroseconds;
    uint32_t loopDeadlineOverrunCount;
};

Result Initialize();

void Update();

void RecordLoopDuration(
    uint32_t durationMicroseconds);

const Snapshot& GetSnapshot();

bool IsInitialized();

} // namespace WashTrac::SystemHealth
