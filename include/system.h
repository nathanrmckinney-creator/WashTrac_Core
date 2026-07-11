/******************************************************************************
 *
 *  Project:
 *      WashTrac Core
 *
 *  File:
 *      system.h
 *
 *  Description:
 *      Global system definitions used throughout the WashTrac Core firmware.
 *      Every source file in the project includes this header.
 *
 *  Copyright:
 *      © 2026 WashTrac
 *
 ******************************************************************************/

#pragma once

#include <stdint.h>

namespace WashTrac
{

//----------------------------------------------------------
// Firmware Information
//----------------------------------------------------------

constexpr const char* PROJECT_NAME      = "WashTrac Core";
constexpr const char* FIRMWARE_VERSION  = "1.0.0-alpha";
constexpr uint16_t    CONFIG_VERSION    = 1;
constexpr uint16_t    HARDWARE_REVISION = 1;

//----------------------------------------------------------
// Timing
//----------------------------------------------------------

constexpr uint32_t SYSTEM_TICK_MS = 10;

//----------------------------------------------------------
// Queue
//----------------------------------------------------------

constexpr uint8_t MAX_QUEUE_DEPTH = 2;

//----------------------------------------------------------
// Relay Count
//----------------------------------------------------------

constexpr uint8_t RELAY_COUNT = 6;

//----------------------------------------------------------
// Input Count
//----------------------------------------------------------

constexpr uint8_t INPUT_COUNT = 6;

//----------------------------------------------------------
// Result Codes
//----------------------------------------------------------

enum class Result
{
    OK,
    ERROR,
    INVALID_PARAMETER,
    TIMEOUT,
    STORAGE_FAILURE,
    CRC_FAILURE,
    NOT_INITIALIZED,
    UNSUPPORTED
};

//----------------------------------------------------------
// Wash State
//----------------------------------------------------------

enum class WashState
{
    IDLE,
    WAIT_FOR_BUSY,
    BUSY
};

//----------------------------------------------------------
// Request Source
//----------------------------------------------------------

enum class RequestSource
{
    KEYPAD,
    DASHBOARD,
    API,
    MEMBERSHIP,
    MAINTENANCE
};

} // namespace WashTrac