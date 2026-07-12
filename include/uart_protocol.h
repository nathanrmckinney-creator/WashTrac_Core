/******************************************************************************
 *
 *  Project:
 *      WashTrac Core
 *
 *  Module:
 *      UART Protocol Transport
 *
 *  File:
 *      uart_protocol.h
 *
 *  Description:
 *      Defines the fixed-memory, newline-delimited UART transport used for
 *      communication between WashTrac Core and the CM5 controller.
 *
 *  Copyright:
 *      © 2026 WashTrac
 *
 ******************************************************************************/

#pragma once

#include "system.h"

#include <cstddef>

namespace WashTrac::UartProtocol
{

constexpr std::size_t MAXIMUM_LINE_LENGTH = 512U;

Result Initialize();

void Update();

bool ReadLine(
    char* buffer,
    std::size_t bufferSize);

Result WriteLine(
    const char* text);

bool IsInitialized();

} // namespace WashTrac::UartProtocol