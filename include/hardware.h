/******************************************************************************
 *
 *  Project:
 *      WashTrac Core
 *
 *  Module:
 *      Hardware Definition
 *
 *  File:
 *      hardware.h
 *
 *  Description:
 *      Central hardware definition for the WashTrac controller.
 *      Every GPIO assignment exists here and nowhere else.
 *
 ******************************************************************************/

#pragma once

#include <cstdint>

#include "driver/gpio.h"

namespace WashTrac::Hardware
{

//----------------------------------------------------------
// Relay Outputs
//----------------------------------------------------------

constexpr gpio_num_t RELAY_1 = GPIO_NUM_8;
constexpr gpio_num_t RELAY_2 = GPIO_NUM_17;
constexpr gpio_num_t RELAY_3 = GPIO_NUM_16;
constexpr gpio_num_t RELAY_4 = GPIO_NUM_15;
constexpr gpio_num_t RELAY_5 = GPIO_NUM_7;
constexpr gpio_num_t RELAY_6 = GPIO_NUM_6;

//----------------------------------------------------------
// Digital Inputs
//----------------------------------------------------------

constexpr gpio_num_t INPUT_1 = GPIO_NUM_40;  // Wash Busy
constexpr gpio_num_t INPUT_2 = GPIO_NUM_41;  // Spare
constexpr gpio_num_t INPUT_3 = GPIO_NUM_42;  // Spare
constexpr gpio_num_t INPUT_4 = GPIO_NUM_45;  // Spare
constexpr gpio_num_t INPUT_5 = GPIO_NUM_47;  // Spare
constexpr gpio_num_t INPUT_6 = GPIO_NUM_48;  // Spare

//----------------------------------------------------------
// Relay Logic
//----------------------------------------------------------

// Relay 1 (SSR)
// HIGH = Contact Closed
// LOW  = Contact Open

constexpr bool SSR_ACTIVE_LEVEL   = true;
constexpr bool SSR_INACTIVE_LEVEL = false;

// Mechanical Relays
// HIGH = Energized
// LOW  = Released

constexpr bool RELAY_ACTIVE_LEVEL   = true;
constexpr bool RELAY_INACTIVE_LEVEL = false;

//----------------------------------------------------------
// Input Logic
//----------------------------------------------------------

// Input 1 - Wash Busy
// HIGH = Wash Busy
// LOW  = Wash Complete / Not Busy

constexpr bool WASH_BUSY_ACTIVE_LEVEL = true;

//----------------------------------------------------------
// UART
//----------------------------------------------------------

constexpr uint32_t UART_BAUD_RATE = 115200;

} // namespace WashTrac::Hardware
