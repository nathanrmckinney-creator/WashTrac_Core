/******************************************************************************
 *
 *  Project:
 *      WashTrac Core
 *
 *  File:
 *      lte_manager.h
 *
 *  Description:
 *      LTE modem manager for the Air780E. Handles modem initialization,
 *      AT command scheduling, status tracking, diagnostics, and network
 *      registration. This module is the single interface between the
 *      firmware and the LTE modem.
 *
 ******************************************************************************/

#pragma once

#include <stdint.h>

namespace WashTrac::LTE
{

enum class ModemState : uint8_t
{
    Off = 0,
    Initializing,
    Ready,
    Registering,
    Online,
    Error
};

struct Status
{
    ModemState state;

    bool modemPresent;
    bool simPresent;
    bool registered;

    int8_t rssi;
    uint8_t ber;

    char carrier[32];
    char imei[24];
    char iccid[32];
    char ipAddress[24];

    uint32_t uptimeSeconds;
};

bool Initialize();

void Update();

const Status& GetStatus();

bool IsOnline();

bool IsRegistered();

bool QueueATCommand(const char* command);

void Reset();

}