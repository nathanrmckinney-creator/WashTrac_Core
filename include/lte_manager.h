/******************************************************************************
 *
 *  Project:
 *      WashTrac Core
 *
 *  File:
 *      lte_manager.h
 *
 *  Description:
 *      Production LTE modem manager interface for the Air780E. Handles modem
 *      initialization, APN persistence, packet-data session control,
 *      automatic reconnect, network registration, identity retrieval,
 *      diagnostics, counters, and cached modem status.
 *
 *  Copyright:
 *      © 2026 WashTrac
 *
 ******************************************************************************/

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace WashTrac::LTE
{

constexpr size_t APN_MAX_LENGTH = 64U;

enum class ModemState : uint8_t
{
    Off = 0,
    Initializing,
    Ready,
    Registering,
    Connecting,
    Online,
    Disconnecting,
    Reconnecting,
    Restarting,
    Error
};

enum class RegistrationState : uint8_t
{
    Unknown = 0,
    NotRegistered,
    RegisteredHome,
    Searching,
    RegistrationDenied,
    RegisteredRoaming
};

enum class DataSessionState : uint8_t
{
    Inactive = 0,
    Configuring,
    Attaching,
    Activating,
    Active,
    Deactivating,
    Error
};

enum class DisconnectReason : uint8_t
{
    None = 0,
    RegistrationLost,
    PacketAttachLost,
    PDPContextLost,
    NoIPAddress,
    CommandTimeout,
    CommandRejected,
    ModemRestart,
    ManualDisconnect,
    InitializationFailure,
    Unknown
};

struct RegistrationDetails
{
    RegistrationState state;

    bool registered;
    bool roaming;

    uint16_t mcc;
    uint16_t mnc;

    uint32_t tac;
    uint32_t cellId;

    uint8_t accessTechnology;

    char carrier[32];
};

struct SignalDetails
{
    int8_t rssi;
    uint8_t ber;

    int16_t rsrp;
    int16_t rsrq;
    int16_t sinr;
};

struct Counters
{
    uint32_t modemRestartCount;
    uint32_t reconnectCount;
    uint32_t registrationFailureCount;
    uint32_t dataSessionFailureCount;
    uint32_t commandTimeoutCount;
    uint32_t commandErrorCount;
};

struct Diagnostics
{
    DisconnectReason lastDisconnectReason;

    int32_t lastCmeError;
    int32_t lastCmsError;

    uint32_t lastSuccessfulCommandAgeSeconds;
    uint32_t lastRegistrationAgeSeconds;
    uint32_t lastDataSessionAgeSeconds;

    char lastCommand[96];
    char lastError[96];
};

struct Configuration
{
    char apn[APN_MAX_LENGTH];

    bool automaticReconnectEnabled;

    uint32_t reconnectInitialDelaySeconds;
    uint32_t reconnectMaximumDelaySeconds;
};

struct Status
{
    ModemState state;
    DataSessionState dataSessionState;

    bool modemPresent;
    bool simPresent;
    bool packetAttached;
    bool dataSessionActive;

    RegistrationDetails registration;
    SignalDetails signal;
    Counters counters;
    Diagnostics diagnostics;
    Configuration configuration;

    char imei[24];
    char imsi[24];
    char iccid[32];
    char firmwareVersion[64];
    char ipAddress[48];

    uint32_t uptimeSeconds;
    uint32_t modemUptimeSeconds;
};

bool Initialize();

void Update();

const Status& GetStatus();

const Configuration& GetConfiguration();

bool IsOnline();

bool IsRegistered();

bool IsDataSessionActive();

bool QueueATCommand(const char* command);

bool SetAPN(const char* apn);

const char* GetAPN();

bool SaveConfiguration();

bool ReloadConfiguration();

bool RestoreDefaultConfiguration();

bool ConnectDataSession();

bool DisconnectDataSession();

bool SetAutomaticReconnectEnabled(bool enabled);

bool SetReconnectDelays(
    uint32_t initialDelaySeconds,
    uint32_t maximumDelaySeconds);

bool RestartModem();

void Reset();

} // namespace WashTrac::LTE