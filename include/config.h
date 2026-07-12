/******************************************************************************
 *
 *  Project:
 *      WashTrac Core
 *
 *  Module:
 *      Configuration Manager
 *
 *  File:
 *      config.h
 *
 *  Description:
 *      Defines the persistent WashTrac Core configuration and the public
 *      interface used to load, validate, save, and restore factory defaults.
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

namespace WashTrac
{

//------------------------------------------------------------------------------
// Configuration constants
//------------------------------------------------------------------------------

constexpr uint32_t CONFIG_MAGIC = 0x57545243U;  // ASCII: WTRC

constexpr std::size_t RELAY_NAME_LENGTH = 32U;
constexpr std::size_t INPUT_NAME_LENGTH = 32U;

constexpr uint16_t MIN_RELAY_DURATION_SECONDS = 1U;
constexpr uint16_t MAX_RELAY_DURATION_SECONDS = 600U;

constexpr uint16_t MAX_RELAY_DELAY_SECONDS = 600U;

constexpr uint16_t DEFAULT_RELAY_ON_DELAY_SECONDS = 0U;
constexpr uint16_t DEFAULT_RELAY_DURATION_SECONDS = 2U;
constexpr uint16_t DEFAULT_RELAY_OFF_DELAY_SECONDS = 0U;

constexpr uint16_t DEFAULT_WASH_BUSY_RELEASE_DELAY_SECONDS = 5U;
constexpr uint16_t DEFAULT_INTER_WASH_DELAY_SECONDS = 3U;

constexpr uint16_t DEFAULT_WASH_START_RETRY_DELAY_SECONDS = 3U;
constexpr uint8_t DEFAULT_WASH_START_MAX_ATTEMPTS = 3U;

//------------------------------------------------------------------------------
// Persistent configuration structures
//------------------------------------------------------------------------------

struct ConfigHeader
{
    uint32_t magic;
    uint16_t configVersion;
    uint16_t structureSize;
    uint32_t crc32;
};

struct RelayConfig
{
    bool enabled;
    std::array<char, RELAY_NAME_LENGTH> name;

    uint16_t onDelaySeconds;
    uint16_t durationSeconds;
    uint16_t offDelaySeconds;
};

struct InputConfig
{
    bool enabled;
    bool inverted;
    std::array<char, INPUT_NAME_LENGTH> name;
};

struct CoreConfig
{
    ConfigHeader header;

    std::array<RelayConfig, RELAY_COUNT> relays;
    std::array<InputConfig, INPUT_COUNT> inputs;

    uint16_t washBusyReleaseDelaySeconds;
    uint16_t interWashDelaySeconds;

    uint16_t washStartRetryDelaySeconds;
    uint8_t washStartMaxAttempts;
};

//------------------------------------------------------------------------------
// Configuration Manager
//------------------------------------------------------------------------------

class ConfigurationManager
{
public:
    ConfigurationManager() = delete;

    static Result Initialize();
    static Result Load();
    static Result Save();
    static Result ResetFactory();

    static const CoreConfig& Get();

    static Result SetRelay(
        uint8_t relayNumber,
        bool enabled,
        const char* name,
        uint16_t onDelaySeconds,
        uint16_t durationSeconds,
        uint16_t offDelaySeconds);

    static Result SetInput(
        uint8_t inputNumber,
        bool enabled,
        bool inverted,
        const char* name);

    static Result SetWashBusyReleaseDelay(
        uint16_t releaseDelaySeconds);
    
    static Result SetInterWashDelay(
        uint16_t delaySeconds);

    static bool IsConfigurationValid();

    static bool IsInitialized();

private:
    static void LoadFactoryDefaults();
    static bool Validate(const CoreConfig& config);
    static uint32_t CalculateCrc(const CoreConfig& config);
};

} // namespace WashTrac