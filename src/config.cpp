/******************************************************************************
 *
 *  Project:
 *      WashTrac Core
 *
 *  Module:
 *      Configuration Manager
 *
 *  File:
 *      config.cpp
 *
 *  Description:
 *      Production Configuration Manager implementation providing:
 *        - NVS initialization and recovery
 *        - Persistent configuration storage
 *        - Configuration validation
 *        - CRC32 integrity checking
 *        - Factory-default restoration
 *        - Whole-second relay timing configuration
 *
 ******************************************************************************/

#include "config.h"

#include "esp_crc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

#include <array>
#include <cstring>

namespace
{

constexpr const char* LOG_TAG = "ConfigManager";

constexpr const char* NVS_NAMESPACE = "washtrac";
constexpr const char* NVS_CONFIG_KEY = "core_config";

WashTrac::CoreConfig g_config{};
bool g_initialized = false;

template <std::size_t Size>
void CopyName(std::array<char, Size>& destination, const char* source)
{
    destination.fill('\0');

    if (source == nullptr)
        return;

    std::strncpy(destination.data(), source, Size - 1U);
    destination[Size - 1U] = '\0';
}

template <std::size_t Size>
bool IsNameTerminated(const std::array<char, Size>& name)
{
    return name[Size - 1U] == '\0';
}

} // namespace

namespace WashTrac
{

Result ConfigurationManager::Initialize()
{
    if (g_initialized)
        return Result::OK;

    esp_err_t err = nvs_flash_init();

    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_LOGW(LOG_TAG, "NVS requires reinitialization.");

        err = nvs_flash_erase();

        if (err != ESP_OK)
        {
            ESP_LOGE(
                LOG_TAG,
                "Failed to erase NVS: %s",
                esp_err_to_name(err));

            return Result::STORAGE_FAILURE;
        }

        err = nvs_flash_init();
    }

    if (err != ESP_OK)
    {
        ESP_LOGE(
            LOG_TAG,
            "Failed to initialize NVS: %s",
            esp_err_to_name(err));

        return Result::STORAGE_FAILURE;
    }

    g_initialized = true;

    const Result loadResult = Load();

    if (loadResult != Result::OK)
    {
        ESP_LOGW(
            LOG_TAG,
            "No valid stored configuration. Loading factory defaults.");

        LoadFactoryDefaults();

        const Result saveResult = Save();

        if (saveResult != Result::OK)
        {
            g_initialized = false;
            return saveResult;
        }
    }

    ESP_LOGI(LOG_TAG, "Configuration Manager initialized.");

    return Result::OK;
}

Result ConfigurationManager::Load()
{
    if (!g_initialized)
        return Result::NOT_INITIALIZED;

    nvs_handle_t handle = 0;

    esp_err_t err = nvs_open(
        NVS_NAMESPACE,
        NVS_READONLY,
        &handle);

    if (err != ESP_OK)
    {
        ESP_LOGW(
            LOG_TAG,
            "Unable to open configuration storage: %s",
            esp_err_to_name(err));

        return Result::STORAGE_FAILURE;
    }

    CoreConfig storedConfig{};
    std::size_t storedSize = sizeof(storedConfig);

    err = nvs_get_blob(
        handle,
        NVS_CONFIG_KEY,
        &storedConfig,
        &storedSize);

    nvs_close(handle);

    if (err != ESP_OK)
    {
        ESP_LOGW(
            LOG_TAG,
            "Unable to read stored configuration: %s",
            esp_err_to_name(err));

        return Result::STORAGE_FAILURE;
    }

    if (storedSize != sizeof(CoreConfig))
    {
        ESP_LOGW(
            LOG_TAG,
            "Stored configuration size is invalid.");

        return Result::STORAGE_FAILURE;
    }

    if (!Validate(storedConfig))
    {
        ESP_LOGW(
            LOG_TAG,
            "Stored configuration failed validation.");

        return Result::STORAGE_FAILURE;
    }

    const uint32_t storedCrc = storedConfig.header.crc32;
    const uint32_t calculatedCrc = CalculateCrc(storedConfig);

    if (storedCrc != calculatedCrc)
    {
        ESP_LOGW(
            LOG_TAG,
            "Stored configuration CRC is invalid.");

        return Result::STORAGE_FAILURE;
    }

    g_config = storedConfig;

    ESP_LOGI(LOG_TAG, "Configuration loaded from NVS.");

    return Result::OK;
}

Result ConfigurationManager::Save()
{
    if (!g_initialized)
        return Result::NOT_INITIALIZED;

    g_config.header.magic = CONFIG_MAGIC;
    g_config.header.configVersion = CONFIG_VERSION;
    g_config.header.structureSize =
        static_cast<uint16_t>(sizeof(CoreConfig));
    g_config.header.crc32 = 0U;

    if (!Validate(g_config))
    {
        ESP_LOGE(
            LOG_TAG,
            "Refusing to save invalid configuration.");

        return Result::INVALID_PARAMETER;
    }

    g_config.header.crc32 = CalculateCrc(g_config);

    nvs_handle_t handle = 0;

    esp_err_t err = nvs_open(
        NVS_NAMESPACE,
        NVS_READWRITE,
        &handle);

    if (err != ESP_OK)
    {
        ESP_LOGE(
            LOG_TAG,
            "Unable to open configuration storage: %s",
            esp_err_to_name(err));

        return Result::STORAGE_FAILURE;
    }

    err = nvs_set_blob(
        handle,
        NVS_CONFIG_KEY,
        &g_config,
        sizeof(g_config));

    if (err == ESP_OK)
        err = nvs_commit(handle);

    nvs_close(handle);

    if (err != ESP_OK)
    {
        ESP_LOGE(
            LOG_TAG,
            "Unable to save configuration: %s",
            esp_err_to_name(err));

        return Result::STORAGE_FAILURE;
    }

    ESP_LOGI(LOG_TAG, "Configuration saved to NVS.");

    return Result::OK;
}

Result ConfigurationManager::ResetFactory()
{
    if (!g_initialized)
        return Result::NOT_INITIALIZED;

    LoadFactoryDefaults();

    const Result result = Save();

    if (result == Result::OK)
        ESP_LOGI(LOG_TAG, "Factory configuration restored.");

    return result;
}

const CoreConfig& ConfigurationManager::Get()
{
    return g_config;
}

Result ConfigurationManager::SetRelay(
    uint8_t relayNumber,
    bool enabled,
    const char* name,
    uint16_t onDelaySeconds,
    uint16_t durationSeconds,
    uint16_t offDelaySeconds)
{
    if (!g_initialized)
        return Result::NOT_INITIALIZED;

    if (relayNumber < 1U || relayNumber > RELAY_COUNT)
        return Result::INVALID_PARAMETER;

    if (onDelaySeconds > MAX_RELAY_DELAY_SECONDS)
        return Result::INVALID_PARAMETER;

    if (durationSeconds < MIN_RELAY_DURATION_SECONDS ||
        durationSeconds > MAX_RELAY_DURATION_SECONDS)
    {
        return Result::INVALID_PARAMETER;
    }

    if (offDelaySeconds > MAX_RELAY_DELAY_SECONDS)
        return Result::INVALID_PARAMETER;

    /*
     * Relay 1 is the Wash Start SSR and must always remain available.
     */
    if (relayNumber == 1U && !enabled)
        return Result::INVALID_PARAMETER;

    RelayConfig& relay = g_config.relays[relayNumber - 1U];

    relay.enabled = enabled;
    relay.onDelaySeconds = onDelaySeconds;
    relay.durationSeconds = durationSeconds;
    relay.offDelaySeconds = offDelaySeconds;

    CopyName(relay.name, name);

    return Result::OK;
}

Result ConfigurationManager::SetInput(
    uint8_t inputNumber,
    bool enabled,
    bool inverted,
    const char* name)
{
    if (!g_initialized)
        return Result::NOT_INITIALIZED;

    if (inputNumber < 1U || inputNumber > INPUT_COUNT)
        return Result::INVALID_PARAMETER;

    InputConfig& input =
        g_config.inputs[inputNumber - 1U];

    /*
     * Input 1 is permanently assigned to Wash Busy.
     */
    if (inputNumber == 1U)
    {
        input.enabled = true;
        input.inverted = false;
        CopyName(input.name, "Wash Busy");
    }
    /*
     * Input 2 is permanently assigned to the E-Stop circuit.
     * Default logic is inverted:
     *     Voltage present = healthy
     *     Voltage absent  = E-Stop active
     */
    else if (inputNumber == 2U)
    {
        input.enabled = true;
        input.inverted = inverted;
        CopyName(input.name, "E-Stop");
    }
    else
    {
        input.enabled = enabled;
        input.inverted = inverted;
        CopyName(input.name, name);
    }

    return Result::OK;
}

Result ConfigurationManager::SetWashBusyReleaseDelay(
    uint16_t releaseDelaySeconds)
{
    if (!g_initialized)
        return Result::NOT_INITIALIZED;

    if (releaseDelaySeconds == 0U ||
        releaseDelaySeconds > MAX_RELAY_DELAY_SECONDS)
    {
        return Result::INVALID_PARAMETER;
    }

    g_config.washBusyReleaseDelaySeconds =
        releaseDelaySeconds;

    return Result::OK;
}

Result ConfigurationManager::SetInterWashDelay(
    uint16_t delaySeconds)
{
    if (!g_initialized)
        return Result::NOT_INITIALIZED;

    if (delaySeconds == 0U ||
        delaySeconds > MAX_RELAY_DELAY_SECONDS)
    {
        return Result::INVALID_PARAMETER;
    }

    g_config.interWashDelaySeconds = delaySeconds;

    return Result::OK;
}

bool ConfigurationManager::IsInitialized()
{
    return g_initialized;
}

void ConfigurationManager::LoadFactoryDefaults()
{
    g_config = {};

    g_config.header.magic = CONFIG_MAGIC;
    g_config.header.configVersion = CONFIG_VERSION;
    g_config.header.structureSize =
        static_cast<uint16_t>(sizeof(CoreConfig));
    g_config.header.crc32 = 0U;

    for (std::size_t i = 0U; i < RELAY_COUNT; ++i)
    {
        RelayConfig& relay = g_config.relays[i];

        relay.enabled = false;
        relay.onDelaySeconds =
            DEFAULT_RELAY_ON_DELAY_SECONDS;
        relay.durationSeconds =
            DEFAULT_RELAY_DURATION_SECONDS;
        relay.offDelaySeconds =
            DEFAULT_RELAY_OFF_DELAY_SECONDS;
    }

    /*
     * Relay 1 controls the active-high Wash Start SSR.
     * GPIO Manager remains responsible for initializing the physical output LOW.
     */
    g_config.relays[0].enabled = true;

    CopyName(g_config.relays[0].name, "Wash Start");
    CopyName(g_config.relays[1].name, "Relay 2");
    CopyName(g_config.relays[2].name, "Relay 3");
    CopyName(g_config.relays[3].name, "Relay 4");
    CopyName(g_config.relays[4].name, "Relay 5");
    CopyName(g_config.relays[5].name, "Relay 6");

    for (std::size_t i = 0U; i < INPUT_COUNT; ++i)
{
    g_config.inputs[i].enabled = false;
    g_config.inputs[i].inverted = false;
}

    g_config.inputs[0].enabled = true;
    g_config.inputs[0].inverted = false;

    g_config.inputs[1].enabled = true;
    g_config.inputs[1].inverted = true;

    CopyName(g_config.inputs[0].name, "Wash Busy");
    CopyName(g_config.inputs[1].name, "E-Stop");
    CopyName(g_config.inputs[2].name, "Input 3");
    CopyName(g_config.inputs[3].name, "Input 4");
    CopyName(g_config.inputs[4].name, "Input 5");
    CopyName(g_config.inputs[5].name, "Input 6");

    g_config.washBusyReleaseDelaySeconds =
        DEFAULT_WASH_BUSY_RELEASE_DELAY_SECONDS;
    
    g_config.interWashDelaySeconds =
        DEFAULT_INTER_WASH_DELAY_SECONDS;

    g_config.washStartRetryDelaySeconds =
        DEFAULT_WASH_START_RETRY_DELAY_SECONDS;

    g_config.washStartMaxAttempts =
        DEFAULT_WASH_START_MAX_ATTEMPTS;

    g_config.header.crc32 = CalculateCrc(g_config);
}

bool ConfigurationManager::Validate(const CoreConfig& config)
{
    if (config.header.magic != CONFIG_MAGIC)
        return false;

    if (config.header.configVersion != CONFIG_VERSION)
        return false;

    if (config.header.structureSize != sizeof(CoreConfig))
        return false;

    if (!config.relays[0].enabled)
        return false;

    for (const RelayConfig& relay : config.relays)
    {
        if (!IsNameTerminated(relay.name))
            return false;

        if (relay.onDelaySeconds > MAX_RELAY_DELAY_SECONDS)
            return false;

        if (relay.durationSeconds < MIN_RELAY_DURATION_SECONDS ||
            relay.durationSeconds > MAX_RELAY_DURATION_SECONDS)
        {
            return false;
        }

        if (relay.offDelaySeconds > MAX_RELAY_DELAY_SECONDS)
            return false;
    }

    if (!config.inputs[0].enabled)
        return false;
    if (!config.inputs[1].enabled)
        return false;
    
    for (const InputConfig& input : config.inputs)
    {
        if (!IsNameTerminated(input.name))
            return false;
    }

    if (config.washBusyReleaseDelaySeconds == 0U ||
    config.washBusyReleaseDelaySeconds >
        MAX_RELAY_DELAY_SECONDS)
    {
            return false;
    }

    if (config.interWashDelaySeconds == 0U ||
    config.interWashDelaySeconds >
        MAX_RELAY_DELAY_SECONDS)
    {
            return false;
    }
    if (config.washStartRetryDelaySeconds == 0U ||
    config.washStartRetryDelaySeconds >
        MAX_RELAY_DELAY_SECONDS)
    {
            return false;
    }

    if (config.washStartMaxAttempts == 0U)
    {
            return false;
    }
    return true;
}

uint32_t ConfigurationManager::CalculateCrc(
    const CoreConfig& config)
{
    CoreConfig crcConfig = config;
    crcConfig.header.crc32 = 0U;

    return esp_crc32_le(
        0U,
        reinterpret_cast<const uint8_t*>(&crcConfig),
        sizeof(crcConfig));
}

} // namespace WashTrac