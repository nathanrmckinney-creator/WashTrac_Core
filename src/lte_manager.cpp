/******************************************************************************
 *
 *  Project:
 *      WashTrac Core
 *
 *  File:
 *      lte_manager.cpp
 *
 *  Description:
 *      Production Air780E LTE modem manager implementation. Provides a
 *      non-blocking UART-driven AT command engine, persistent APN
 *      configuration, packet-data session activation, automatic reconnect,
 *      registration and signal diagnostics, modem identity collection,
 *      timeout/retry handling, counters, and cached modem status.
 *
 *      Hardware:
 *          UART1 TX -> GPIO46 (ESP32_TX)
 *          UART1 RX -> GPIO3  (ESP32_RX)
 *
 *  Copyright:
 *      © 2026 WashTrac
 *
 ******************************************************************************/

#include "lte_manager.h"

#include <ctype.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

namespace WashTrac::LTE
{
namespace
{

constexpr uart_port_t LTE_UART_PORT = UART_NUM_1;
constexpr int LTE_UART_TX_PIN = 46;
constexpr int LTE_UART_RX_PIN = 3;
constexpr int LTE_UART_BAUD_RATE = 115200;

constexpr size_t UART_RX_BUFFER_SIZE = 1024U;
constexpr size_t UART_TX_BUFFER_SIZE = 512U;

constexpr size_t COMMAND_QUEUE_DEPTH = 16U;
constexpr size_t COMMAND_MAX_LENGTH = 96U;
constexpr size_t RESPONSE_BUFFER_SIZE = 768U;
constexpr size_t LINE_BUFFER_SIZE = 192U;

constexpr uint32_t COMMAND_TIMEOUT_MS = 3000U;
constexpr uint32_t REGISTRATION_TIMEOUT_MS = 10000U;
constexpr uint32_t DATA_SESSION_TIMEOUT_MS = 30000U;
constexpr uint32_t MODEM_RESTART_TIMEOUT_MS = 20000U;

constexpr uint32_t INITIAL_RETRY_DELAY_MS = 1000U;
constexpr uint32_t ERROR_RETRY_DELAY_MS = 5000U;
constexpr uint32_t STATUS_REFRESH_INTERVAL_MS = 10000U;
constexpr uint32_t IDENTITY_REFRESH_INTERVAL_MS = 300000U;
constexpr uint32_t DIAGNOSTIC_REFRESH_INTERVAL_MS = 30000U;

constexpr uint32_t DEFAULT_RECONNECT_INITIAL_SECONDS = 5U;
constexpr uint32_t DEFAULT_RECONNECT_MAXIMUM_SECONDS = 300U;

constexpr uint8_t MAX_COMMAND_RETRIES = 2U;

constexpr const char* NVS_NAMESPACE = "lte";
constexpr const char* NVS_KEY_APN = "apn";
constexpr const char* NVS_KEY_AUTO_RECONNECT = "auto_recon";
constexpr const char* NVS_KEY_RECONNECT_INITIAL = "recon_init";
constexpr const char* NVS_KEY_RECONNECT_MAXIMUM = "recon_max";
constexpr const char* NVS_KEY_RESTART_COUNT = "restart_cnt";

constexpr const char* TAG = "WashTracLTE";

enum class InitStep : uint8_t
{
    None = 0,
    Attention,
    EchoOff,
    ExtendedErrors,
    SimStatus,
    Imei,
    Imsi,
    Iccid,
    Firmware,
    RegistrationUrc,
    Registration,
    PacketAttach,
    Signal,
    ExtendedSignal,
    Carrier,
    ConfigurePdp,
    ActivatePdp,
    IpAddress,
    Complete
};

enum class Operation : uint8_t
{
    None = 0,
    Initialization,
    StatusRefresh,
    IdentityRefresh,
    ConnectData,
    DisconnectData,
    RestartModem,
    ManualCommand
};

struct CommandEntry
{
    char text[COMMAND_MAX_LENGTH];
    Operation operation;
};

Status g_status{};
bool g_initialized = false;

CommandEntry g_commandQueue[COMMAND_QUEUE_DEPTH]{};
size_t g_queueHead = 0U;
size_t g_queueTail = 0U;
size_t g_queueCount = 0U;

bool g_commandActive = false;
char g_activeCommand[COMMAND_MAX_LENGTH]{};
Operation g_activeOperation = Operation::None;

char g_responseBuffer[RESPONSE_BUFFER_SIZE]{};
size_t g_responseLength = 0U;
char g_lineBuffer[LINE_BUFFER_SIZE]{};
size_t g_lineLength = 0U;

uint64_t g_commandStartedMs = 0U;
uint32_t g_activeTimeoutMs = COMMAND_TIMEOUT_MS;
uint8_t g_commandRetryCount = 0U;

InitStep g_initStep = InitStep::None;
uint64_t g_nextActionMs = 0U;
uint64_t g_lastStatusRefreshMs = 0U;
uint64_t g_lastIdentityRefreshMs = 0U;
uint64_t g_lastDiagnosticRefreshMs = 0U;
uint64_t g_lastSuccessfulCommandMs = 0U;
uint64_t g_lastRegistrationMs = 0U;
uint64_t g_lastDataSessionMs = 0U;
uint64_t g_modemStartedMs = 0U;

uint64_t g_reconnectDueMs = 0U;
uint32_t g_reconnectDelaySeconds = DEFAULT_RECONNECT_INITIAL_SECONDS;
bool g_manualDisconnect = false;
bool g_connectRequested = false;
bool g_disconnectRequested = false;
bool g_restartRequested = false;

uint64_t NowMs()
{
    return static_cast<uint64_t>(esp_timer_get_time() / 1000LL);
}

uint32_t AgeSeconds(uint64_t timestampMs)
{
    if (timestampMs == 0U)
    {
        return 0U;
    }

    const uint64_t now = NowMs();

    if (now < timestampMs)
    {
        return 0U;
    }

    const uint64_t seconds = (now - timestampMs) / 1000ULL;

    if (seconds > UINT32_MAX)
    {
        return UINT32_MAX;
    }

    return static_cast<uint32_t>(seconds);
}

void CopyText(char* destination, size_t destinationSize, const char* source)
{
    if ((destination == nullptr) || (destinationSize == 0U))
    {
        return;
    }

    if (source == nullptr)
    {
        destination[0] = '\0';
        return;
    }

    (void)snprintf(destination, destinationSize, "%s", source);
}

void SetLastError(const char* text)
{
    CopyText(
        g_status.diagnostics.lastError,
        sizeof(g_status.diagnostics.lastError),
        text);
}

bool IsPrintableApnCharacter(char value)
{
    return
        isalnum(static_cast<unsigned char>(value)) ||
        (value == '.') ||
        (value == '-') ||
        (value == '_');
}

bool IsValidAPN(const char* apn)
{
    if (apn == nullptr)
    {
        return false;
    }

    const size_t length = strnlen(apn, APN_MAX_LENGTH);

    if (length >= APN_MAX_LENGTH)
    {
        return false;
    }

    for (size_t index = 0U; index < length; ++index)
    {
        if (!IsPrintableApnCharacter(apn[index]))
        {
            return false;
        }
    }

    return true;
}

void SetDefaultConfiguration()
{
    memset(&g_status.configuration, 0, sizeof(g_status.configuration));

    g_status.configuration.apn[0] = '\0';
    g_status.configuration.automaticReconnectEnabled = true;
    g_status.configuration.reconnectInitialDelaySeconds =
        DEFAULT_RECONNECT_INITIAL_SECONDS;
    g_status.configuration.reconnectMaximumDelaySeconds =
        DEFAULT_RECONNECT_MAXIMUM_SECONDS;
}

void ResetStatus()
{
    Configuration configuration = g_status.configuration;
    Counters counters = g_status.counters;

    memset(&g_status, 0, sizeof(g_status));

    g_status.configuration = configuration;
    g_status.counters = counters;

    g_status.state = ModemState::Off;
    g_status.dataSessionState = DataSessionState::Inactive;

    g_status.signal.rssi = -1;
    g_status.signal.ber = 99U;
    g_status.signal.rsrp = INT16_MIN;
    g_status.signal.rsrq = INT16_MIN;
    g_status.signal.sinr = INT16_MIN;

    g_status.registration.state = RegistrationState::Unknown;
    CopyText(
        g_status.registration.carrier,
        sizeof(g_status.registration.carrier),
        "Unknown");

    g_status.diagnostics.lastDisconnectReason = DisconnectReason::None;
    g_status.diagnostics.lastCmeError = -1;
    g_status.diagnostics.lastCmsError = -1;
}

bool LoadConfigurationFromNVS()
{
    SetDefaultConfiguration();

    nvs_handle_t handle = 0;
    const esp_err_t openResult =
        nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);

    if (openResult == ESP_ERR_NVS_NOT_FOUND)
    {
        return true;
    }

    if (openResult != ESP_OK)
    {
        ESP_LOGW(
            TAG,
            "Unable to open LTE NVS namespace: %s",
            esp_err_to_name(openResult));
        return false;
    }

    size_t apnLength = sizeof(g_status.configuration.apn);
    esp_err_t result = nvs_get_str(
        handle,
        NVS_KEY_APN,
        g_status.configuration.apn,
        &apnLength);

    if ((result != ESP_OK) && (result != ESP_ERR_NVS_NOT_FOUND))
    {
        ESP_LOGW(TAG, "Unable to read APN: %s", esp_err_to_name(result));
    }

    uint8_t automaticReconnect =
        g_status.configuration.automaticReconnectEnabled ? 1U : 0U;

    result = nvs_get_u8(
        handle,
        NVS_KEY_AUTO_RECONNECT,
        &automaticReconnect);

    if (result == ESP_OK)
    {
        g_status.configuration.automaticReconnectEnabled =
            automaticReconnect != 0U;
    }

    uint32_t reconnectInitial =
        g_status.configuration.reconnectInitialDelaySeconds;

    result = nvs_get_u32(
        handle,
        NVS_KEY_RECONNECT_INITIAL,
        &reconnectInitial);

    if ((result == ESP_OK) && (reconnectInitial > 0U))
    {
        g_status.configuration.reconnectInitialDelaySeconds =
            reconnectInitial;
    }

    uint32_t reconnectMaximum =
        g_status.configuration.reconnectMaximumDelaySeconds;

    result = nvs_get_u32(
        handle,
        NVS_KEY_RECONNECT_MAXIMUM,
        &reconnectMaximum);

    if ((result == ESP_OK) &&
        (reconnectMaximum >=
         g_status.configuration.reconnectInitialDelaySeconds))
    {
        g_status.configuration.reconnectMaximumDelaySeconds =
            reconnectMaximum;
    }

    uint32_t restartCount = 0U;
    result = nvs_get_u32(handle, NVS_KEY_RESTART_COUNT, &restartCount);

    if (result == ESP_OK)
    {
        g_status.counters.modemRestartCount = restartCount;
    }

    nvs_close(handle);
    return true;
}

bool SaveConfigurationToNVS()
{
    nvs_handle_t handle = 0;
    esp_err_t result =
        nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);

    if (result != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "Unable to open LTE NVS namespace for writing: %s",
            esp_err_to_name(result));
        return false;
    }

    result = nvs_set_str(
        handle,
        NVS_KEY_APN,
        g_status.configuration.apn);

    if (result == ESP_OK)
    {
        result = nvs_set_u8(
            handle,
            NVS_KEY_AUTO_RECONNECT,
            g_status.configuration.automaticReconnectEnabled ? 1U : 0U);
    }

    if (result == ESP_OK)
    {
        result = nvs_set_u32(
            handle,
            NVS_KEY_RECONNECT_INITIAL,
            g_status.configuration.reconnectInitialDelaySeconds);
    }

    if (result == ESP_OK)
    {
        result = nvs_set_u32(
            handle,
            NVS_KEY_RECONNECT_MAXIMUM,
            g_status.configuration.reconnectMaximumDelaySeconds);
    }

    if (result == ESP_OK)
    {
        result = nvs_set_u32(
            handle,
            NVS_KEY_RESTART_COUNT,
            g_status.counters.modemRestartCount);
    }

    if (result == ESP_OK)
    {
        result = nvs_commit(handle);
    }

    nvs_close(handle);

    if (result != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "Unable to save LTE configuration: %s",
            esp_err_to_name(result));
        return false;
    }

    return true;
}

void ResetCommandEngine()
{
    memset(g_commandQueue, 0, sizeof(g_commandQueue));
    g_queueHead = 0U;
    g_queueTail = 0U;
    g_queueCount = 0U;

    g_commandActive = false;
    g_activeCommand[0] = '\0';
    g_activeOperation = Operation::None;

    g_responseBuffer[0] = '\0';
    g_responseLength = 0U;

    g_lineBuffer[0] = '\0';
    g_lineLength = 0U;

    g_commandStartedMs = 0U;
    g_activeTimeoutMs = COMMAND_TIMEOUT_MS;
    g_commandRetryCount = 0U;
}

bool IsQueueFull()
{
    return g_queueCount >= COMMAND_QUEUE_DEPTH;
}

bool IsQueueEmpty()
{
    return g_queueCount == 0U;
}

bool EnqueueCommand(const char* command, Operation operation)
{
    if ((command == nullptr) || (command[0] == '\0') || IsQueueFull())
    {
        return false;
    }

    const size_t commandLength = strnlen(command, COMMAND_MAX_LENGTH);

    if (commandLength >= COMMAND_MAX_LENGTH)
    {
        return false;
    }

    CopyText(
        g_commandQueue[g_queueTail].text,
        sizeof(g_commandQueue[g_queueTail].text),
        command);

    g_commandQueue[g_queueTail].operation = operation;

    g_queueTail = (g_queueTail + 1U) % COMMAND_QUEUE_DEPTH;
    ++g_queueCount;

    return true;
}

bool DequeueCommand(
    char* destination,
    size_t destinationSize,
    Operation& operation)
{
    if ((destination == nullptr) ||
        (destinationSize == 0U) ||
        IsQueueEmpty())
    {
        return false;
    }

    CopyText(
        destination,
        destinationSize,
        g_commandQueue[g_queueHead].text);

    operation = g_commandQueue[g_queueHead].operation;

    g_commandQueue[g_queueHead].text[0] = '\0';
    g_commandQueue[g_queueHead].operation = Operation::None;

    g_queueHead = (g_queueHead + 1U) % COMMAND_QUEUE_DEPTH;
    --g_queueCount;

    return true;
}

void AppendResponseCharacter(char value)
{
    if (g_responseLength >= (RESPONSE_BUFFER_SIZE - 1U))
    {
        return;
    }

    g_responseBuffer[g_responseLength++] = value;
    g_responseBuffer[g_responseLength] = '\0';
}

bool StartsWith(const char* text, const char* prefix)
{
    if ((text == nullptr) || (prefix == nullptr))
    {
        return false;
    }

    return strncmp(text, prefix, strlen(prefix)) == 0;
}

const char* SkipWhitespace(const char* text)
{
    if (text == nullptr)
    {
        return "";
    }

    while ((*text == ' ') ||
           (*text == '\t') ||
           (*text == ':') ||
           (*text == ','))
    {
        ++text;
    }

    return text;
}

void TrimLine(char* text)
{
    if (text == nullptr)
    {
        return;
    }

    size_t length = strlen(text);

    while ((length > 0U) &&
           ((text[length - 1U] == '\r') ||
            (text[length - 1U] == '\n') ||
            (text[length - 1U] == ' ') ||
            (text[length - 1U] == '\t')))
    {
        text[length - 1U] = '\0';
        --length;
    }

    size_t start = 0U;

    while ((text[start] == ' ') || (text[start] == '\t'))
    {
        ++start;
    }

    if (start > 0U)
    {
        memmove(text, text + start, strlen(text + start) + 1U);
    }
}

uint32_t ParseUnsignedValue(const char* text, int base)
{
    if (text == nullptr)
    {
        return 0U;
    }

    char* end = nullptr;
    const unsigned long value = strtoul(text, &end, base);

    if (end == text)
    {
        return 0U;
    }

    if (value > UINT32_MAX)
    {
        return UINT32_MAX;
    }

    return static_cast<uint32_t>(value);
}

void UpdateRegistrationState(int registrationCode)
{
    g_status.registration.registered =
        (registrationCode == 1) ||
        (registrationCode == 5);

    g_status.registration.roaming = registrationCode == 5;

    switch (registrationCode)
    {
        case 0:
            g_status.registration.state =
                RegistrationState::NotRegistered;
            break;

        case 1:
            g_status.registration.state =
                RegistrationState::RegisteredHome;
            break;

        case 2:
            g_status.registration.state =
                RegistrationState::Searching;
            break;

        case 3:
            g_status.registration.state =
                RegistrationState::RegistrationDenied;
            break;

        case 5:
            g_status.registration.state =
                RegistrationState::RegisteredRoaming;
            break;

        default:
            g_status.registration.state =
                RegistrationState::Unknown;
            break;
    }

    if (g_status.registration.registered)
    {
        g_lastRegistrationMs = NowMs();

        if (g_status.dataSessionActive)
        {
            g_status.state = ModemState::Online;
        }
        else if (g_status.state != ModemState::Connecting)
        {
            g_status.state = ModemState::Ready;
        }
    }
    else if (g_status.modemPresent && g_status.simPresent)
    {
        if (g_status.state == ModemState::Online)
        {
            g_status.diagnostics.lastDisconnectReason =
                DisconnectReason::RegistrationLost;
        }

        g_status.dataSessionActive = false;
        g_status.packetAttached = false;
        g_status.dataSessionState = DataSessionState::Inactive;
        g_status.state = ModemState::Registering;
    }
}

void ParseRegistration(const char* line)
{
    const char* payload = strchr(line, ':');

    if (payload == nullptr)
    {
        return;
    }

    ++payload;

    char copy[LINE_BUFFER_SIZE]{};
    CopyText(copy, sizeof(copy), payload);

    char* save = nullptr;
    char* token = strtok_r(copy, ",", &save);
    int fieldIndex = 0;
    int registrationCode = -1;

    while (token != nullptr)
    {
        TrimLine(token);

        if (fieldIndex == 1)
        {
            registrationCode = atoi(token);
        }
        else if (fieldIndex == 2)
        {
            while (*token == '"')
            {
                ++token;
            }

            char* quote = strchr(token, '"');

            if (quote != nullptr)
            {
                *quote = '\0';
            }

            g_status.registration.tac =
                ParseUnsignedValue(token, 16);
        }
        else if (fieldIndex == 3)
        {
            while (*token == '"')
            {
                ++token;
            }

            char* quote = strchr(token, '"');

            if (quote != nullptr)
            {
                *quote = '\0';
            }

            g_status.registration.cellId =
                ParseUnsignedValue(token, 16);
        }
        else if (fieldIndex == 4)
        {
            g_status.registration.accessTechnology =
                static_cast<uint8_t>(atoi(token));
        }

        token = strtok_r(nullptr, ",", &save);
        ++fieldIndex;
    }

    if (registrationCode < 0)
    {
        const char* comma = strrchr(line, ',');

        if (comma != nullptr)
        {
            registrationCode = atoi(comma + 1);
        }
    }

    if (registrationCode >= 0)
    {
        UpdateRegistrationState(registrationCode);
    }
}

void ParseSignal(const char* line)
{
    int rssi = 99;
    int ber = 99;

    const char* payload = strchr(line, ':');

    if (payload == nullptr)
    {
        return;
    }

    if (sscanf(payload + 1, " %d,%d", &rssi, &ber) != 2)
    {
        return;
    }

    if ((rssi >= 0) && (rssi <= 31))
    {
        g_status.signal.rssi = static_cast<int8_t>(rssi);
    }
    else
    {
        g_status.signal.rssi = -1;
    }

    if ((ber >= 0) && (ber <= 99))
    {
        g_status.signal.ber = static_cast<uint8_t>(ber);
    }
}

void ParseExtendedSignal(const char* line)
{
    const char* payload = strchr(line, ':');

    if (payload == nullptr)
    {
        return;
    }

    int rxlev = 99;
    int ber = 99;
    int rscp = 255;
    int ecno = 255;
    int rsrq = 255;
    int rsrp = 255;

    const int count = sscanf(
        payload + 1,
        " %d,%d,%d,%d,%d,%d",
        &rxlev,
        &ber,
        &rscp,
        &ecno,
        &rsrq,
        &rsrp);

    if (count >= 6)
    {
        if ((rsrp >= 0) && (rsrp <= 97))
        {
            g_status.signal.rsrp =
                static_cast<int16_t>(-140 + rsrp);
        }

        if ((rsrq >= 0) && (rsrq <= 34))
        {
            g_status.signal.rsrq =
                static_cast<int16_t>((-195 + (rsrq * 5)) / 10);
        }
    }
}

void ParseCarrier(const char* line)
{
    const char* firstQuote = strchr(line, '"');

    if (firstQuote != nullptr)
    {
        const char* secondQuote = strchr(firstQuote + 1, '"');

        if (secondQuote != nullptr)
        {
            const size_t length =
                static_cast<size_t>(
                    secondQuote - firstQuote - 1);

            if ((length > 0U) &&
                (length < sizeof(g_status.registration.carrier)))
            {
                memcpy(
                    g_status.registration.carrier,
                    firstQuote + 1,
                    length);

                g_status.registration.carrier[length] = '\0';
            }
        }
    }

    const char* numericOperator = nullptr;
    const char* search = line;

    while ((search = strchr(search, '"')) != nullptr)
    {
        ++search;

        if (isdigit(static_cast<unsigned char>(*search)))
        {
            numericOperator = search;
        }
    }

    if (numericOperator == nullptr)
    {
        return;
    }

    char digits[8]{};
    size_t count = 0U;

    while (isdigit(static_cast<unsigned char>(*numericOperator)) &&
           (count < (sizeof(digits) - 1U)))
    {
        digits[count++] = *numericOperator++;
    }

    digits[count] = '\0';

    if ((count == 5U) || (count == 6U))
    {
        char mccText[4]{};
        memcpy(mccText, digits, 3U);

        g_status.registration.mcc =
            static_cast<uint16_t>(atoi(mccText));

        g_status.registration.mnc =
            static_cast<uint16_t>(atoi(digits + 3U));
    }
}

void ParsePacketAttach(const char* line)
{
    int attached = 0;
    const char* payload = strchr(line, ':');

    if ((payload != nullptr) &&
        (sscanf(payload + 1, " %d", &attached) == 1))
    {
        g_status.packetAttached = attached == 1;

        if (!g_status.packetAttached &&
            g_status.dataSessionActive)
        {
            g_status.dataSessionActive = false;
            g_status.dataSessionState =
                DataSessionState::Inactive;
            g_status.diagnostics.lastDisconnectReason =
                DisconnectReason::PacketAttachLost;
        }
    }
}

void ParsePdpState(const char* line)
{
    int contextId = 0;
    int active = 0;
    const char* payload = strchr(line, ':');

    if ((payload != nullptr) &&
        (sscanf(payload + 1, " %d,%d", &contextId, &active) == 2) &&
        (contextId == 1))
    {
        g_status.dataSessionActive = active == 1;
        g_status.dataSessionState =
            g_status.dataSessionActive
                ? DataSessionState::Active
                : DataSessionState::Inactive;

        if (g_status.dataSessionActive)
        {
            g_lastDataSessionMs = NowMs();
        }
    }
}

void ParseIpAddress(const char* line)
{
    const char* payload = strchr(line, ':');

    if (payload == nullptr)
    {
        payload = line;
    }
    else
    {
        ++payload;
    }

    const char* comma = strrchr(payload, ',');

    if (comma != nullptr)
    {
        payload = comma + 1;
    }

    payload = SkipWhitespace(payload);

    char parsed[sizeof(g_status.ipAddress)]{};
    size_t writeIndex = 0U;

    while ((*payload != '\0') &&
           (*payload != '\r') &&
           (*payload != '\n') &&
           (writeIndex < (sizeof(parsed) - 1U)))
    {
        if ((*payload != '"') &&
            (*payload != ' ') &&
            (*payload != '\t'))
        {
            parsed[writeIndex++] = *payload;
        }

        ++payload;
    }

    parsed[writeIndex] = '\0';

    if ((strchr(parsed, '.') != nullptr) ||
        (strchr(parsed, ':') != nullptr))
    {
        CopyText(
            g_status.ipAddress,
            sizeof(g_status.ipAddress),
            parsed);

        g_status.dataSessionActive = true;
        g_status.dataSessionState = DataSessionState::Active;
        g_lastDataSessionMs = NowMs();

        if (g_status.registration.registered)
        {
            g_status.state = ModemState::Online;
        }
    }
}

void ParseNumericIdentity(
    const char* line,
    char* destination,
    size_t destinationSize,
    size_t minimumLength)
{
    if ((line == nullptr) ||
        (destination == nullptr) ||
        (destinationSize == 0U))
    {
        return;
    }

    char digits[40]{};
    size_t digitCount = 0U;

    for (size_t index = 0U; line[index] != '\0'; ++index)
    {
        if (isdigit(static_cast<unsigned char>(line[index])))
        {
            if (digitCount < (sizeof(digits) - 1U))
            {
                digits[digitCount++] = line[index];
            }
        }
    }

    digits[digitCount] = '\0';

    if (digitCount >= minimumLength)
    {
        CopyText(destination, destinationSize, digits);
    }
}

void ParseFirmwareVersion(const char* line)
{
    if ((line == nullptr) || (line[0] == '\0'))
    {
        return;
    }

    if (StartsWith(line, "AT") ||
        StartsWith(line, "+CGMR:") ||
        StartsWith(line, "OK") ||
        StartsWith(line, "ERROR"))
    {
        const char* payload = strchr(line, ':');

        if (payload != nullptr)
        {
            payload = SkipWhitespace(payload + 1);

            if (payload[0] != '\0')
            {
                CopyText(
                    g_status.firmwareVersion,
                    sizeof(g_status.firmwareVersion),
                    payload);
            }
        }

        return;
    }

    CopyText(
        g_status.firmwareVersion,
        sizeof(g_status.firmwareVersion),
        line);
}

void ParseCommandError(const char* line)
{
    int errorCode = -1;

    if (StartsWith(line, "+CME ERROR"))
    {
        const char* payload = strchr(line, ':');

        if (payload != nullptr)
        {
            errorCode = atoi(payload + 1);
        }

        g_status.diagnostics.lastCmeError = errorCode;
    }
    else if (StartsWith(line, "+CMS ERROR"))
    {
        const char* payload = strchr(line, ':');

        if (payload != nullptr)
        {
            errorCode = atoi(payload + 1);
        }

        g_status.diagnostics.lastCmsError = errorCode;
    }

    SetLastError(line);
}

void ProcessResponseLine(char* line)
{
    TrimLine(line);

    if (line[0] == '\0')
    {
        return;
    }

    if ((strcmp(line, "OK") == 0) ||
        (strcmp(line, "ERROR") == 0))
    {
        if (strcmp(line, "ERROR") == 0)
        {
            SetLastError(line);
        }

        return;
    }

    if (StartsWith(line, "+CME ERROR") ||
        StartsWith(line, "+CMS ERROR"))
    {
        ParseCommandError(line);
        return;
    }

    if (StartsWith(line, "+CPIN:"))
    {
        g_status.simPresent =
            strstr(line, "READY") != nullptr;
        return;
    }

    if (StartsWith(line, "+CREG:") ||
        StartsWith(line, "+CGREG:") ||
        StartsWith(line, "+CEREG:"))
    {
        ParseRegistration(line);
        return;
    }

    if (StartsWith(line, "+CGATT:"))
    {
        ParsePacketAttach(line);
        return;
    }

    if (StartsWith(line, "+CGACT:"))
    {
        ParsePdpState(line);
        return;
    }

    if (StartsWith(line, "+CSQ:"))
    {
        ParseSignal(line);
        return;
    }

    if (StartsWith(line, "+CESQ:"))
    {
        ParseExtendedSignal(line);
        return;
    }

    if (StartsWith(line, "+COPS:"))
    {
        ParseCarrier(line);
        return;
    }

    if (StartsWith(line, "+CGPADDR:") ||
        StartsWith(line, "+CGCONTRDP:"))
    {
        ParseIpAddress(line);
        return;
    }

    if (StartsWith(line, "+ICCID:") ||
        StartsWith(line, "+CCID:"))
    {
        ParseNumericIdentity(
            line,
            g_status.iccid,
            sizeof(g_status.iccid),
            18U);
        return;
    }

    if ((strcmp(g_activeCommand, "AT+CGSN") == 0) ||
        (strcmp(g_activeCommand, "AT+GSN") == 0))
    {
        ParseNumericIdentity(
            line,
            g_status.imei,
            sizeof(g_status.imei),
            14U);
        return;
    }

    if (strcmp(g_activeCommand, "AT+CIMI") == 0)
    {
        ParseNumericIdentity(
            line,
            g_status.imsi,
            sizeof(g_status.imsi),
            14U);
        return;
    }

    if ((strcmp(g_activeCommand, "AT+CCID") == 0) ||
        (strcmp(g_activeCommand, "AT+ICCID") == 0))
    {
        ParseNumericIdentity(
            line,
            g_status.iccid,
            sizeof(g_status.iccid),
            18U);
        return;
    }

    if ((strcmp(g_activeCommand, "AT+CGMR") == 0) ||
        (strcmp(g_activeCommand, "ATI") == 0))
    {
        ParseFirmwareVersion(line);
    }
}

void ProcessCompletedLine()
{
    g_lineBuffer[g_lineLength] = '\0';
    ProcessResponseLine(g_lineBuffer);
    g_lineLength = 0U;
    g_lineBuffer[0] = '\0';
}

void ProcessIncomingByte(uint8_t value)
{
    AppendResponseCharacter(static_cast<char>(value));

    if (value == '\n')
    {
        ProcessCompletedLine();
        return;
    }

    if (value == '\r')
    {
        return;
    }

    if (g_lineLength < (LINE_BUFFER_SIZE - 1U))
    {
        g_lineBuffer[g_lineLength++] =
            static_cast<char>(value);

        g_lineBuffer[g_lineLength] = '\0';
    }
}

bool ResponseContains(const char* text)
{
    return
        (text != nullptr) &&
        (g_responseBuffer[0] != '\0') &&
        (strstr(g_responseBuffer, text) != nullptr);
}

bool ResponseSucceeded()
{
    return
        ResponseContains("\r\nOK\r\n") ||
        ResponseContains("\nOK\r\n") ||
        (strcmp(g_responseBuffer, "OK\r\n") == 0);
}

bool ResponseFailed()
{
    return
        ResponseContains("ERROR") ||
        ResponseContains("+CME ERROR") ||
        ResponseContains("+CMS ERROR");
}

uint32_t TimeoutForCommand(const char* command)
{
    if (command == nullptr)
    {
        return COMMAND_TIMEOUT_MS;
    }

    if (StartsWith(command, "AT+CREG?") ||
        StartsWith(command, "AT+CGREG?") ||
        StartsWith(command, "AT+CEREG?") ||
        StartsWith(command, "AT+COPS?"))
    {
        return REGISTRATION_TIMEOUT_MS;
    }

    if (StartsWith(command, "AT+CGATT=") ||
        StartsWith(command, "AT+CGACT="))
    {
        return DATA_SESSION_TIMEOUT_MS;
    }

    if (StartsWith(command, "AT+CFUN=1,1"))
    {
        return MODEM_RESTART_TIMEOUT_MS;
    }

    return COMMAND_TIMEOUT_MS;
}

void SendActiveCommand()
{
    char commandFrame[COMMAND_MAX_LENGTH + 4U]{};

    const int frameLength = snprintf(
        commandFrame,
        sizeof(commandFrame),
        "%s\r\n",
        g_activeCommand);

    if ((frameLength <= 0) ||
        (static_cast<size_t>(frameLength) >=
         sizeof(commandFrame)))
    {
        g_commandActive = false;
        SetLastError("LTE command frame overflow");
        return;
    }

    g_responseBuffer[0] = '\0';
    g_responseLength = 0U;
    g_lineBuffer[0] = '\0';
    g_lineLength = 0U;

    (void)uart_flush_input(LTE_UART_PORT);

    const int bytesWritten = uart_write_bytes(
        LTE_UART_PORT,
        commandFrame,
        frameLength);

    CopyText(
        g_status.diagnostics.lastCommand,
        sizeof(g_status.diagnostics.lastCommand),
        g_activeCommand);

    if (bytesWritten != frameLength)
    {
        ESP_LOGW(
            TAG,
            "UART write incomplete for command: %s",
            g_activeCommand);
    }

    g_commandStartedMs = NowMs();
    g_activeTimeoutMs = TimeoutForCommand(g_activeCommand);
    g_commandActive = true;
}

void StartNextCommand()
{
    if (g_commandActive || IsQueueEmpty())
    {
        return;
    }

    if (!DequeueCommand(
            g_activeCommand,
            sizeof(g_activeCommand),
            g_activeOperation))
    {
        return;
    }

    g_commandRetryCount = 0U;
    SendActiveCommand();
}

bool EnqueuePdpConfiguration(Operation operation)
{
    if (g_status.configuration.apn[0] == '\0')
    {
        return EnqueueCommand(
            "AT+CGDCONT=1,\"IP\"",
            operation);
    }

    char command[COMMAND_MAX_LENGTH]{};

    const int length = snprintf(
        command,
        sizeof(command),
        "AT+CGDCONT=1,\"IP\",\"%s\"",
        g_status.configuration.apn);

    if ((length <= 0) ||
        (static_cast<size_t>(length) >= sizeof(command)))
    {
        SetLastError("APN command exceeds LTE command buffer");
        return false;
    }

    return EnqueueCommand(command, operation);
}

void ScheduleReconnect(DisconnectReason reason)
{
    g_status.diagnostics.lastDisconnectReason = reason;

    if (!g_status.configuration.automaticReconnectEnabled ||
        g_manualDisconnect ||
        !g_initialized)
    {
        return;
    }

    const uint64_t delayMs =
        static_cast<uint64_t>(g_reconnectDelaySeconds) * 1000ULL;

    g_reconnectDueMs = NowMs() + delayMs;
    g_status.state = ModemState::Reconnecting;

    uint64_t nextDelay =
        static_cast<uint64_t>(g_reconnectDelaySeconds) * 2ULL;

    if (nextDelay >
        g_status.configuration.reconnectMaximumDelaySeconds)
    {
        nextDelay =
            g_status.configuration.reconnectMaximumDelaySeconds;
    }

    g_reconnectDelaySeconds =
        static_cast<uint32_t>(nextDelay);
}

void ResetReconnectBackoff()
{
    g_reconnectDelaySeconds =
        g_status.configuration.reconnectInitialDelaySeconds;

    g_reconnectDueMs = 0U;
}

void CompleteDataConnection()
{
    if (g_status.registration.registered &&
        g_status.packetAttached &&
        g_status.dataSessionActive &&
        (g_status.ipAddress[0] != '\0'))
    {
        g_status.state = ModemState::Online;
        g_status.dataSessionState = DataSessionState::Active;
        g_lastDataSessionMs = NowMs();
        g_manualDisconnect = false;
        ResetReconnectBackoff();
    }
    else
    {
        ++g_status.counters.dataSessionFailureCount;

        g_status.dataSessionState = DataSessionState::Error;

        ScheduleReconnect(
            g_status.registration.registered
                ? DisconnectReason::NoIPAddress
                : DisconnectReason::RegistrationLost);
    }
}

void AdvanceInitialization(bool commandSucceeded)
{
    if (!commandSucceeded)
    {
        if (g_initStep == InitStep::Attention)
        {
            g_status.modemPresent = false;
            g_status.state = ModemState::Error;
            g_status.diagnostics.lastDisconnectReason =
                DisconnectReason::InitializationFailure;
            g_nextActionMs = NowMs() + ERROR_RETRY_DELAY_MS;
            g_initStep = InitStep::None;
        }

        return;
    }

    switch (g_initStep)
    {
        case InitStep::Attention:
            g_status.modemPresent = true;
            g_status.state = ModemState::Initializing;
            g_modemStartedMs = NowMs();
            g_initStep = InitStep::EchoOff;
            (void)EnqueueCommand("ATE0", Operation::Initialization);
            break;

        case InitStep::EchoOff:
            g_initStep = InitStep::ExtendedErrors;
            (void)EnqueueCommand(
                "AT+CMEE=2",
                Operation::Initialization);
            break;

        case InitStep::ExtendedErrors:
            g_initStep = InitStep::SimStatus;
            (void)EnqueueCommand(
                "AT+CPIN?",
                Operation::Initialization);
            break;

        case InitStep::SimStatus:
            g_initStep = InitStep::Imei;
            (void)EnqueueCommand(
                "AT+CGSN",
                Operation::Initialization);
            break;

        case InitStep::Imei:
            g_initStep = InitStep::Imsi;
            (void)EnqueueCommand(
                "AT+CIMI",
                Operation::Initialization);
            break;

        case InitStep::Imsi:
            g_initStep = InitStep::Iccid;
            (void)EnqueueCommand(
                "AT+CCID",
                Operation::Initialization);
            break;

        case InitStep::Iccid:
            g_initStep = InitStep::Firmware;
            (void)EnqueueCommand(
                "AT+CGMR",
                Operation::Initialization);
            break;

        case InitStep::Firmware:
            g_initStep = InitStep::RegistrationUrc;
            (void)EnqueueCommand(
                "AT+CEREG=2",
                Operation::Initialization);
            break;

        case InitStep::RegistrationUrc:
            g_initStep = InitStep::Registration;
            g_status.state = ModemState::Registering;
            (void)EnqueueCommand(
                "AT+CEREG?",
                Operation::Initialization);
            break;

        case InitStep::Registration:
            g_initStep = InitStep::PacketAttach;
            (void)EnqueueCommand(
                "AT+CGATT?",
                Operation::Initialization);
            break;

        case InitStep::PacketAttach:
            g_initStep = InitStep::Signal;
            (void)EnqueueCommand(
                "AT+CSQ",
                Operation::Initialization);
            break;

        case InitStep::Signal:
            g_initStep = InitStep::ExtendedSignal;
            (void)EnqueueCommand(
                "AT+CESQ",
                Operation::Initialization);
            break;

        case InitStep::ExtendedSignal:
            g_initStep = InitStep::Carrier;
            (void)EnqueueCommand(
                "AT+COPS?",
                Operation::Initialization);
            break;

        case InitStep::Carrier:
            g_initStep = InitStep::ConfigurePdp;

            if (!EnqueuePdpConfiguration(
                    Operation::Initialization))
            {
                g_status.dataSessionState =
                    DataSessionState::Error;
            }
            else
            {
                g_status.dataSessionState =
                    DataSessionState::Configuring;
            }
            break;

        case InitStep::ConfigurePdp:
            g_initStep = InitStep::ActivatePdp;
            g_status.state = ModemState::Connecting;
            g_status.dataSessionState =
                DataSessionState::Attaching;

            (void)EnqueueCommand(
                "AT+CGATT=1",
                Operation::Initialization);
            (void)EnqueueCommand(
                "AT+CGACT=1,1",
                Operation::Initialization);
            break;

        case InitStep::ActivatePdp:
            g_initStep = InitStep::IpAddress;
            g_status.dataSessionState =
                DataSessionState::Activating;

            (void)EnqueueCommand(
                "AT+CGACT?",
                Operation::Initialization);
            (void)EnqueueCommand(
                "AT+CGPADDR=1",
                Operation::Initialization);
            break;

        case InitStep::IpAddress:
            g_initStep = InitStep::Complete;

            if (g_status.dataSessionActive)
            {
                CompleteDataConnection();
            }
            else if (g_status.registration.registered)
            {
                g_status.state = ModemState::Ready;
                ScheduleReconnect(
                    DisconnectReason::PDPContextLost);
            }
            else
            {
                g_status.state = ModemState::Registering;
                ++g_status.counters.registrationFailureCount;
                ScheduleReconnect(
                    DisconnectReason::RegistrationLost);
            }

            g_lastStatusRefreshMs = NowMs();
            g_lastIdentityRefreshMs = NowMs();
            g_lastDiagnosticRefreshMs = NowMs();
            break;

        case InitStep::Complete:
        case InitStep::None:
        default:
            break;
    }
}

void FinishActiveCommand(bool succeeded)
{
    const Operation completedOperation = g_activeOperation;
    const bool wasInitializationCommand =
        completedOperation == Operation::Initialization;

    if (succeeded)
    {
        g_status.modemPresent = true;
        g_lastSuccessfulCommandMs = NowMs();
        g_status.diagnostics.lastError[0] = '\0';
    }
    else
    {
        ++g_status.counters.commandErrorCount;

        if (completedOperation == Operation::ConnectData)
        {
            ++g_status.counters.dataSessionFailureCount;
            g_status.dataSessionState =
                DataSessionState::Error;
            ScheduleReconnect(
                DisconnectReason::CommandRejected);
        }
    }

    g_commandActive = false;
    g_activeCommand[0] = '\0';
    g_activeOperation = Operation::None;
    g_commandStartedMs = 0U;
    g_activeTimeoutMs = COMMAND_TIMEOUT_MS;
    g_commandRetryCount = 0U;

    if (wasInitializationCommand)
    {
        AdvanceInitialization(succeeded);
        return;
    }

    if (completedOperation == Operation::DisconnectData &&
        IsQueueEmpty())
    {
        g_status.packetAttached = false;
        g_status.dataSessionActive = false;
        g_status.dataSessionState =
            DataSessionState::Inactive;
        g_status.ipAddress[0] = '\0';
        g_status.state =
            g_status.registration.registered
                ? ModemState::Ready
                : ModemState::Registering;
    }

    if (completedOperation == Operation::RestartModem)
    {
        ResetCommandEngine();
        g_initStep = InitStep::None;
        g_status.state = ModemState::Restarting;
        g_nextActionMs = NowMs() + MODEM_RESTART_TIMEOUT_MS;
    }
}

void RetryOrFailActiveCommand(bool timedOut)
{
    if (g_commandRetryCount < MAX_COMMAND_RETRIES)
    {
        ++g_commandRetryCount;
        SendActiveCommand();
        return;
    }

    if (timedOut)
    {
        ++g_status.counters.commandTimeoutCount;
        g_status.diagnostics.lastDisconnectReason =
            DisconnectReason::CommandTimeout;
        SetLastError("LTE command timed out");
    }

    FinishActiveCommand(false);
}

void PollUart()
{
    uint8_t buffer[128]{};

    while (true)
    {
        const int bytesRead = uart_read_bytes(
            LTE_UART_PORT,
            buffer,
            sizeof(buffer),
            0);

        if (bytesRead <= 0)
        {
            break;
        }

        for (int index = 0; index < bytesRead; ++index)
        {
            ProcessIncomingByte(buffer[index]);
        }
    }
}

void UpdateActiveCommand()
{
    if (!g_commandActive)
    {
        return;
    }

    if (ResponseSucceeded())
    {
        FinishActiveCommand(true);
        return;
    }

    if (ResponseFailed())
    {
        RetryOrFailActiveCommand(false);
        return;
    }

    const uint64_t now = NowMs();

    if ((now - g_commandStartedMs) >= g_activeTimeoutMs)
    {
        RetryOrFailActiveCommand(true);
    }
}

void StartInitialization()
{
    if (g_commandActive || !IsQueueEmpty())
    {
        return;
    }

    g_status.state = ModemState::Initializing;
    g_status.modemPresent = false;
    g_status.simPresent = false;
    g_status.packetAttached = false;
    g_status.dataSessionActive = false;
    g_status.dataSessionState =
        DataSessionState::Inactive;
    g_status.registration.registered = false;
    g_status.registration.roaming = false;
    g_status.ipAddress[0] = '\0';

    g_initStep = InitStep::Attention;

    (void)EnqueueCommand(
        "AT",
        Operation::Initialization);
}

void ScheduleStatusRefresh()
{
    if (!g_initialized ||
        g_commandActive ||
        !IsQueueEmpty() ||
        (g_initStep != InitStep::Complete))
    {
        return;
    }

    const uint64_t now = NowMs();

    if ((now - g_lastStatusRefreshMs) >=
        STATUS_REFRESH_INTERVAL_MS)
    {
        g_lastStatusRefreshMs = now;

        (void)EnqueueCommand(
            "AT+CPIN?",
            Operation::StatusRefresh);
        (void)EnqueueCommand(
            "AT+CEREG?",
            Operation::StatusRefresh);
        (void)EnqueueCommand(
            "AT+CGATT?",
            Operation::StatusRefresh);
        (void)EnqueueCommand(
            "AT+CGACT?",
            Operation::StatusRefresh);
        (void)EnqueueCommand(
            "AT+CSQ",
            Operation::StatusRefresh);
        (void)EnqueueCommand(
            "AT+COPS?",
            Operation::StatusRefresh);
        (void)EnqueueCommand(
            "AT+CGPADDR=1",
            Operation::StatusRefresh);
    }

    if ((now - g_lastDiagnosticRefreshMs) >=
        DIAGNOSTIC_REFRESH_INTERVAL_MS)
    {
        g_lastDiagnosticRefreshMs = now;

        (void)EnqueueCommand(
            "AT+CESQ",
            Operation::StatusRefresh);
    }

    if ((now - g_lastIdentityRefreshMs) >=
        IDENTITY_REFRESH_INTERVAL_MS)
    {
        g_lastIdentityRefreshMs = now;

        (void)EnqueueCommand(
            "AT+CGSN",
            Operation::IdentityRefresh);
        (void)EnqueueCommand(
            "AT+CIMI",
            Operation::IdentityRefresh);
        (void)EnqueueCommand(
            "AT+CCID",
            Operation::IdentityRefresh);
        (void)EnqueueCommand(
            "AT+CGMR",
            Operation::IdentityRefresh);
    }
}

void StartDataConnection()
{
    if (g_commandActive ||
        !IsQueueEmpty() ||
        (g_initStep != InitStep::Complete))
    {
        return;
    }

    g_connectRequested = false;
    g_manualDisconnect = false;

    g_status.state = ModemState::Connecting;
    g_status.dataSessionState =
        DataSessionState::Configuring;

    if (!EnqueuePdpConfiguration(Operation::ConnectData))
    {
        g_status.dataSessionState =
            DataSessionState::Error;
        ScheduleReconnect(
            DisconnectReason::CommandRejected);
        return;
    }

    (void)EnqueueCommand(
        "AT+CGATT=1",
        Operation::ConnectData);
    (void)EnqueueCommand(
        "AT+CGACT=1,1",
        Operation::ConnectData);
    (void)EnqueueCommand(
        "AT+CGATT?",
        Operation::ConnectData);
    (void)EnqueueCommand(
        "AT+CGACT?",
        Operation::ConnectData);
    (void)EnqueueCommand(
        "AT+CGPADDR=1",
        Operation::ConnectData);
}

void StartDataDisconnection()
{
    if (g_commandActive || !IsQueueEmpty())
    {
        return;
    }

    g_disconnectRequested = false;
    g_connectRequested = false;
    g_manualDisconnect = true;
    g_reconnectDueMs = 0U;

    g_status.state = ModemState::Disconnecting;
    g_status.dataSessionState =
        DataSessionState::Deactivating;
    g_status.diagnostics.lastDisconnectReason =
        DisconnectReason::ManualDisconnect;

    (void)EnqueueCommand(
        "AT+CGACT=0,1",
        Operation::DisconnectData);
    (void)EnqueueCommand(
        "AT+CGATT=0",
        Operation::DisconnectData);
}

void StartModemRestart()
{
    if (g_commandActive || !IsQueueEmpty())
    {
        return;
    }

    g_restartRequested = false;
    g_manualDisconnect = false;
    g_connectRequested = false;
    g_disconnectRequested = false;
    g_reconnectDueMs = 0U;

    ++g_status.counters.modemRestartCount;
    (void)SaveConfigurationToNVS();

    g_status.state = ModemState::Restarting;
    g_status.diagnostics.lastDisconnectReason =
        DisconnectReason::ModemRestart;

    (void)EnqueueCommand(
        "AT+CFUN=1,1",
        Operation::RestartModem);
}

void UpdateAutomaticReconnect()
{
    if (!g_initialized ||
        !g_status.configuration.automaticReconnectEnabled ||
        g_manualDisconnect ||
        g_connectRequested ||
        g_disconnectRequested ||
        g_restartRequested ||
        g_commandActive ||
        !IsQueueEmpty() ||
        (g_initStep != InitStep::Complete))
    {
        return;
    }

    if (g_status.dataSessionActive &&
        g_status.registration.registered &&
        (g_status.ipAddress[0] != '\0'))
    {
        ResetReconnectBackoff();
        return;
    }

    if (g_reconnectDueMs == 0U)
    {
        ScheduleReconnect(
            g_status.registration.registered
                ? DisconnectReason::PDPContextLost
                : DisconnectReason::RegistrationLost);
        return;
    }

    if (NowMs() >= g_reconnectDueMs)
    {
        ++g_status.counters.reconnectCount;
        g_connectRequested = true;
        g_reconnectDueMs = 0U;
    }
}

void UpdateDiagnosticAges()
{
    g_status.diagnostics.lastSuccessfulCommandAgeSeconds =
        AgeSeconds(g_lastSuccessfulCommandMs);

    g_status.diagnostics.lastRegistrationAgeSeconds =
        AgeSeconds(g_lastRegistrationMs);

    g_status.diagnostics.lastDataSessionAgeSeconds =
        AgeSeconds(g_lastDataSessionMs);
}

} // namespace

bool Initialize()
{
    if (g_initialized)
    {
        return true;
    }

    memset(&g_status, 0, sizeof(g_status));
    SetDefaultConfiguration();

    (void)LoadConfigurationFromNVS();

    const Configuration configuration =
        g_status.configuration;
    const Counters counters =
        g_status.counters;

    ResetStatus();

    g_status.configuration = configuration;
    g_status.counters = counters;

    ResetCommandEngine();

    const uart_config_t uartConfig = {
        .baud_rate = LTE_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
        .flags = {
            .allow_pd = 0,
            .backup_before_sleep = 0
        }
    };

    esp_err_t result =
        uart_param_config(LTE_UART_PORT, &uartConfig);

    if (result != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "uart_param_config failed: %s",
            esp_err_to_name(result));

        g_status.state = ModemState::Error;
        SetLastError("LTE UART parameter configuration failed");
        return false;
    }

    result = uart_set_pin(
        LTE_UART_PORT,
        LTE_UART_TX_PIN,
        LTE_UART_RX_PIN,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE);

    if (result != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "uart_set_pin failed: %s",
            esp_err_to_name(result));

        g_status.state = ModemState::Error;
        SetLastError("LTE UART pin configuration failed");
        return false;
    }

    result = uart_driver_install(
        LTE_UART_PORT,
        UART_RX_BUFFER_SIZE,
        UART_TX_BUFFER_SIZE,
        0,
        nullptr,
        0);

    if ((result != ESP_OK) &&
        (result != ESP_ERR_INVALID_STATE))
    {
        ESP_LOGE(
            TAG,
            "uart_driver_install failed: %s",
            esp_err_to_name(result));

        g_status.state = ModemState::Error;
        SetLastError("LTE UART driver installation failed");
        return false;
    }

    result = uart_flush_input(LTE_UART_PORT);

    if (result != ESP_OK)
    {
        ESP_LOGW(
            TAG,
            "uart_flush_input failed: %s",
            esp_err_to_name(result));
    }

    g_initialized = true;
    g_status.state = ModemState::Initializing;
    g_nextActionMs = NowMs() + INITIAL_RETRY_DELAY_MS;

    g_reconnectDelaySeconds =
        g_status.configuration.reconnectInitialDelaySeconds;

    ESP_LOGI(
        TAG,
        "LTE manager initialized on UART%d TX=%d RX=%d",
        static_cast<int>(LTE_UART_PORT),
        LTE_UART_TX_PIN,
        LTE_UART_RX_PIN);

    return true;
}

void Update()
{
    if (!g_initialized)
    {
        return;
    }

    const uint64_t now = NowMs();

    g_status.uptimeSeconds =
        static_cast<uint32_t>(now / 1000ULL);

    g_status.modemUptimeSeconds =
        g_modemStartedMs == 0U
            ? 0U
            : AgeSeconds(g_modemStartedMs);

    PollUart();
    UpdateActiveCommand();
    UpdateDiagnosticAges();

    if ((g_initStep == InitStep::None) &&
        !g_commandActive &&
        IsQueueEmpty() &&
        (now >= g_nextActionMs))
    {
        StartInitialization();
    }

    if (g_restartRequested)
    {
        StartModemRestart();
    }
    else if (g_disconnectRequested)
    {
        StartDataDisconnection();
    }
    else if (g_connectRequested)
    {
        StartDataConnection();
    }

    UpdateAutomaticReconnect();
    ScheduleStatusRefresh();
    StartNextCommand();
}

const Status& GetStatus()
{
    return g_status;
}

const Configuration& GetConfiguration()
{
    return g_status.configuration;
}

bool IsOnline()
{
    return
        g_initialized &&
        g_status.modemPresent &&
        g_status.simPresent &&
        g_status.registration.registered &&
        g_status.packetAttached &&
        g_status.dataSessionActive &&
        (g_status.ipAddress[0] != '\0') &&
        (g_status.state == ModemState::Online);
}

bool IsRegistered()
{
    return
        g_initialized &&
        g_status.registration.registered;
}

bool IsDataSessionActive()
{
    return
        g_initialized &&
        g_status.dataSessionActive;
}

bool QueueATCommand(const char* command)
{
    if (!g_initialized || (command == nullptr))
    {
        return false;
    }

    if ((command[0] != 'A') || (command[1] != 'T'))
    {
        return false;
    }

    return EnqueueCommand(
        command,
        Operation::ManualCommand);
}

bool SetAPN(const char* apn)
{
    if (!IsValidAPN(apn))
    {
        return false;
    }

    if (strcmp(g_status.configuration.apn, apn) == 0)
    {
        return true;
    }

    char previous[APN_MAX_LENGTH]{};

    CopyText(
        previous,
        sizeof(previous),
        g_status.configuration.apn);

    CopyText(
        g_status.configuration.apn,
        sizeof(g_status.configuration.apn),
        apn);

    if (!SaveConfigurationToNVS())
    {
        CopyText(
            g_status.configuration.apn,
            sizeof(g_status.configuration.apn),
            previous);

        return false;
    }

    if (g_initialized)
    {
        g_connectRequested = true;
        g_manualDisconnect = false;
    }

    return true;
}

const char* GetAPN()
{
    return g_status.configuration.apn;
}

bool SaveConfiguration()
{
    return SaveConfigurationToNVS();
}

bool ReloadConfiguration()
{
    Configuration previous = g_status.configuration;

    if (!LoadConfigurationFromNVS())
    {
        g_status.configuration = previous;
        return false;
    }

    g_reconnectDelaySeconds =
        g_status.configuration.reconnectInitialDelaySeconds;

    return true;
}

bool RestoreDefaultConfiguration()
{
    SetDefaultConfiguration();
    g_reconnectDelaySeconds =
        g_status.configuration.reconnectInitialDelaySeconds;

    if (!SaveConfigurationToNVS())
    {
        return false;
    }

    if (g_initialized)
    {
        g_connectRequested = true;
        g_manualDisconnect = false;
    }

    return true;
}

bool ConnectDataSession()
{
    if (!g_initialized)
    {
        return false;
    }

    if (g_status.dataSessionActive &&
        g_status.packetAttached &&
        (g_status.ipAddress[0] != '\0'))
    {
        return true;
    }

    g_manualDisconnect = false;
    g_disconnectRequested = false;
    g_connectRequested = true;
    g_reconnectDueMs = 0U;

    return true;
}

bool DisconnectDataSession()
{
    if (!g_initialized)
    {
        return false;
    }

    g_connectRequested = false;
    g_disconnectRequested = true;
    g_reconnectDueMs = 0U;

    return true;
}

bool SetAutomaticReconnectEnabled(bool enabled)
{
    if (g_status.configuration.automaticReconnectEnabled == enabled)
    {
        return true;
    }

    const bool previous =
        g_status.configuration.automaticReconnectEnabled;

    g_status.configuration.automaticReconnectEnabled = enabled;

    if (!SaveConfigurationToNVS())
    {
        g_status.configuration.automaticReconnectEnabled = previous;
        return false;
    }

    if (!enabled)
    {
        g_reconnectDueMs = 0U;
    }
    else if (g_initialized &&
             !g_status.dataSessionActive &&
             !g_manualDisconnect)
    {
        ScheduleReconnect(
            DisconnectReason::PDPContextLost);
    }

    return true;
}

bool RestartModem()
{
    if (!g_initialized)
    {
        return false;
    }

    g_restartRequested = true;
    return true;
}

void Reset()
{
    if (!g_initialized)
    {
        return;
    }

    ResetCommandEngine();

    const Configuration configuration =
        g_status.configuration;
    const Counters counters =
        g_status.counters;

    ResetStatus();

    g_status.configuration = configuration;
    g_status.counters = counters;

    g_initStep = InitStep::None;
    g_nextActionMs = NowMs() + ERROR_RETRY_DELAY_MS;

    g_reconnectDueMs = 0U;
    g_reconnectDelaySeconds =
        g_status.configuration.reconnectInitialDelaySeconds;

    g_manualDisconnect = false;
    g_connectRequested = false;
    g_disconnectRequested = false;
    g_restartRequested = false;

    (void)uart_flush_input(LTE_UART_PORT);

    ESP_LOGI(TAG, "LTE manager reset requested");
}

} // namespace WashTrac::LTE