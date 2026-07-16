/******************************************************************************
 *
 *  Project:
 *      WashTrac Core
 *
 *  File:
 *      lte_manager.cpp
 *
 *  Description:
 *      Air780E LTE modem manager implementation. Provides a non-blocking
 *      UART-driven AT command engine, modem initialization, SIM detection,
 *      network registration, signal reporting, identity collection, IP
 *      reporting, timeout handling, retries, and cached modem status.
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

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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

constexpr size_t COMMAND_QUEUE_DEPTH = 8U;
constexpr size_t COMMAND_MAX_LENGTH = 96U;
constexpr size_t RESPONSE_BUFFER_SIZE = 512U;
constexpr size_t LINE_BUFFER_SIZE = 160U;

constexpr uint32_t COMMAND_TIMEOUT_MS = 3000U;
constexpr uint32_t REGISTRATION_TIMEOUT_MS = 10000U;
constexpr uint32_t INITIAL_RETRY_DELAY_MS = 1000U;
constexpr uint32_t ERROR_RETRY_DELAY_MS = 5000U;
constexpr uint32_t STATUS_REFRESH_INTERVAL_MS = 10000U;
constexpr uint32_t IDENTITY_REFRESH_INTERVAL_MS = 300000U;
constexpr uint8_t MAX_COMMAND_RETRIES = 2U;

constexpr const char* TAG = "WashTracLTE";

enum class InitStep : uint8_t
{
    None = 0,
    Attention,
    EchoOff,
    SimStatus,
    Imei,
    Iccid,
    Registration,
    Signal,
    Carrier,
    IpAddress,
    Complete
};

struct CommandEntry
{
    char text[COMMAND_MAX_LENGTH];
};

Status g_status{};
bool g_initialized = false;

CommandEntry g_commandQueue[COMMAND_QUEUE_DEPTH]{};
size_t g_queueHead = 0U;
size_t g_queueTail = 0U;
size_t g_queueCount = 0U;

bool g_commandActive = false;
char g_activeCommand[COMMAND_MAX_LENGTH]{};
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

uint64_t NowMs()
{
    return static_cast<uint64_t>(esp_timer_get_time() / 1000LL);
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

void ResetStatus()
{
    memset(&g_status, 0, sizeof(g_status));

    g_status.state = ModemState::Off;
    g_status.rssi = -1;
    g_status.ber = 99U;

    CopyText(g_status.carrier, sizeof(g_status.carrier), "Unknown");
    CopyText(g_status.imei, sizeof(g_status.imei), "");
    CopyText(g_status.iccid, sizeof(g_status.iccid), "");
    CopyText(g_status.ipAddress, sizeof(g_status.ipAddress), "");
}

void ResetCommandEngine()
{
    memset(g_commandQueue, 0, sizeof(g_commandQueue));
    g_queueHead = 0U;
    g_queueTail = 0U;
    g_queueCount = 0U;

    g_commandActive = false;
    g_activeCommand[0] = '\0';

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

bool EnqueueCommand(const char* command)
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

    g_queueTail = (g_queueTail + 1U) % COMMAND_QUEUE_DEPTH;
    ++g_queueCount;

    return true;
}

bool DequeueCommand(char* destination, size_t destinationSize)
{
    if ((destination == nullptr) || (destinationSize == 0U) || IsQueueEmpty())
    {
        return false;
    }

    CopyText(
        destination,
        destinationSize,
        g_commandQueue[g_queueHead].text);

    g_commandQueue[g_queueHead].text[0] = '\0';
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

    while ((*text == ' ') || (*text == '\t') || (*text == ':'))
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

void ParseRegistration(const char* line)
{
    const char* comma = strrchr(line, ',');

    if (comma == nullptr)
    {
        return;
    }

    const int registrationCode = atoi(comma + 1);

    g_status.registered =
        (registrationCode == 1) ||
        (registrationCode == 5);

    if (g_status.registered)
    {
        g_status.state = ModemState::Online;
    }
    else if (g_status.modemPresent && g_status.simPresent)
    {
        g_status.state = ModemState::Registering;
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
        g_status.rssi = static_cast<int8_t>(rssi);
    }
    else
    {
        g_status.rssi = -1;
    }

    if ((ber >= 0) && (ber <= 99))
    {
        g_status.ber = static_cast<uint8_t>(ber);
    }
}

void ParseCarrier(const char* line)
{
    const char* firstQuote = strchr(line, '"');

    if (firstQuote == nullptr)
    {
        return;
    }

    const char* secondQuote = strchr(firstQuote + 1, '"');

    if (secondQuote == nullptr)
    {
        return;
    }

    const size_t length = static_cast<size_t>(secondQuote - firstQuote - 1);

    if ((length == 0U) || (length >= sizeof(g_status.carrier)))
    {
        return;
    }

    memcpy(g_status.carrier, firstQuote + 1, length);
    g_status.carrier[length] = '\0';
}

void ParseIpAddress(const char* line)
{
    const char* payload = strchr(line, ':');

    if (payload == nullptr)
    {
        return;
    }

    payload = SkipWhitespace(payload + 1);

    const char* comma = strrchr(payload, ',');

    if (comma != nullptr)
    {
        payload = SkipWhitespace(comma + 1);
    }

    char parsed[sizeof(g_status.ipAddress)]{};
    size_t writeIndex = 0U;

    while ((*payload != '\0') &&
           (*payload != '\r') &&
           (*payload != '\n') &&
           (writeIndex < (sizeof(parsed) - 1U)))
    {
        if ((*payload != '"') && (*payload != ' '))
        {
            parsed[writeIndex++] = *payload;
        }

        ++payload;
    }

    parsed[writeIndex] = '\0';

    if ((strchr(parsed, '.') != nullptr) || (strchr(parsed, ':') != nullptr))
    {
        CopyText(g_status.ipAddress, sizeof(g_status.ipAddress), parsed);
    }
}

void ParseNumericIdentity(
    const char* line,
    char* destination,
    size_t destinationSize,
    size_t minimumLength)
{
    if ((line == nullptr) || (destination == nullptr) || (destinationSize == 0U))
    {
        return;
    }

    char digits[40]{};
    size_t digitCount = 0U;

    for (size_t index = 0U; line[index] != '\0'; ++index)
    {
        if ((line[index] >= '0') && (line[index] <= '9'))
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

void ProcessResponseLine(char* line)
{
    TrimLine(line);

    if (line[0] == '\0')
    {
        return;
    }

    if (strcmp(line, "OK") == 0)
    {
        return;
    }

    if ((strcmp(line, "ERROR") == 0) ||
        StartsWith(line, "+CME ERROR") ||
        StartsWith(line, "+CMS ERROR"))
    {
        return;
    }

    if (StartsWith(line, "+CPIN:"))
    {
        g_status.simPresent = strstr(line, "READY") != nullptr;
        return;
    }

    if (StartsWith(line, "+CREG:") ||
        StartsWith(line, "+CGREG:") ||
        StartsWith(line, "+CEREG:"))
    {
        ParseRegistration(line);
        return;
    }

    if (StartsWith(line, "+CSQ:"))
    {
        ParseSignal(line);
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

    if ((strcmp(g_activeCommand, "AT+CCID") == 0) ||
        (strcmp(g_activeCommand, "AT+ICCID") == 0))
    {
        ParseNumericIdentity(
            line,
            g_status.iccid,
            sizeof(g_status.iccid),
            18U);
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
        g_lineBuffer[g_lineLength++] = static_cast<char>(value);
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
    return ResponseContains("\r\nOK\r\n") ||
           ResponseContains("\nOK\r\n") ||
           (strcmp(g_responseBuffer, "OK\r\n") == 0);
}

bool ResponseFailed()
{
    return ResponseContains("ERROR") ||
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

    if (StartsWith(command, "AT+CFUN=1,1"))
    {
        return 15000U;
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
        (static_cast<size_t>(frameLength) >= sizeof(commandFrame)))
    {
        g_commandActive = false;
        return;
    }

    g_responseBuffer[0] = '\0';
    g_responseLength = 0U;
    g_lineBuffer[0] = '\0';
    g_lineLength = 0U;

    uart_flush_input(LTE_UART_PORT);

    const int bytesWritten = uart_write_bytes(
        LTE_UART_PORT,
        commandFrame,
        frameLength);

    if (bytesWritten != frameLength)
    {
        ESP_LOGW(TAG, "UART write incomplete for command: %s", g_activeCommand);
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

    if (!DequeueCommand(g_activeCommand, sizeof(g_activeCommand)))
    {
        return;
    }

    g_commandRetryCount = 0U;
    SendActiveCommand();
}

void AdvanceInitialization(bool commandSucceeded)
{
    if (!commandSucceeded)
    {
        if (g_initStep == InitStep::Attention)
        {
            g_status.modemPresent = false;
            g_status.state = ModemState::Error;
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
            g_initStep = InitStep::EchoOff;
            (void)EnqueueCommand("ATE0");
            break;

        case InitStep::EchoOff:
            g_initStep = InitStep::SimStatus;
            (void)EnqueueCommand("AT+CPIN?");
            break;

        case InitStep::SimStatus:
            g_initStep = InitStep::Imei;
            (void)EnqueueCommand("AT+CGSN");
            break;

        case InitStep::Imei:
            g_initStep = InitStep::Iccid;
            (void)EnqueueCommand("AT+CCID");
            break;

        case InitStep::Iccid:
            g_initStep = InitStep::Registration;
            g_status.state = ModemState::Registering;
            (void)EnqueueCommand("AT+CEREG?");
            break;

        case InitStep::Registration:
            g_initStep = InitStep::Signal;
            (void)EnqueueCommand("AT+CSQ");
            break;

        case InitStep::Signal:
            g_initStep = InitStep::Carrier;
            (void)EnqueueCommand("AT+COPS?");
            break;

        case InitStep::Carrier:
            g_initStep = InitStep::IpAddress;
            (void)EnqueueCommand("AT+CGPADDR=1");
            break;

        case InitStep::IpAddress:
            g_initStep = InitStep::Complete;

            if (g_status.registered)
            {
                g_status.state = ModemState::Online;
            }
            else
            {
                g_status.state = ModemState::Ready;
            }

            g_lastStatusRefreshMs = NowMs();
            g_lastIdentityRefreshMs = NowMs();
            break;

        case InitStep::Complete:
        case InitStep::None:
        default:
            break;
    }
}

void FinishActiveCommand(bool succeeded)
{
    const bool wasInitializationCommand =
        (g_initStep != InitStep::None) &&
        (g_initStep != InitStep::Complete);

    if (succeeded)
    {
        g_status.modemPresent = true;
    }

    g_commandActive = false;
    g_activeCommand[0] = '\0';
    g_commandStartedMs = 0U;
    g_activeTimeoutMs = COMMAND_TIMEOUT_MS;
    g_commandRetryCount = 0U;

    if (wasInitializationCommand)
    {
        AdvanceInitialization(succeeded);
    }
}

void RetryOrFailActiveCommand()
{
    if (g_commandRetryCount < MAX_COMMAND_RETRIES)
    {
        ++g_commandRetryCount;
        SendActiveCommand();
        return;
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
        RetryOrFailActiveCommand();
        return;
    }

    const uint64_t now = NowMs();

    if ((now - g_commandStartedMs) >= g_activeTimeoutMs)
    {
        RetryOrFailActiveCommand();
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
    g_status.registered = false;

    g_initStep = InitStep::Attention;
    (void)EnqueueCommand("AT");
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

    if ((now - g_lastStatusRefreshMs) >= STATUS_REFRESH_INTERVAL_MS)
    {
        g_lastStatusRefreshMs = now;

        (void)EnqueueCommand("AT+CPIN?");
        (void)EnqueueCommand("AT+CEREG?");
        (void)EnqueueCommand("AT+CSQ");
        (void)EnqueueCommand("AT+COPS?");
        (void)EnqueueCommand("AT+CGPADDR=1");
    }

    if ((now - g_lastIdentityRefreshMs) >= IDENTITY_REFRESH_INTERVAL_MS)
    {
        g_lastIdentityRefreshMs = now;

        (void)EnqueueCommand("AT+CGSN");
        (void)EnqueueCommand("AT+CCID");
    }
}

} // namespace

bool Initialize()
{
    if (g_initialized)
    {
        return true;
    }

    ResetStatus();
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

    esp_err_t result = uart_param_config(LTE_UART_PORT, &uartConfig);

    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(result));
        g_status.state = ModemState::Error;
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
        ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(result));
        g_status.state = ModemState::Error;
        return false;
    }

    result = uart_driver_install(
        LTE_UART_PORT,
        UART_RX_BUFFER_SIZE,
        UART_TX_BUFFER_SIZE,
        0,
        nullptr,
        0);

    if ((result != ESP_OK) && (result != ESP_ERR_INVALID_STATE))
    {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(result));
        g_status.state = ModemState::Error;
        return false;
    }

    result = uart_flush_input(LTE_UART_PORT);

    if (result != ESP_OK)
    {
        ESP_LOGW(TAG, "uart_flush_input failed: %s", esp_err_to_name(result));
    }

    g_initialized = true;
    g_status.state = ModemState::Initializing;
    g_nextActionMs = NowMs() + INITIAL_RETRY_DELAY_MS;

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

    g_status.uptimeSeconds =
        static_cast<uint32_t>(NowMs() / 1000ULL);

    PollUart();
    UpdateActiveCommand();

    const uint64_t now = NowMs();

    if ((g_initStep == InitStep::None) &&
        !g_commandActive &&
        IsQueueEmpty() &&
        (now >= g_nextActionMs))
    {
        StartInitialization();
    }

    ScheduleStatusRefresh();
    StartNextCommand();
}

const Status& GetStatus()
{
    return g_status;
}

bool IsOnline()
{
    return
        g_initialized &&
        g_status.modemPresent &&
        g_status.simPresent &&
        g_status.registered &&
        (g_status.state == ModemState::Online);
}

bool IsRegistered()
{
    return g_initialized && g_status.registered;
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

    return EnqueueCommand(command);
}

void Reset()
{
    if (!g_initialized)
    {
        return;
    }

    ResetCommandEngine();

    g_status.state = ModemState::Initializing;
    g_status.modemPresent = false;
    g_status.simPresent = false;
    g_status.registered = false;
    g_status.rssi = -1;
    g_status.ber = 99U;

    CopyText(g_status.carrier, sizeof(g_status.carrier), "Unknown");
    CopyText(g_status.ipAddress, sizeof(g_status.ipAddress), "");

    g_initStep = InitStep::None;
    g_nextActionMs = NowMs() + ERROR_RETRY_DELAY_MS;

    uart_flush_input(LTE_UART_PORT);

    ESP_LOGI(TAG, "LTE manager reset requested");
}

} // namespace WashTrac::LTE