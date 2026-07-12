/******************************************************************************
 *
 *  Project:
 *      WashTrac Core
 *
 *  Module:
 *      Service Console
 *
 *  File:
 *      service_console.cpp
 *
 *  Description:
 *      Production read-only UART service console providing:
 *          - Fixed-size command buffering
 *          - No dynamic memory allocation
 *          - Read-only diagnostic commands
 *          - Runtime status, health, input, relay, fault, queue,
 *            configuration, and event reporting
 *
 ******************************************************************************/

#include "service_console.h"

#include "config.h"
#include "event_logger.h"
#include "fault_manager.h"
#include "input_manager.h"
#include "relay_scheduler.h"
#include "state_machine.h"
#include "system_health.h"
#include "wash_queue.h"

#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"

#include <array>
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace
{

constexpr const char* LOG_TAG = "ServiceConsole";

constexpr uart_port_t CONSOLE_UART = UART_NUM_0;
constexpr int CONSOLE_BAUD_RATE = 115200;
constexpr std::size_t RX_BUFFER_SIZE = 128U;
constexpr std::size_t COMMAND_BUFFER_SIZE = 64U;
constexpr TickType_t UART_READ_TIMEOUT_TICKS = 0U;

std::array<char, COMMAND_BUFFER_SIZE> g_commandBuffer{};
std::size_t g_commandLength = 0U;
bool g_initialized = false;
bool g_promptVisible = false;

const char* StateToString(
    const WashTrac::StateMachine::SystemState state)
{
    using WashTrac::StateMachine::SystemState;

    switch (state)
    {
        case SystemState::Idle:
            return "Idle";

        case SystemState::StartingWash:
            return "StartingWash";

        case SystemState::WaitingForBusy:
            return "WaitingForBusy";

        case SystemState::WashRunning:
            return "WashRunning";

        case SystemState::ReleaseDelay:
            return "ReleaseDelay";

        case SystemState::QueueDelay:
            return "QueueDelay";

        case SystemState::RetryDelay:
            return "RetryDelay";

        case SystemState::Fault:
            return "Fault";

        case SystemState::EStop:
            return "EStop";

        default:
            return "Unknown";
    }
}

const char* FaultToString(
    const WashTrac::Faults::FaultCode code)
{
    using WashTrac::Faults::FaultCode;

    switch (code)
    {
        case FaultCode::None:
            return "None";

        case FaultCode::WashStartTimeout:
            return "Wash Start Timeout";

        case FaultCode::RelaySchedulerFailure:
            return "Relay Scheduler Failure";

        case FaultCode::QueueFailure:
            return "Queue Failure";

        case FaultCode::ConfigurationFailure:
            return "Configuration Failure";

        case FaultCode::EStopActive:
            return "E-Stop Active";

        case FaultCode::InvalidState:
            return "Invalid State";

        case FaultCode::InternalError:
            return "Internal Error";

        default:
            return "Unknown";
    }
}

const char* RelayStateToString(
    const WashTrac::Relays::RelayState state)
{
    using WashTrac::Relays::RelayState;

    switch (state)
    {
        case RelayState::Idle:
            return "Idle";

        case RelayState::WaitingOnDelay:
            return "WaitingOnDelay";

        case RelayState::Active:
            return "Active";

        case RelayState::WaitingOffDelay:
            return "WaitingOffDelay";

        default:
            return "Unknown";
    }
}

const char* EventToString(
    const WashTrac::Events::EventCode code)
{
    using WashTrac::Events::EventCode;

    switch (code)
    {
        case EventCode::None:
            return "None";

        case EventCode::Boot:
            return "Boot";

        case EventCode::ConfigurationLoaded:
            return "ConfigurationLoaded";

        case EventCode::ConfigurationSaved:
            return "ConfigurationSaved";

        case EventCode::FactoryReset:
            return "FactoryReset";

        case EventCode::WashQueued:
            return "WashQueued";

        case EventCode::WashDequeued:
            return "WashDequeued";

        case EventCode::WashStartAttempt:
            return "WashStartAttempt";

        case EventCode::WashStarted:
            return "WashStarted";

        case EventCode::WashCompleted:
            return "WashCompleted";

        case EventCode::RetryStarted:
            return "RetryStarted";

        case EventCode::FaultRaised:
            return "FaultRaised";

        case EventCode::FaultCleared:
            return "FaultCleared";

        case EventCode::EStopActive:
            return "EStopActive";

        case EventCode::EStopCleared:
            return "EStopCleared";

        case EventCode::QueueCleared:
            return "QueueCleared";

        case EventCode::SystemInitialized:
            return "SystemInitialized";

        default:
            return "Unknown";
    }
}

void PrintPrompt()
{
    std::printf("\r\nWashTrac> ");
    std::fflush(stdout);
    g_promptVisible = true;
}

void PrintHeader(
    const char* title)
{
    std::printf(
        "\r\n%s\r\n"
        "--------------------------------\r\n",
        title);
}

void PrintHelp()
{
    PrintHeader("WashTrac Core Service Console");

    std::printf(
        "help       Show available commands\r\n"
        "version    Show firmware and hardware versions\r\n"
        "status     Show consolidated controller status\r\n"
        "health     Show runtime health information\r\n"
        "state      Show state-machine status\r\n"
        "queue      Show pending wash queue status\r\n"
        "faults     Show current and previous faults\r\n"
        "inputs     Show all input states\r\n"
        "relays     Show all relay states\r\n"
        "config     Show active configuration\r\n"
        "events     Show stored runtime events\r\n");
}

void PrintVersion()
{
    PrintHeader("Version");

    std::printf(
        "Project.............%s\r\n"
        "Firmware............%s\r\n"
        "Hardware Revision...%u\r\n"
        "Config Version......%u\r\n",
        WashTrac::PROJECT_NAME,
        WashTrac::FIRMWARE_VERSION,
        static_cast<unsigned>(WashTrac::HARDWARE_REVISION),
        static_cast<unsigned>(WashTrac::CONFIG_VERSION));
}

void PrintState()
{
    PrintHeader("State");

    std::printf(
        "Current State.......%s\r\n"
        "Busy................%s\r\n"
        "Faulted.............%s\r\n"
        "Retry Count.........%u\r\n",
        StateToString(WashTrac::StateMachine::GetState()),
        WashTrac::StateMachine::IsBusy() ? "Yes" : "No",
        WashTrac::StateMachine::IsFaulted() ? "Yes" : "No",
        static_cast<unsigned>(
            WashTrac::StateMachine::GetRetryCount()));
}

void PrintQueue()
{
    PrintHeader("Queue");

    std::printf(
        "Pending Washes......%u\r\n"
        "Maximum.............%u\r\n"
        "Empty...............%s\r\n"
        "Full................%s\r\n",
        static_cast<unsigned>(WashTrac::WashQueue::Count()),
        static_cast<unsigned>(
            WashTrac::WashQueue::MAX_PENDING_WASHES),
        WashTrac::WashQueue::IsEmpty() ? "Yes" : "No",
        WashTrac::WashQueue::IsFull() ? "Yes" : "No");
}

void PrintHealth()
{
    const WashTrac::SystemHealth::Snapshot& health =
        WashTrac::SystemHealth::GetSnapshot();

    PrintHeader("System Health");

    std::printf(
        "Uptime..............%llu ms\r\n"
        "Free Heap...........%lu bytes\r\n"
        "Minimum Free Heap...%lu bytes\r\n"
        "Largest Block.......%lu bytes\r\n"
        "Stack High-Water....%lu bytes\r\n"
        "Last Loop...........%lu us\r\n"
        "Maximum Loop........%lu us\r\n"
        "Loop Overruns.......%lu\r\n",
        static_cast<unsigned long long>(
            health.uptimeMilliseconds),
        static_cast<unsigned long>(
            health.freeHeapBytes),
        static_cast<unsigned long>(
            health.minimumFreeHeapBytes),
        static_cast<unsigned long>(
            health.largestFreeHeapBlockBytes),
        static_cast<unsigned long>(
            health.mainTaskStackHighWaterBytes),
        static_cast<unsigned long>(
            health.lastLoopDurationMicroseconds),
        static_cast<unsigned long>(
            health.maximumLoopDurationMicroseconds),
        static_cast<unsigned long>(
            health.loopDeadlineOverrunCount));
}

void PrintFaults()
{
    const WashTrac::Faults::FaultStatus& current =
        WashTrac::Faults::GetStatus();

    const WashTrac::Faults::FaultStatus& previous =
        WashTrac::Faults::GetPreviousStatus();

    PrintHeader("Faults");

    std::printf(
        "Current Code........%s\r\n"
        "Current Active......%s\r\n"
        "Current Occurrence..%lu\r\n"
        "Current Timestamp...%lu\r\n"
        "Previous Code.......%s\r\n"
        "Previous Active.....%s\r\n"
        "Previous Occurrence.%lu\r\n"
        "Previous Timestamp..%lu\r\n",
        FaultToString(current.code),
        current.active ? "Yes" : "No",
        static_cast<unsigned long>(
            current.occurrenceCount),
        static_cast<unsigned long>(
            current.timestamp),
        FaultToString(previous.code),
        previous.active ? "Yes" : "No",
        static_cast<unsigned long>(
            previous.occurrenceCount),
        static_cast<unsigned long>(
            previous.timestamp));
}

void PrintInputs()
{
    PrintHeader("Inputs");

    for (uint8_t inputNumber = 1U;
         inputNumber <= WashTrac::INPUT_COUNT;
         ++inputNumber)
    {
        const bool active =
            WashTrac::Inputs::ReadInput(inputNumber);

        std::printf(
            "Input %u.............%s\r\n",
            static_cast<unsigned>(inputNumber),
            active ? "Active" : "Inactive");
    }

    std::printf(
        "Wash Busy...........%s\r\n"
        "E-Stop..............%s\r\n",
        WashTrac::Inputs::IsWashBusy()
            ? "Active"
            : "Inactive",
        WashTrac::Inputs::IsEStopActive()
            ? "Active"
            : "Healthy");
}

void PrintRelays()
{
    PrintHeader("Relays");

    for (uint8_t relayNumber = 1U;
         relayNumber <= WashTrac::RELAY_COUNT;
         ++relayNumber)
    {
        const auto relayId =
            static_cast<WashTrac::Relays::RelayId>(
                relayNumber);

        const auto state =
            WashTrac::Relays::GetState(relayId);

        std::printf(
            "Relay %u.............%-16s "
            "Active=%s Running=%s\r\n",
            static_cast<unsigned>(relayNumber),
            RelayStateToString(state),
            WashTrac::Relays::IsActive(relayId)
                ? "Yes"
                : "No",
            WashTrac::Relays::IsRunning(relayId)
                ? "Yes"
                : "No");
    }
}

void PrintConfig()
{
    const WashTrac::CoreConfig& config =
        WashTrac::ConfigurationManager::Get();

    PrintHeader("Configuration");

    std::printf(
        "Magic...............0x%08lX\r\n"
        "Config Version......%u\r\n"
        "Structure Size......%u\r\n"
        "CRC32...............0x%08lX\r\n"
        "Busy Release Delay..%u s\r\n"
        "Inter-Wash Delay....%u s\r\n"
        "Retry Delay.........%u s\r\n"
        "Maximum Attempts....%u\r\n",
        static_cast<unsigned long>(
            config.header.magic),
        static_cast<unsigned>(
            config.header.configVersion),
        static_cast<unsigned>(
            config.header.structureSize),
        static_cast<unsigned long>(
            config.header.crc32),
        static_cast<unsigned>(
            config.washBusyReleaseDelaySeconds),
        static_cast<unsigned>(
            config.interWashDelaySeconds),
        static_cast<unsigned>(
            config.washStartRetryDelaySeconds),
        static_cast<unsigned>(
            config.washStartMaxAttempts));

    std::printf("\r\nRelay Configuration\r\n");

    for (std::size_t index = 0U;
         index < WashTrac::RELAY_COUNT;
         ++index)
    {
        const WashTrac::RelayConfig& relay =
            config.relays[index];

        std::printf(
            "%u: %-16s Enabled=%s On=%u Duration=%u Off=%u\r\n",
            static_cast<unsigned>(index + 1U),
            relay.name.data(),
            relay.enabled ? "Yes" : "No",
            static_cast<unsigned>(
                relay.onDelaySeconds),
            static_cast<unsigned>(
                relay.durationSeconds),
            static_cast<unsigned>(
                relay.offDelaySeconds));
    }

    std::printf("\r\nInput Configuration\r\n");

    for (std::size_t index = 0U;
         index < WashTrac::INPUT_COUNT;
         ++index)
    {
        const WashTrac::InputConfig& input =
            config.inputs[index];

        std::printf(
            "%u: %-16s Enabled=%s Inverted=%s\r\n",
            static_cast<unsigned>(index + 1U),
            input.name.data(),
            input.enabled ? "Yes" : "No",
            input.inverted ? "Yes" : "No");
    }
}

void PrintEvents()
{
    PrintHeader("Events");

    const std::size_t eventCount =
        WashTrac::Events::Count();

    std::printf(
        "Stored Events.......%u\r\n"
        "Capacity............%u\r\n"
        "Full................%s\r\n\r\n",
        static_cast<unsigned>(eventCount),
        static_cast<unsigned>(
            WashTrac::Events::MAX_EVENTS),
        WashTrac::Events::IsFull() ? "Yes" : "No");

    for (std::size_t index = 0U;
         index < eventCount;
         ++index)
    {
        WashTrac::Events::Event event{};

        if (!WashTrac::Events::Get(index, event))
            continue;

        std::printf(
            "%03u  Tick=%lu  %-22s Value=%lu\r\n",
            static_cast<unsigned>(index),
            static_cast<unsigned long>(
                event.timestamp),
            EventToString(event.code),
            static_cast<unsigned long>(
                event.value));
    }
}

void PrintStatus()
{
    PrintHeader("Controller Status");

    const WashTrac::SystemHealth::Snapshot& health =
        WashTrac::SystemHealth::GetSnapshot();

    std::printf(
        "State...............%s\r\n"
        "Wash Busy...........%s\r\n"
        "E-Stop..............%s\r\n"
        "Pending Washes......%u\r\n"
        "Active Fault........%s\r\n"
        "Free Heap...........%lu bytes\r\n"
        "Maximum Loop........%lu us\r\n"
        "Loop Overruns.......%lu\r\n",
        StateToString(WashTrac::StateMachine::GetState()),
        WashTrac::Inputs::IsWashBusy()
            ? "Active"
            : "Inactive",
        WashTrac::Inputs::IsEStopActive()
            ? "Active"
            : "Healthy",
        static_cast<unsigned>(
            WashTrac::WashQueue::Count()),
        FaultToString(
            WashTrac::Faults::GetCode()),
        static_cast<unsigned long>(
            health.freeHeapBytes),
        static_cast<unsigned long>(
            health.maximumLoopDurationMicroseconds),
        static_cast<unsigned long>(
            health.loopDeadlineOverrunCount));
}

void NormalizeCommand()
{
    std::size_t writeIndex = 0U;
    bool previousWasSpace = true;

    for (std::size_t readIndex = 0U;
         readIndex < g_commandLength;
         ++readIndex)
    {
        const unsigned char rawCharacter =
            static_cast<unsigned char>(
                g_commandBuffer[readIndex]);

        if (std::isspace(rawCharacter) != 0)
        {
            if (!previousWasSpace &&
                writeIndex < COMMAND_BUFFER_SIZE - 1U)
            {
                g_commandBuffer[writeIndex++] = ' ';
            }

            previousWasSpace = true;
            continue;
        }

        if (writeIndex < COMMAND_BUFFER_SIZE - 1U)
        {
            g_commandBuffer[writeIndex++] =
                static_cast<char>(
                    std::tolower(rawCharacter));
        }

        previousWasSpace = false;
    }

    if (writeIndex > 0U &&
        g_commandBuffer[writeIndex - 1U] == ' ')
    {
        --writeIndex;
    }

    g_commandBuffer[writeIndex] = '\0';
    g_commandLength = writeIndex;
}

void ExecuteCommand()
{
    NormalizeCommand();

    if (g_commandLength == 0U)
        return;

    if (std::strcmp(g_commandBuffer.data(), "help") == 0)
    {
        PrintHelp();
    }
    else if (std::strcmp(
                 g_commandBuffer.data(),
                 "version") == 0)
    {
        PrintVersion();
    }
    else if (std::strcmp(
                 g_commandBuffer.data(),
                 "status") == 0)
    {
        PrintStatus();
    }
    else if (std::strcmp(
                 g_commandBuffer.data(),
                 "health") == 0)
    {
        PrintHealth();
    }
    else if (std::strcmp(
                 g_commandBuffer.data(),
                 "state") == 0)
    {
        PrintState();
    }
    else if (std::strcmp(
                 g_commandBuffer.data(),
                 "queue") == 0)
    {
        PrintQueue();
    }
    else if (std::strcmp(
                 g_commandBuffer.data(),
                 "faults") == 0)
    {
        PrintFaults();
    }
    else if (std::strcmp(
                 g_commandBuffer.data(),
                 "inputs") == 0)
    {
        PrintInputs();
    }
    else if (std::strcmp(
                 g_commandBuffer.data(),
                 "relays") == 0)
    {
        PrintRelays();
    }
    else if (std::strcmp(
                 g_commandBuffer.data(),
                 "config") == 0)
    {
        PrintConfig();
    }
    else if (std::strcmp(
                 g_commandBuffer.data(),
                 "events") == 0)
    {
        PrintEvents();
    }
    else
    {
        std::printf(
            "\r\nUnknown command: %s\r\n"
            "Type 'help' for available commands.\r\n",
            g_commandBuffer.data());
    }
}

void ResetCommandBuffer()
{
    g_commandBuffer.fill('\0');
    g_commandLength = 0U;
}

void ProcessCharacter(
    const char character)
{
    if (character == '\r' ||
        character == '\n')
    {
        if (g_commandLength > 0U)
        {
            ExecuteCommand();
            ResetCommandBuffer();
        }

        PrintPrompt();
        return;
    }

    if (character == '\b' ||
        character == 0x7F)
    {
        if (g_commandLength > 0U)
        {
            --g_commandLength;
            g_commandBuffer[g_commandLength] = '\0';
            std::printf("\b \b");
            std::fflush(stdout);
        }

        return;
    }

    const unsigned char rawCharacter =
        static_cast<unsigned char>(character);

    if (std::isprint(rawCharacter) == 0)
        return;

    if (g_commandLength >= COMMAND_BUFFER_SIZE - 1U)
    {
        std::printf(
            "\r\nCommand too long. Maximum length is %u characters.\r\n",
            static_cast<unsigned>(
                COMMAND_BUFFER_SIZE - 1U));

        ResetCommandBuffer();
        PrintPrompt();
        return;
    }

    g_commandBuffer[g_commandLength++] = character;
    g_commandBuffer[g_commandLength] = '\0';

    std::printf("%c", character);
    std::fflush(stdout);
    g_promptVisible = false;
}

} // namespace

namespace WashTrac::ServiceConsole
{

Result Initialize()
{
    if (g_initialized)
        return Result::OK;

    if (!uart_is_driver_installed(CONSOLE_UART))
    {
        const esp_err_t installResult =
            uart_driver_install(
                CONSOLE_UART,
                static_cast<int>(RX_BUFFER_SIZE),
                0,
                0,
                nullptr,
                0);

        if (installResult != ESP_OK)
        {
            ESP_LOGE(
                LOG_TAG,
                "UART driver installation failed: %s",
                esp_err_to_name(installResult));

            return Result::ERROR;
        }
    }

    uart_config_t uartConfig{};

    uartConfig.baud_rate = CONSOLE_BAUD_RATE;
    uartConfig.data_bits = UART_DATA_8_BITS;
    uartConfig.parity = UART_PARITY_DISABLE;
    uartConfig.stop_bits = UART_STOP_BITS_1;
    uartConfig.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uartConfig.rx_flow_ctrl_thresh = 0U;
    uartConfig.source_clk = UART_SCLK_DEFAULT;

    esp_err_t result =
        uart_param_config(
            CONSOLE_UART,
            &uartConfig);

    if (result != ESP_OK)
    {
        ESP_LOGE(
            LOG_TAG,
            "UART configuration failed: %s",
            esp_err_to_name(result));

        return Result::ERROR;
    }

    ResetCommandBuffer();

    g_initialized = true;

    ESP_LOGI(
        LOG_TAG,
        "Service Console initialized on UART %u at %d baud.",
        static_cast<unsigned>(CONSOLE_UART),
        CONSOLE_BAUD_RATE);

    std::printf(
        "\r\nWashTrac Core Service Console\r\n"
        "Type 'help' for available commands.\r\n");

    PrintPrompt();

    return Result::OK;
}

void Update()
{
    if (!g_initialized)
        return;

    uint8_t receivedBytes[RX_BUFFER_SIZE]{};

    const int byteCount =
        uart_read_bytes(
            CONSOLE_UART,
            receivedBytes,
            sizeof(receivedBytes),
            UART_READ_TIMEOUT_TICKS);

    if (byteCount <= 0)
        return;

    for (int index = 0;
         index < byteCount;
         ++index)
    {
        ProcessCharacter(
            static_cast<char>(receivedBytes[index]));
    }
}

bool IsInitialized()
{
    return g_initialized;
}

} // namespace WashTrac::ServiceConsole