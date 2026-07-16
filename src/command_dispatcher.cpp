/******************************************************************************
 *
 *  Project:
 *      WashTrac Core
 *
 *  Module:
 *      CM5 Command Dispatcher
 *
 *  File:
 *      command_dispatcher.cpp
 *
 *  Description:
 *      Production fixed-memory dispatcher for commands received from the CM5.
 *
 *      Responsibilities:
 *          - Service the dedicated UART2 transport
 *          - Read complete newline-delimited messages
 *          - Parse and validate Version 1 JSON commands
 *          - Execute commands through existing production modules
 *          - Serialize and queue JSON responses
 *          - Use no dynamic memory allocation
 *
 *  Copyright:
 *      © 2026 WashTrac
 *
 ******************************************************************************/

#include "command_dispatcher.h"

#include "config.h"
#include "diagnostics_manager.h"
#include "fault_manager.h"
#include "input_manager.h"
#include "json_protocol.h"
#include "lte_manager.h"
#include "relay_scheduler.h"
#include "state_machine.h"
#include "system_health.h"
#include "uart_protocol.h"
#include "wash_queue.h"

#include "esp_log.h"
#include "esp_system.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace
{

constexpr const char* LOG_TAG = "CommandDispatcher";

std::array<char, WashTrac::UartProtocol::MAXIMUM_LINE_LENGTH>
    g_receiveBuffer{};

std::array<char, WashTrac::JsonProtocol::RESPONSE_LENGTH>
    g_responseBuffer{};

bool g_initialized = false;

class Writer
{
public:
    Writer(char* output, const std::size_t outputSize)
        : output_(output),
          outputSize_(outputSize),
          length_(0U),
          valid_(output != nullptr && outputSize > 0U)
    {
        if (valid_)
        {
            output_[0] = '\0';
        }
    }

    bool Append(const char* const text)
    {
        if (!valid_ || text == nullptr)
        {
            return false;
        }

        const std::size_t textLength = std::strlen(text);

        if (length_ + textLength >= outputSize_)
        {
            valid_ = false;
            return false;
        }

        std::memcpy(output_ + length_, text, textLength);
        length_ += textLength;
        output_[length_] = '\0';

        return true;
    }

    bool AppendUnsigned(const uint64_t value)
    {
        char number[32]{};

        const int count =
            std::snprintf(
                number,
                sizeof(number),
                "%llu",
                static_cast<unsigned long long>(value));

        if (count <= 0 ||
            static_cast<std::size_t>(count) >= sizeof(number))
        {
            valid_ = false;
            return false;
        }

        return Append(number);
    }

    bool AppendSigned(const int64_t value)
    {
        char number[32]{};

        const int count =
            std::snprintf(
                number,
                sizeof(number),
                "%lld",
                static_cast<long long>(value));

        if (count <= 0 ||
            static_cast<std::size_t>(count) >= sizeof(number))
        {
            valid_ = false;
            return false;
        }

        return Append(number);
    }

    bool AppendBoolean(const bool value)
    {
        return Append(value ? "true" : "false");
    }

    bool AppendEscapedString(const char* const text)
    {
        if (text == nullptr || !Append("\""))
        {
            return false;
        }

        for (std::size_t index = 0U;
             text[index] != '\0';
             ++index)
        {
            const unsigned char character =
                static_cast<unsigned char>(text[index]);

            switch (character)
            {
                case '"':
                    if (!Append("\\\"")) return false;
                    break;

                case '\\':
                    if (!Append("\\\\")) return false;
                    break;

                case '\b':
                    if (!Append("\\b")) return false;
                    break;

                case '\f':
                    if (!Append("\\f")) return false;
                    break;

                case '\n':
                    if (!Append("\\n")) return false;
                    break;

                case '\r':
                    if (!Append("\\r")) return false;
                    break;

                case '\t':
                    if (!Append("\\t")) return false;
                    break;

                default:
                {
                    if (character < 0x20U)
                    {
                        valid_ = false;
                        return false;
                    }

                    char characterText[2] =
                    {
                        static_cast<char>(character),
                        '\0'
                    };

                    if (!Append(characterText))
                    {
                        return false;
                    }

                    break;
                }
            }
        }

        return Append("\"");
    }

    bool IsValid() const
    {
        return valid_;
    }

private:
    char* output_;
    std::size_t outputSize_;
    std::size_t length_;
    bool valid_;
};

const char* RequestId(
    const WashTrac::JsonProtocol::Message& message)
{
    return message.hasRequestId
        ? message.requestId
        : nullptr;
}

const char* StateToString(
    const WashTrac::StateMachine::SystemState state)
{
    using WashTrac::StateMachine::SystemState;

    switch (state)
    {
        case SystemState::Idle:
            return "idle";

        case SystemState::StartingWash:
            return "starting_wash";

        case SystemState::WaitingForBusy:
            return "waiting_for_busy";

        case SystemState::WashRunning:
            return "wash_running";

        case SystemState::ReleaseDelay:
            return "release_delay";

        case SystemState::QueueDelay:
            return "queue_delay";

        case SystemState::RetryDelay:
            return "retry_delay";

        case SystemState::Fault:
            return "fault";

        case SystemState::EStop:
            return "e_stop";

        default:
            return "unknown";
    }
}

const char* LteStateToString(
    const WashTrac::LTE::ModemState state)
{
    using WashTrac::LTE::ModemState;

    switch (state)
    {
        case ModemState::Off:
            return "off";

        case ModemState::Initializing:
            return "initializing";

        case ModemState::Ready:
            return "ready";

        case ModemState::Registering:
            return "registering";

        case ModemState::Online:
            return "online";

        case ModemState::Error:
            return "error";

        default:
            return "unknown";
    }
}

const char* ResultToError(const WashTrac::Result result)
{
    using WashTrac::Result;

    switch (result)
    {
        case Result::INVALID_PARAMETER:
            return "invalid_parameter";

        case Result::NOT_INITIALIZED:
            return "not_initialized";

        case Result::STORAGE_FAILURE:
            return "storage_failure";

        case Result::CRC_FAILURE:
            return "crc_failure";

        case Result::TIMEOUT:
            return "timeout";

        case Result::UNSUPPORTED:
            return "unsupported";

        case Result::ERROR:
        default:
            return "operation_failed";
    }
}

WashTrac::Result SendResponse()
{
    return WashTrac::UartProtocol::WriteLine(
        g_responseBuffer.data());
}

void SendStandardError(
    const WashTrac::JsonProtocol::Message& message,
    const char* const operation,
    const char* const error)
{
    const WashTrac::Result serializationResult =
        WashTrac::JsonProtocol::WriteError(
            RequestId(message),
            operation,
            error,
            g_responseBuffer.data(),
            g_responseBuffer.size());

    if (serializationResult != WashTrac::Result::OK)
    {
        ESP_LOGE(
            LOG_TAG,
            "Unable to serialize error response for %s.",
            operation);

        return;
    }

    if (SendResponse() != WashTrac::Result::OK)
    {
        ESP_LOGW(
            LOG_TAG,
            "Unable to queue error response for %s.",
            operation);
    }
}

void SendStandardResult(
    const WashTrac::JsonProtocol::Message& message,
    const char* const operation,
    const WashTrac::Result result)
{
    const WashTrac::Result serializationResult =
        result == WashTrac::Result::OK
            ? WashTrac::JsonProtocol::WriteOk(
                  RequestId(message),
                  operation,
                  g_responseBuffer.data(),
                  g_responseBuffer.size())
            : WashTrac::JsonProtocol::WriteError(
                  RequestId(message),
                  operation,
                  ResultToError(result),
                  g_responseBuffer.data(),
                  g_responseBuffer.size());

    if (serializationResult != WashTrac::Result::OK)
    {
        ESP_LOGE(
            LOG_TAG,
            "Unable to serialize response for %s.",
            operation);

        return;
    }

    if (SendResponse() != WashTrac::Result::OK)
    {
        ESP_LOGW(
            LOG_TAG,
            "Unable to queue response for %s.",
            operation);
    }
}

bool BeginDataResponse(
    Writer& writer,
    const WashTrac::JsonProtocol::Message& message,
    const char* const operation)
{
    if (!writer.Append("{\"type\":\"ok\""))
    {
        return false;
    }

    if (message.hasRequestId)
    {
        if (!writer.Append(",\"request_id\":") ||
            !writer.AppendEscapedString(message.requestId))
        {
            return false;
        }
    }

    return writer.Append(",\"operation\":") &&
           writer.AppendEscapedString(operation);
}

void SendBuiltResponse(
    const WashTrac::JsonProtocol::Message& message,
    const char* const operation,
    const Writer& writer)
{
    if (!writer.IsValid())
    {
        SendStandardError(
            message,
            operation,
            "response_too_large");

        return;
    }

    if (SendResponse() != WashTrac::Result::OK)
    {
        ESP_LOGW(
            LOG_TAG,
            "Unable to queue response for %s.",
            operation);
    }
}

void HandlePing(
    const WashTrac::JsonProtocol::Message& message)
{
    Writer writer(
        g_responseBuffer.data(),
        g_responseBuffer.size());

    BeginDataResponse(writer, message, "ping");
    writer.Append(",\"firmware\":");
    writer.AppendEscapedString(WashTrac::FIRMWARE_VERSION);
    writer.Append(",\"hardware_revision\":");
    writer.AppendUnsigned(WashTrac::HARDWARE_REVISION);
    writer.Append("}");

    SendBuiltResponse(message, "ping", writer);
}

void HandleStatus(
    const WashTrac::JsonProtocol::Message& message)
{
    const WashTrac::SystemHealth::Snapshot& health =
        WashTrac::SystemHealth::GetSnapshot();

    Writer writer(
        g_responseBuffer.data(),
        g_responseBuffer.size());

    BeginDataResponse(writer, message, "status");

    writer.Append(",\"state\":");
    writer.AppendEscapedString(
        StateToString(
            WashTrac::StateMachine::GetState()));

    writer.Append(",\"busy\":");
    writer.AppendBoolean(
        WashTrac::StateMachine::IsBusy());

    writer.Append(",\"wash_busy_input\":");
    writer.AppendBoolean(
        WashTrac::Inputs::IsWashBusy());

    writer.Append(",\"e_stop\":");
    writer.AppendBoolean(
        WashTrac::Inputs::IsEStopActive());

    writer.Append(",\"fault_active\":");
    writer.AppendBoolean(
        WashTrac::Faults::IsActive());

    writer.Append(",\"fault_code\":");
    writer.AppendUnsigned(
        static_cast<uint8_t>(
            WashTrac::Faults::GetCode()));

    writer.Append(",\"queue_count\":");
    writer.AppendUnsigned(
        WashTrac::WashQueue::Count());

    writer.Append(",\"queue_full\":");
    writer.AppendBoolean(
        WashTrac::WashQueue::IsFull());

    writer.Append(",\"retry_count\":");
    writer.AppendUnsigned(
        WashTrac::StateMachine::GetRetryCount());

    writer.Append(",\"uptime_ms\":");
    writer.AppendUnsigned(
        health.uptimeMilliseconds);

    writer.Append("}");

    SendBuiltResponse(message, "status", writer);
}

void HandleStartWash(
    const WashTrac::JsonProtocol::Message& message)
{
    if (WashTrac::Inputs::IsEStopActive() ||
        WashTrac::StateMachine::GetState() ==
            WashTrac::StateMachine::SystemState::EStop)
    {
        SendStandardError(
            message,
            "start_wash",
            "e_stop_active");

        return;
    }

    if (WashTrac::WashQueue::IsFull())
    {
        SendStandardError(
            message,
            "start_wash",
            "queue_full");

        return;
    }

    const WashTrac::Result result =
        WashTrac::WashQueue::Enqueue();

    if (result != WashTrac::Result::OK)
    {
        SendStandardResult(
            message,
            "start_wash",
            result);

        return;
    }

    Writer writer(
        g_responseBuffer.data(),
        g_responseBuffer.size());

    BeginDataResponse(writer, message, "start_wash");
    writer.Append(",\"queue_count\":");
    writer.AppendUnsigned(
        WashTrac::WashQueue::Count());
    writer.Append("}");

    SendBuiltResponse(message, "start_wash", writer);
}

void HandleQueueStatus(
    const WashTrac::JsonProtocol::Message& message)
{
    Writer writer(
        g_responseBuffer.data(),
        g_responseBuffer.size());

    BeginDataResponse(writer, message, "queue_status");

    writer.Append(",\"count\":");
    writer.AppendUnsigned(
        WashTrac::WashQueue::Count());

    writer.Append(",\"maximum\":");
    writer.AppendUnsigned(
        WashTrac::WashQueue::MAX_PENDING_WASHES);

    writer.Append(",\"empty\":");
    writer.AppendBoolean(
        WashTrac::WashQueue::IsEmpty());

    writer.Append(",\"full\":");
    writer.AppendBoolean(
        WashTrac::WashQueue::IsFull());

    writer.Append("}");

    SendBuiltResponse(message, "queue_status", writer);
}


void AppendUnsignedArray(
    Writer& writer,
    const std::array<uint32_t, WashTrac::INPUT_COUNT>& values)
{
    writer.Append("[");

    for (std::size_t index = 0U;
         index < values.size();
         ++index)
    {
        if (index > 0U)
        {
            writer.Append(",");
        }

        writer.AppendUnsigned(values[index]);
    }

    writer.Append("]");
}

void AppendRelayStateArray(
    Writer& writer,
    const std::array<
        WashTrac::Relays::RelayState,
        WashTrac::RELAY_COUNT>& values)
{
    writer.Append("[");

    for (std::size_t index = 0U;
         index < values.size();
         ++index)
    {
        if (index > 0U)
        {
            writer.Append(",");
        }

        writer.AppendUnsigned(
            static_cast<uint8_t>(values[index]));
    }

    writer.Append("]");
}

void AppendRelayCountArray(
    Writer& writer,
    const std::array<uint32_t, WashTrac::RELAY_COUNT>& values)
{
    writer.Append("[");

    for (std::size_t index = 0U;
         index < values.size();
         ++index)
    {
        if (index > 0U)
        {
            writer.Append(",");
        }

        writer.AppendUnsigned(values[index]);
    }

    writer.Append("]");
}

void HandleDiagnostics(
    const WashTrac::JsonProtocol::Message& message)
{
    const WashTrac::Diagnostics::Snapshot& snapshot =
        WashTrac::Diagnostics::GetSnapshot();

    const WashTrac::CoreConfig& config =
        WashTrac::ConfigurationManager::Get();

    Writer writer(
        g_responseBuffer.data(),
        g_responseBuffer.size());

    BeginDataResponse(writer, message, "diagnostics");

    writer.Append(",\"state\":");
    writer.AppendUnsigned(
        static_cast<uint8_t>(
            snapshot.systemState));

    writer.Append(",\"retry_count\":");
    writer.AppendUnsigned(
        snapshot.retryCount);

    writer.Append(",\"queue_count\":");
    writer.AppendUnsigned(
        snapshot.pendingWashCount);

    writer.Append(",\"fault_code\":");
    writer.AppendUnsigned(
        static_cast<uint8_t>(
            snapshot.currentFault.code));

    writer.Append(",\"fault_active\":");
    writer.AppendBoolean(
        snapshot.currentFault.active);

    /*
     * Production input diagnostics payload.
     *
     * Each entry combines the current logical input state from the
     * Diagnostics Manager with the live configuration held by the
     * Configuration Manager. This gives the CM5 everything required to
     * display ACTIVE, INACTIVE, and E-STOP without additional commands.
     */
    writer.Append(",\"inputs\":[");

    for (std::size_t index = 0U;
         index < config.inputs.size();
         ++index)
    {
        if (index > 0U)
        {
            writer.Append(",");
        }

        const WashTrac::InputConfig& input =
            config.inputs[index];

        writer.Append("{\"number\":");
        writer.AppendUnsigned(index + 1U);

        writer.Append(",\"name\":");
        writer.AppendEscapedString(
            input.name.data());

        writer.Append(",\"enabled\":");
        writer.AppendBoolean(
            input.enabled);

        writer.Append(",\"inverted\":");
        writer.AppendBoolean(
            input.inverted);

        writer.Append(",\"state\":");
        writer.AppendBoolean(
            snapshot.inputStates[index]);

        writer.Append("}");
    }

    writer.Append("]");

    writer.Append(",\"input_transitions\":");
    AppendUnsignedArray(
        writer,
        snapshot.inputTransitionCounts);

    writer.Append(",\"relay_states\":");
    AppendRelayStateArray(
        writer,
        snapshot.relayStates);

    writer.Append(",\"relay_activations\":");
    AppendRelayCountArray(
        writer,
        snapshot.relayActivationCounts);

    writer.Append(",\"free_heap\":");
    writer.AppendUnsigned(
        snapshot.systemHealth.freeHeapBytes);

    writer.Append(",\"min_free_heap\":");
    writer.AppendUnsigned(
        snapshot.systemHealth.minimumFreeHeapBytes);

    writer.Append(",\"max_loop_us\":");
    writer.AppendUnsigned(
        snapshot.systemHealth.maximumLoopDurationMicroseconds);

    writer.Append(",\"overruns\":");
    writer.AppendUnsigned(
        snapshot.systemHealth.loopDeadlineOverrunCount);

    writer.Append(",\"updates\":");
    writer.AppendUnsigned(
        snapshot.updateCount);

    writer.Append("}");

    SendBuiltResponse(message, "diagnostics", writer);
}

void HandleGetConfig(
    const WashTrac::JsonProtocol::Message& message)
{
    const WashTrac::CoreConfig& config =
        WashTrac::ConfigurationManager::Get();

    Writer writer(
        g_responseBuffer.data(),
        g_responseBuffer.size());

    BeginDataResponse(writer, message, "get_config");

    writer.Append(",\"relays\":[");

    for (std::size_t index = 0U;
         index < config.relays.size();
         ++index)
    {
        if (index > 0U)
        {
            writer.Append(",");
        }

        const WashTrac::RelayConfig& relay =
            config.relays[index];

        writer.Append("[");
        writer.AppendUnsigned(index + 1U);
        writer.Append(",");
        writer.AppendBoolean(relay.enabled);
        writer.Append(",");
        writer.AppendEscapedString(relay.name.data());
        writer.Append(",");
        writer.AppendUnsigned(relay.onDelaySeconds);
        writer.Append(",");
        writer.AppendUnsigned(relay.durationSeconds);
        writer.Append(",");
        writer.AppendUnsigned(relay.offDelaySeconds);
        writer.Append("]");
    }

    writer.Append("],\"inputs\":[");

    for (std::size_t index = 0U;
         index < config.inputs.size();
         ++index)
    {
        if (index > 0U)
        {
            writer.Append(",");
        }

        const WashTrac::InputConfig& input =
            config.inputs[index];

        writer.Append("[");
        writer.AppendUnsigned(index + 1U);
        writer.Append(",");
        writer.AppendBoolean(input.enabled);
        writer.Append(",");
        writer.AppendBoolean(input.inverted);
        writer.Append(",");
        writer.AppendEscapedString(input.name.data());
        writer.Append("]");
    }

    writer.Append("],\"busy_release_delay\":");
    writer.AppendUnsigned(
        config.washBusyReleaseDelaySeconds);

    writer.Append(",\"inter_wash_delay\":");
    writer.AppendUnsigned(
        config.interWashDelaySeconds);

    writer.Append(",\"retry_delay\":");
    writer.AppendUnsigned(
        config.washStartRetryDelaySeconds);

    writer.Append(",\"max_attempts\":");
    writer.AppendUnsigned(
        config.washStartMaxAttempts);

    writer.Append("}");

    SendBuiltResponse(message, "get_config", writer);
}

void HandleSetRelay(
    const WashTrac::JsonProtocol::Message& message)
{
    const WashTrac::Result result =
        WashTrac::ConfigurationManager::SetRelay(
            message.relayNumber,
            message.enabled,
            message.name,
            message.onDelaySeconds,
            message.durationSeconds,
            message.offDelaySeconds);

    SendStandardResult(
        message,
        "set_relay",
        result);
}

void HandleSetInput(
    const WashTrac::JsonProtocol::Message& message)
{
    const WashTrac::Result result =
        WashTrac::ConfigurationManager::SetInput(
            message.inputNumber,
            message.enabled,
            message.inverted,
            message.name);

    SendStandardResult(
        message,
        "set_input",
        result);
}

void HandleSetTiming(
    const WashTrac::JsonProtocol::Message& message)
{
    WashTrac::Result result =
        WashTrac::Result::OK;

    if (message.hasWashBusyReleaseDelaySeconds)
    {
        result =
            WashTrac::ConfigurationManager::
                SetWashBusyReleaseDelay(
                    message.washBusyReleaseDelaySeconds);
    }

    if (result == WashTrac::Result::OK &&
        message.hasInterWashDelaySeconds)
    {
        result =
            WashTrac::ConfigurationManager::
                SetInterWashDelay(
                    message.interWashDelaySeconds);
    }

    SendStandardResult(
        message,
        "set_timing",
        result);
}

void HandleSaveConfig(
    const WashTrac::JsonProtocol::Message& message)
{
    const WashTrac::Result result =
        WashTrac::ConfigurationManager::Save();

    SendStandardResult(
        message,
        "save_config",
        result);
}


void HandleLteStatus(
    const WashTrac::JsonProtocol::Message& message)
{
    const WashTrac::LTE::Status& status =
        WashTrac::LTE::GetStatus();

    Writer writer(
        g_responseBuffer.data(),
        g_responseBuffer.size());

    BeginDataResponse(writer, message, "lte_status");

    writer.Append(",\"state\":");
    writer.AppendEscapedString(
        LteStateToString(status.state));

    writer.Append(",\"modem_present\":");
    writer.AppendBoolean(status.modemPresent);

    writer.Append(",\"sim_present\":");
    writer.AppendBoolean(status.simPresent);

    writer.Append(",\"registered\":");
    writer.AppendBoolean(status.registered);

    writer.Append(",\"online\":");
    writer.AppendBoolean(
        WashTrac::LTE::IsOnline());

    writer.Append(",\"carrier\":");
    writer.AppendEscapedString(status.carrier);

    writer.Append(",\"ip_address\":");
    writer.AppendEscapedString(status.ipAddress);

    writer.Append(",\"uptime_seconds\":");
    writer.AppendUnsigned(status.uptimeSeconds);

    writer.Append("}");

    SendBuiltResponse(message, "lte_status", writer);
}

void HandleLteSignal(
    const WashTrac::JsonProtocol::Message& message)
{
    const WashTrac::LTE::Status& status =
        WashTrac::LTE::GetStatus();

    Writer writer(
        g_responseBuffer.data(),
        g_responseBuffer.size());

    BeginDataResponse(writer, message, "lte_signal");

    writer.Append(",\"rssi\":");
    writer.AppendSigned(status.rssi);

    writer.Append(",\"ber\":");
    writer.AppendUnsigned(status.ber);

    writer.Append(",\"registered\":");
    writer.AppendBoolean(status.registered);

    writer.Append(",\"online\":");
    writer.AppendBoolean(
        WashTrac::LTE::IsOnline());

    writer.Append("}");

    SendBuiltResponse(message, "lte_signal", writer);
}

void HandleLteInfo(
    const WashTrac::JsonProtocol::Message& message)
{
    const WashTrac::LTE::Status& status =
        WashTrac::LTE::GetStatus();

    Writer writer(
        g_responseBuffer.data(),
        g_responseBuffer.size());

    BeginDataResponse(writer, message, "lte_info");

    writer.Append(",\"state\":");
    writer.AppendEscapedString(
        LteStateToString(status.state));

    writer.Append(",\"imei\":");
    writer.AppendEscapedString(status.imei);

    writer.Append(",\"iccid\":");
    writer.AppendEscapedString(status.iccid);

    writer.Append(",\"carrier\":");
    writer.AppendEscapedString(status.carrier);

    writer.Append(",\"ip_address\":");
    writer.AppendEscapedString(status.ipAddress);

    writer.Append(",\"modem_present\":");
    writer.AppendBoolean(status.modemPresent);

    writer.Append(",\"sim_present\":");
    writer.AppendBoolean(status.simPresent);

    writer.Append(",\"registered\":");
    writer.AppendBoolean(status.registered);

    writer.Append("}");

    SendBuiltResponse(message, "lte_info", writer);
}

void HandleLteRestart(
    const WashTrac::JsonProtocol::Message& message)
{
    WashTrac::LTE::Reset();

    SendStandardResult(
        message,
        "lte_restart",
        WashTrac::Result::OK);
}

void HandleFactoryReset(
    const WashTrac::JsonProtocol::Message& message)
{
    const WashTrac::Result result =
        WashTrac::ConfigurationManager::ResetFactory();

    SendStandardResult(
        message,
        "factory_reset",
        result);

    if (result == WashTrac::Result::OK)
    {
        vTaskDelay(pdMS_TO_TICKS(250));
        esp_restart();
    }
}

void Dispatch(
    const WashTrac::JsonProtocol::Message& message)
{
    using WashTrac::JsonProtocol::Command;

    switch (message.command)
    {
        case Command::Ping:
            HandlePing(message);
            break;

        case Command::Status:
            HandleStatus(message);
            break;

        case Command::StartWash:
            HandleStartWash(message);
            break;

        case Command::QueueStatus:
            HandleQueueStatus(message);
            break;

        case Command::Diagnostics:
            HandleDiagnostics(message);
            break;

        case Command::GetConfig:
            HandleGetConfig(message);
            break;

        case Command::SetRelay:
            HandleSetRelay(message);
            break;

        case Command::SetInput:
            HandleSetInput(message);
            break;

        case Command::SetTiming:
            HandleSetTiming(message);
            break;

        case Command::SaveConfig:
            HandleSaveConfig(message);
            break;

        case Command::FactoryReset:
            HandleFactoryReset(message);
            break;

        case Command::LteStatus:
            HandleLteStatus(message);
            break;

        case Command::LteSignal:
            HandleLteSignal(message);
            break;

        case Command::LteInfo:
            HandleLteInfo(message);
            break;

        case Command::LteRestart:
            HandleLteRestart(message);
            break;

        case Command::Unknown:
        default:
            SendStandardError(
                message,
                "unknown",
                "unsupported_command");
            break;
    }
}

void HandleLine(const char* const line)
{
    WashTrac::JsonProtocol::Message message{};

    const WashTrac::Result parseResult =
        WashTrac::JsonProtocol::Parse(
            line,
            message);

    if (parseResult != WashTrac::Result::OK)
    {
        const WashTrac::Result serializationResult =
            WashTrac::JsonProtocol::WriteError(
                nullptr,
                "parse",
                "invalid_message",
                g_responseBuffer.data(),
                g_responseBuffer.size());

        if (serializationResult == WashTrac::Result::OK)
        {
            if (SendResponse() != WashTrac::Result::OK)
            {
                ESP_LOGW(
                    LOG_TAG,
                    "Unable to queue invalid-message response.");
            }
        }

        return;
    }

    Dispatch(message);
}

} // namespace

namespace WashTrac::CommandDispatcher
{

Result Initialize()
{
    if (g_initialized)
    {
        return Result::OK;
    }

    if (!UartProtocol::IsInitialized() ||
        !ConfigurationManager::IsInitialized() ||
        !Inputs::IsInitialized() ||
        !Relays::IsInitialized() ||
        !WashQueue::IsInitialized() ||
        !Faults::IsInitialized() ||
        !SystemHealth::IsInitialized() ||
        !Diagnostics::IsInitialized())
    {
        ESP_LOGE(
            LOG_TAG,
            "Initialization failed because a required subsystem "
            "is not initialized.");

        return Result::NOT_INITIALIZED;
    }

    g_receiveBuffer.fill('\0');
    g_responseBuffer.fill('\0');

    g_initialized = true;

    ESP_LOGI(
        LOG_TAG,
        "CM5 Command Dispatcher initialized.");

    return Result::OK;
}

void Update()
{
    if (!g_initialized)
    {
        return;
    }

    UartProtocol::Update();

    while (UartProtocol::ReadLine(
               g_receiveBuffer.data(),
               g_receiveBuffer.size()))
    {
        HandleLine(g_receiveBuffer.data());
        g_receiveBuffer.fill('\0');
    }

    /*
     * Run the transport again so a response queued during this update can
     * begin transmitting without waiting for the next system-loop iteration.
     */
    UartProtocol::Update();
}

bool IsInitialized()
{
    return g_initialized;
}

} // namespace WashTrac::CommandDispatcher