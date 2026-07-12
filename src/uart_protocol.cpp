/******************************************************************************
 *
 *  Project:
 *      WashTrac Core
 *
 *  Module:
 *      UART Protocol Transport
 *
 *  File:
 *      uart_protocol.cpp
 *
 *  Description:
 *      Production fixed-memory UART transport for CM5 communication.
 *
 *      The transport provides:
 *          - Dedicated UART2 operation at 115200 baud, 8-N-1
 *          - GPIO assignment through the central Hardware Definition
 *          - Non-blocking receive and transmit processing
 *          - Newline-delimited message framing
 *          - Fixed-depth receive and transmit queues
 *          - Oversized-line rejection and recovery
 *          - No dynamic memory allocation
 *
 *  Copyright:
 *      © 2026 WashTrac
 *
 ******************************************************************************/

#include "uart_protocol.h"

#include "hardware.h"

#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace
{

constexpr const char* LOG_TAG = "UartProtocol";

constexpr std::size_t UART_RX_RING_BUFFER_SIZE = 1024U;
constexpr std::size_t RECEIVE_CHUNK_SIZE = 128U;
constexpr std::size_t MESSAGE_QUEUE_DEPTH = 4U;
constexpr TickType_t UART_READ_TIMEOUT_TICKS = 0U;

struct Message
{
    std::array<char, WashTrac::UartProtocol::MAXIMUM_LINE_LENGTH> data{};
    std::size_t length = 0U;
};

std::array<Message, MESSAGE_QUEUE_DEPTH> g_receiveQueue{};
std::size_t g_receiveHead = 0U;
std::size_t g_receiveTail = 0U;
std::size_t g_receiveCount = 0U;

std::array<Message, MESSAGE_QUEUE_DEPTH> g_transmitQueue{};
std::size_t g_transmitHead = 0U;
std::size_t g_transmitTail = 0U;
std::size_t g_transmitCount = 0U;
std::size_t g_transmitOffset = 0U;

Message g_receiveAssembly{};
bool g_discardingOversizedLine = false;
bool g_initialized = false;

void ResetMessage(Message& message)
{
    message.data.fill('\0');
    message.length = 0U;
}

void ResetTransportState()
{
    for (Message& message : g_receiveQueue)
    {
        ResetMessage(message);
    }

    for (Message& message : g_transmitQueue)
    {
        ResetMessage(message);
    }

    ResetMessage(g_receiveAssembly);

    g_receiveHead = 0U;
    g_receiveTail = 0U;
    g_receiveCount = 0U;

    g_transmitHead = 0U;
    g_transmitTail = 0U;
    g_transmitCount = 0U;
    g_transmitOffset = 0U;

    g_discardingOversizedLine = false;
}

void QueueCompletedReceiveLine()
{
    if (g_receiveAssembly.length == 0U)
    {
        ResetMessage(g_receiveAssembly);
        return;
    }

    if (g_receiveCount >= MESSAGE_QUEUE_DEPTH)
    {
        ESP_LOGW(
            LOG_TAG,
            "Receive queue full. Dropping completed UART line.");

        ResetMessage(g_receiveAssembly);
        return;
    }

    Message& destination =
        g_receiveQueue[g_receiveTail];

    destination = g_receiveAssembly;

    g_receiveTail =
        (g_receiveTail + 1U) % MESSAGE_QUEUE_DEPTH;

    ++g_receiveCount;

    ResetMessage(g_receiveAssembly);
}

void ProcessReceivedByte(const uint8_t byte)
{
    if (byte == static_cast<uint8_t>('\r'))
    {
        return;
    }

    if (byte == static_cast<uint8_t>('\n'))
    {
        if (g_discardingOversizedLine)
        {
            g_discardingOversizedLine = false;
            ResetMessage(g_receiveAssembly);
            return;
        }

        QueueCompletedReceiveLine();
        return;
    }

    if (g_discardingOversizedLine)
    {
        return;
    }

    if (g_receiveAssembly.length >=
        WashTrac::UartProtocol::MAXIMUM_LINE_LENGTH - 1U)
    {
        ESP_LOGW(
            LOG_TAG,
            "UART line exceeded maximum length and is being discarded.");

        g_discardingOversizedLine = true;
        ResetMessage(g_receiveAssembly);
        return;
    }

    g_receiveAssembly.data[g_receiveAssembly.length++] =
        static_cast<char>(byte);

    g_receiveAssembly.data[g_receiveAssembly.length] = '\0';
}

void ReceiveAvailableBytes()
{
    std::array<uint8_t, RECEIVE_CHUNK_SIZE> receivedBytes{};

    while (true)
    {
        const int byteCount =
            uart_read_bytes(
                WashTrac::Hardware::CM5_UART,
                receivedBytes.data(),
                receivedBytes.size(),
                UART_READ_TIMEOUT_TICKS);

        if (byteCount <= 0)
        {
            return;
        }

        for (int index = 0;
             index < byteCount;
             ++index)
        {
            ProcessReceivedByte(
                receivedBytes[static_cast<std::size_t>(index)]);
        }

        if (static_cast<std::size_t>(byteCount) <
            receivedBytes.size())
        {
            return;
        }
    }
}

void TransmitAvailableBytes()
{
    while (g_transmitCount > 0U)
    {
        Message& message =
            g_transmitQueue[g_transmitHead];

        if (g_transmitOffset >= message.length)
        {
            ResetMessage(message);

            g_transmitHead =
                (g_transmitHead + 1U) % MESSAGE_QUEUE_DEPTH;

            --g_transmitCount;
            g_transmitOffset = 0U;
            continue;
        }

        const std::size_t remaining =
            message.length - g_transmitOffset;

        const int bytesWritten =
            uart_tx_chars(
                WashTrac::Hardware::CM5_UART,
                message.data.data() + g_transmitOffset,
                remaining);

        if (bytesWritten <= 0)
        {
            return;
        }

        g_transmitOffset +=
            static_cast<std::size_t>(bytesWritten);

        if (static_cast<std::size_t>(bytesWritten) < remaining)
        {
            return;
        }
    }
}

} // namespace

namespace WashTrac::UartProtocol
{

Result Initialize()
{
    if (g_initialized)
    {
        return Result::OK;
    }

    uart_config_t configuration{};

    configuration.baud_rate =
        static_cast<int>(Hardware::UART_BAUD_RATE);
    configuration.data_bits = UART_DATA_8_BITS;
    configuration.parity = UART_PARITY_DISABLE;
    configuration.stop_bits = UART_STOP_BITS_1;
    configuration.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    configuration.rx_flow_ctrl_thresh = 0U;
    configuration.source_clk = UART_SCLK_DEFAULT;

    esp_err_t result =
        uart_param_config(
            Hardware::CM5_UART,
            &configuration);

    if (result != ESP_OK)
    {
        ESP_LOGE(
            LOG_TAG,
            "UART parameter configuration failed: %s",
            esp_err_to_name(result));

        return Result::ERROR;
    }

    result =
        uart_set_pin(
            Hardware::CM5_UART,
            Hardware::CM5_UART_TX,
            Hardware::CM5_UART_RX,
            UART_PIN_NO_CHANGE,
            UART_PIN_NO_CHANGE);

    if (result != ESP_OK)
    {
        ESP_LOGE(
            LOG_TAG,
            "UART pin configuration failed: %s",
            esp_err_to_name(result));

        return Result::ERROR;
    }

    if (!uart_is_driver_installed(Hardware::CM5_UART))
    {
        result =
            uart_driver_install(
                Hardware::CM5_UART,
                UART_RX_RING_BUFFER_SIZE,
                0,
                0,
                nullptr,
                0);

        if (result != ESP_OK)
        {
            ESP_LOGE(
                LOG_TAG,
                "UART driver installation failed: %s",
                esp_err_to_name(result));

            return Result::ERROR;
        }
    }

    result =
        uart_flush_input(
            Hardware::CM5_UART);

    if (result != ESP_OK)
    {
        ESP_LOGE(
            LOG_TAG,
            "UART receive buffer flush failed: %s",
            esp_err_to_name(result));

        return Result::ERROR;
    }

    ResetTransportState();

    g_initialized = true;

    ESP_LOGI(
        LOG_TAG,
        "CM5 UART transport initialized on UART%u, TX GPIO%u, "
        "RX GPIO%u at %lu baud.",
        static_cast<unsigned>(Hardware::CM5_UART),
        static_cast<unsigned>(Hardware::CM5_UART_TX),
        static_cast<unsigned>(Hardware::CM5_UART_RX),
        static_cast<unsigned long>(Hardware::UART_BAUD_RATE));

    return Result::OK;
}

void Update()
{
    if (!g_initialized)
    {
        return;
    }

    ReceiveAvailableBytes();
    TransmitAvailableBytes();
}

bool ReadLine(
    char* const buffer,
    const std::size_t bufferSize)
{
    if (!g_initialized ||
        buffer == nullptr ||
        bufferSize == 0U ||
        g_receiveCount == 0U)
    {
        return false;
    }

    const Message& message =
        g_receiveQueue[g_receiveHead];

    if (bufferSize <= message.length)
    {
        return false;
    }

    std::memcpy(
        buffer,
        message.data.data(),
        message.length + 1U);

    ResetMessage(
        g_receiveQueue[g_receiveHead]);

    g_receiveHead =
        (g_receiveHead + 1U) % MESSAGE_QUEUE_DEPTH;

    --g_receiveCount;

    return true;
}

Result WriteLine(const char* const text)
{
    if (!g_initialized)
    {
        return Result::NOT_INITIALIZED;
    }

    if (text == nullptr)
    {
        return Result::INVALID_PARAMETER;
    }

    const std::size_t textLength =
        std::strlen(text);

    if (textLength == 0U ||
        textLength > MAXIMUM_LINE_LENGTH - 2U)
    {
        return Result::INVALID_PARAMETER;
    }

    if (g_transmitCount >= MESSAGE_QUEUE_DEPTH)
    {
        return Result::ERROR;
    }

    Message& message =
        g_transmitQueue[g_transmitTail];

    ResetMessage(message);

    std::memcpy(
        message.data.data(),
        text,
        textLength);

    message.data[textLength] = '\n';
    message.length = textLength + 1U;
    message.data[message.length] = '\0';

    g_transmitTail =
        (g_transmitTail + 1U) % MESSAGE_QUEUE_DEPTH;

    ++g_transmitCount;

    return Result::OK;
}

bool IsInitialized()
{
    return g_initialized;
}

} // namespace WashTrac::UartProtocol