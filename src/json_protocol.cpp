/******************************************************************************
 *
 *  Project:
 *      WashTrac Core
 *
 *  Module:
 *      JSON Protocol
 *
 *  File:
 *      json_protocol.cpp
 *
 *  Description:
 *      Production fixed-memory JSON parser and serializer for the WashTrac
 *      Core Version 1 CM5 protocol.
 *
 *      The implementation:
 *          - Uses no dynamic memory allocation
 *          - Accepts one top-level JSON object
 *          - Supports strings, unsigned integers, booleans, and null
 *          - Rejects malformed, duplicate, nested, and unsupported fields
 *          - Performs strict command-specific validation
 *          - Escapes all serialized JSON strings
 *
 *  Copyright:
 *      © 2026 WashTrac
 *
 ******************************************************************************/

#include "json_protocol.h"

#include <array>
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>

namespace
{

using WashTrac::JsonProtocol::Command;
using WashTrac::JsonProtocol::Message;

constexpr std::size_t KEY_LENGTH = 48U;
constexpr std::size_t STRING_VALUE_LENGTH = 96U;

enum class ValueType : uint8_t
{
    String,
    UnsignedInteger,
    Boolean,
    Null
};

struct ParsedValue
{
    ValueType type;
    std::array<char, STRING_VALUE_LENGTH> stringValue;
    uint32_t unsignedValue;
    bool booleanValue;
};

class Parser
{
public:
    explicit Parser(const char* text)
        : text_(text),
          position_(0U)
    {
    }

    bool ParseMessage(Message& message)
    {
        ResetMessage(message);

        if (text_ == nullptr)
        {
            return false;
        }

        SkipWhitespace();

        if (!Consume('{'))
        {
            return false;
        }

        SkipWhitespace();

        if (Consume('}'))
        {
            return false;
        }

        while (true)
        {
            std::array<char, KEY_LENGTH> key{};

            if (!ParseString(key.data(), key.size()))
            {
                return false;
            }

            SkipWhitespace();

            if (!Consume(':'))
            {
                return false;
            }

            SkipWhitespace();

            ParsedValue value{};

            if (!ParseValue(value))
            {
                return false;
            }

            if (!ApplyField(key.data(), value, message))
            {
                return false;
            }

            SkipWhitespace();

            if (Consume('}'))
            {
                break;
            }

            if (!Consume(','))
            {
                return false;
            }

            SkipWhitespace();
        }

        SkipWhitespace();

        if (text_[position_] != '\0')
        {
            return false;
        }

        return ValidateMessage(message);
    }

private:
    static void ResetMessage(Message& message)
    {
        message = {};
        message.command = Command::Unknown;
    }

    void SkipWhitespace()
    {
        while (text_[position_] != '\0' &&
               std::isspace(
                   static_cast<unsigned char>(
                       text_[position_])) != 0)
        {
            ++position_;
        }
    }

    bool Consume(const char expected)
    {
        if (text_[position_] != expected)
        {
            return false;
        }

        ++position_;
        return true;
    }

    bool ParseString(
        char* output,
        const std::size_t outputSize)
    {
        if (output == nullptr ||
            outputSize == 0U ||
            !Consume('"'))
        {
            return false;
        }

        std::size_t outputLength = 0U;

        while (true)
        {
            const char character = text_[position_++];

            if (character == '\0')
            {
                return false;
            }

            if (character == '"')
            {
                output[outputLength] = '\0';
                return true;
            }

            if (static_cast<unsigned char>(character) < 0x20U)
            {
                return false;
            }

            char decodedCharacter = character;

            if (character == '\\')
            {
                const char escaped = text_[position_++];

                switch (escaped)
                {
                    case '"':
                    case '\\':
                    case '/':
                        decodedCharacter = escaped;
                        break;

                    case 'b':
                        decodedCharacter = '\b';
                        break;

                    case 'f':
                        decodedCharacter = '\f';
                        break;

                    case 'n':
                        decodedCharacter = '\n';
                        break;

                    case 'r':
                        decodedCharacter = '\r';
                        break;

                    case 't':
                        decodedCharacter = '\t';
                        break;

                    default:
                        return false;
                }
            }

            if (outputLength >= outputSize - 1U)
            {
                return false;
            }

            output[outputLength++] = decodedCharacter;
        }
    }

    bool ParseUnsignedInteger(uint32_t& value)
    {
        if (!std::isdigit(
                static_cast<unsigned char>(
                    text_[position_])))
        {
            return false;
        }

        uint64_t parsedValue = 0ULL;

        while (std::isdigit(
                   static_cast<unsigned char>(
                       text_[position_])) != 0)
        {
            parsedValue =
                (parsedValue * 10ULL) +
                static_cast<uint64_t>(
                    text_[position_] - '0');

            if (parsedValue >
                std::numeric_limits<uint32_t>::max())
            {
                return false;
            }

            ++position_;
        }

        value = static_cast<uint32_t>(parsedValue);
        return true;
    }

    bool MatchLiteral(const char* literal)
    {
        const std::size_t length =
            std::strlen(literal);

        if (std::strncmp(
                text_ + position_,
                literal,
                length) != 0)
        {
            return false;
        }

        position_ += length;
        return true;
    }

    bool ParseValue(ParsedValue& value)
    {
        value = {};

        if (text_[position_] == '"')
        {
            value.type = ValueType::String;

            return ParseString(
                value.stringValue.data(),
                value.stringValue.size());
        }

        if (std::isdigit(
                static_cast<unsigned char>(
                    text_[position_])) != 0)
        {
            value.type =
                ValueType::UnsignedInteger;

            return ParseUnsignedInteger(
                value.unsignedValue);
        }

        if (MatchLiteral("true"))
        {
            value.type = ValueType::Boolean;
            value.booleanValue = true;
            return true;
        }

        if (MatchLiteral("false"))
        {
            value.type = ValueType::Boolean;
            value.booleanValue = false;
            return true;
        }

        if (MatchLiteral("null"))
        {
            value.type = ValueType::Null;
            return true;
        }

        return false;
    }

    static bool CopyString(
        char* destination,
        const std::size_t destinationSize,
        const char* source)
    {
        const std::size_t sourceLength =
            std::strlen(source);

        if (sourceLength >= destinationSize)
        {
            return false;
        }

        std::memcpy(
            destination,
            source,
            sourceLength + 1U);

        return true;
    }

    static Command ParseCommand(const char* command)
    {
        if (std::strcmp(command, "ping") == 0)
            return Command::Ping;

        if (std::strcmp(command, "status") == 0)
            return Command::Status;

        if (std::strcmp(command, "start_wash") == 0)
            return Command::StartWash;

        if (std::strcmp(command, "queue_status") == 0)
            return Command::QueueStatus;

        if (std::strcmp(command, "diagnostics") == 0)
            return Command::Diagnostics;

        if (std::strcmp(command, "get_config") == 0)
            return Command::GetConfig;

        if (std::strcmp(command, "set_relay") == 0)
            return Command::SetRelay;

        if (std::strcmp(command, "set_input") == 0)
            return Command::SetInput;

        if (std::strcmp(command, "set_timing") == 0)
            return Command::SetTiming;

        if (std::strcmp(command, "save_config") == 0)
            return Command::SaveConfig;

        return Command::Unknown;
    }

    static bool AssignUnsigned16(
        const ParsedValue& value,
        uint16_t& destination,
        bool& present)
    {
        if (present ||
            value.type != ValueType::UnsignedInteger ||
            value.unsignedValue >
                std::numeric_limits<uint16_t>::max())
        {
            return false;
        }

        destination =
            static_cast<uint16_t>(
                value.unsignedValue);

        present = true;
        return true;
    }

    static bool ApplyField(
        const char* key,
        const ParsedValue& value,
        Message& message)
    {
        if (std::strcmp(key, "cmd") == 0)
        {
            if (message.command != Command::Unknown ||
                value.type != ValueType::String)
            {
                return false;
            }

            message.command =
                ParseCommand(
                    value.stringValue.data());

            return message.command != Command::Unknown;
        }

        if (std::strcmp(key, "request_id") == 0)
        {
            if (message.hasRequestId ||
                value.type != ValueType::String ||
                !CopyString(
                    message.requestId,
                    sizeof(message.requestId),
                    value.stringValue.data()))
            {
                return false;
            }

            message.hasRequestId = true;
            return true;
        }

        if (std::strcmp(key, "name") == 0)
        {
            if (message.hasName ||
                value.type != ValueType::String ||
                !CopyString(
                    message.name,
                    sizeof(message.name),
                    value.stringValue.data()))
            {
                return false;
            }

            message.hasName = true;
            return true;
        }

        if (std::strcmp(key, "relay") == 0)
        {
            if (message.hasRelayNumber ||
                value.type != ValueType::UnsignedInteger ||
                value.unsignedValue >
                    std::numeric_limits<uint8_t>::max())
            {
                return false;
            }

            message.relayNumber =
                static_cast<uint8_t>(
                    value.unsignedValue);

            message.hasRelayNumber = true;
            return true;
        }

        if (std::strcmp(key, "input") == 0)
        {
            if (message.hasInputNumber ||
                value.type != ValueType::UnsignedInteger ||
                value.unsignedValue >
                    std::numeric_limits<uint8_t>::max())
            {
                return false;
            }

            message.inputNumber =
                static_cast<uint8_t>(
                    value.unsignedValue);

            message.hasInputNumber = true;
            return true;
        }

        if (std::strcmp(key, "enabled") == 0)
        {
            if (message.hasEnabled ||
                value.type != ValueType::Boolean)
            {
                return false;
            }

            message.enabled = value.booleanValue;
            message.hasEnabled = true;
            return true;
        }

        if (std::strcmp(key, "inverted") == 0)
        {
            if (message.hasInverted ||
                value.type != ValueType::Boolean)
            {
                return false;
            }

            message.inverted = value.booleanValue;
            message.hasInverted = true;
            return true;
        }

        if (std::strcmp(key, "on_delay") == 0)
        {
            return AssignUnsigned16(
                value,
                message.onDelaySeconds,
                message.hasOnDelaySeconds);
        }

        if (std::strcmp(key, "duration") == 0)
        {
            return AssignUnsigned16(
                value,
                message.durationSeconds,
                message.hasDurationSeconds);
        }

        if (std::strcmp(key, "off_delay") == 0)
        {
            return AssignUnsigned16(
                value,
                message.offDelaySeconds,
                message.hasOffDelaySeconds);
        }

        if (std::strcmp(key, "busy_release_delay") == 0)
        {
            return AssignUnsigned16(
                value,
                message.washBusyReleaseDelaySeconds,
                message.hasWashBusyReleaseDelaySeconds);
        }

        if (std::strcmp(key, "inter_wash_delay") == 0)
        {
            return AssignUnsigned16(
                value,
                message.interWashDelaySeconds,
                message.hasInterWashDelaySeconds);
        }

        return false;
    }

    static bool ValidateMessage(const Message& message)
    {
        if (message.command == Command::Unknown)
        {
            return false;
        }

        switch (message.command)
        {
            case Command::Ping:
            case Command::Status:
            case Command::StartWash:
            case Command::QueueStatus:
            case Command::Diagnostics:
            case Command::GetConfig:
            case Command::SaveConfig:
                return true;

            case Command::SetRelay:
                return message.hasRelayNumber &&
                       message.hasEnabled &&
                       message.hasName &&
                       message.hasOnDelaySeconds &&
                       message.hasDurationSeconds &&
                       message.hasOffDelaySeconds;

            case Command::SetInput:
                return message.hasInputNumber &&
                       message.hasEnabled &&
                       message.hasInverted &&
                       message.hasName;

            case Command::SetTiming:
                return message.hasWashBusyReleaseDelaySeconds ||
                       message.hasInterWashDelaySeconds;

            default:
                return false;
        }
    }

    const char* text_;
    std::size_t position_;
};

class Writer
{
public:
    Writer(
        char* output,
        const std::size_t outputSize)
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

    bool Append(const char* text)
    {
        if (!valid_ || text == nullptr)
        {
            return false;
        }

        const std::size_t textLength =
            std::strlen(text);

        if (length_ + textLength >= outputSize_)
        {
            valid_ = false;
            return false;
        }

        std::memcpy(
            output_ + length_,
            text,
            textLength);

        length_ += textLength;
        output_[length_] = '\0';

        return true;
    }

    bool AppendEscapedString(const char* text)
    {
        if (!Append("\""))
        {
            return false;
        }

        for (std::size_t index = 0U;
             text[index] != '\0';
             ++index)
        {
            const unsigned char character =
                static_cast<unsigned char>(
                    text[index]);

            switch (character)
            {
                case '"':
                    if (!Append("\\\""))
                        return false;
                    break;

                case '\\':
                    if (!Append("\\\\"))
                        return false;
                    break;

                case '\b':
                    if (!Append("\\b"))
                        return false;
                    break;

                case '\f':
                    if (!Append("\\f"))
                        return false;
                    break;

                case '\n':
                    if (!Append("\\n"))
                        return false;
                    break;

                case '\r':
                    if (!Append("\\r"))
                        return false;
                    break;

                case '\t':
                    if (!Append("\\t"))
                        return false;
                    break;

                default:
                    if (character < 0x20U)
                    {
                        return false;
                    }

                    char characterText[2] =
                    {
                        static_cast<char>(character),
                        '\0'
                    };

                    if (!Append(characterText))
                        return false;
                    break;
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

WashTrac::Result WriteResponse(
    const bool success,
    const char* requestId,
    const char* operation,
    const char* error,
    char* output,
    const std::size_t outputSize)
{
    if (operation == nullptr ||
        output == nullptr ||
        outputSize == 0U)
    {
        return WashTrac::Result::INVALID_PARAMETER;
    }

    Writer writer(output, outputSize);

    writer.Append("{\"type\":");
    writer.AppendEscapedString(success ? "ok" : "error");

    if (requestId != nullptr &&
        requestId[0] != '\0')
    {
        writer.Append(",\"request_id\":");
        writer.AppendEscapedString(requestId);
    }

    writer.Append(",\"operation\":");
    writer.AppendEscapedString(operation);

    if (!success)
    {
        if (error == nullptr ||
            error[0] == '\0')
        {
            return WashTrac::Result::INVALID_PARAMETER;
        }

        writer.Append(",\"error\":");
        writer.AppendEscapedString(error);
    }

    writer.Append("}");

    return writer.IsValid()
        ? WashTrac::Result::OK
        : WashTrac::Result::ERROR;
}

} // namespace

namespace WashTrac::JsonProtocol
{

Result Parse(
    const char* const json,
    Message& message)
{
    Parser parser(json);

    return parser.ParseMessage(message)
        ? Result::OK
        : Result::INVALID_PARAMETER;
}

Result WriteOk(
    const char* const requestId,
    const char* const operation,
    char* const output,
    const std::size_t outputSize)
{
    return WriteResponse(
        true,
        requestId,
        operation,
        nullptr,
        output,
        outputSize);
}

Result WriteError(
    const char* const requestId,
    const char* const operation,
    const char* const error,
    char* const output,
    const std::size_t outputSize)
{
    return WriteResponse(
        false,
        requestId,
        operation,
        error,
        output,
        outputSize);
}

const char* CommandToString(const Command command)
{
    switch (command)
    {
        case Command::Ping:
            return "ping";

        case Command::Status:
            return "status";

        case Command::StartWash:
            return "start_wash";

        case Command::QueueStatus:
            return "queue_status";

        case Command::Diagnostics:
            return "diagnostics";

        case Command::GetConfig:
            return "get_config";

        case Command::SetRelay:
            return "set_relay";

        case Command::SetInput:
            return "set_input";

        case Command::SetTiming:
            return "set_timing";

        case Command::SaveConfig:
            return "save_config";

        default:
            return "unknown";
    }
}

} // namespace WashTrac::JsonProtocol