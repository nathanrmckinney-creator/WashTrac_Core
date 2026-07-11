/******************************************************************************
 *
 *  Project:
 *      WashTrac Core
 *
 *  Module:
 *      GPIO Manager
 *
 *  File:
 *      gpio_manager.cpp
 *
 *  Description:
 *      Initializes all relay outputs and digital inputs.
 *      Provides direct GPIO access through the interface declared in
 *      gpio_manager.h.
 *
 ******************************************************************************/

#include "gpio_manager.h"

#include <array>
#include <cstddef>
#include <cstdint>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

#include "hardware.h"

namespace WashTrac::GPIO
{
namespace
{

constexpr const char* TAG = "GPIO_MANAGER";

constexpr std::array<gpio_num_t, 6> RELAY_PINS = {
    Hardware::RELAY_1,
    Hardware::RELAY_2,
    Hardware::RELAY_3,
    Hardware::RELAY_4,
    Hardware::RELAY_5,
    Hardware::RELAY_6
};

constexpr std::array<gpio_num_t, 6> INPUT_PINS = {
    Hardware::INPUT_1,
    Hardware::INPUT_2,
    Hardware::INPUT_3,
    Hardware::INPUT_4,
    Hardware::INPUT_5,
    Hardware::INPUT_6
};

int InactiveLevel(const std::size_t relay_index)
{
    return relay_index == 0U
        ? static_cast<int>(Hardware::SSR_INACTIVE_LEVEL)
        : static_cast<int>(Hardware::RELAY_INACTIVE_LEVEL);
}

int ActiveLevel(const std::size_t relay_index)
{
    return relay_index == 0U
        ? static_cast<int>(Hardware::SSR_ACTIVE_LEVEL)
        : static_cast<int>(Hardware::RELAY_ACTIVE_LEVEL);
}

esp_err_t ConfigureRelayOutputs()
{
    uint64_t pin_mask = 0U;

    for (const gpio_num_t pin : RELAY_PINS)
    {
        pin_mask |= (1ULL << static_cast<uint32_t>(pin));
    }

    gpio_config_t config = {};
    config.pin_bit_mask = pin_mask;
    config.mode = GPIO_MODE_OUTPUT;
    config.pull_up_en = GPIO_PULLUP_DISABLE;
    config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    config.intr_type = GPIO_INTR_DISABLE;

    const esp_err_t config_result = gpio_config(&config);
    if (config_result != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "Relay output configuration failed: %s",
            esp_err_to_name(config_result));
        return config_result;
    }

    for (std::size_t index = 0U; index < RELAY_PINS.size(); ++index)
    {
        const esp_err_t level_result =
            gpio_set_level(RELAY_PINS[index], InactiveLevel(index));

        if (level_result != ESP_OK)
        {
            ESP_LOGE(
                TAG,
                "Failed to force relay %u inactive: %s",
                static_cast<unsigned>(index + 1U),
                esp_err_to_name(level_result));
            return level_result;
        }
    }

    return ESP_OK;
}

esp_err_t ConfigureInputs()
{
    uint64_t pin_mask = 0U;

    for (const gpio_num_t pin : INPUT_PINS)
    {
        pin_mask |= (1ULL << static_cast<uint32_t>(pin));
    }

    gpio_config_t config = {};
    config.pin_bit_mask = pin_mask;
    config.mode = GPIO_MODE_INPUT;
    config.pull_up_en = GPIO_PULLUP_DISABLE;
    config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    config.intr_type = GPIO_INTR_DISABLE;

    const esp_err_t result = gpio_config(&config);
    if (result != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "Input configuration failed: %s",
            esp_err_to_name(result));
    }

    return result;
}

bool ReadInputByIndex(const std::size_t index)
{
    return gpio_get_level(INPUT_PINS[index]) != 0;
}

void SetRelayByIndex(const std::size_t index, const bool active)
{
    const int level = active ? ActiveLevel(index) : InactiveLevel(index);

    const esp_err_t result = gpio_set_level(RELAY_PINS[index], level);
    if (result != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "Failed to set relay %u: %s",
            static_cast<unsigned>(index + 1U),
            esp_err_to_name(result));
    }
}

} // namespace

esp_err_t Initialize()
{
    const esp_err_t relay_result = ConfigureRelayOutputs();
    if (relay_result != ESP_OK)
    {
        return relay_result;
    }

    const esp_err_t input_result = ConfigureInputs();
    if (input_result != ESP_OK)
    {
        return input_result;
    }

    ESP_LOGI(TAG, "GPIO Manager initialized; all relays inactive");
    return ESP_OK;
}

bool ReadInput1()
{
    return ReadInputByIndex(0U);
}

bool ReadInput2()
{
    return ReadInputByIndex(1U);
}

bool ReadInput3()
{
    return ReadInputByIndex(2U);
}

bool ReadInput4()
{
    return ReadInputByIndex(3U);
}

bool ReadInput5()
{
    return ReadInputByIndex(4U);
}

bool ReadInput6()
{
    return ReadInputByIndex(5U);
}

void SetRelay1(const bool active)
{
    SetRelayByIndex(0U, active);
}

void SetRelay2(const bool active)
{
    SetRelayByIndex(1U, active);
}

void SetRelay3(const bool active)
{
    SetRelayByIndex(2U, active);
}

void SetRelay4(const bool active)
{
    SetRelayByIndex(3U, active);
}

void SetRelay5(const bool active)
{
    SetRelayByIndex(4U, active);
}

void SetRelay6(const bool active)
{
    SetRelayByIndex(5U, active);
}

} // namespace WashTrac::GPIO
