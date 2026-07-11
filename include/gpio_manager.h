#pragma once

#include "driver/gpio.h"
#include "esp_err.h"

namespace WashTrac::GPIO
{
esp_err_t Initialize();

bool ReadInput1();   // Wash Busy
bool ReadInput2();
bool ReadInput3();
bool ReadInput4();
bool ReadInput5();
bool ReadInput6();

void SetRelay1(bool active);
void SetRelay2(bool active);
void SetRelay3(bool active);
void SetRelay4(bool active);
void SetRelay5(bool active);
void SetRelay6(bool active);
}
