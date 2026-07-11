#pragma once

#include <cstdint>

namespace WashTrac::StateMachine
{

enum class SystemState
{
    Idle,
    StartingWash,
    WaitingForBusy,
    WashRunning,
    ReleaseDelay,
    QueueDelay,
    RetryDelay,
    Fault,
    EStop
};

void Initialize();

void Update();

SystemState GetState();

bool IsBusy();

bool IsFaulted();

void ClearFault();

uint8_t GetRetryCount();

void EnterEStop();

} // namespace WashTrac::StateMachine