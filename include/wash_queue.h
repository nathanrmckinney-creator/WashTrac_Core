/******************************************************************************
 *
 *  Project:
 *      WashTrac Core
 *
 *  Module:
 *      Wash Queue
 *
 *  File:
 *      wash_queue.h
 *
 *  Description:
 *      Defines the public interface for managing pending WashTrac wash
 *      requests.
 *
 *      The queue stores only pending washes. The wash currently being
 *      processed is not counted as queued.
 *
 *  Copyright:
 *      © 2026 WashTrac
 *
 ******************************************************************************/

#pragma once

#include "system.h"

#include <cstddef>
#include <cstdint>

namespace WashTrac::WashQueue
{

constexpr std::size_t MAX_PENDING_WASHES = 2U;

Result Initialize();

Result Enqueue();

Result Dequeue();

void Clear();

std::size_t Count();

bool IsEmpty();

bool IsFull();

bool IsInitialized();

} // namespace WashTrac::WashQueue