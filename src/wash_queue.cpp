/******************************************************************************
 *
 *  Project:
 *      WashTrac Core
 *
 *  Module:
 *      Wash Queue
 *
 *  File:
 *      wash_queue.cpp
 *
 *  Description:
 *      Wash Queue implementation providing:
 *          - Initialization
 *          - Pending wash tracking
 *          - Maximum queue-depth enforcement
 *          - Queue removal and clearing
 *
 ******************************************************************************/

#include "wash_queue.h"

#include "event_logger.h"

#include "esp_log.h"

namespace
{

constexpr const char* LOG_TAG = "WashQueue";

struct WashQueueState
{
    std::size_t pendingCount;
    bool initialized;
};

WashQueueState g_state{};

} // namespace

namespace WashTrac::WashQueue
{

Result Initialize()
{
    if (g_state.initialized)
        return Result::OK;

    g_state.pendingCount = 0U;

    WashTrac::Events::Log(
        WashTrac::Events::EventCode::QueueCleared);
    g_state.initialized = true;

    ESP_LOGI(LOG_TAG, "Wash Queue initialized.");

    return Result::OK;
}

Result Enqueue()
{
    if (!g_state.initialized)
        return Result::NOT_INITIALIZED;

    if (IsFull())
        return Result::INVALID_PARAMETER;

    ++g_state.pendingCount;

    WashTrac::Events::Log(
        WashTrac::Events::EventCode::WashQueued,
        static_cast<uint32_t>(g_state.pendingCount));

    ESP_LOGI(
        LOG_TAG,
        "Wash queued. Pending washes: %u",
        static_cast<unsigned>(g_state.pendingCount));

    return Result::OK;
}

Result Dequeue()
{
    if (!g_state.initialized)
        return Result::NOT_INITIALIZED;

    if (IsEmpty())
        return Result::INVALID_PARAMETER;

    --g_state.pendingCount;

    WashTrac::Events::Log(
        WashTrac::Events::EventCode::WashDequeued,
        static_cast<uint32_t>(g_state.pendingCount));

    ESP_LOGI(
        LOG_TAG,
        "Wash removed from queue. Pending washes: %u",
        static_cast<unsigned>(g_state.pendingCount));

    return Result::OK;
}

void Clear()
{
    if (!g_state.initialized)
        return;

    g_state.pendingCount = 0U;

    WashTrac::Events::Log(
        WashTrac::Events::EventCode::QueueCleared);

    ESP_LOGI(LOG_TAG, "Wash Queue cleared.");
}

std::size_t Count()
{
    if (!g_state.initialized)
        return 0U;

    return g_state.pendingCount;
}

bool IsEmpty()
{
    return Count() == 0U;
}

bool IsFull()
{
    return Count() >= MAX_PENDING_WASHES;
}

bool IsInitialized()
{
    return g_state.initialized;
}

} // namespace WashTrac::WashQueue