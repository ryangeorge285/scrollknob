#include "mouse_task.h"
#include "semaphore_guard.h"
#include "util.h"
#include <BleMouse.h>
#include "led_manager.h"
#include <BLEDevice.h>
#include <BLEServer.h>

MouseTask::MouseTask(const uint8_t task_core) : Task<MouseTask>("Mouse", 5000, 1, task_core)
{
    // Create queue to receive knob state updates
    knob_state_queue_ = xQueueCreate(1, sizeof(PB_SmartKnobState));
    assert(knob_state_queue_ != NULL);

    // Create mutex for thread safety
    mutex_ = xSemaphoreCreateMutex();
    assert(mutex_ != NULL);

    // Initialize logger to null
    logger_ = nullptr;
}

MouseTask::~MouseTask()
{
    // Clean up resources
    vQueueDelete(knob_state_queue_);
    vSemaphoreDelete(mutex_);
}

QueueHandle_t MouseTask::getKnobStateQueue()
{
    return knob_state_queue_;
}

void MouseTask::run()
{
    log("Mouse task started");

    LEDManager::getInstance()->setMode(LED_MODE_BT_PAIRING);

    bleMouse.begin();

    bool was_connected = false;

    // Main task loop
    while (1)
    {
        bool is_connected = bleMouse.isConnected();

        // Handle connection state changes
        if (is_connected && !was_connected)
        {
            LEDManager::getInstance()->setMode(LED_MODE_NORMAL);
            log("BLE Mouse connected");
        }
        else if (!is_connected && was_connected)
        {
            LEDManager::getInstance()->setMode(LED_MODE_ERROR);
            log("BLE Mouse disconnected");
            break;
        }

        was_connected = is_connected;

        // Check for knob state updates from the queue
        if (xQueueReceive(knob_state_queue_, &state_, portMAX_DELAY) == pdTRUE)
        {
            bool substantial_change = (previous_state_.current_position != state_.current_position) || (previous_state_.config.detent_strength_unit != state_.config.detent_strength_unit) || (previous_state_.config.endstop_strength_unit != state_.config.endstop_strength_unit) || (previous_state_.config.min_position != state_.config.min_position) || (previous_state_.config.max_position != state_.config.max_position);

            if (substantial_change)
            {
                if (is_connected)
                {
                    bleMouse.move(0, 0, state_.current_position - previous_state_.current_position);
                }
                char buf[100];
                snprintf(buf, sizeof(buf), "Received knob state: position=%d", state_.current_position);
                log(buf);

                previous_state_ = state_;
            }
        }

        // Small delay to prevent task from consuming too much CPU
        delay(5);
    }
}

void MouseTask::log(const char *msg)
{
    if (logger_ != nullptr)
    {
        logger_->log(msg);
    }
}

void MouseTask::setLogger(Logger *logger)
{
    logger_ = logger;
}