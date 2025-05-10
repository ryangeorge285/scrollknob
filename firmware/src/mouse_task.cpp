#include "mouse_task.h"
#include "semaphore_guard.h"
#include "util.h"
#include <BleMouse.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include "pmw3389_sensor.h"
#include "ads1220_adc.h"

MouseTask::MouseTask(const uint8_t task_core, MotorTask &motor_task) : Task<MouseTask>("Mouse", 5000, 1, task_core), motor_task_(motor_task)
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

    led_manager_->setMode(LED_MODE_BT_PAIRING);

    mouseSensor.setLogger(logger_);
    mouseSensor.init();
    mouseSensor.enableBurst();

    bleMouse.begin();

    SPI.end();
    SPI.begin(PIN_PMW3389_SCK, PIN_PMW3389_MISO, PIN_PMW3389_MOSI);

    adsController.setLogger(logger_);
    adsController.init();
    adsController.calibrate();

    bool was_connected = false;

    int16_t dx = 0, dy = 0;
    byte burst[12];
    bool mouseMotion = false;

    unsigned long last_update = millis();
    bool knobMotion = false;

    // Main task loop
    while (1)
    {

        bool is_connected = bleMouse.isConnected();

        // Handle connection state changes
        if (is_connected && !was_connected)
        {
            led_manager_->setMode(LED_MODE_NORMAL);
            log("BLE Mouse connected");
        }
        else if (!is_connected && was_connected)
        {
            led_manager_->setMode(LED_MODE_ERROR);
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

                // if (compass_sensor_ != nullptr)
                // {
                //     char buf[100];
                //     snprintf(buf, sizeof(buf), "Received compass reading: %f", compass_sensor_->getCurrentHeading());
                //     log(buf);
                //     // Use the heading value
                // }

                previous_state_ = state_;
                knobMotion = true;
            }

            mouseSensor.readMouseMovement(mouseMotion, dx, dy);

            if (mouseMotion)
            {
                bleMouse.move(constrain(dx, -127, 127),
                              constrain(dy, -127, 127),
                              0);
                dx = dy = 0;
            }
            static bool pressed;
            static uint8_t press_readings;
            float press_value_unit = adsController.readStrainGauge(false);

            char buf[100];
            snprintf(buf, sizeof(buf), "Press value: %f", press_value_unit);
            log(buf);

            if (!pressed && press_value_unit > 1.45)
            {
                press_readings++;
                if (press_readings > 2)
                {
                    bleMouse.press();
                    motor_task_.playHaptic(true);
                    pressed = true;
                    press_count_++;
                }
            }
            else if (pressed && press_value_unit < 1.3)
            {
                press_readings++;
                if (press_readings > 2)
                {
                    bleMouse.release();
                    motor_task_.playHaptic(false);
                    pressed = false;
                }
            }
            else
            {
                press_readings = 0;
            }

#if DEEP_SLEEP_ENABLED
            if (knobMotion || mouseMotion)
            {
                last_update = millis();
                knobMotion = false;
                mouseMotion = false;
            }
            else if (millis() - last_update > 5000)
            {
                log("No activity, going to sleep");

                mouseSensor.shutdown();

                delay(100);
                esp_deep_sleep_start();
            }
#endif
        }

        vTaskDelay(pdMS_TO_TICKS(1));
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
