#include "mouse_task.h"
#include "semaphore_guard.h"
#include "util.h"
#include <BleMouse.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include "pmw3389_sensor.h"
#include "ads1220_adc.h"
#include <esp_system.h>
#include "driver/rtc_io.h"
#include "driver/uart.h" // Add this line
#include <esp_sleep.h>
#include "configuration.h" // Make sure Configuration is included

MouseTask::MouseTask(const uint8_t task_core, MotorTask &motor_task) : Task<MouseTask>("Mouse", 10000, 1, task_core), motor_task_(motor_task)
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

static const char *resetReasonToStr(esp_reset_reason_t reset_reason)
{
    // First check if this was a wakeup from deep sleep
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

    switch (wakeup_reason)
    {
    case ESP_SLEEP_WAKEUP_EXT0:
        return "Wakeup: external signal using RTC_IO";
    case ESP_SLEEP_WAKEUP_EXT1:
        return "Wakeup: external signal using RTC_CNTL";
    case ESP_SLEEP_WAKEUP_TIMER:
        return "Wakeup: timer";
    case ESP_SLEEP_WAKEUP_TOUCHPAD:
        return "Wakeup: touchpad";
    case ESP_SLEEP_WAKEUP_ULP:
        return "Wakeup: ULP program";
    default:
        // Not a deep sleep wakeup, use the reset reason
        switch (reset_reason)
        {
        case ESP_RST_POWERON:
            return "Power on reset";
        case ESP_RST_EXT:
            return "External reset";
        case ESP_RST_SW:
            return "Software reset";
        case ESP_RST_PANIC:
            return "Panic reset";
        case ESP_RST_INT_WDT:
            return "Interrupt watchdog reset";
        case ESP_RST_TASK_WDT:
            return "Task watchdog reset";
        case ESP_RST_WDT:
            return "RTC watchdog reset";
        case ESP_RST_DEEPSLEEP:
            return "Deep sleep reset";
        case ESP_RST_BROWNOUT:
            return "Brownout reset";
        case ESP_RST_SDIO:
            return "SDIO reset";
        default:
            return "Unknown reset reason";
        }
    }
}

void MouseTask::run()
{
    log("Mouse task started");

    led_manager_->setMode(LED_MODE_BT_PAIRING);

    mouseSensor.setLogger(logger_);
    mouseSensor.init();
    mouseSensor.enableBurst();

    SPI.end();
    SPI.begin(PIN_VSPI_SCK, PIN_VSPI_MISO, PIN_VSPI_MOSI);

    adsController->init();
    
    esp_reset_reason_t reason = esp_reset_reason();
    log(resetReasonToStr(reason));
    bool was_connected = false;

    int16_t dx = 0, dy = 0;
    byte burst[12];
    bool mouseMotion = false;

    unsigned long last_update = millis();
    bool knobMotion = false;
    BLEDevice::init("ScrollWheel");
    bleMouse.begin();

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
        }

        was_connected = is_connected;

        // Check for knob state updates from the queue
        if (xQueueReceive(knob_state_queue_, &state_, pdMS_TO_TICKS(50)) == pdTRUE)
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
                char buf[100];
                snprintf(buf, sizeof(buf), "Mouse Movement: {%i,%i}", dx, dy);
                log(buf);
                bleMouse.move(constrain(dx, -127, 127),
                              constrain(dy, -127, 127),
                              0);
                dx = dy = 0;
            }
            static bool pressed;
            static int currentClickStatus = 0;
            static uint8_t press_readings;

            StrainData strain = adsController->read(false);

            // char buf[100];
            // snprintf(buf, sizeof(buf), "Press value: %f", strain.force);
            // log(buf);

            if (!pressed && strain.force > 0.9)
            {
                press_readings++;
                if (press_readings > 2)
                {
                    if (strain.clickStatus == 1)
                    {
                        char buf[100];
                        snprintf(buf, sizeof(buf), "Left Clicked: Force%f", strain.force);
                        log(buf);
                        bleMouse.press(MOUSE_LEFT);
                        currentClickStatus = 1;
                    }
                    else if (strain.clickStatus == 2)
                    {
                        char buf[100];
                        snprintf(buf, sizeof(buf), "Right Clicked: Force%f", strain.force);
                        log(buf);
                        bleMouse.press(MOUSE_RIGHT);
                        currentClickStatus = 2;
                    }
                    motor_task_.playHaptic(true);
                    pressed = true;
                    press_count_++;
                }
            }
            else if (pressed && strain.force < 0.85)
            {
                press_readings++;
                if (press_readings > 2)
                {
                    char buf[100];
                    snprintf(buf, sizeof(buf), "Releasing %s", currentClickStatus == 1 ? "Left Click" : "Right Click");
                    log(buf);
                    if (currentClickStatus == 1)
                    {
                        bleMouse.release(MOUSE_LEFT);
                    }
                    else if (currentClickStatus == 2)
                    {
                        bleMouse.release(MOUSE_RIGHT);
                    }
                    motor_task_.playHaptic(false);
                    pressed = false;
                    currentClickStatus = 0;
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
            else if (millis() - last_update > 6000 * SLEEP_MIN)
            {
                log("No activity, going to sleep");

                // mouseSensor.shutdown();

                // esp_sleep_disable_uart_wakeup(0);
                rtc_gpio_pulldown_dis((gpio_num_t)PIN_PMW3389_MOTION);
                rtc_gpio_pullup_en((gpio_num_t)PIN_PMW3389_MOTION);
                esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_PMW3389_MOTION, (esp_sleep_ext1_wakeup_mode_t)0);

                char buf[100];
                snprintf(buf, sizeof(buf), "Motion Pin: %i", digitalRead(PIN_PMW3389_MOTION));
                log(buf);

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
