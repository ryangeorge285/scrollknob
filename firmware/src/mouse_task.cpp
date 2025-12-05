#include "mouse_task.h"
#include "semaphore_guard.h"
#include "util.h"
#include <algorithm>
#include <BleMouse.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include "pmw3389_sensor.h"
#include "ads1220_adc.h"
#include <esp_system.h>
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
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
    press_count_ = 0;
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

static void clearAllBleBonds(Logger *logger)
{
    int dev_num = esp_ble_get_bond_device_num();
    if (dev_num <= 0)
    {
        return;
    }
    esp_ble_bond_dev_t *dev_list = (esp_ble_bond_dev_t *)malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
    if (!dev_list)
    {
        if (logger != nullptr)
        {
            logger->log("BLE: failed to alloc bond list");
        }
        return;
    }
    if (esp_ble_get_bond_device_list(&dev_num, dev_list) == ESP_OK)
    {
        for (int i = 0; i < dev_num; ++i)
        {
            esp_ble_remove_bond_device(dev_list[i].bd_addr);
        }
        if (logger != nullptr)
        {
            char buf[80];
            snprintf(buf, sizeof(buf), "BLE: cleared %d bonded device(s)", dev_num);
            logger->log(buf);
        }
    }
    free(dev_list);
}

void MouseTask::run()
{
    log("Mouse task started");

    led_manager_->setMode(LED_MODE_BT_PAIRING);

    // Free classic BT memory and bring up BLE before touching other peripherals
    esp_err_t bt_release = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if (bt_release != ESP_OK && bt_release != ESP_ERR_INVALID_STATE)
    {
        char buf[100];
        snprintf(buf, sizeof(buf), "BT mem release failed: %d", bt_release);
        log(buf);
    }
    bleMouse.begin();
    log("BLE Mouse init started");

    mouseSensor.setLogger(logger_);
    mouseSensor.init();
    mouseSensor.enableBurst();

    SPI.end();
    SPI.begin(PIN_VSPI_SCK, PIN_VSPI_MISO, PIN_VSPI_MOSI);

    const bool ads_ready = (adsController != nullptr) && adsController->init();
    if (!ads_ready)
    {
        log("ADS1220 not detected, click/press input disabled");
    }

    esp_reset_reason_t reason = esp_reset_reason();
    log(resetReasonToStr(reason));
    bool was_connected = false;

    int16_t dx = 0, dy = 0;
    bool mouseMotion = false;
    uint32_t last_mouse_spi_us = 0;
    const uint32_t spi_quiet_window_us = 300;
    static bool pressed;
    static int currentClickStatus = 0;
    static uint8_t press_readings;

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
            disconnect_streak_ = 0;
            last_connect_ms_ = millis();
        }
        else if (!is_connected && was_connected)
        {
            led_manager_->setMode(LED_MODE_BT_PAIRING);
            unsigned long since_connect = millis() - last_connect_ms_;
            if (since_connect < 3000)
            {
                disconnect_streak_ = std::min<uint8_t>(250, (uint8_t)(disconnect_streak_ + 1));
            }
            else
            {
                disconnect_streak_ = 0;
            }
            if (disconnect_streak_ >= 3)
            {
                log("BLE quick disconnects detected, clearing bonds");
                clearAllBleBonds(logger_);
                disconnect_streak_ = 0;
            }
            log("BLE Mouse disconnected, restarting advertising");
            BLEDevice::startAdvertising();
        }
        else if (!is_connected)
        {
            static unsigned long last_adv_kick = 0;
            if (millis() - last_adv_kick > 5000)
            {
                BLEDevice::startAdvertising();
                last_adv_kick = millis();
            }
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
            last_mouse_spi_us = micros();

            if (mouseMotion)
            {
                char buf[100];
                snprintf(buf, sizeof(buf), "Mouse Movement: {%i,%i}", -dx, -dy);
                log(buf);
                bleMouse.move(constrain(-dx, -127, 127),
                              constrain(-dy, -127, 127),
                              0);
                dx = dy = 0;
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

        if (ads_ready)
        {
            StrainData strain = adsController->read(false);

            // char buf[120];
            // snprintf(buf, sizeof(buf), "Force=%.3f X=%.3f Y=%.3f ClickStatus=%d", strain.force, strain.x, strain.y, strain.clickStatus);
            // log(buf);

            const bool in_spi_quiet_period = (micros() - last_mouse_spi_us) < spi_quiet_window_us;
            if (in_spi_quiet_period)
            {
                press_readings = 0;
            }
            else
            {
                if (!pressed && strain.force > 0.9f)
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
                else if (pressed && strain.force < 0.85f)
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
            }
        }
        else
        {
            press_readings = 0;
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
