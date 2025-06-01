#include "logger.h"
#include "task.h"
#include "proto_gen/smartknob.pb.h"
#include "freertos/queue.h"
#include <BleMouse.h>
#include "compass_sensor.h"
#include "led_manager.h"
#include "pmw3389_sensor.h"
#include "ads1220_adc.h"
#include "motor_task.h"
#include "configuration.h"

class MouseTask : public Task<MouseTask>
{
    friend class Task<MouseTask>; // Allow base Task to invoke protected run()

public:
    MouseTask(const uint8_t task_core, MotorTask &motor_task);
    ~MouseTask();

    QueueHandle_t getKnobStateQueue();
    void setLogger(Logger *logger);

    void setCompassSensor(CompassSensor *compassSensor) { compass_sensor_ = compassSensor; }

    void setLEDManager(LEDManager *led_manager) { led_manager_ = led_manager; }

    void setConfiguration(Configuration *config)
    {
        configuration_ = config;
    }

protected:
    void run();

private:
    MotorTask &motor_task_;
    Configuration *configuration_;

    BleMouse bleMouse = BleMouse("ScrollWheel", "ESP32S3", 100);
    PMW3389 mouseSensor;
    ADS1220Controller adsController;

    QueueHandle_t knob_state_queue_;
    CompassSensor *compass_sensor_;

    LEDManager *led_manager_ = nullptr;

    uint8_t press_count_;

    PB_SmartKnobState state_;
    PB_SmartKnobState previous_state_;
    SemaphoreHandle_t mutex_;
    Logger *logger_;
    void log(const char *msg);
};
