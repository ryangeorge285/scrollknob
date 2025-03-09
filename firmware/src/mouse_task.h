#include "logger.h"
#include "task.h"
#include "proto_gen/smartknob.pb.h"
#include "freertos/queue.h"
#include <BleMouse.h>
#include "compass_sensor.h"

class MouseTask : public Task<MouseTask>
{
    friend class Task<MouseTask>; // Allow base Task to invoke protected run()

public:
    MouseTask(const uint8_t task_core);
    ~MouseTask();

    QueueHandle_t getKnobStateQueue();
    void setLogger(Logger *logger);

    void setCompassSensor(CompassSensor *compassSensor) { compass_sensor_ = compassSensor; }

protected:
    void run();

private:
    BleMouse bleMouse = BleMouse("ScrollWheel", "ESP32S3", 100);

    QueueHandle_t knob_state_queue_;
    CompassSensor *compass_sensor_;

    PB_SmartKnobState state_;
    PB_SmartKnobState previous_state_;
    SemaphoreHandle_t mutex_;
    Logger *logger_;
    void log(const char *msg);
};
