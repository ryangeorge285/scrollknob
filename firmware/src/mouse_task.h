#include "logger.h"
#include "task.h"
#include "proto_gen/smartknob.pb.h"
#include "freertos/queue.h"
#include <BleMouse.h>

class MouseTask : public Task<MouseTask>
{
    friend class Task<MouseTask>; // Allow base Task to invoke protected run()

public:
    MouseTask(const uint8_t task_core);
    ~MouseTask();

    QueueHandle_t getKnobStateQueue();
    void setLogger(Logger *logger);

protected:
    void run();

private:
    BleMouse bleMouse = BleMouse("ScrollWheel", "ESP32S3", 100);

    QueueHandle_t knob_state_queue_;

    PB_SmartKnobState state_;
    PB_SmartKnobState previous_state_;
    SemaphoreHandle_t mutex_;
    Logger *logger_;
    void log(const char *msg);
};
