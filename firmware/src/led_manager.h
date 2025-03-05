#include <FastLED.h>
#include <mutex>

#define LED_MODE_NORMAL 0
#define LED_MODE_CALIBRATION 1
#define LED_MODE_BT_PAIRING 2
#define LED_MODE_ERROR 3

class LEDManager
{
private:
    static LEDManager *instance_;
    std::mutex mutex_;

    uint8_t current_mode_;
    uint8_t current_hue_;
    uint8_t brightness;

    LEDManager();

    CRGB leds[NUM_LEDS];

    void adjustGamma();

public:
    static LEDManager *getInstance();

    void init();

    void setMode(uint8_t mode);

    void updateState(float press_value_unit, bool pressed = false);

    void setBrightness(uint8_t brightness);

    void updateLEDs();
};