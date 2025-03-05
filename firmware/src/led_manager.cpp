#include "led_manager.h"
#include "util.h"

LEDManager *LEDManager::instance_ = nullptr;

LEDManager::LEDManager()
{
    setMode(LED_MODE_NORMAL);
    brightness = 255;
}

LEDManager *LEDManager::getInstance()
{
    if (instance_ == nullptr)
    {
        instance_ = new LEDManager();
    }
    return instance_;
}

void LEDManager::setMode(uint8_t mode)
{
    std::lock_guard<std::mutex> lock(mutex_);
    this->current_mode_ = mode;
    if (mode == LED_MODE_NORMAL)
    {
        current_hue_ = 200;
    }
    else if (mode == LED_MODE_CALIBRATION)
    {
        current_hue_ = 32;
    }
    else if (mode == LED_MODE_BT_PAIRING)
    {
        current_hue_ = 160;
    }
    else if (mode == LED_MODE_ERROR)
    {
        current_hue_ = 0;
    }
    else
    {
        current_hue_ = 0;
    }

    for (uint8_t i = 0; i < NUM_LEDS; i++)
    {
        // leds[i].setHSV(current_hue_, 255 - 180 * CLAMP(press_value_unit, (float)0, (float)1) - 75 * pressed, brightness);
        leds[i].setHSV(current_hue_, 255, brightness);
        leds[i].r = dim8_video(leds[i].r);
        leds[i].g = dim8_video(leds[i].g);
        leds[i].b = dim8_video(leds[i].b);
    }
}

void LEDManager::updateState(float press_value_unit, bool pressed)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (current_mode_ == LED_MODE_NORMAL)
    {
        fill_solid(leds, NUM_LEDS, CHSV(current_hue_, 255, brightness));
        adjustGamma();
    }
    else if (current_mode_ == LED_MODE_BT_PAIRING)
    {
        int a = millis();
        byte b = triwave8(a / 15);

        byte breath = ease8InOutCubic(b);
        breath = map(breath, 0, 255, 50, brightness);
        fill_solid(leds, NUM_LEDS, CHSV(current_hue_, 255, breath));
        // fill_gradient(leds,NUM_LEDS,CHSV(0,255,MIN_BRIGHTNESS),CHSV(0,255,breath), CHSV(0,255,breath), CHSV(0,255,MIN_BRIGHTNESS),LONGEST_HUES);  // uncomment this to smooth gradient looking effect

        adjustGamma();
    }
}

void LEDManager::setBrightness(uint8_t brightness)
{
    std::lock_guard<std::mutex> lock(mutex_);
    this->brightness = brightness;
}

void LEDManager::updateLEDs()
{
    std::lock_guard<std::mutex> lock(mutex_);
    FastLED.show();
}

void LEDManager::init()
{
    std::lock_guard<std::mutex> lock(mutex_);
    FastLED.addLeds<SK6812, PIN_LED_DATA, GRB>(leds, NUM_LEDS);
}

void LEDManager::adjustGamma() // https://www.reddit.com/r/FastLED/comments/b2mlvf/gamma_correction/eity0u1?utm_source=share&utm_medium=web2x
{
    for (uint16_t i = 0; i < NUM_LEDS; i++)
    {
        leds[i].r = dim8_video(leds[i].r);
        leds[i].g = dim8_video(leds[i].g);
        leds[i].b = dim8_video(leds[i].b);
    }
}