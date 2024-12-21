#define LGFX_USE_V1

#include <PCA9557.h>
#include <lvgl.h>
#include <DHT20.h>
#include <LovyanGFX.hpp>
#include <lgfx/v1/platforms/esp32s3/Panel_RGB.hpp>
#include <lgfx/v1/platforms/esp32s3/Bus_RGB.hpp>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include "ui.h"

#include <driver/i2c.h>
//#include <TAMC_GT911.h>


#define TFT_BL 2
#define LED_PIN 38
#define DHT_SDA 19
#define DHT_SCL 20

class LGFX : public lgfx::LGFX_Device
{
public:
  lgfx::Bus_RGB _bus_instance;
  lgfx::Panel_RGB _panel_instance;
  lgfx::Light_PWM _light_instance;
  lgfx::Touch_GT911 _touch_instance;
  LGFX(void)
  {
    {
      auto cfg = _panel_instance.config();
      cfg.memory_width  = 800;
      cfg.memory_height = 480;
      cfg.panel_width   = 800;
      cfg.panel_height  = 480;
      cfg.offset_x      = 0;
      cfg.offset_y      = 0;
      _panel_instance.config(cfg);
    }

    {
      auto cfg = _bus_instance.config();
      cfg.panel = &_panel_instance;

      cfg.pin_d0  = GPIO_NUM_15; // B0
      cfg.pin_d1  = GPIO_NUM_7;  // B1
      cfg.pin_d2  = GPIO_NUM_6;  // B2
      cfg.pin_d3  = GPIO_NUM_5;  // B3
      cfg.pin_d4  = GPIO_NUM_4;  // B4

      cfg.pin_d5  = GPIO_NUM_9;  // G0
      cfg.pin_d6  = GPIO_NUM_46; // G1
      cfg.pin_d7  = GPIO_NUM_3;  // G2
      cfg.pin_d8  = GPIO_NUM_8;  // G3
      cfg.pin_d9  = GPIO_NUM_16; // G4
      cfg.pin_d10 = GPIO_NUM_1;  // G5

      cfg.pin_d11 = GPIO_NUM_14; // R0
      cfg.pin_d12 = GPIO_NUM_21; // R1
      cfg.pin_d13 = GPIO_NUM_47; // R2
      cfg.pin_d14 = GPIO_NUM_48; // R3
      cfg.pin_d15 = GPIO_NUM_45; // R4

      cfg.pin_henable = GPIO_NUM_41;
      cfg.pin_vsync   = GPIO_NUM_40;
      cfg.pin_hsync   = GPIO_NUM_39;
      cfg.pin_pclk    = GPIO_NUM_0;
      cfg.freq_write  = 12000000;

      cfg.hsync_polarity    = 0;
      cfg.hsync_front_porch = 40;
      cfg.hsync_pulse_width = 48;
      cfg.hsync_back_porch  = 40;

      cfg.vsync_polarity    = 0;
      cfg.vsync_front_porch = 1;
      cfg.vsync_pulse_width = 31;
      cfg.vsync_back_porch  = 13;

      cfg.pclk_active_neg = 1;
      cfg.de_idle_high    = 0;
      cfg.pclk_idle_high  = 0;

      _bus_instance.config(cfg);
    }
    _panel_instance.setBus(&_bus_instance);

    {
      auto cfg = _light_instance.config();
      cfg.pin_bl = GPIO_NUM_2;
      _light_instance.config(cfg);
    }
    _panel_instance.light(&_light_instance);

    {
      auto cfg = _touch_instance.config();
      cfg.x_min       = 0;
      cfg.x_max       = 799;
      cfg.y_min       = 0;
      cfg.y_max       = 479;
      cfg.pin_int     = -1;
      cfg.pin_rst     = -1;
      cfg.bus_shared  = false;
      cfg.offset_rotation = 0;
      cfg.i2c_port    = I2C_NUM_1;
      cfg.pin_sda     = GPIO_NUM_19;
      cfg.pin_scl     = GPIO_NUM_20;
      cfg.freq        = 400000;
      cfg.i2c_addr    = 0x14;
      _touch_instance.config(cfg);
      _panel_instance.setTouch(&_touch_instance);
    }
    setPanel(&_panel_instance);
  }
};

LGFX lcd; //

#include "touch.h"

DHT20 dht20; //

static uint32_t screenWidth;
static uint32_t screenHeight;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t disp_draw_buf[800 * 480 / 15];
static lv_disp_drv_t disp_drv;
int led = 0;

void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);
    
    lcd.pushImageDMA(area->x1, area->y1, w, h, (lgfx::rgb565_t*)&color_p->full);
    lv_disp_flush_ready(disp);
}

void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {
    if (touch_has_signal()) {
        if (touch_touched()) {
            data->state = LV_INDEV_STATE_PR;
            data->point.x = touch_last_x;
            data->point.y = touch_last_y;
            Serial.printf("Touch: x=%d, y=%d\n", data->point.x, data->point.y);
        } else if (touch_released()) {
            data->state = LV_INDEV_STATE_REL;
        }
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
    delay(15);
}

void initDisplay() {
    lcd.begin();
    lcd.fillScreen(TFT_BLACK);
    lcd.setTextSize(2);
    
    ledcSetup(1, 300, 8);
    ledcAttachPin(TFT_BL, 1);
    ledcWrite(1, 255);
    
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, LOW);
    delay(500);
    digitalWrite(TFT_BL, HIGH);
}

void initLVGL() {
    lv_init();
    touch_init();

    screenWidth = lcd.width();
    screenHeight = lcd.height();

    lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, NULL, screenWidth * screenHeight / 15);

    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);
}

void setup() {
    Serial.begin(9600);
    
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    
    Wire.begin(DHT_SDA, DHT_SCL);
    dht20.begin();

    initDisplay();
    initLVGL();
    ui_init();
    
    lv_timer_handler();
}

void updateSensorData() {
    char buffer[6];
    int temperature = (int)dht20.getTemperature();
    int humidity = (int)dht20.getHumidity();
    
    snprintf(buffer, sizeof(buffer), "%d", temperature);
    lv_label_set_text(ui_Label1, buffer);
    
    snprintf(buffer, sizeof(buffer), "%d", humidity);
    lv_label_set_text(ui_Label2, buffer);
}

void loop() {
    updateSensorData();
    
    digitalWrite(LED_PIN, led ? HIGH : LOW);
    
    lv_timer_handler();
    delay(10);
}