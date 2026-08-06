#include "../../hal/touch_hal.h"
#include "../../hal/imu_hal.h"
#include "board.h"
#include <Arduino.h>
#include <Wire.h>
#include <TouchDrvCSTXXX.hpp>

static TouchDrvCST92xx touch;

static volatile bool     touch_data_ready = false;
static volatile bool     touch_pressed = false;
static volatile uint16_t touch_x = 0;
static volatile uint16_t touch_y = 0;

static void IRAM_ATTR touch_isr(void) {
    touch_data_ready = true;
}

void touch_hal_init(void) {
    touch.setPins(TP_RST, TP_INT);
    if (!touch.begin(Wire, CST9220_ADDR, IIC_SDA, IIC_SCL)) {
        Serial.println("Touch init failed");
        return;
    }
    touch.setMaxCoordinates(LCD_WIDTH, LCD_HEIGHT);
    // C6 2.16 panel mapping. The original values (swap=true, mirrorX=true)
    // were calibrated to the old display orientation that force-wrote MADCTL
    // 0x30 (MV transpose + ML). The display now runs at the CO5300 class
    // default (MADCTL 0x00 — USB-port-on-the-side orientation), so the touch
    // mapping is re-derived to match. SensorLib applies swap then mirror
    // (TouchDrvInterface::updateXY); tap-tested on C6 hardware, the raw
    // CST9217 coordinates map straight through in this orientation — no swap,
    // no mirror.
    touch.setSwapXY(false);
    touch.setMirrorXY(false, false);
    pinMode(TP_INT, INPUT_PULLUP);
    attachInterrupt(TP_INT, touch_isr, FALLING);
    Serial.println("Touch init OK");
}

// The controller is glued to the panel, so it always reports in physical panel
// space. LVGL, meanwhile, lays out on an unrotated canvas and display.cpp maps
// that canvas onto the panel at flush time. Undo that map here, or every tap
// lands in the wrong place once the board is turned.
//
// display.cpp sends LVGL (x,y) to the panel as:
//   r=1  -> (S-1-y, x)      r=2 -> (S-1-x, S-1-y)      r=3 -> (y, S-1-x)
// so the inverses below bring a panel point back into LVGL space.
static void unrotate_point(uint16_t* x, uint16_t* y) {
    const uint16_t S  = LCD_WIDTH;   // square panel: one side serves both axes
    const uint16_t px = *x, py = *y;

    switch (imu_hal_rotation_quadrant()) {
    case 1: *x = py;         *y = S - 1 - px; break;
    case 2: *x = S - 1 - px; *y = S - 1 - py; break;
    case 3: *x = S - 1 - py; *y = px;         break;
    default: break;                  // quadrant 0 — panel space is LVGL space
    }
}

void touch_hal_read(uint16_t* x, uint16_t* y, bool* pressed) {
    if (touch_data_ready) {
        touch_data_ready = false;
        int16_t tx[5], ty[5];
        uint8_t n = touch.getPoint(tx, ty, touch.getSupportTouchPoint());
        if (n > 0) {
            touch_pressed = true;
            touch_x = (uint16_t)tx[0];
            touch_y = (uint16_t)ty[0];
        } else {
            touch_pressed = false;
        }
    }
    *x = touch_x;
    *y = touch_y;
    unrotate_point(x, y);
    *pressed = touch_pressed;
}
