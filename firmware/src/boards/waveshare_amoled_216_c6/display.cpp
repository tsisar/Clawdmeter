#include "../../hal/display_hal.h"
#include "../../hal/imu_hal.h"
#include "../../brightness.h"
#include "board.h"
#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <esp_heap_caps.h>
#include <lvgl.h>

// C6 AMOLED-2.16 uses a CO5300 AMOLED panel (per the Waveshare
// ESP32-C6-Touch-AMOLED-2.16 spec) — the same controller as the S3
// AMOLED-2.16 sibling, so we drive it with Arduino_CO5300 and reuse that
// class's vendor-correct init rather than the SH8601 class + a hand-patched
// sequence. LCD reset is not wired to any MCU GPIO; the panel boots from its
// internal power-on reset (rst = GFX_NOT_DEFINED).
//
// The CO5300's MADCTL only does axis flips, not row/column exchange, so 90°
// and 270° cannot come from the panel — rotation is a CPU pixel remap here in
// the flush path, exactly as on the S3 2.16.

// Render strip used when rotating in software. Must be at least as large as
// the biggest LVGL partial flush chunk, which main.cpp caps at
// LCD_WIDTH x BUF_LINES. This board has no PSRAM, so main.cpp's PSRAM-free
// branch applies (BUF_LINES=20) and the strip comes out of internal SRAM at
// 480 x 20 x 2 = 19 KB. Keep this in step with BUF_LINES in main.cpp.
#define ROT_BUF_LINES 20
static uint16_t* rot_buf = nullptr;

static Arduino_DataBus* bus = nullptr;
static Arduino_CO5300*  gfx = nullptr;

void display_hal_init(void) {
    bus = new Arduino_ESP32QSPI(
        LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3);
    // CO5300 constructor: (bus, rst, rotation, w, h, col_off1..2, row_off1..2).
    // No reset GPIO on this board; the 480-wide panel is full-width so all
    // offsets are 0 — matches the S3 AMOLED-2.16 instantiation.
    gfx = new Arduino_CO5300(
        bus, GFX_NOT_DEFINED, 0 /* rotation disabled */,
        LCD_WIDTH, LCD_HEIGHT, 0, 0, 0, 0);
}

// Arduino_CO5300::begin() already issues SLPOUT, SPI-mode control, pixel
// format, brightness-control, DISPON and a default MADCTL. The ONLY thing it
// does not set is this panel's manufacturer page-0x20 driving-voltage
// registers (0x19/0x1C) — without them the panel stays black even with the
// rails up. Set just those; everything else the SH8601-era hack also wrote
// (0xC4/0x36/0x53/0x51/0x63/0x29) is now covered by the class init.
//
// Note: we deliberately do NOT restore the old MADCTL 0x30 (MV transpose).
// The CO5300 class default (rotation-0, MADCTL 0x00) orients the panel with
// the USB port on the side, which is the preferred desk orientation for this
// board.
static void send_panel_driving_init(Arduino_DataBus* b) {
    b->beginWrite();
    b->writeC8D8(0xFE, 0x20);    // enter manufacturer command page 0x20
    b->writeC8D8(0x19, 0x10);    // panel driving voltage
    b->writeC8D8(0x1C, 0xA0);    // panel driving voltage
    b->writeC8D8(0xFE, 0x00);    // back to user command page
    b->endWrite();
    delay(20);
}

void display_hal_begin(void) {
    gfx->begin();
    send_panel_driving_init(bus);   // panel-specific regs the class init omits
    gfx->fillScreen(0x0000);
    gfx->setBrightness(200);

    // Rotation strip from internal SRAM — there is no PSRAM on this SoC. On
    // failure rot_buf stays null and the draw path silently falls back to
    // unrotated output rather than dropping frames.
    rot_buf = (uint16_t*)heap_caps_malloc(LCD_WIDTH * ROT_BUF_LINES * 2,
                                          MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!rot_buf) Serial.println("Rotation strip alloc failed — rotation disabled");
}

void display_hal_set_brightness(uint8_t level) {
    if (gfx) gfx->setBrightness(level);
}

void display_hal_fill_screen(uint16_t color) {
    if (gfx) gfx->fillScreen(color);
}

// Rotate a w×h strip into rot_buf and compute destination coordinates on the
// 480×480 panel. Src is row-major over the rectangle (sx, sy, w, h). The panel
// is square, so one side length serves both axes.
static void rotate_strip(const uint16_t* src, int32_t w, int32_t h,
                         int32_t sx, int32_t sy, uint8_t r,
                         int32_t* dx, int32_t* dy, int32_t* dw, int32_t* dh) {
    const int S = LCD_WIDTH;

    switch (r) {
    case 1: // 90° CW: (x,y) -> (S-1-y, x)
        *dw = h; *dh = w;
        *dx = S - sy - h;
        *dy = sx;
        for (int32_t y = 0; y < h; y++) {
            for (int32_t x = 0; x < w; x++) {
                rot_buf[x * h + (h - 1 - y)] = src[y * w + x];
            }
        }
        break;
    case 2: // 180°: (x,y) -> (S-1-x, S-1-y)
        *dw = w; *dh = h;
        *dx = S - sx - w;
        *dy = S - sy - h;
        for (int32_t y = 0; y < h; y++) {
            for (int32_t x = 0; x < w; x++) {
                rot_buf[(h - 1 - y) * w + (w - 1 - x)] = src[y * w + x];
            }
        }
        break;
    case 3: // 270° CW: (x,y) -> (y, S-1-x)
        *dw = h; *dh = w;
        *dx = sy;
        *dy = S - sx - w;
        for (int32_t y = 0; y < h; y++) {
            for (int32_t x = 0; x < w; x++) {
                rot_buf[(w - 1 - x) * h + y] = src[y * w + x];
            }
        }
        break;
    default:
        *dx = sx; *dy = sy; *dw = w; *dh = h;
        break;
    }
}

void display_hal_draw_bitmap(int32_t x, int32_t y, int32_t w, int32_t h,
                             const uint16_t* pixels) {
    if (!gfx) return;
    uint8_t r = imu_hal_rotation_quadrant();
    // LVGL's partial chunking already keeps w*h within one draw buffer, which
    // is what rot_buf is sized to. The check is insurance against BUF_LINES in
    // main.cpp drifting away from ROT_BUF_LINES: degrade to an unrotated frame
    // rather than run off the end of the strip.
    if (r == 0 || !rot_buf || w * h > LCD_WIDTH * ROT_BUF_LINES) {
        gfx->draw16bitRGBBitmap(x, y, (uint16_t*)pixels, w, h);
        return;
    }
    int32_t dx, dy, dw, dh;
    rotate_strip(pixels, w, h, x, y, r, &dx, &dy, &dw, &dh);
    gfx->draw16bitRGBBitmap(dx, dy, rot_buf, dw, dh);
}

// On rotation change, blank the panel, force a full LVGL redraw at the new
// orientation, then ramp brightness back up over ~125ms so the transition
// reads as deliberate rather than as a glitch.
void display_hal_tick(void) {
    static uint8_t  last_rotation = 0;
    static uint8_t  ramp_step = 0;     // 0=idle, 1..4=ramping
    static uint32_t ramp_last = 0;

    uint8_t rot = imu_hal_rotation_quadrant();
    if (rot != last_rotation) {
        display_hal_set_brightness(0);
        last_rotation = rot;
        lv_obj_invalidate(lv_screen_active());
        ramp_step = 1;
        return;
    }

    if (ramp_step == 0) return;
    uint32_t now = millis();
    if (now - ramp_last < 25) return;
    ramp_last = now;

    // Ramp back to the user's chosen brightness (not a hardcoded level), so a
    // physical rotation doesn't reset what they set via PWR-short-press.
    static const uint8_t pct[] = {30, 60, 85, 100};
    uint8_t target = brightness_get();
    display_hal_set_brightness((uint8_t)(((uint16_t)target * pct[ramp_step - 1]) / 100));
    if (ramp_step >= 4) ramp_step = 0;
    else                ramp_step++;
}

// CO5300 requires even-aligned flush regions.
void display_hal_round_area(int32_t* x1, int32_t* y1, int32_t* x2, int32_t* y2) {
    *x1 = *x1 & ~1;
    *y1 = *y1 & ~1;
    *x2 = *x2 | 1;
    *y2 = *y2 | 1;
}
