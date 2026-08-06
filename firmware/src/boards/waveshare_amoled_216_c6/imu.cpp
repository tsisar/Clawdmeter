#include "../../hal/imu_hal.h"
#include "board.h"
#include <Arduino.h>
#include <Wire.h>
#include <SensorQMI8658.hpp>

// QMI8658 accelerometer drives auto-rotation, same as the S3 AMOLED-2.16
// sibling: the CO5300 cannot rotate in hardware, so this quadrant is consumed
// by display.cpp (CPU pixel remap in the flush path) and touch.cpp (inverse
// remap of the reported point).
//
// The gate is deliberately slow — a quadrant flip forces a full-screen redraw,
// which on the C6's single RISC-V core costs enough that jitter would show.

#define IMU_POLL_MS       100    // ~10 Hz
#define STABLE_TIME_MS    300    // orientation must hold this long before rotating
#define TILT_THRESHOLD    0.5f   // ~30° from axis (sin 30° ≈ 0.5)

static SensorQMI8658 imu;
static uint8_t  current_rotation   = 0;
static uint8_t  candidate_rotation = 0;
static uint32_t candidate_since    = 0;
static uint32_t last_poll_ms       = 0;
static bool     imu_ok             = false;

// Accelerometer axes → quadrant. Quadrant 0 is the CO5300 class default (USB
// port on the side) that touch.cpp's swap/mirror settings are calibrated
// against.
//
// The X branch is INVERTED relative to the S3 2.16 port. Both mappings were
// established on hardware, since the QMI8658's mounting orientation on this
// carrier is not documented anywhere: with the S3 mapping verbatim the two
// side-on positions came out on each other's quadrant, and it is the X branch
// that drives those on this board — not Y, as the S3's layout would suggest.
static uint8_t accel_to_rotation(float ax, float ay) {
    float abs_ax = fabsf(ax);
    float abs_ay = fabsf(ay);
    if (abs_ax < TILT_THRESHOLD && abs_ay < TILT_THRESHOLD) {
        return 255;  // ambiguous (face-up/down) — hold the current quadrant
    }
    if (abs_ay > abs_ax) return (ay > 0) ? 3 : 1;
    return (ax > 0) ? 2 : 0;
}

void imu_hal_init(void) {
    if (!imu.begin(Wire, QMI8658_L_SLAVE_ADDRESS, IIC_SDA, IIC_SCL)) {
        Serial.println("QMI8658 init failed — rotation stays at quadrant 0");
        return;
    }
    imu.configAccelerometer(
        SensorQMI8658::ACC_RANGE_4G,
        SensorQMI8658::ACC_ODR_LOWPOWER_21Hz,
        SensorQMI8658::LPF_MODE_3);
    imu.enableAccelerometer();
    imu_ok = true;
    Serial.println("QMI8658 init OK (auto-rotation enabled)");
}

void imu_hal_tick(void) {
    if (!imu_ok) return;
    uint32_t now = millis();
    if (now - last_poll_ms < IMU_POLL_MS) return;
    last_poll_ms = now;

    float ax, ay, az;
    if (!imu.getAccelerometer(ax, ay, az)) return;

    uint8_t target = accel_to_rotation(ax, ay);
    if (target == 255 || target == current_rotation) {
        candidate_rotation = current_rotation;
        return;
    }
    if (target != candidate_rotation) {
        candidate_rotation = target;
        candidate_since = now;
    } else if (now - candidate_since >= STABLE_TIME_MS) {
        current_rotation = target;
        Serial.printf("Rotation: %d\n", current_rotation);
    }
}

uint8_t imu_hal_rotation_quadrant(void) { return current_rotation; }