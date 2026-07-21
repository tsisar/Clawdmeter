#pragma once
#include "data.h"
#include "ble.h"

enum screen_t {
    SCREEN_SPLASH,
    SCREEN_USAGE,
    SCREEN_COUNT,
};

void ui_init(void);
void ui_update(const UsageData* data);
void ui_tick_anim(void);
void ui_show_screen(screen_t screen);
void ui_toggle_splash(void);
screen_t ui_get_current_screen(void);
void ui_update_ble_status(ble_state_t state, const char* name, const char* mac);
void ui_update_battery(int percent, bool charging);
// Pairing window: render the state ble_pairing_view() reports. The code is
// only meaningful in the CONFIRM and PASSKEY views — in WAITING there is no
// code yet, because the host has not chosen a pairing method.
void ui_update_pairing(ble_pair_view_t view, uint32_t code);
// Hold-to-pair gesture feedback. 0 = idle, 1 = holding (keep going),
// 2 = armed (release now). Floats above whatever screen is showing.
void ui_update_pair_arm(int stage);
