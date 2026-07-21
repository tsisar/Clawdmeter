#pragma once
#include <stdint.h>

enum ble_state_t {
    BLE_STATE_INIT,
    BLE_STATE_ADVERTISING,
    BLE_STATE_CONNECTED,
    BLE_STATE_DISCONNECTED,
};

void ble_init(void);
void ble_tick(void);
ble_state_t ble_get_state(void);
const char* ble_get_device_name(void);
const char* ble_get_mac_address(void);
void ble_clear_bonds(void);
bool ble_has_bonds(void);

// Pairing window: opened by the hold-PWR gesture. While open the device is
// discoverable by name and displays a 4-digit PIN the connecting host must
// type (BLE passkey entry). Outside the window the device advertises
// anonymously and new pairings are impossible.
void ble_pairing_begin(void);
void ble_pairing_cancel(void);
bool ble_pairing_active(void);

// What the pairing screen should be showing. Which of the two code flows runs
// is the HOST's choice, not ours — the SMP IO-capability matrix decides it
// only once the host starts pairing, so the device cannot know upfront and
// must not display a code before then.
enum ble_pair_view_t {
    BLE_PAIR_VIEW_NONE = 0,
    BLE_PAIR_VIEW_WAITING,   // window open, host hasn't started pairing yet
    BLE_PAIR_VIEW_CONFIRM,   // numeric comparison: show code, PWR confirms
    BLE_PAIR_VIEW_PASSKEY,   // passkey display: host types this code
};
ble_pair_view_t ble_pairing_view(void);
uint32_t        ble_pairing_code(void);  // meaningless unless view is CONFIRM/PASSKEY

// Numeric comparison. GUI Bluetooth agents (GNOME, Windows) declare
// DisplayYesNo, which per the IO-capability matrix means "both sides show the
// same number and the human confirms it" — they cannot do the
// type-the-device's-code flow at all.
bool ble_confirm_pending(void);
void ble_confirm_accept(void);
void ble_confirm_reject(void);
bool ble_has_data(void);
const char* ble_get_data(void);
void ble_send_ack(void);
void ble_send_nack(void);
void ble_request_refresh(void);

// BLE HID keyboard
void ble_keyboard_press(uint8_t key, uint8_t modifier);
void ble_keyboard_release(void);
