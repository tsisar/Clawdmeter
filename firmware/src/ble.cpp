#include "ble.h"
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <NimBLEHIDDevice.h>
#include <Preferences.h>
#include <esp_random.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#define DEVICE_NAME "Clawdmeter"

// Custom GATT UUIDs for data channel
#define SERVICE_UUID        "4c41555a-4465-7669-6365-000000000001"
#define RX_CHAR_UUID        "4c41555a-4465-7669-6365-000000000002"  // host writes here
#define TX_CHAR_UUID        "4c41555a-4465-7669-6365-000000000003"  // device ack/nack notifies
#define REQ_CHAR_UUID       "4c41555a-4465-7669-6365-000000000004"  // device-initiated refresh request

#define BLE_BUF_SIZE 512

// HID keyboard report descriptor (standard 6-KRO boot-protocol-compatible).
// Includes the LED output report (Num/Caps/Scroll Lock indicators) — without
// it macOS's Keyboard Setup Assistant flags the device as "unidentifiable"
// because the descriptor doesn't look like a complete keyboard.
static const uint8_t HID_REPORT_MAP[] = {
    0x05, 0x01,  // Usage Page (Generic Desktop)
    0x09, 0x06,  // Usage (Keyboard)
    0xA1, 0x01,  // Collection (Application)
    0x85, 0x01,  //   Report ID (1)
    0x05, 0x07,  //   Usage Page (Key Codes)
    0x19, 0xE0,  //   Usage Minimum (224) - Left Control
    0x29, 0xE7,  //   Usage Maximum (231) - Right GUI
    0x15, 0x00,  //   Logical Minimum (0)
    0x25, 0x01,  //   Logical Maximum (1)
    0x75, 0x01,  //   Report Size (1)
    0x95, 0x08,  //   Report Count (8)
    0x81, 0x02,  //   Input (Data, Variable, Absolute) - Modifier byte
    0x95, 0x01,  //   Report Count (1)
    0x75, 0x08,  //   Report Size (8)
    0x81, 0x01,  //   Input (Constant) - Reserved byte
    // LED output report — required for macOS to treat this as a full keyboard.
    0x95, 0x05,  //   Report Count (5)
    0x75, 0x01,  //   Report Size (1)
    0x05, 0x08,  //   Usage Page (LEDs)
    0x19, 0x01,  //   Usage Minimum (Num Lock)
    0x29, 0x05,  //   Usage Maximum (Kana)
    0x91, 0x02,  //   Output (Data, Variable, Absolute) - LED report
    0x95, 0x01,  //   Report Count (1)
    0x75, 0x03,  //   Report Size (3)
    0x91, 0x01,  //   Output (Constant) - LED report padding
    0x95, 0x06,  //   Report Count (6)
    0x75, 0x08,  //   Report Size (8)
    0x15, 0x00,  //   Logical Minimum (0)
    0x25, 0x65,  //   Logical Maximum (101)
    0x05, 0x07,  //   Usage Page (Key Codes)
    0x19, 0x00,  //   Usage Minimum (0)
    0x29, 0x65,  //   Usage Maximum (101)
    0x81, 0x00,  //   Input (Data, Array) - Key array (6 keys)
    0xC0,        // End Collection
};

static NimBLEServer* server = nullptr;
static NimBLEHIDDevice* hid_dev = nullptr;
static NimBLECharacteristic* input_kbd = nullptr;
static NimBLECharacteristic* tx_char = nullptr;
static NimBLECharacteristic* rx_char = nullptr;
static NimBLECharacteristic* req_char = nullptr;

static ble_state_t state = BLE_STATE_INIT;
static bool need_advertise = false;

// One-shot supervision-timeout pushback (see onConnParamsUpdate). Written by
// NimBLE host-task callbacks, consumed by ble_tick() on the loop task.
static const uint16_t CONN_HANDLE_NONE  = 0xFFFF;
static const uint16_t DESIRED_TIMEOUT   = 600;   // ×10ms = 6s, matches PPCP
static volatile uint16_t param_fix_handle = CONN_HANDLE_NONE;  // pending retry
static volatile uint32_t param_fix_at_ms  = 0;                 // when to send it
static volatile uint16_t param_fix_spent  = CONN_HANDLE_NONE;  // one per connection
static char rx_buf[BLE_BUF_SIZE];
static volatile bool data_ready = false;
static volatile bool has_received_data = false;
static char mac_str[18];

// --- Owner allow-list + pairing window --------------------------------------
//
// The board is a BLE peripheral that any central in range could connect to and
// write usage data to. To stop the display showing a stranger's account, it
// keeps an allow-list of up to MAX_OWNERS identity addresses, persisted in NVS
// in most-recently-connected-first order. Only a listed owner (over a
// bonded+encrypted link) may write usage data; the display shows whichever
// owner wrote last.
//
// Admission is gated by a pairing window (the hold-PWR gesture): while it is
// open the device advertises its name and displays a 4-digit PIN that the
// connecting host must type (BLE passkey entry, MITM). A machine that pairs
// then lands in slot 0; when the list is full the last slot — the machine
// that has been away longest — is evicted and its bond deleted. Outside the
// window the device advertises anonymously (no name) and new pairings cannot
// complete: the passkey handed to the stack is random and never displayed.
#define MAX_OWNERS 3
#define PAIRING_WINDOW_MS 60000  // PIN shown / name advertised for this long
#define PAIRING_GRACE_MS  30000  // PIN typed near the deadline still admits
static Preferences prefs;
static char owner_addrs[MAX_OWNERS][18] = {};  // identity addrs, MRU-first
// Per-slot "was admitted through the PIN" flag, persisted alongside the
// addresses. Slots admitted via the pairing window are authenticated; slots
// adopted from a pre-MITM build are not. An authenticated slot may only ever
// re-encrypt over an authenticated link, so a stranger spoofing that owner's
// address cannot Just-Works its way back into the slot.
static bool owner_auth[MAX_OWNERS] = {};
static int  owner_count = 0;
static const char* ZERO_ADDR = "00:00:00:00:00:00";

// The owner list and pairing state are touched from two tasks: NimBLE's host
// task (every callback below) and the Arduino loop task (ble_tick, the gesture
// entry points, ble_clear_bonds). owner_touch/owner_admit_front memmove the
// arrays, so an unguarded reader can compare against a half-shifted address.
// This mutex is held ONLY around in-RAM list work — never across a NimBLE call
// or an NVS write, either of which could block the host task or deadlock.
static SemaphoreHandle_t owner_mux = nullptr;
#define OWNERS_LOCK()   do { if (owner_mux) xSemaphoreTakeRecursive(owner_mux, portMAX_DELAY); } while (0)
#define OWNERS_UNLOCK() do { if (owner_mux) xSemaphoreGiveRecursive(owner_mux); } while (0)

// NVS writes take tens of ms and must not happen on the BLE host task — a
// stalled host task shows up as supervision timeouts. Callbacks mark the list
// dirty; ble_tick (loop task) does the actual persisting.
static volatile bool owners_dirty = false;

static volatile bool     pairing_mode = false;    // window open: discoverable + PIN on screen
static uint32_t          pairing_deadline_ms = 0; // when the window closes
static uint32_t          admission_until_ms = 0;  // deadline + grace
static volatile bool     admission_armed = false; // guards admission_until_ms against millis() wrap
// Passkey-display path only (KeyboardOnly hosts). Minted inside
// onPassKeyDisplay rather than when the window opens: the host picks the
// pairing method, so a code shown upfront is usually the wrong one — the
// common GUI path is numeric comparison with a completely different number.
static volatile uint32_t pairing_pin_val = 0;
static volatile bool     passkey_shown = false;
static volatile bool     pin_requested = false;   // stack ran an authenticated flow this window
// Which connection that flow belongs to. Without it, machine A's routine
// reconnect would close the window out from under machine B, which is still
// looking at a code. CONN_HANDLE_NONE means "unknown peer" (the passkey-display
// callback carries no connection info), which falls back to closing on any
// completed ceremony.
static volatile uint16_t ceremony_handle = 0xFFFF;

// Numeric-comparison state. onConfirmPassKey lands on the NimBLE host task and
// must return immediately — a human takes seconds to look at the screen — so it
// parks the request here and the loop task injects the answer on a button press.
static volatile bool     confirm_pending = false;
static volatile uint16_t confirm_handle  = 0xFFFF;
static volatile uint32_t confirm_code    = 0;

// Bonds that already existed at boot with an empty owner list — the only
// addresses the pre-allow-list migration path may adopt (see
// onAuthenticationComplete). Snapshotted once in ble_init.
static char legacy_bonds[MAX_OWNERS][18] = {};
static int  legacy_bond_count = 0;

static bool is_legacy_bond(const char* id) {
    for (int i = 0; i < legacy_bond_count; i++) {
        if (strcmp(legacy_bonds[i], id) == 0) return true;
    }
    return false;
}

// Admission stays open for a grace period past the visible window so a PIN
// typed just before the deadline still lands (SMP finishes after expiry).
// The armed flag matters: a bare timestamp comparison flips back to "open"
// once millis() has advanced ~24.8 days past the deadline and the signed
// difference wraps, silently re-opening admission. ble_tick disarms it.
static bool admission_open() {
    return admission_armed && (int32_t)(millis() - admission_until_ms) < 0;
}

// NVS key for owner slot i: "owner" for slot 0 (back-compat with the
// single-owner builds that shipped before), "owner1"/"owner2" after.
static const char* owner_key(int i, char* buf, size_t len) {
    if (i == 0) return "owner";
    snprintf(buf, len, "owner%d", i);
    return buf;
}

// Callers that act on the result must hold the lock across both the lookup and
// the action — the index is only meaningful while the array is still.
static int owner_index(const char* id) {
    for (int i = 0; i < owner_count; i++) {
        if (strcmp(owner_addrs[i], id) == 0) return i;
    }
    return -1;
}

// Persist the list. MUST run on the loop task only (NVS blocks); callers on
// the host task set owners_dirty instead and let ble_tick get here. Snapshots
// under the lock, then writes with the lock released.
static void persist_owners_now() {
    char    snap[MAX_OWNERS][18];
    bool    snap_auth[MAX_OWNERS];
    int     snap_count;
    OWNERS_LOCK();
    memcpy(snap, owner_addrs, sizeof(snap));
    memcpy(snap_auth, owner_auth, sizeof(snap_auth));
    snap_count = owner_count;
    OWNERS_UNLOCK();

    char key[8];
    uint8_t auth_mask = 0;
    prefs.begin("clawd", false);
    for (int i = 0; i < MAX_OWNERS; i++) {
        const char* k = owner_key(i, key, sizeof(key));
        if (i < snap_count) {
            prefs.putString(k, snap[i]);
            if (snap_auth[i]) auth_mask |= (1u << i);
        } else {
            prefs.remove(k);
        }
    }
    prefs.putUChar("ownerauth", auth_mask);
    prefs.end();
}

// Safe from either task: queue the write for ble_tick.
static void save_owners() {
    owners_dirty = true;
}

// Loop task only (ble_clear_bonds).
static void clear_owners() {
    OWNERS_LOCK();
    owner_count = 0;
    memset(owner_auth, 0, sizeof(owner_auth));
    memset(owner_addrs, 0, sizeof(owner_addrs));
    OWNERS_UNLOCK();
    owners_dirty = false;  // superseded by the explicit wipe below
    char key[8];
    prefs.begin("clawd", false);
    for (int i = 0; i < MAX_OWNERS; i++) prefs.remove(owner_key(i, key, sizeof(key)));
    prefs.remove("ownerauth");
    prefs.end();
}

static void load_owners() {
    char key[8];
    owner_count = 0;
    prefs.begin("clawd", true);
    // Absent key ⇒ 0 ⇒ every slot unauthenticated, which is exactly right for
    // an owner list written by a build that predates the PIN flow.
    uint8_t auth_mask = prefs.getUChar("ownerauth", 0);
    for (int i = 0; i < MAX_OWNERS; i++) {
        String o = prefs.getString(owner_key(i, key, sizeof(key)), "");
        if (o.length() != 17) continue;  // "aa:bb:cc:dd:ee:ff"
        strncpy(owner_addrs[owner_count], o.c_str(), sizeof(owner_addrs[0]) - 1);
        owner_addrs[owner_count][sizeof(owner_addrs[0]) - 1] = '\0';
        // Mask bits track the ORIGINAL slot index, which survives the compaction
        // this loop performs when an intermediate slot is missing or corrupt.
        owner_auth[owner_count] = (auth_mask >> i) & 1u;
        Serial.printf("BLE: owner loaded = %s (pin-paired=%d)\n",
            owner_addrs[owner_count], owner_auth[owner_count] ? 1 : 0);
        owner_count++;
    }
    prefs.end();
}

// Move slot idx to the front (most-recently-active first) so eviction always
// hits the machine that has been away longest. `persist` skips the NVS write
// for high-frequency callers (every accepted RX write) — RAM order is what
// eviction consults, and it is persisted anyway whenever the list changes
// membership; losing pure reorderings across a reboot is harmless.
static void owner_touch(int idx, bool persist) {
    if (idx <= 0) return;
    char tmp[sizeof(owner_addrs[0])];
    bool tmp_auth = owner_auth[idx];
    memcpy(tmp, owner_addrs[idx], sizeof(tmp));
    memmove(owner_addrs[1], owner_addrs[0], (size_t)idx * sizeof(owner_addrs[0]));
    memmove(owner_auth + 1, owner_auth, (size_t)idx * sizeof(owner_auth[0]));
    memcpy(owner_addrs[0], tmp, sizeof(tmp));
    owner_auth[0] = tmp_auth;
    if (persist) save_owners();
}

// Delete a stored bond by its printed address, going through the bond table so
// the address TYPE (public vs static-random identity) is preserved — a
// NimBLEAddress built from the bare string would default to public and miss
// random-identity peers (macOS).
static bool delete_bond_by_str(const char* addr_str) {
    int n = NimBLEDevice::getNumBonds();
    for (int i = 0; i < n; i++) {
        NimBLEAddress a = NimBLEDevice::getBondedAddress(i);
        if (strcmp(a.toString().c_str(), addr_str) == 0) {
            return NimBLEDevice::deleteBond(a);
        }
    }
    return true;  // already gone
}

// Delete every stored bond that isn't an owner, so the board only stays paired
// to listed machines. Removing a bond shifts the indices, so restart from 0 —
// but only while deletes actually succeed: ble_gap_unpair() can fail (EBUSY
// against a connected peer), and retrying a bond that won't go would spin this
// loop forever inside the NimBLE host task.
static void prune_foreign_bonds() {
    if (owner_count == 0) return;
    bool removed;
    do {
        removed = false;
        int n = NimBLEDevice::getNumBonds();
        for (int i = 0; i < n; i++) {
            NimBLEAddress a = NimBLEDevice::getBondedAddress(i);
            if (owner_index(a.toString().c_str()) < 0) {
                Serial.printf("BLE: pruning non-owner bond %s\n", a.toString().c_str());
                removed = NimBLEDevice::deleteBond(a);
                if (!removed) Serial.println("BLE: prune failed, giving up this pass");
                break;
            }
        }
    } while (removed);
}

// Insert a newly-paired machine at slot 0. When the list is full the last
// slot is evicted: bond deleted and, if it happens to be connected, dropped —
// per the pairing-window UX, the newest machine always wins a seat.
static void owner_admit_front(const char* id, bool authenticated) {
    if (owner_count == MAX_OWNERS) {
        const char* evict = owner_addrs[MAX_OWNERS - 1];
        Serial.printf("BLE: evicting stalest owner %s\n", evict);
        // Drop the link BEFORE unpairing: ble_gap_unpair() refuses (EBUSY) to
        // delete the bond of a connected peer. getPeerDevices() returns a copy,
        // so disconnecting inside the walk is safe.
        if (server) {
            for (uint16_t h : server->getPeerDevices()) {
                NimBLEConnInfo pi = server->getPeerInfoByHandle(h);
                if (strcmp(pi.getIdAddress().toString().c_str(), evict) == 0) {
                    server->disconnect(h);
                }
            }
        }
        // The disconnect is asynchronous, so this delete may still hit EBUSY;
        // the peer is no longer an owner either way, so the prune pass below
        // (and every later one) will finish the job.
        delete_bond_by_str(evict);
        owner_count--;
    }
    memmove(owner_addrs[1], owner_addrs[0], (size_t)owner_count * sizeof(owner_addrs[0]));
    memmove(owner_auth + 1, owner_auth, (size_t)owner_count * sizeof(owner_auth[0]));
    strncpy(owner_addrs[0], id, sizeof(owner_addrs[0]) - 1);
    owner_addrs[0][sizeof(owner_addrs[0]) - 1] = '\0';
    owner_auth[0] = authenticated;
    owner_count++;
    save_owners();
    Serial.printf("BLE: owner admitted = %s (%d/%d, pin-paired=%d)\n",
        id, owner_count, MAX_OWNERS, authenticated ? 1 : 0);
    prune_foreign_bonds();
}

// Close the pairing window (success, timeout, or cancel) and fall back to
// anonymous advertising.
static void pairing_close(const char* why) {
    if (!pairing_mode && !admission_armed) return;
    pairing_mode = false;
    admission_armed = false;
    pin_requested = false;
    passkey_shown = false;
    ceremony_handle = CONN_HANDLE_NONE;
    need_advertise = true;
    Serial.printf("BLE: pairing window closed (%s)\n", why);
}

static void start_advertising() {
    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    adv->reset();
    if (pairing_mode) {
        // Pairing window — full discoverable advertising:
        //   flags (3) + appearance (4) + HID service 0x1812 (4) + name "Clawdmeter" (12)
        //   = 23 bytes. macOS Bluetooth Settings only surfaces BLE-only devices
        //   that explicitly advertise the standard HID service UUID (0x1812) —
        //   without it the device is recognized internally but hidden from the
        //   GUI nearby-devices list.
        adv->setDiscoverableMode(BLE_GAP_DISC_MODE_GEN);
        adv->setAppearance(HID_KEYBOARD);
        adv->addServiceUUID(NimBLEUUID((uint16_t)0x1812));  // BLE HID Service
        adv->setName(DEVICE_NAME);
        // Scan response carries the 128-bit custom data-service UUID for
        // active scanners.
        NimBLEAdvertisementData scanResp;
        scanResp.setCompleteServices(NimBLEUUID(SERVICE_UUID));
        adv->setScanResponseData(scanResp);
        adv->enableScanResponse(true);
    } else {
        // Stealth (default) — connectable but anonymous: no name, no
        // appearance, no service UUIDs, non-discoverable flags. Bonded
        // machines reconnect by address (bluez direct connect, macOS HID
        // reconnect — neither needs the name), while scanners see only an
        // unnamed address.
        adv->setDiscoverableMode(BLE_GAP_DISC_MODE_NON);
        // The scan response must be OVERWRITTEN, not merely left unset:
        // NimBLE's start() only pushes scan-response bytes when one is
        // enabled and non-empty, so simply disabling it leaves the previous
        // pairing window's response (with the custom service UUID) live in
        // the controller, and active scanners would still see it. TX power is
        // a standard, non-identifying field that costs 3 bytes.
        NimBLEAdvertisementData scanResp;
        scanResp.addTxPower();
        adv->setScanResponseData(scanResp);
        adv->enableScanResponse(true);
    }
    bool ok = adv->start();
    // Only reflect ADVERTISING in the UI state when no client is connected.
    // With MAX_CONNECTIONS=2, onConnect re-advertises to fill the second slot;
    // without this guard the UI would flip CONNECTED → ADVERTISING on every
    // first connect and never come back until a second client arrived.
    if (!server || server->getConnectedCount() == 0) {
        state = BLE_STATE_ADVERTISING;
    }
    Serial.printf("BLE: advertising start=%s (connected=%u)\n",
        ok ? "OK" : "FAILED",
        server ? (unsigned)server->getConnectedCount() : 0);
}

// Unencrypted-link watchdog bookkeeping: a stranger that connects to the
// anonymous advertisement but never encrypts must not camp a connection slot.
// Written from NimBLE host-task callbacks, consumed by ble_tick().
// Guarded by the same mutex as the owner list (see OWNERS_LOCK).
static uint32_t conn_started_ms[CONFIG_BT_NIMBLE_MAX_CONNECTIONS];
static uint16_t conn_handles[CONFIG_BT_NIMBLE_MAX_CONNECTIONS] = {
    CONN_HANDLE_NONE, CONN_HANDLE_NONE};
#define ENC_WATCHDOG_MS 30000

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* s, NimBLEConnInfo& info) override {
        state = BLE_STATE_CONNECTED;
        Serial.printf("BLE: connected from %s (active=%u)\n",
            info.getAddress().toString().c_str(),
            (unsigned)s->getConnectedCount());
        OWNERS_LOCK();
        for (int i = 0; i < CONFIG_BT_NIMBLE_MAX_CONNECTIONS; i++) {
            if (conn_handles[i] == CONN_HANDLE_NONE) {
                conn_started_ms[i] = millis();
                conn_handles[i] = info.getConnHandle();
                break;
            }
        }
        OWNERS_UNLOCK();
        // Log negotiated link timing — the difference between guessing and
        // knowing when debugging disconnects (e.g. reason=520 supervision
        // timeouts are only explainable next to the negotiated timeout).
        // Units: interval ×1.25ms, timeout ×10ms, latency = skippable events.
        Serial.printf("BLE: connparams itvl=%u(%.2fms) lat=%u timeout=%u(%ums)\n",
            info.getConnInterval(), info.getConnInterval() * 1.25f,
            info.getConnLatency(), info.getConnTimeout(), info.getConnTimeout() * 10);
        // Keep advertising while a connection slot is still free so a second
        // central (e.g. the host daemon alongside an OS-held HID link) can
        // discover and connect. NimBLE auto-stops advertising on each accept.
        if (s->getConnectedCount() < CONFIG_BT_NIMBLE_MAX_CONNECTIONS) {
            need_advertise = true;
        }
    }

    void onDisconnect(NimBLEServer* s, NimBLEConnInfo& info, int reason) override {
        // Only flip the UI state to DISCONNECTED when the last client leaves.
        if (s->getConnectedCount() == 0) state = BLE_STATE_DISCONNECTED;
        need_advertise = true;
        // Drop any pending/spent param pushback for this handle — NimBLE
        // reuses conn handles, so stale state would leak onto the next link.
        if (param_fix_handle == info.getConnHandle()) param_fix_handle = CONN_HANDLE_NONE;
        if (param_fix_spent  == info.getConnHandle()) param_fix_spent  = CONN_HANDLE_NONE;
        OWNERS_LOCK();
        for (int i = 0; i < CONFIG_BT_NIMBLE_MAX_CONNECTIONS; i++) {
            if (conn_handles[i] == info.getConnHandle()) conn_handles[i] = CONN_HANDLE_NONE;
        }
        OWNERS_UNLOCK();
        // A code left on the screen for a peer that has gone away would invite
        // the user to confirm nothing at all.
        if (confirm_pending && confirm_handle == info.getConnHandle()) {
            confirm_pending = false;
            Serial.println("BLE: pending confirmation dropped (peer disconnected)");
        }
        Serial.printf("BLE: disconnected (reason=%d, remaining=%u)\n",
            reason, (unsigned)s->getConnectedCount());
    }

    // Centrals re-negotiate parameters mid-connection. Windows in particular
    // clamps the supervision timeout to 2s once an app GATT session goes
    // active (captured on hardware; it honors 9.6s while only its HID driver
    // holds the link) — and a 2s window is tight enough that ordinary radio
    // gaps kill the link (HCI 0x208 → reason=520), which was the constant
    // daemon reconnect churn. Push back ONCE per connection: schedule a
    // deferred LL connection-parameter request for the same interval range but
    // a 6s timeout (the mechanism Microsoft's accessory guidelines prescribe).
    // Deferred ~2s so it can't race the central's own in-flight update
    // transaction (Windows has a documented late-instant bug there), and
    // one-shot so a central that re-clamps doesn't trigger an update war.
    void onConnParamsUpdate(NimBLEConnInfo& info) override {
        Serial.printf("BLE: connparams update itvl=%u(%.2fms) lat=%u timeout=%u(%ums)\n",
            info.getConnInterval(), info.getConnInterval() * 1.25f,
            info.getConnLatency(), info.getConnTimeout(), info.getConnTimeout() * 10);
        if (info.getConnTimeout() < DESIRED_TIMEOUT &&
            info.getConnHandle() != param_fix_spent) {
            param_fix_handle = info.getConnHandle();
            param_fix_at_ms  = millis() + 2000;
        }
    }

    // The stack asks for the passkey to show whenever a passkey-entry pairing
    // starts (our IO cap is DISPLAY_ONLY + MITM). Inside the pairing window we
    // hand it the PIN that is on the screen; outside it we hand back a fresh
    // random number that is displayed nowhere — the peer cannot know it, so
    // pairing outside the window cannot succeed.
    uint32_t onPassKeyDisplay() override {
        if (admission_open()) {
            // Mint it now, and only now — this is the moment we know the host
            // actually wants a typed code. 1000..9999: no leading zero, so
            // what is typed always matches what is shown.
            pairing_pin_val = 1000 + (esp_random() % 9000);
            passkey_shown = true;
            pin_requested = true;
            ceremony_handle = CONN_HANDLE_NONE;  // callback carries no conn info
            Serial.printf("BLE: passkey %04lu — host should type it\n",
                (unsigned long)pairing_pin_val);
            return pairing_pin_val;
        }
        // Outside the window: a value that is displayed nowhere, so pairing
        // cannot succeed.
        return 100000 + (esp_random() % 900000);
    }

    // Numeric comparison: the stack derived a 6-digit code from the pairing
    // exchange and both ends must show the same one. Park it for the loop task
    // — returning without injecting keeps the SMP procedure open. Outside the
    // pairing window we refuse straight away rather than putting a stranger's
    // code on the screen for the user to rubber-stamp.
    void onConfirmPassKey(NimBLEConnInfo& info, uint32_t pin) override {
        if (!admission_open()) {
            Serial.printf("BLE: numeric-comparison request outside the pairing window from %s"
                          " — rejected\n", info.getAddress().toString().c_str());
            NimBLEDevice::injectConfirmPasskey(info, false);
            return;
        }
        pin_requested   = true;
        confirm_code    = pin;
        confirm_handle  = info.getConnHandle();
        ceremony_handle = info.getConnHandle();
        confirm_pending = true;
        Serial.printf("BLE: confirm code %06lu from %s — awaiting PWR press\n",
            (unsigned long)pin, info.getAddress().toString().c_str());
    }

    // Gate on the owner allow-list. Runs on every encryption establishment:
    // fresh pairings AND bonded reconnects (both raise ENC_CHANGE). Note this
    // callback also fires when SMP FAILS — everything below is gated on
    // isEncrypted so a failed attempt admits nothing.
    void onAuthenticationComplete(NimBLEConnInfo& info) override {
        std::string id = info.getIdAddress().toString();
        Serial.printf("BLE: auth complete peer=%s bonded=%d enc=%d\n",
            id.c_str(), info.isBonded() ? 1 : 0, info.isEncrypted() ? 1 : 0);
        // Bonded reconnects START at the central's clamped parameters (no
        // later update event fires), so the supervision-timeout pushback must
        // also arm here, not just in onConnParamsUpdate.
        if (info.getConnTimeout() < DESIRED_TIMEOUT &&
            info.getConnHandle() != param_fix_spent) {
            param_fix_handle = info.getConnHandle();
            param_fix_at_ms  = millis() + 2000;
        }
        if (!info.isEncrypted() || id == ZERO_ADDR) return;  // failed/aborted SMP

        const bool  authed = info.isAuthenticated();
        const char* reject_why = nullptr;

        // Decide under the lock so the index stays valid across the check and
        // the mutation; the NimBLE teardown for a rejection happens after.
        OWNERS_LOCK();
        int idx = owner_index(id.c_str());
        if (idx >= 0) {
            // Being on the list is NOT enough. A slot that was won with the PIN
            // must keep presenting an authenticated link: an unauthenticated one
            // from that same address means the peer re-paired Just Works (its OS
            // dropped the bond, or someone is spoofing the address), and letting
            // it through would be a PIN-free way back into an occupied slot.
            // Legacy slots (adopted from a pre-MITM build) are exempt until they
            // re-pair through the window, which upgrades them.
            if (owner_auth[idx] && !authed) {
                reject_why = "unauthenticated re-pair of a pin-paired slot";
            } else {
                // A known owner that re-paired through the window closes it —
                // leaving "Ready to pair" up after a ceremony this peer just
                // completed is stale. Gated on the ceremony belonging to THIS
                // connection so a routine bonded reconnect (the daemon's 60s
                // cycle) can't slam the window shut on a different machine
                // that is still looking at a code.
                if (admission_open() && pin_requested && authed &&
                    (ceremony_handle == CONN_HANDLE_NONE ||
                     ceremony_handle == info.getConnHandle())) {
                    if (!owner_auth[idx]) {
                        owner_auth[idx] = true;
                        save_owners();
                        Serial.printf("BLE: owner %s upgraded to pin-paired\n", id.c_str());
                    }
                    pairing_close("owner re-paired");
                }
                owner_touch(idx, true);
            }
        } else if (admission_open()) {
            // Encryption alone does NOT prove the PIN was typed: MITM is only
            // *requested* unless BLE_SM_LVL>=3, and a central advertising
            // NoInputNoOutput IO capabilities negotiates Just Works and arrives
            // here encrypted and bonded but UNauthenticated, never having seen
            // the PIN. isAuthenticated() is set only for passkey/numeric-
            // comparison/OOB pairing, so it is the real proof of PIN entry.
            if (authed) {
                owner_admit_front(id.c_str(), true);
                pairing_close("new machine paired");
            } else {
                reject_why = "unauthenticated (Just Works) pairing";
            }
        } else if (owner_count == 0 && is_legacy_bond(id.c_str())) {
            // Migration: firmware upgraded from a pre-allow-list build — a bond
            // predating this boot exists but NVS holds no owner list. Restricted
            // to the boot-time bond snapshot so a stranger that Just-Works pairs
            // with a factory-fresh device cannot adopt itself as the owner. The
            // slot inherits the old bond's security level until the machine
            // re-pairs through the window, which upgrades it.
            owner_admit_front(id.c_str(), authed);
        } else {
            reject_why = "not an owner and no pairing window is open";
        }
        OWNERS_UNLOCK();

        if (reject_why) {
            Serial.printf("BLE: rejecting %s — %s\n", id.c_str(), reject_why);
            NimBLEDevice::deleteBond(info.getIdAddress());
            server->disconnect(info);
        }
    }

};

class RxCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* chr, NimBLEConnInfo& info) override {
        // Only accept usage data over an encrypted link from a listed owner —
        // admission happens exclusively in onAuthenticationComplete, never
        // here. With several owners listed, whoever writes last is what the
        // screen shows. The RAM-only touch keeps the actively-writing machine
        // at the front so eviction targets the machine that's truly stalest.
        std::string id = info.getIdAddress().toString();
        if (!info.isEncrypted()) {
            Serial.println("BLE: dropping RX write from unencrypted link");
            return;
        }
        OWNERS_LOCK();
        int idx = owner_index(id.c_str());
        if (idx >= 0) owner_touch(idx, false);
        OWNERS_UNLOCK();
        if (idx < 0) {
            Serial.printf("BLE: dropping RX write from non-owner %s\n", id.c_str());
            return;
        }
        std::string val = chr->getValue();
        size_t len = std::min(val.length(), (size_t)(BLE_BUF_SIZE - 1));
        memcpy(rx_buf, val.c_str(), len);
        rx_buf[len] = '\0';
        data_ready = true;
        has_received_data = true;
    }
};

// When the daemon enables notifications on the refresh char, ask for data
// if we have none yet. Firing on subscribe (not on connect) ensures the
// notification isn't dropped before the daemon's CCCD write completes.
class ReqCallbacks : public NimBLECharacteristicCallbacks {
    void onSubscribe(NimBLECharacteristic* chr, NimBLEConnInfo& info, uint16_t subValue) override {
        Serial.printf("BLE: req_char onSubscribe subValue=%u has_data=%d\n", subValue, has_received_data ? 1 : 0);
        if (subValue != 0 && !has_received_data) {
            ble_request_refresh();
        }
    }
};

void ble_init(void) {
    owner_mux = xSemaphoreCreateRecursiveMutex();
    NimBLEDevice::init(DEVICE_NAME);
    // Bonding + MITM + Secure Connections, with a display-only IO capability:
    // new pairings run passkey entry — we show a PIN, the host types it.
    // Pre-existing bonds re-encrypt with their stored LTK and never re-pair,
    // so upgrading firmware does not force anyone through the PIN flow.
    NimBLEDevice::setSecurityAuth(true, true, true);
    // DisplayYesNo, not DisplayOnly. Per the SMP IO matrix (ble_sm_sc.c), a
    // DisplayOnly responder facing the DisplayYesNo agent that every desktop
    // GUI registers resolves to Just Works — the computer never asks for
    // anything and the link comes up unauthenticated, which we then have to
    // reject. DisplayYesNo instead yields numeric comparison with GUI agents
    // and still yields passkey-display with KeyboardOnly/KeyboardDisplay ones;
    // both set the authenticated bit.
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_YESNO);

    // Restore the owner allow-list and drop any stale non-owner bonds so the
    // board stays paired to just those machines across reboots.
    load_owners();
    // Snapshot bonds that exist with no owner list — machines paired by a
    // pre-allow-list build. Only these may be adopted without a pairing
    // window; anything that bonds later has to come through the PIN.
    if (owner_count == 0) {
        int n = NimBLEDevice::getNumBonds();
        for (int i = 0; i < n && legacy_bond_count < MAX_OWNERS; i++) {
            NimBLEAddress a = NimBLEDevice::getBondedAddress(i);
            strncpy(legacy_bonds[legacy_bond_count], a.toString().c_str(),
                    sizeof(legacy_bonds[0]) - 1);
            legacy_bonds[legacy_bond_count][sizeof(legacy_bonds[0]) - 1] = '\0';
            Serial.printf("BLE: legacy bond %s eligible for adoption\n",
                legacy_bonds[legacy_bond_count]);
            legacy_bond_count++;
        }
    }
    prune_foreign_bonds();

    // Format MAC address
    NimBLEAddress addr = NimBLEDevice::getAddress();
    snprintf(mac_str, sizeof(mac_str), "%s", addr.toString().c_str());
    for (int i = 0; mac_str[i]; i++) {
        if (mac_str[i] >= 'a' && mac_str[i] <= 'f') mac_str[i] -= 32;
    }

    server = NimBLEDevice::createServer();
    static ServerCallbacks serverCb;
    server->setCallbacks(&serverCb);

    // --- HID keyboard service ---
    hid_dev = new NimBLEHIDDevice(server);
    hid_dev->setReportMap((uint8_t*)HID_REPORT_MAP, sizeof(HID_REPORT_MAP));
    hid_dev->setManufacturer("Anthropic");
    // PnP ID: (vendorIdSource, vendorId, productId, version).
    // Source 1 = Bluetooth SIG, vendor 0x02E5 = Espressif. Originally claimed
    // Apple's USB vendor 0x05AC + Magic Keyboard product 0x820A — macOS
    // validates Apple-claimed HIDs against known device IDs and silently
    // refuses to surface a Connect button for spoofers.
    hid_dev->setPnp(0x01, 0x02E5, 0x0001, 0x0100);
    // country=33 (US ANSI). Setting this to 0 ("not supported") causes macOS
    // to launch the Keyboard Setup Assistant on first pair asking the user
    // to identify the layout — we only ever send Space / Shift+Tab so the
    // physical layout is irrelevant; advertise a known one to skip the wizard.
    hid_dev->setHidInfo(33, 0x02);
    hid_dev->setBatteryLevel(100);
    input_kbd = hid_dev->getInputReport(1);  // report ID 1

    // --- Custom data service ---
    NimBLEService* svc = server->createService(SERVICE_UUID);

    rx_char = svc->createCharacteristic(
        RX_CHAR_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
    );
    static RxCallbacks rxCb;
    rx_char->setCallbacks(&rxCb);

    tx_char = svc->createCharacteristic(
        TX_CHAR_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );

    req_char = svc->createCharacteristic(
        REQ_CHAR_UUID,
        NIMBLE_PROPERTY::NOTIFY
    );
    static ReqCallbacks reqCb;
    req_char->setCallbacks(&reqCb);

    svc->start();
    server->start();
    start_advertising();

    Serial.printf("BLE: init complete, MAC=%s\n", mac_str);
}

void ble_tick(void) {
    // Owner-list writes queued by host-task callbacks land here, on the loop
    // task, so NVS never blocks the BLE stack.
    if (owners_dirty) {
        owners_dirty = false;
        persist_owners_now();
    }
    if (need_advertise) {
        need_advertise = false;
        start_advertising();
    }
    // Pairing window expiry: hide the PIN and fall back to anonymous
    // advertising. Admission stays open for PAIRING_GRACE_MS more so a PIN
    // typed right before the deadline still completes (admission_open()).
    if (pairing_mode && (int32_t)(millis() - pairing_deadline_ms) >= 0) {
        pairing_mode = false;
        pin_requested = false;
        need_advertise = true;
        Serial.println("BLE: pairing window timed out");
    }
    // Disarm admission once the grace period lapses. Leaving it armed would
    // let the signed millis() comparison in admission_open() wrap back to
    // "open" roughly 24.8 days later, silently re-enabling PIN-less admission.
    if (admission_armed && !pairing_mode &&
        (int32_t)(millis() - admission_until_ms) >= 0) {
        admission_armed = false;
        Serial.println("BLE: pairing admission grace expired");
    }
    // Unencrypted-link watchdog: a peer that connects but never encrypts
    // (a stranger poking at the anonymous advertisement, or an evicted
    // machine retrying with a deleted bond) is dropped so it can't camp a
    // connection slot. Paused while admission is open — a human typing the
    // pairing PIN legitimately sits unencrypted for a while.
    static uint32_t last_watchdog_ms = 0;
    if (server && !admission_open() && millis() - last_watchdog_ms >= 5000) {
        last_watchdog_ms = millis();
        // Claim the expired handles under the lock and clear the slots in the
        // same breath, so a disconnect+reconnect racing in on the host task
        // can't have its FRESH handle torn down by this stale scan.
        uint16_t expired[CONFIG_BT_NIMBLE_MAX_CONNECTIONS];
        int n_expired = 0;
        OWNERS_LOCK();
        for (int i = 0; i < CONFIG_BT_NIMBLE_MAX_CONNECTIONS; i++) {
            uint16_t h = conn_handles[i];
            if (h == CONN_HANDLE_NONE) continue;
            if (millis() - conn_started_ms[i] < ENC_WATCHDOG_MS) continue;
            expired[n_expired++] = h;
            conn_handles[i] = CONN_HANDLE_NONE;  // done watching either way
        }
        OWNERS_UNLOCK();
        for (int i = 0; i < n_expired; i++) {
            NimBLEConnInfo pi = server->getPeerInfoByHandle(expired[i]);
            if (!pi.isEncrypted()) {
                Serial.printf("BLE: dropping unencrypted camper %s\n",
                    pi.getAddress().toString().c_str());
                server->disconnect(expired[i]);
            }
        }
    }
    // Deferred one-shot supervision-timeout pushback (see onConnParamsUpdate).
    if (param_fix_handle != CONN_HANDLE_NONE &&
        (int32_t)(millis() - param_fix_at_ms) >= 0) {
        uint16_t h = param_fix_handle;
        param_fix_handle = CONN_HANDLE_NONE;
        param_fix_spent  = h;
        if (server && server->getConnectedCount() > 0) {
            Serial.println("BLE: requesting 6s supervision timeout");
            server->updateConnParams(h, 12, 24, 0, DESIRED_TIMEOUT);
        }
    }
}

ble_state_t ble_get_state(void) {
    return state;
}

const char* ble_get_device_name(void) {
    return DEVICE_NAME;
}

const char* ble_get_mac_address(void) {
    return mac_str;
}

// Open the pairing window: pick a fresh 4-digit PIN (1000..9999 — no leading
// zero, so what's typed always matches what's shown), go discoverable, and
// let the UI display the code. Ends on success, timeout, or cancel.
void ble_pairing_begin(void) {
    pairing_mode = true;
    pin_requested = false;
    passkey_shown = false;
    pairing_pin_val = 0;
    pairing_deadline_ms = millis() + PAIRING_WINDOW_MS;
    admission_until_ms  = pairing_deadline_ms + PAIRING_GRACE_MS;
    admission_armed = true;
    need_advertise = true;
    Serial.println("BLE: pairing window open — waiting for a host to start pairing");

    // With every connection slot taken (typically the OS's HID link plus the
    // daemon's data link) NimBLE stops advertising entirely — the PIN would be
    // on screen while nothing could possibly find the device. Free the stalest
    // owner's slot, matching the "newest machine wins a seat" policy. Its bond
    // survives, so it reconnects on its own once a slot frees up again.
    if (server && server->getConnectedCount() >= CONFIG_BT_NIMBLE_MAX_CONNECTIONS) {
        char stalest[18] = {0};
        OWNERS_LOCK();
        if (owner_count > 0) strncpy(stalest, owner_addrs[owner_count - 1], sizeof(stalest) - 1);
        OWNERS_UNLOCK();
        for (uint16_t h : server->getPeerDevices()) {
            NimBLEConnInfo pi = server->getPeerInfoByHandle(h);
            // Prefer the stalest owner; fall back to whatever is connected if
            // the peer list and the owner list disagree.
            if (stalest[0] == '\0' ||
                strcmp(pi.getIdAddress().toString().c_str(), stalest) == 0) {
                Serial.printf("BLE: freeing a connection slot for pairing (dropping %s)\n",
                    pi.getAddress().toString().c_str());
                server->disconnect(h);
                break;
            }
        }
    }
}

void ble_pairing_cancel(void) {
    if (confirm_pending) { ble_confirm_reject(); return; }
    pairing_close("cancelled");
}

bool     ble_confirm_pending(void) { return confirm_pending; }

// Both run on the loop task (button handling), so calling into NimBLE here is
// safe and can't stall the host task.
void ble_confirm_accept(void) {
    if (!confirm_pending || !server) return;
    confirm_pending = false;
    NimBLEConnInfo pi = server->getPeerInfoByHandle(confirm_handle);
    Serial.println("BLE: numeric comparison accepted");
    NimBLEDevice::injectConfirmPasskey(pi, true);
}

void ble_confirm_reject(void) {
    if (!confirm_pending || !server) return;
    confirm_pending = false;
    NimBLEConnInfo pi = server->getPeerInfoByHandle(confirm_handle);
    Serial.println("BLE: numeric comparison rejected");
    NimBLEDevice::injectConfirmPasskey(pi, false);
}

bool ble_pairing_active(void) {
    return pairing_mode;
}

// A pending numeric comparison outranks everything: it stays up even after the
// 60s window lapses, because the computer is showing that code right now and
// the user still has to answer it.
ble_pair_view_t ble_pairing_view(void) {
    if (confirm_pending)             return BLE_PAIR_VIEW_CONFIRM;
    if (pairing_mode && passkey_shown) return BLE_PAIR_VIEW_PASSKEY;
    if (pairing_mode)                return BLE_PAIR_VIEW_WAITING;
    return BLE_PAIR_VIEW_NONE;
}

uint32_t ble_pairing_code(void) {
    return confirm_pending ? confirm_code : pairing_pin_val;
}

void ble_clear_bonds(void) {
    NimBLEDevice::deleteAllBonds();
    clear_owners();  // release ownership so the board can be handed to other machines
    // Drop the migration snapshot too: after an explicit wipe nothing is
    // grandfathered, so every machine — including the one that was paired a
    // moment ago — has to come back through the pairing window and the PIN.
    legacy_bond_count = 0;
    pairing_close("bonds cleared");
    Serial.println("BLE: bonds cleared");
    if (state == BLE_STATE_CONNECTED) {
        server->disconnect(server->getPeerInfo(0).getConnHandle());
    }
    need_advertise = true;
}

bool ble_has_bonds(void) {
    return NimBLEDevice::getNumBonds() > 0;
}

bool ble_has_data(void) {
    return data_ready;
}

const char* ble_get_data(void) {
    data_ready = false;
    return rx_buf;
}

void ble_send_ack(void) {
    if (state == BLE_STATE_CONNECTED && tx_char) {
        tx_char->setValue("{\"ack\":true}");
        tx_char->notify();
    }
}

void ble_send_nack(void) {
    if (state == BLE_STATE_CONNECTED && tx_char) {
        tx_char->setValue("{\"err\":true}");
        tx_char->notify();
    }
}

void ble_request_refresh(void) {
    if (state == BLE_STATE_CONNECTED && req_char) {
        uint8_t v = 0x01;
        req_char->setValue(&v, 1);
        req_char->notify();
        Serial.println("BLE: refresh requested");
    }
}

void ble_keyboard_press(uint8_t key, uint8_t modifier) {
    if (state != BLE_STATE_CONNECTED || !input_kbd) return;
    // HID report: [modifier, reserved, key1, key2, key3, key4, key5, key6]
    uint8_t report[8] = {modifier, 0, key, 0, 0, 0, 0, 0};
    input_kbd->setValue(report, sizeof(report));
    input_kbd->notify();
}

void ble_keyboard_release(void) {
    if (state != BLE_STATE_CONNECTED || !input_kbd) return;
    uint8_t report[8] = {0};
    input_kbd->setValue(report, sizeof(report));
    input_kbd->notify();
}
