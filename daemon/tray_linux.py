#!/usr/bin/env python3
"""Linux system-tray control panel for the claude-usage-daemon systemd unit.

Does NOT run the daemon itself — the daemon already runs as a systemd --user
service (see README/CLAUDE.md). This is a thin GUI shell around:

    systemctl --user status|start|stop|restart claude-usage-daemon
    journalctl --user -u claude-usage-daemon -f

Menu:
  - status header (non-clickable): ActiveState + a substate detail
  - Start / Stop / Restart (enabled based on current state)
  - View Logs... opens a window live-tailing journalctl
  - Quit (quits the tray only; does not touch the daemon)

Usage::

    python3 daemon/tray_linux.py

Requires: PyGObject (Gtk 3, AyatanaAppIndicator3 or AppIndicator3), Pillow.
On Debian/Ubuntu: apt install python3-gi gir1.2-ayatanaappindicator3-0.1 python3-pil
"""
import fcntl
import os
import subprocess
import sys
import tempfile

import gi

gi.require_version("Gtk", "3.0")
from gi.repository import Gtk, GLib, GdkPixbuf

try:
    gi.require_version("AyatanaAppIndicator3", "0.1")
    from gi.repository import AyatanaAppIndicator3 as AppIndicator3
except ValueError:
    gi.require_version("AppIndicator3", "0.1")
    from gi.repository import AppIndicator3

try:
    gi.require_version("Notify", "0.7")
    from gi.repository import Notify
    Notify.init("Clawdmeter")
    _HAVE_NOTIFY = True
except (ValueError, ImportError):
    _HAVE_NOTIFY = False

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from daemon import autostart_linux  # noqa: E402

UNIT = "claude-usage-daemon"
POLL_SECONDS = 3

# Corner-bubble colors per systemd ActiveState (RGBA). "activating"/"deactivating"
# share the amber "in flux" color with "reloading".
STATE_COLORS = {
    "active": (60, 200, 90, 255),        # green
    "activating": (240, 180, 40, 255),   # amber
    "deactivating": (240, 180, 40, 255), # amber
    "reloading": (240, 180, 40, 255),    # amber
    "inactive": (140, 140, 140, 255),    # gray
    "failed": (220, 60, 60, 255),        # red
}
DEFAULT_COLOR = (140, 140, 140, 255)


# ---------------------------------------------------------------------------
# single-instance guard (flock on a lockfile in the user's runtime dir)
# ---------------------------------------------------------------------------

def _acquire_single_instance():
    """Return an open lockfile handle if this is the only tray instance, else None.

    Uses flock on a file under XDG_RUNTIME_DIR — the kernel releases the lock
    automatically if the process dies, so there's no stale-lock cleanup needed.
    """
    runtime_dir = os.environ.get("XDG_RUNTIME_DIR", tempfile.gettempdir())
    lock_path = os.path.join(runtime_dir, "clawdmeter-tray.lock")
    fh = open(lock_path, "w")
    try:
        fcntl.flock(fh, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        fh.close()
        return None
    return fh


# ---------------------------------------------------------------------------
# icon rendering — brand logo + colored corner bubble per ActiveState
# ---------------------------------------------------------------------------

def _build_state_icons(cache_dir):
    """Render one PNG per known ActiveState into cache_dir, return name->path."""
    from PIL import Image, ImageDraw

    base = autostart_linux.load_brand_logo()
    size = 64
    paths = {}
    for state, color in STATE_COLORS.items():
        icon = base.resize((size, size), Image.LANCZOS).convert("RGBA")
        draw = ImageDraw.Draw(icon)
        r = size // 3
        x0, y0 = size - r - 1, size - r - 1
        x1, y1 = size - 2, size - 2
        draw.ellipse([x0, y0, x1, y1], fill=color)
        path = os.path.join(cache_dir, f"clawdmeter-{state}.png")
        icon.save(path)
        paths[state] = path
    return paths


# ---------------------------------------------------------------------------
# systemctl / journalctl helpers
# ---------------------------------------------------------------------------

def _systemctl(*args):
    return subprocess.run(
        ["systemctl", "--user", *args],
        capture_output=True, text=True, timeout=10,
    )


def get_state():
    """Return (active_state, sub_state) e.g. ("active", "running")."""
    result = _systemctl("show", UNIT, "--property=ActiveState,SubState", "--value")
    lines = result.stdout.strip().splitlines()
    if len(lines) < 2:
        return "unknown", ""
    return lines[0], lines[1]


def do_start(_item=None):
    _systemctl("start", UNIT)


def do_stop(_item=None):
    _systemctl("stop", UNIT)


def do_restart(_item=None):
    _systemctl("restart", UNIT)


# ---------------------------------------------------------------------------
# log viewer window
# ---------------------------------------------------------------------------

class LogWindow(Gtk.Window):
    def __init__(self):
        super().__init__(title=f"{UNIT} — logs")
        self.set_default_size(760, 480)

        scroller = Gtk.ScrolledWindow()
        self.textview = Gtk.TextView()
        self.textview.set_editable(False)
        self.textview.set_monospace(True)
        self.buffer = self.textview.get_buffer()
        scroller.add(self.textview)
        self.add(scroller)

        self._proc = subprocess.Popen(
            ["journalctl", "--user", "-u", UNIT, "-n", "200", "-f", "-o", "cat"],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1,
        )
        GLib.io_add_watch(self._proc.stdout, GLib.IO_IN | GLib.IO_HUP, self._on_output)
        self.connect("destroy", self._on_destroy)

    def _on_output(self, source, condition):
        if condition & GLib.IO_HUP:
            return False
        line = source.readline()
        if not line:
            return False
        end = self.buffer.get_end_iter()
        self.buffer.insert(end, line)
        self.textview.scroll_to_iter(self.buffer.get_end_iter(), 0.0, False, 0, 0)
        return True

    def _on_destroy(self, _widget):
        if self._proc.poll() is None:
            self._proc.terminate()


# ---------------------------------------------------------------------------
# tray app
# ---------------------------------------------------------------------------

class TrayApp:
    def __init__(self):
        cache_dir = os.path.join(GLib.get_user_cache_dir(), "clawdmeter-tray")
        os.makedirs(cache_dir, exist_ok=True)
        self._icons = _build_state_icons(cache_dir)

        self.indicator = AppIndicator3.Indicator.new(
            "clawdmeter", self._icons["inactive"],
            AppIndicator3.IndicatorCategory.APPLICATION_STATUS,
        )
        self.indicator.set_status(AppIndicator3.IndicatorStatus.ACTIVE)
        self.indicator.set_icon_full(self._icons["inactive"], "Clawdmeter")

        self._log_window = None
        self._prev_state = None

        self.status_item = Gtk.MenuItem(label="Checking status...")
        self.status_item.set_sensitive(False)
        self.start_item = Gtk.MenuItem(label="Start")
        self.start_item.connect("activate", self._on_start)
        self.stop_item = Gtk.MenuItem(label="Stop")
        self.stop_item.connect("activate", self._on_stop)
        self.restart_item = Gtk.MenuItem(label="Restart")
        self.restart_item.connect("activate", self._on_restart)
        logs_item = Gtk.MenuItem(label="View Logs...")
        logs_item.connect("activate", self._on_view_logs)
        self.autostart_item = Gtk.CheckMenuItem(label="Start at login")
        self.autostart_item.set_active(autostart_linux.is_enabled())
        self._autostart_handler = self.autostart_item.connect("toggled", self._on_toggle_autostart)
        quit_item = Gtk.MenuItem(label="Quit")
        quit_item.connect("activate", self._on_quit)

        menu = Gtk.Menu()
        for item in (self.status_item, Gtk.SeparatorMenuItem(),
                     self.start_item, self.stop_item, self.restart_item,
                     Gtk.SeparatorMenuItem(), logs_item, self.autostart_item,
                     Gtk.SeparatorMenuItem(), quit_item):
            menu.append(item)
        menu.show_all()
        self.indicator.set_menu(menu)

        self._refresh()
        GLib.timeout_add_seconds(POLL_SECONDS, self._refresh)

    def _on_start(self, _item):
        do_start()
        GLib.timeout_add(500, lambda: self._refresh() and False)

    def _on_stop(self, _item):
        do_stop()
        GLib.timeout_add(500, lambda: self._refresh() and False)

    def _on_restart(self, _item):
        do_restart()
        GLib.timeout_add(500, lambda: self._refresh() and False)

    def _on_view_logs(self, _item):
        if self._log_window is None or not self._log_window.get_visible():
            self._log_window = LogWindow()
            self._log_window.show_all()
        else:
            self._log_window.present()

    def _on_toggle_autostart(self, item):
        if item.get_active():
            autostart_linux.enable()
        else:
            autostart_linux.disable()

    def _on_quit(self, _item):
        Gtk.main_quit()

    def _refresh(self):
        active_state, sub_state = get_state()
        icon_path = self._icons.get(active_state, self._icons["inactive"])
        self.indicator.set_icon_full(icon_path, "Clawdmeter")

        label = f"{active_state}"
        if sub_state:
            label += f" ({sub_state})"
        self.status_item.set_label(label)

        self.start_item.set_sensitive(active_state not in ("active", "activating"))
        self.stop_item.set_sensitive(active_state in ("active", "activating", "reloading"))
        self.restart_item.set_sensitive(True)

        if _HAVE_NOTIFY and active_state == "failed" and self._prev_state != "failed":
            Notify.Notification.new(
                "Clawdmeter", f"{UNIT} failed", "dialog-error"
            ).show()
        self._prev_state = active_state
        return True


def main():
    lock = _acquire_single_instance()
    if lock is None:
        print("Clawdmeter tray is already running.", file=sys.stderr)
        return

    # Self-heals the app-grid launcher entry + icon on every start (cheap; keeps
    # Exec= pointing at the right path even if the repo moves).
    autostart_linux.install_launcher()

    TrayApp()
    Gtk.main()


if __name__ == "__main__":
    main()