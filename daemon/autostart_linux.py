"""Login-autostart + application-launcher entries for the Linux tray (APP-01 parity).

Two separate .desktop files, both regular XDG desktop entries:

  ~/.config/autostart/clawdmeter-tray.desktop
      Toggled by the user (tray menu "Start at login" / enable()/disable()).
      GNOME (and any XDG-autostart-compliant DE) launches this at login.

  ~/.local/share/applications/clawdmeter-tray.desktop
      Always installed (install_launcher()) so "Clawdmeter Tray" shows up in
      the Activities/app-grid search for manual launch, independent of the
      autostart toggle.

Both Exec= lines resolve the interpreter and script path fresh each write —
never hard-coded, since the repo may live at a different path per machine
(CLAUDE.md "repoint ExecStart" lesson from the systemd unit).

Public API mirrors autostart_windows.py:
  enable(tray_script=None)  -- write/overwrite the autostart entry
  disable()                 -- remove the autostart entry; idempotent when absent
  is_enabled()              -- True if the autostart entry currently exists
  install_launcher()        -- (re)write the app-grid launcher entry + icon
"""
import os
import shlex
import shutil
import sys
import time

_AUTOSTART_DIR = os.path.expanduser("~/.config/autostart")
_APPLICATIONS_DIR = os.path.expanduser("~/.local/share/applications")
_ICON_DIR = os.path.expanduser("~/.local/share/icons")
_ICON_PATH = os.path.join(_ICON_DIR, "clawdmeter.png")
_DESKTOP_NAME = "clawdmeter-tray.desktop"
_APP_NAME = "Clawdmeter"

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_DEFAULT_SCRIPT = os.path.join(_REPO_ROOT, "daemon", "tray_linux.py")
_LOGO_PNG = os.path.join(_REPO_ROOT, "assets", "logo_80.png")

# Launcher icon canvas vs. how much of it the brand mark fills. XDG icon
# convention leaves a margin around the mark instead of running edge-to-edge
# (that's what makes launcher icons look cramped/homemade next to system apps).
_ICON_CANVAS = 256
_ICON_FILL = 0.8


def log(msg: str) -> None:
    """Log in the daemon [HH:MM:SS] style."""
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def _autostart_path() -> str:
    return os.path.join(_AUTOSTART_DIR, _DESKTOP_NAME)


def _launcher_path() -> str:
    return os.path.join(_APPLICATIONS_DIR, _DESKTOP_NAME)


def _command(tray_script: str | None = None) -> str:
    """Build the quoted `python3 /abs/path/tray_linux.py` exec line.

    Uses whatever `python3` resolves to on PATH — PyGObject is a system
    package (python3-gi), not something a venv typically carries, so the
    system interpreter is the correct target here (unlike the Windows
    base-vs-venv split in autostart_windows.py).
    """
    python = shutil.which("python3") or sys.executable
    script = os.path.abspath(tray_script if tray_script is not None else _DEFAULT_SCRIPT)
    return f"{shlex.quote(python)} {shlex.quote(script)}"


def load_brand_logo():
    """Load the master brand mark as a Pillow RGBA image.

    Reads assets/logo_80.png (true-color, anti-aliased) rather than
    firmware/src/logo.h, which is the same mark baked down to RGB565 for the
    firmware's memory budget — a lossy source we don't need on the desktop.
    """
    from PIL import Image

    return Image.open(_LOGO_PNG).convert("RGBA")


def _install_icon() -> str:
    """Render a padded, high-quality launcher icon to _ICON_PATH. Returns the path.

    Upscales the 80x80 master with LANCZOS (smooth, not blocky) onto a larger
    canvas with margin around it, matching how launcher icons are conventionally
    composed (see _ICON_CANVAS / _ICON_FILL above).
    """
    from PIL import Image

    os.makedirs(_ICON_DIR, exist_ok=True)
    base = load_brand_logo()
    content = round(_ICON_CANVAS * _ICON_FILL)
    mark = base.resize((content, content), Image.LANCZOS)
    icon = Image.new("RGBA", (_ICON_CANVAS, _ICON_CANVAS), (0, 0, 0, 0))
    offset = ((_ICON_CANVAS - content) // 2, (_ICON_CANVAS - content) // 2)
    icon.paste(mark, offset, mark)
    icon.save(_ICON_PATH)
    return _ICON_PATH


def _desktop_entry(cmd: str, icon_path: str, autostart: bool) -> str:
    lines = [
        "[Desktop Entry]",
        "Type=Application",
        f"Name={_APP_NAME}",
        "Comment=Tray control panel for the claude-usage-daemon service",
        f"Exec={cmd}",
        f"Icon={icon_path}",
        "Terminal=false",
        "Categories=Utility;",
    ]
    if autostart:
        lines.append("X-GNOME-Autostart-enabled=true")
    return "\n".join(lines) + "\n"


def install_launcher(tray_script: str | None = None) -> None:
    """(Re)write the app-grid launcher entry and brand icon.

    Idempotent and cheap — safe to call on every tray startup so the paths
    self-heal if the repo moves.
    """
    os.makedirs(_APPLICATIONS_DIR, exist_ok=True)
    icon_path = _install_icon()
    cmd = _command(tray_script)
    with open(_launcher_path(), "w") as f:
        f.write(_desktop_entry(cmd, icon_path, autostart=False))


def enable(tray_script: str | None = None) -> None:
    """Write (or overwrite) the XDG autostart entry."""
    os.makedirs(_AUTOSTART_DIR, exist_ok=True)
    icon_path = _install_icon()
    cmd = _command(tray_script)
    with open(_autostart_path(), "w") as f:
        f.write(_desktop_entry(cmd, icon_path, autostart=True))
    log(f"Autostart enabled: {cmd}")


def disable() -> None:
    """Remove the XDG autostart entry. Idempotent — no error if already absent."""
    try:
        os.remove(_autostart_path())
        log("Autostart disabled")
    except FileNotFoundError:
        pass  # already absent — idempotent


def is_enabled() -> bool:
    """Return True if the autostart entry currently exists.

    Checks the live filesystem on every call so external changes (e.g. the
    user deleting the file manually) are reflected immediately.
    """
    return os.path.isfile(_autostart_path())