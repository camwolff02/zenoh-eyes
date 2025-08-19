#!/usr/bin/env python3
# Electric Eyes — Rich TUI with Zenoh + FlatBuffers publisher
# Arrows/Vim steer LR/UD, +/- eyelid separation, b blink, r reset, q quit.

import sys, os, json, time, math, shutil, select
from pathlib import Path
from dataclasses import dataclass
import flatbuffers
import zenoh
from zenoh import Config

# ------------------ Rich UI ------------------
from rich.console import Console
from rich.live import Live
from rich.panel import Panel
from rich.columns import Columns
from rich.text import Text
from rich.table import Table
from rich.style import Style

# ------------------ Schema -------------------
from electric_eyes import EyeCommand

# ------------------ CLI ----------------------
import argparse
def parse_args():
    p = argparse.ArgumentParser(description="Electric Eyes (Rich TUI)")
    # Limits
    p.add_argument("--lr-min", type=int, default=-90)
    p.add_argument("--lr-max", type=int, default=90)
    p.add_argument("--ud-min", type=int, default=-90)
    p.add_argument("--ud-max", type=int, default=90)
    p.add_argument("--sep-min", type=int, default=0)
    p.add_argument("--sep-max", type=int, default=90)
    # Steps
    p.add_argument("--step-lr", type=int, default=2)
    p.add_argument("--step-ud", type=int, default=2)
    p.add_argument("--step-sep", type=int, default=2)
    # Initial values
    p.add_argument("--lr0", type=int, default=0)
    p.add_argument("--ud0", type=int, default=0)
    p.add_argument("--sep0", type=int, default=90)
    # Zenoh
    p.add_argument("--keyexpr", default="robot/eye_command")
    p.add_argument("--endpoint", action="append", help="Zenoh endpoint, e.g. tcp/192.168.68.136:7447 (repeatable)")
    p.add_argument("--rate-hz", type=float, default=30.0, help="publish rate (Hz)")
    return p.parse_args()

# ------------------ Utils --------------------
def clamp(v, lo, hi): return max(lo, min(hi, v))

def build_eye_cmd(lr, ud, sep, blink):
    b = flatbuffers.Builder(0)
    EyeCommand.EyeCommandStart(b)
    EyeCommand.EyeCommandAddLookLr(b, int(lr))
    EyeCommand.EyeCommandAddLookUd(b, int(ud))
    EyeCommand.EyeCommandAddEyeSep(b, int(sep))
    EyeCommand.EyeCommandAddBlink(b, bool(blink))
    eye = EyeCommand.EyeCommandEnd(b)
    b.Finish(eye)
    return b.Output()

# ------------------ Key reader ----------------
class KeyReader:
    """
    Cross-platform nonblocking key reader.
    - Linux/macOS: termios + select
    - Windows: msvcrt
    Returns strings: 'LEFT','RIGHT','UP','DOWN','-','+','=','b','r','q','h','j','k','l','1','2'
    """
    def __init__(self):
        self.win = os.name == "nt"
        if self.win:
            import msvcrt
            self.msvcrt = msvcrt
        else:
            import termios, tty
            self.termios = termios
            self.tty = tty
            self.fd = sys.stdin.fileno()
            self.old = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)

    def close(self):
        if not self.win:
            self.termios.tcsetattr(self.fd, self.termios.TCSADRAIN, self.old)

    def get_key(self):
        if self.win:
            if not self.msvcrt.kbhit():
                return None
            ch = self.msvcrt.getwch()
            if ch in ("\x00", "\xe0"):
                code = self.msvcrt.getwch()
                return {
                    "K": "UP", "P": "DOWN", "M": "RIGHT", "H": "LEFT"
                }.get(code, None)
            return self._map_char(ch)
        else:
            r, _, _ = select.select([sys.stdin], [], [], 0)
            if not r:
                return None
            ch = sys.stdin.read(1)
            if ch == "\x1b":  # ESC sequence
                # Read rest if available
                time.sleep(0.001)
                r, _, _ = select.select([sys.stdin], [], [], 0)
                if r:
                    rest = sys.stdin.read(2)
                    seq = ch + rest
                    return {
                        "\x1b[A":"UP", "\x1b[B":"DOWN", "\x1b[C":"RIGHT", "\x1b[D":"LEFT"
                    }.get(seq, None)
                return None
            return self._map_char(ch)

    @staticmethod
    def _map_char(ch):
        mapping = {
            "q":"QUIT", "Q":"QUIT",
            "b":"BLINK", "B":"BLINK",
            "r":"RESET", "R":"RESET",
            "-":"SEP_DOWN", "_":"SEP_DOWN",
            "+":"SEP_UP", "=":"SEP_UP",
            "1":"SEP_DOWN", "2":"SEP_UP",
            "h":"LEFT", "l":"RIGHT", "k":"UP", "j":"DOWN",
        }
        return mapping.get(ch, None)

# ------------------ Eye renderer --------------
S_SCLERA = Style(bgcolor="white")
S_IRIS   = Style(bgcolor="#1e90ff")     # base blue
S_IRIS_LIGHT = Style(bgcolor="#5ec8ff") # near-side tint
S_IRIS_DARK  = Style(bgcolor="#0b5ed7") # far-side ring/shadow
S_PUPIL  = Style(bgcolor="black")
S_VEIN   = Style(color="red", bgcolor="white")


def render_eye(width, height, lr, ud, sep) -> Text:
    """
    Colored eye with perspective and rounded eyelids:
      - Sclera (white), veins (red) that MOVE with the eyeball (texture shift)
      - Iris (blue with lighting) & pupil (black) foreshorten with LR/UD
      - Eyelids (SEP) occlude along elliptical arcs (rounded), not flat bands
    Conventions: LR>0 = LEFT, UD>0 = UP.
    """
    from math import cos, pi, sqrt

    width = max(32, width)
    height = max(12, height)

    txt = Text(no_wrap=True)
    cx, cy = width // 2, height // 2
    rx, ry = max(8, (width - 2) // 2), max(4, (height - 2) // 2)  # eye ellipse radii

    # Eyelid aperture from SEP (0..90) -> [0..1]
    aperture = max(0.0, min(1.0, sep / 90.0))
    # Coarse row-level max opening (used to skip entire rows early)
    open_half_y = int(ry * aperture)

    # ---- Constant base sizes ----
    base_r = min(rx, ry)
    iris_base  = max(2, int(base_r * 0.50))
    pupil_base = max(1, int(iris_base * 0.38))

    # ---- Perspective foreshortening (cosine squash) ----
    s_h = 0.35 + 0.65 * cos(abs(lr) * pi / 180.0)  # horizontal squash
    s_v = 0.35 + 0.65 * cos(abs(ud) * pi / 180.0)  # vertical   squash

    iris_rx  = max(1, int(iris_base  * s_h))
    iris_ry  = max(1, int(iris_base  * s_v))
    pupil_rx = max(1, int(pupil_base * s_h))
    pupil_ry = max(1, int(pupil_base * s_v))

    # ---- Iris/pupil center (LR>0 left, UD>0 up) ----
    # (matches your current sign convention)
    k_parallax_x = 0.6
    k_parallax_y = 0.6
    iris_cx = cx - int((-lr / 90.0) * rx * k_parallax_x)
    iris_cy = cy - int((ud  / 90.0) * ry * k_parallax_y)

    # We allow the iris to pass under lids (more realistic). If you want to clamp
    # it within the opening, re-enable a clamp here.

    # ---- Vein texture shift (veins “stick” to the eyeball) ----
    tshift_x = int((-lr / 90.0) * rx * k_parallax_x)
    tshift_y = int((ud  / 90.0) * ry * k_parallax_y)

    # Direction for iris lighting (near/far side)
    d_lr = max(-1.0, min(1.0, -lr / 90.0))
    d_ud = max(-1.0, min(1.0,  ud / 90.0))

    # Precompute denominators
    rx2 = rx * rx
    ry2 = ry * ry

    for y in range(height):
        # Quick row-level occlusion: blank rows entirely outside the maximum opening
        if aperture <= 0.0 or (y < cy - open_half_y) or (y > cy + open_half_y):
            txt.append(" " * width)
            if y != height - 1: txt.append("\n")
            continue

        row = Text(no_wrap=True)
        for x in range(width):
            ex, ey = x - cx, y - cy

            # Eye boundary (ellipse)
            in_eye = (ex * ex) / rx2 + (ey * ey) / ry2 <= 1.0
            if not in_eye:
                row.append(" ")
                continue

            # ---- Rounded eyelids: per-x opening half-height ----
            # At each column, the visible vertical half-span is:
            #   y_cap(x) = aperture * ry * sqrt(1 - (ex^2 / rx^2))
            # This traces an ellipse so the lids are curved, not flat.
            inner = 1.0 - (ex * ex) / rx2
            if inner <= 0.0:
                # Far horizontal extremes of the eye: fully occluded
                row.append(" ")
                continue
            y_cap = int(aperture * ry * sqrt(inner))
            y_top = cy - y_cap
            y_bot = cy + y_cap
            if y < y_top or y > y_bot:
                row.append(" ")
                continue

            # Ellipse membership for iris/pupil (perspective-scaled)
            dx_i, dy_i = x - iris_cx, y - iris_cy
            u = dx_i / max(1, iris_rx)
            v = dy_i / max(1, iris_ry)
            r2_iris = u * u + v * v

            up = dx_i / max(1, pupil_rx)
            vp = dy_i / max(1, pupil_ry)
            r2_pupil = up * up + vp * vp

            if r2_pupil <= 1.0:
                row.append(" ", style=S_PUPIL)
            elif r2_iris <= 1.0:
                # Iris shading: dark rim + near/far lighting
                if r2_iris >= 0.75:
                    style = S_IRIS_DARK
                else:
                    dot = (u * d_lr) + (v * (-d_ud))
                    if dot > 0.25:
                        style = S_IRIS_LIGHT
                    elif dot < -0.25:
                        style = S_IRIS_DARK
                    else:
                        style = S_IRIS
                row.append(" ", style=style)
            else:
                # ---------- Sclera with moving veins ----------
                tx = x + tshift_x
                ty = y + tshift_y
                # mild perspective stretch so veins compress with rotation
                tx_s = int(cx + (tx - cx) / max(0.7, s_h))
                ty_s = int(cy + (ty - cy) / max(0.7, s_v))
                if ((tx_s * 13 + ty_s * 7) % 97 == 0) or ((tx_s + 2 * ty_s) % 59 == 0 and (tx_s ^ ty_s) & 3 == 0):
                    row.append("╱" if (tx_s + ty_s) % 2 else "╲", style=S_VEIN)
                else:
                    row.append(" ", style=S_SCLERA)

        txt.append(row)
        if y != height - 1: txt.append("\n")
    return txt


def _render_eye(width, height, lr, ud, sep) -> Text:
    """
    Colored eye with perspective:
      - Sclera (white), veins (red) that MOVE with the eyeball (texture shift)
      - Iris (blue with lighting) & pupil (black) foreshorten with LR/UD
      - Eyelids (SEP) only occlude by blanking rows; no scaling
    Conventions: LR>0 = LEFT, UD>0 = UP.
    """
    width = max(32, width)
    height = max(12, height)

    txt = Text(no_wrap=True)
    cx, cy = width // 2, height // 2
    rx, ry = max(8, (width - 2) // 2), max(4, (height - 2) // 2)  # eye ellipse radii

    # Eyelid aperture from SEP (0..90) -> [0..1]
    aperture = max(0.0, min(1.0, sep / 90.0))
    open_half_y = max(0, int(ry * aperture))

    # ---- Constant base sizes ----
    base_r = min(rx, ry)
    iris_base  = max(2, int(base_r * 0.50))
    pupil_base = max(1, int(iris_base * 0.38))

    # ---- Perspective foreshortening (cosine squash) ----
    from math import cos, pi
    s_h = 0.35 + 0.65 * cos(abs(lr) * pi / 180.0)  # horizontal squash
    s_v = 0.35 + 0.65 * cos(abs(ud) * pi / 180.0)  # vertical   squash

    iris_rx  = max(1, int(iris_base  * s_h))
    iris_ry  = max(1, int(iris_base  * s_v))
    pupil_rx = max(1, int(pupil_base * s_h))
    pupil_ry = max(1, int(pupil_base * s_v))

    # ---- Iris/pupil center (LR>0 left, UD>0 up) ----
    # Parallax factor (how much the texture shifts per degree). Tune 0.6..0.8 to taste.
    k_parallax_x = 0.6
    k_parallax_y = 0.6

    iris_cx = cx - int((-lr / 90.0) * rx * k_parallax_x)
    iris_cy = cy - int((ud / 90.0) * ry * k_parallax_y)

    # Keep visible when not fully closed (optional)
    if open_half_y > 1:
        iris_cy = max(cy - open_half_y + 1, min(cy + open_half_y - 1, iris_cy))
    iris_cx = max(cx - rx + 1, min(cx + rx - 1, iris_cx))

    # ---- Vein texture shift (makes veins rotate with the eyeball) ----
    # Shift the sclera texture *in the same direction* as the iris center moves.
    tshift_x = int((-lr / 90.0) * rx * k_parallax_x)
    tshift_y = int((ud / 90.0) * ry * k_parallax_y)

    # Direction vector for iris lighting (near/far side)
    d_lr = max(-1.0, min(1.0, -lr / 90.0))
    d_ud = max(-1.0, min(1.0, ud / 90.0))

    for y in range(height):
        # Eyelid occlusion rows
        if (y < cy - open_half_y) or (y > cy + open_half_y):
            txt.append(" " * width)
            if y != height - 1: txt.append("\n")
            continue

        row = Text(no_wrap=True)
        for x in range(width):
            ex, ey = x - cx, y - cy
            in_eye = (ex * ex) / (rx * rx) + (ey * ey) / (ry * ry) <= 1.0
            if not in_eye:
                row.append(" ")
                continue

            # Ellipse membership for iris/pupil (perspective-scaled)
            dx_i, dy_i = x - iris_cx, y - iris_cy
            u = dx_i / max(1, iris_rx)
            v = dy_i / max(1, iris_ry)
            r2_iris = u * u + v * v

            up = dx_i / max(1, pupil_rx)
            vp = dy_i / max(1, pupil_ry)
            r2_pupil = up * up + vp * vp

            if r2_pupil <= 1.0:
                row.append(" ", style=S_PUPIL)
            elif r2_iris <= 1.0:
                # Iris with limbal darkening + near/far lighting
                if r2_iris >= 0.75:
                    style = S_IRIS_DARK
                else:
                    dot = (u * d_lr) + (v * (-d_ud))
                    if dot > 0.25:
                        style = S_IRIS_LIGHT
                    elif dot < -0.25:
                        style = S_IRIS_DARK
                    else:
                        style = S_IRIS
                row.append(" ", style=style)
            else:
                # ---------- Sclera with moving veins ----------
                # Sample the vein pattern in *eye space* by shifting texture coords.
                # This anchors veins to the eyeball so they rotate with LR/UD.
                tx = x + tshift_x
                ty = y + tshift_y

                # Optional: mild perspective stretch on the texture so veins compress with rotation.
                tx_s = int(cx + (tx - cx) / max(0.7, s_h))
                ty_s = int(cy + (ty - cy) / max(0.7, s_v))

                # Deterministic vein sprinkle using shifted/stretched coords
                if ((tx_s * 13 + ty_s * 7) % 97 == 0) or ((tx_s + 2 * ty_s) % 59 == 0 and (tx_s ^ ty_s) & 3 == 0):
                    row.append("╱" if (tx_s + ty_s) % 2 else "╲", style=S_VEIN)
                else:
                    row.append(" ", style=S_SCLERA)

        txt.append(row)
        if y != height - 1: txt.append("\n")
    return txt

# ------------------ Panels --------------------
def controls_panel() -> Panel:
    t = Table.grid(padding=(0,1))
    t.add_row("[bold]Controls[/bold]")
    t.add_row("←/h: LR-", "→/l: LR+")
    t.add_row("↑/k: UD+", "↓/j: UD−")
    t.add_row("+/2: SEP +", "-/1: SEP −")
    t.add_row("b: Blink", "r: Reset")
    t.add_row("[dim]q: Quit[/dim]")
    return Panel(t, title="Controls", border_style="magenta")

def state_panel(lr, ud, sep, limits, rate_hz, keyexpr, endpoints) -> Panel:
    def bar(val, lo, hi, width=20):
        ratio = 0 if hi == lo else (val - lo) / (hi - lo)
        filled = int(ratio * width)
        s = "[" + "#" * filled + "-" * (width - filled) + "]"
        return f"{s} {val:>4}"

    eps = ", ".join(endpoints) if endpoints else "(discovery)"
    t = Table.grid(padding=(0,1))
    t.add_row(f"LR {bar(lr, limits['lr'][0], limits['lr'][1])}")
    t.add_row(f"UD {bar(ud, limits['ud'][0], limits['ud'][1])}")
    t.add_row(f"SEP {bar(sep, limits['sep'][0], limits['sep'][1])}")
    t.add_row(f"[dim]Rate:[/dim] {rate_hz:.1f} Hz")
    t.add_row(f"[dim]Keyexpr:[/dim] {keyexpr}")
    t.add_row(f"[dim]Endpoints:[/dim] {eps}")
    return Panel(t, title="State", border_style="cyan")

# ------------------ Zenoh wrapper -------------
class ZPub:
    def __init__(self, keyexpr, endpoints):
        self.keyexpr = keyexpr
        self.endpoints = endpoints or []
        self.session = None
        self.pub = None
    def open(self):
        conf = Config()
        if self.endpoints:
            conf.insert_json5("connect/endpoints", json.dumps(self.endpoints))
        self.session = zenoh.open(conf)
        self.pub = self.session.declare_publisher(self.keyexpr)
    def close(self):
        try:
            if self.pub: self.pub.close()
        finally:
            if self.session: self.session.close()
    def put(self, lr, ud, sep, blink=False):
        if self.pub:
            self.pub.put(build_eye_cmd(lr, ud, sep, blink))

# ------------------ Main loop -----------------
def main():
    args = parse_args()
    console = Console()
    reader = KeyReader()

    # State
    lr = clamp(args.lr0, args.lr_min, args.lr_max)
    ud = clamp(args.ud0, args.ud_min, args.ud_max)
    sep = clamp(args.sep0, args.sep_min, args.sep_max)
    blink_once = False

    limits = {"lr": (args.lr_min, args.lr_max),
              "ud": (args.ud_min, args.ud_max),
              "sep": (args.sep_min, args.sep_max)}
    steps = {"lr": args.step_lr, "ud": args.step_ud, "sep": args.step_sep}

    pub = ZPub(args.keyexpr, args.endpoint)
    try:
        pub.open()
    except Exception as e:
        console.print(f"[red]Zenoh open failed:[/red] {e}")
        return

    interval = 1.0 / max(1e-6, args.rate_hz)
    next_pub = 0.0

    try:
        with Live(console=console, refresh_per_second=30, screen=True) as live:
            while True:
                now = time.time()

                # --- Handle input
                key = reader.get_key()
                if key:
                    if key == "QUIT":
                        break
                    elif key == "BLINK":
                        blink_once = True
                        pub.put(lr, ud, sep, blink=True)  # immediate one-shot
                        next_pub = now + interval
                    elif key == "RESET":
                        lr, ud, sep = 0, 0, limits["sep"][1]
                    elif key == "LEFT":
                        lr = clamp(lr - steps["lr"], *limits["lr"])   # left is positive
                    elif key == "RIGHT":
                        lr = clamp(lr + steps["lr"], *limits["lr"])
                    elif key == "UP":
                        ud = clamp(ud + steps["ud"], *limits["ud"])
                    elif key == "DOWN":
                        ud = clamp(ud - steps["ud"], *limits["ud"])
                    elif key == "SEP_UP":
                        sep = clamp(sep + steps["sep"], *limits["sep"])
                    elif key == "SEP_DOWN":
                        sep = clamp(sep - steps["sep"], *limits["sep"])

                # --- Periodic publish
                if now >= next_pub and not blink_once:
                    pub.put(lr, ud, sep, blink=False)
                    next_pub = now + interval
                blink_once = False

                # --- Layout & render
                term_w = console.size.width
                term_h = console.size.height
                side_w = 32
                eye_w = max(30, term_w - (side_w * 2) - 4)
                eye_h = max(12, term_h - 6)

                eye = Panel(render_eye(eye_w, eye_h, lr, ud, sep),
                            title="Eye", border_style="green")

                controls = controls_panel()
                state = state_panel(lr, ud, sep, limits,
                                    args.rate_hz, args.keyexpr, args.endpoint)

                content = Columns([controls, eye, state], expand=True, equal=False,
                                  column_first=True, width=None)
                live.update(content)

                time.sleep(0.01)

    finally:
        reader.close()
        try:
            pub.close()
        except Exception:
            pass

if __name__ == "__main__":
    main()
