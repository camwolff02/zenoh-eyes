#!/usr/bin/env python3
# Electric Eyes — Rich TUI with Zenoh + FlatBuffers publisher
# Arrows/Vim steer LR/UD, +/- eyelid separation, b blink, r reset, q quit.
# Fixes: state shows blink, visible blink animation, faster arrows, --no-eye to hide eye panel.

import sys, os, json, time, select
import argparse
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
    p.add_argument("--endpoint", action="append",
                   help="Zenoh endpoint, e.g. tcp/192.168.68.136:7447 (repeatable)")
    p.add_argument("--rate-hz", type=float, default=30.0, help="publish rate (Hz)")
    # UI
    p.add_argument("--no-eye", action="store_true",
                   help="Run without the Eye panel (controls + state only)")
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
    We parse full ESC sequences (e.g., "\x1b[1;2A") and return:
      'LEFT','RIGHT','UP','DOWN','SEP_DOWN','SEP_UP','BLINK','RESET','QUIT'
      plus Vim: h,l,k,j and 1/2 for eyelids.
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

    def _map_char(self, ch):
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

    def _map_esc_sequence(self, seq: str):
        # Accept many forms: ESC[A, ESC[1;2A, ESC[5A], etc. We only care about final letter.
        if not seq.startswith("\x1b"):
            return None
        if "A" in seq[-1]:
            return "UP"
        if "B" in seq[-1]:
            return "DOWN"
        if "C" in seq[-1]:
            return "RIGHT"
        if "D" in seq[-1]:
            return "LEFT"
        # Try a simple endswith as a fallback
        if seq.endswith("A"): return "UP"
        if seq.endswith("B"): return "DOWN"
        if seq.endswith("C"): return "RIGHT"
        if seq.endswith("D"): return "LEFT"
        return None

    def get_all_keys(self):
        """Drain all currently pending keys and return a list of mapped actions."""
        actions = []
        if self.win:
            while self.msvcrt.kbhit():
                ch = self.msvcrt.getwch()
                if ch in ("\x00", "\xe0"):
                    code = self.msvcrt.getwch()
                    actions.append({"K":"UP","P":"DOWN","M":"RIGHT","H":"LEFT"}.get(code))
                else:
                    actions.append(self._map_char(ch))
            return [a for a in actions if a]
        else:
            # POSIX: read everything available without sleeps
            while True:
                r, _, _ = select.select([sys.stdin], [], [], 0)
                if not r:
                    break
                ch = sys.stdin.read(1)
                if ch == "\x1b":
                    # gobble the rest of the escape sequence
                    seq = ch
                    while True:
                        r2, _, _ = select.select([sys.stdin], [], [], 0)
                        if not r2:
                            break
                        seq += sys.stdin.read(1)
                    actions.append(self._map_esc_sequence(seq))
                else:
                    actions.append(self._map_char(ch))
            return [a for a in actions if a]

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
    k_parallax_x = 0.6
    k_parallax_y = 0.6
    iris_cx = cx - int((-lr / 90.0) * rx * k_parallax_x)
    iris_cy = cy - int((ud  / 90.0) * ry * k_parallax_y)

    # Vein texture shift to stick on eyeball
    tshift_x = int((-lr / 90.0) * rx * k_parallax_x)
    tshift_y = int((ud  / 90.0) * ry * k_parallax_y)

    # Direction for iris lighting (near/far side)
    d_lr = max(-1.0, min(1.0, -lr / 90.0))
    d_ud = max(-1.0, min(1.0,  ud / 90.0))

    rx2 = rx * rx
    for y in range(height):
        # Quick row-level occlusion (max possible)
        if aperture <= 0.0 or (y < cy - open_half_y) or (y > cy + open_half_y):
            txt.append(" " * width)
            if y != height - 1: txt.append("\n")
            continue

        row = Text(no_wrap=True)
        for x in range(width):
            ex, ey = x - cx, y - cy

            # Eye boundary (ellipse)
            in_eye = (ex * ex) / (rx2) + (ey * ey) / ((ry) * (ry)) <= 1.0
            if not in_eye:
                row.append(" ")
                continue

            # Rounded eyelids: per-x opening half-height
            inner = 1.0 - (ex * ex) / (rx2)
            if inner <= 0.0:
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
                # Sclera with moving veins (texture-space shift & mild squash)
                tx = x + tshift_x
                ty = y + tshift_y
                # mild perspective stretch
                tx_s = int(cx + (tx - cx) / max(0.7, s_h))
                ty_s = int(cy + (ty - cy) / max(0.7, s_v))
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
    t.add_row("←/h: LR−", "→/l: LR+")
    t.add_row("↑/k: UD+", "↓/j: UD−")
    t.add_row("+/=/2: SEP +", "-/_/1: SEP −")
    t.add_row("b: Blink (one-shot)", "r: Reset")
    t.add_row("[dim]q: Quit[/dim]")
    return Panel(t, title="Controls", border_style="magenta")

def state_panel(lr, ud, sep, limits, rate_hz, keyexpr, endpoints, blinking: bool) -> Panel:
    def bar(val, lo, hi, width=20):
        ratio = 0 if hi == lo else (val - lo) / (hi - lo)
        ratio = max(0.0, min(1.0, ratio))
        filled = int(ratio * width)
        s = "[" + "#" * filled + "-" * (width - filled) + "]"
        return f"{s} {val:>4}"

    eps = ", ".join(endpoints) if endpoints else "(discovery)"
    t = Table.grid(padding=(0,1))
    t.add_row(f"LR  {bar(lr, limits['lr'][0],  limits['lr'][1])}")
    t.add_row(f"UD  {bar(ud, limits['ud'][0],  limits['ud'][1])}")
    t.add_row(f"SEP {bar(sep, limits['sep'][0], limits['sep'][1])}")
    t.add_row(f"[dim]Blink:[/dim] {'[bold green]YES[/bold green]' if blinking else '[dim]no[/dim]'}")
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

    # Blink animation state (UI only)
    BLINK_MS = 160  # total duration (close+open)
    blink_active = False
    blink_t0 = 0.0

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

                # --- Handle all pending input (faster arrows)
                for key in reader.get_all_keys():
                    if key == "QUIT":
                        raise KeyboardInterrupt
                    elif key == "BLINK":
                        # one-shot publish + start UI animation
                        pub.put(lr, ud, sep, blink=True)
                        blink_active = True
                        blink_t0 = now
                        next_pub = now + interval
                    elif key == "RESET":
                        lr, ud, sep = 0, 0, limits["sep"][1]
                    elif key == "LEFT":
                        lr = clamp(lr - steps["lr"], *limits["lr"])   # (legend shows LR− on left)
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

                # --- Periodic publish (no continuous blink flag)
                if now >= next_pub:
                    pub.put(lr, ud, sep, blink=False)
                    next_pub = now + interval

                # --- Blink UI animation: compute a temporary visual SEP
                vis_sep = sep
                if blink_active:
                    t_ms = (now - blink_t0) * 1000.0
                    if t_ms >= BLINK_MS:
                        blink_active = False
                    else:
                        # Triangle wave 1→0→1 over BLINK_MS
                        phase = t_ms / BLINK_MS
                        if phase < 0.5:
                            frac = 1.0 - (phase / 0.5)        # 1 -> 0
                        else:
                            frac = (phase - 0.5) / 0.5        # 0 -> 1
                        blink_sep = int(args.sep_max * frac)
                        vis_sep = min(sep, blink_sep)         # blink only occludes

                # --- Layout & render
                term_w = console.size.width
                term_h = console.size.height
                side_w = 32
                eye_w = max(30, term_w - (side_w * 2) - 4)
                eye_h = max(12, term_h - 6)

                panels = [controls_panel()]
                if not args.no_eye:
                    panels.append(Panel(render_eye(eye_w, eye_h, lr, ud, vis_sep),
                                        title="Eye", border_style="green"))
                panels.append(state_panel(lr, ud, sep, limits,
                                          args.rate_hz, args.keyexpr, args.endpoint,
                                          blinking=blink_active))

                content = Columns(panels, expand=True, equal=False, column_first=True)
                live.update(content)

                # Tight frame; input is nonblocking so we don't need a large sleep
                time.sleep(0.005)

    except KeyboardInterrupt:
        pass
    finally:
        reader.close()
        try:
            pub.close()
        except Exception:
            pass

if __name__ == "__main__":
    main()
