#!/usr/bin/env python3
# Electric Eyes — Rich TUI with Zenoh + FlatBuffers publisher

import sys, os, json, time, select
import argparse
import flatbuffers
import zenoh
from zenoh import Config

from rich.console import Console
from rich.live import Live
from rich.panel import Panel
from rich.columns import Columns
from rich.text import Text
from rich.table import Table
from rich.style import Style
from rich.layout import Layout

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
    # Step increment defaults & bounds
    p.add_argument("--step-lr", type=int, default=2)
    p.add_argument("--step-ud", type=int, default=2)
    p.add_argument("--step-sep", type=int, default=2)
    p.add_argument("--step-min", type=int, default=1, help="Minimum step size")
    p.add_argument("--step-max", type=int, default=10, help="Maximum step size")
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
    # in parse_args()
    p.add_argument("--blink-ms", type=int, default=350,
               help="UI blink animation duration in milliseconds")
    p.add_argument("--aspect", type=float, default=1.5,
                help="Vertical compensation factor (rows are taller). 1.0=no compensation; Increasing means more circular, decreasing means more oblong.")

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
    Nonblocking cross-platform key reader with a high-level API.

    Prefers 'blessed' (TERM-aware, readable names) and falls back to a
    hardened manual parser to avoid splitting ESC sequences.

    Returns actions: LEFT, RIGHT, UP, DOWN,
                     SEP_UP, SEP_DOWN, STEP_INC, STEP_DEC,
                     BLINK, RESET, QUIT
                     (plus Vim: h/j/k/l)
    """
    def __init__(self):
        self.win = os.name == "nt"
        self._use_blessed = False
        self._blessed_term = None
        self._blessed_cbreak_ctx = None

        # Try high-level library first
        try:
            from blessed import Terminal  # type: ignore
            self._blessed_term = Terminal()
            # enter cbreak (raw-ish) mode so inkey() is instantaneous
            self._blessed_cbreak_ctx = self._blessed_term.cbreak()
            self._blessed_cbreak_ctx.__enter__()
            self._use_blessed = True
        except Exception:
            self._use_blessed = False

        # Fallback setup (only if blessed unavailable)
        if not self._use_blessed:
            if self.win:
                import msvcrt
                self.msvcrt = msvcrt
            else:
                import termios, tty, fcntl
                self.termios = termios
                self.tty = tty
                self.fcntl = fcntl
                self.fd = sys.stdin.fileno()
                self.old = termios.tcgetattr(self.fd)
                tty.setcbreak(self.fd)
                # Make stdin nonblocking
                fl = fcntl.fcntl(self.fd, fcntl.F_GETFL)
                fcntl.fcntl(self.fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)
            # Buffer to assemble ESC sequences across reads
            self._buf = ""

    def close(self):
        if self._use_blessed:
            try:
                if self._blessed_cbreak_ctx:
                    self._blessed_cbreak_ctx.__exit__(None, None, None)
            except Exception:
                pass
            return
        # Fallback teardown
        if not self.win:
            try:
                self.termios.tcsetattr(self.fd, self.termios.TCSADRAIN, self.old)
            except Exception:
                pass

    # -------- Mapping helpers --------
    @staticmethod
    def _map_action_from_char(ch: str):
        mapping = {
            "q":"QUIT", "Q":"QUIT",
            "b":"BLINK", "B":"BLINK",
            "r":"RESET", "R":"RESET",
            # eyelids on w/s
            "w":"SEP_UP", "W":"SEP_UP",
            "s":"SEP_DOWN", "S":"SEP_DOWN",
            # step size on +/- and =
            "+":"STEP_INC", "=":"STEP_INC",
            "-":"STEP_DEC", "_":"STEP_DEC",
            # movement (vim)
            "h":"LEFT", "l":"RIGHT", "k":"UP", "j":"DOWN",
        }
        return mapping.get(ch)

    @staticmethod
    def _map_action_from_name(name: str):
        # blessed names like 'KEY_LEFT', 'KEY_RIGHT', etc.
        if name in ("KEY_LEFT",):  return "LEFT"
        if name in ("KEY_RIGHT",): return "RIGHT"
        if name in ("KEY_UP",):    return "UP"
        if name in ("KEY_DOWN",):  return "DOWN"
        return None

    @staticmethod
    def _map_esc_sequence(seq: str):
        # CSI/SS3 arrow keys end with A/B/C/D; ensure we only return arrows
        if not seq or seq[0] != "\x1b":
            return None
        final = seq[-1]
        return {"A":"UP", "B":"DOWN", "C":"RIGHT", "D":"LEFT"}.get(final)

    # -------- Public API --------
    def get_all_keys(self):
        """Drain all pending keys and return a list of mapped actions."""
        actions = []

        # ---- Preferred: blessed ----
        if self._use_blessed:
            t = self._blessed_term
            # inkey(timeout=0) returns immediately; loop until no key
            while True:
                k = t.inkey(timeout=0)
                if not k:
                    break
                if k.is_sequence:
                    # Use human-readable name when available
                    name = getattr(k, "name", "") or ""
                    act = self._map_action_from_name(name)
                    if not act:
                        # Fallback to last char of the sequence
                        act = self._map_esc_sequence(str(k))
                else:
                    act = self._map_action_from_char(str(k))
                if act:
                    actions.append(act)
            return actions

        # ---- Fallback: Windows (msvcrt) ----
        if self.win:
            while self.msvcrt.kbhit():
                ch = self.msvcrt.getwch()
                if ch in ("\x00", "\xe0"):
                    code = self.msvcrt.getwch()
                    actions.append({"K":"UP","P":"DOWN","M":"RIGHT","H":"LEFT"}.get(code))
                else:
                    actions.append(self._map_action_from_char(ch))
            return [a for a in actions if a]

        # ---- Fallback: POSIX manual parser with buffering ----
        # Read as many bytes as available, accumulate, then parse greedily.
        # This avoids splitting ESC sequences (no stray 'B' triggering Blink).
        while True:
            r, _, _ = select.select([sys.stdin], [], [], 0)
            if not r:
                break
            try:
                data = sys.stdin.read(1024)
            except BlockingIOError:
                break
            if not data:
                break
            self._buf += data

        i = 0
        n = len(self._buf)
        while i < n:
            ch = self._buf[i]
            if ch != "\x1b":
                act = self._map_action_from_char(ch)
                if act: actions.append(act)
                i += 1
                continue

            # Attempt to parse CSI or SS3 sequence
            # CSI: \x1b [ ... final
            # SS3: \x1b O final
            if i + 1 >= n:
                break  # incomplete, keep for next frame
            nxt = self._buf[i+1]
            if nxt in ("[", "O"):
                # scan forward until we hit a letter A–Z or run out
                j = i + 2
                while j < n and not self._buf[j].isalpha():
                    j += 1
                if j < n:
                    seq = self._buf[i:j+1]
                    act = self._map_esc_sequence(seq)
                    if act: actions.append(act)
                    i = j + 1
                    continue
                else:
                    break  # incomplete; keep for next frame
            else:
                # Lone ESC (user pressed Escape) — ignore to avoid conflicts
                i += 1

        # Drop consumed bytes
        self._buf = self._buf[i:]
        return actions

# ------------------ Eye renderer --------------
S_SCLERA = Style(bgcolor="white")
S_IRIS   = Style(bgcolor="#1e90ff")
S_IRIS_LIGHT = Style(bgcolor="#5ec8ff")
S_IRIS_DARK  = Style(bgcolor="#0b5ed7")
S_PUPIL  = Style(bgcolor="black")
S_VEIN   = Style(color="red", bgcolor="white")

def render_eye(width, height, lr, ud, sep, aspect: float = 0.55) -> Text:
    """
    Draw a circular eye with vertical 'aspect' compensation so it looks round
    in terminals where rows are taller than columns.

    aspect < 1 compresses vertical distances in the math (typical 0.5–0.6).
    """
    from math import cos, pi, sqrt

    width  = max(32, width)
    height = max(12, height)
    txt = Text(no_wrap=True)

    # Center of panel
    cx, cy = width // 2, height // 2

    # ---- FIXED CIRCLE with optical compensation ----
    # For a circle test ex^2 + (ey*aspect)^2 <= r^2 to fit within the panel,
    # r must be <= half-width AND <= (half-height * aspect).
    half_w = (width  - 2) // 2
    half_h = (height - 2) // 2
    r = max(6, min(half_w, int(half_h * max(0.01, aspect))))  # guard aspect>0
    r2 = r * r

    # Eyelid aperture from SEP (0..90) -> [0..1]
    aperture = max(0.0, min(1.0, sep / 90.0))
    # Coarse row-level occlusion bound (in compensated units)
    open_half = int(r * aperture)

    # ---- Sizes from r (constant eye geometry) ----
    iris_base  = max(2, int(r * 0.50))
    pupil_base = max(1, int(iris_base * 0.38))

    # Foreshortening (cosine squash)
    s_h = 0.35 + 0.65 * cos(abs(lr) * pi / 180.0)
    s_v = 0.35 + 0.65 * cos(abs(ud) * pi / 180.0)

    iris_rx  = max(1, int(iris_base  * s_h))
    iris_ry  = max(1, int(iris_base  * s_v))
    pupil_rx = max(1, int(pupil_base * s_h))
    pupil_ry = max(1, int(pupil_base * s_v))

    # Iris/pupil center (LR>0 left, UD>0 up). Vertical displacement uses r/aspect
    # so the eyeball motion looks symmetric with the compensated geometry.
    kx = 0.6
    ky = 0.6
    iris_cx = cx - int((-lr / 90.0) * r * kx)
    iris_cy = cy - int((  ud / 90.0) * (r / max(0.01, aspect)) * ky)

    # Vein texture shift so veins "stick" to eyeball (match iris motion scale)
    tshift_x = int((-lr / 90.0) * r * kx)
    tshift_y = int((  ud / 90.0) * (r / max(0.01, aspect)) * ky)

    # Lighting direction (near/far)
    d_lr = max(-1.0, min(1.0, -lr / 90.0))
    d_ud = max(-1.0, min(1.0,  ud / 90.0))

    for y in range(height):
        # Quick row skip using compensated distance from center
        if aperture <= 0.0 or abs((y - cy) * aspect) > open_half:
            txt.append(" " * width)
            if y != height - 1:
                txt.append("\n")
            continue

        row = Text(no_wrap=True)
        for x in range(width):
            ex = x - cx
            ey = y - cy
            eyc = ey * aspect  # compensated y

            # Circle boundary with compensation
            if (ex * ex + eyc * eyc) > r2:
                row.append(" ")
                continue

            # Rounded eyelids per-x in compensated space:
            # y_cap_c = aperture * sqrt(r^2 - ex^2)
            inner = r2 - ex * ex
            if inner <= 0:
                row.append(" ")
                continue
            y_cap_c = int(aperture * sqrt(inner))  # compensated units
            if abs(eyc) > y_cap_c:
                row.append(" ")
                continue

            # Iris/pupil membership (use compensated dy for circular look)
            dx_i = x - iris_cx
            dy_i_c = (y - iris_cy) * aspect

            u = dx_i   / max(1, iris_rx)
            v = dy_i_c / max(1, iris_ry)
            r2_iris = u * u + v * v

            up = dx_i   / max(1, pupil_rx)
            vp = dy_i_c / max(1, pupil_ry)
            r2_pupil = up * up + vp * vp

            if r2_pupil <= 1.0:
                row.append(" ", style=S_PUPIL)
            elif r2_iris <= 1.0:
                # Iris shading: dark rim + near/far lighting
                if r2_iris >= 0.75:
                    style = S_IRIS_DARK
                else:
                    dot = (u * d_lr) + (v * (-d_ud))
                    if   dot >  0.25: style = S_IRIS_LIGHT
                    elif dot < -0.25: style = S_IRIS_DARK
                    else:             style = S_IRIS
                row.append(" ", style=style)
            else:
                # Sclera with moving veins (keep a bit of perspective squash)
                tx = x + tshift_x
                ty = y + tshift_y
                tx_s = int(cx + (tx - cx) / max(0.7, s_h))
                ty_s = int(cy + (ty - cy) / max(0.7, s_v))
                if ((tx_s * 13 + ty_s * 7) % 97 == 0) or ((tx_s + 2 * ty_s) % 59 == 0 and (tx_s ^ ty_s) & 3 == 0):
                    row.append("╱" if (tx_s + ty_s) % 2 else "╲", style=S_VEIN)
                else:
                    row.append(" ", style=S_SCLERA)

        txt.append(row)
        if y != height - 1:
            txt.append("\n")

    return txt

# ------------------ Panels --------------------
def controls_panel() -> Panel:
    t = Table.grid(padding=(0,1))
    t.add_row("[bold]Controls[/bold]")
    t.add_row("←/h: LR−", "→/l: LR+")
    t.add_row("↑/k: UD+", "↓/j: UD−")
    t.add_row("w: SEP +", "s: SEP −")
    t.add_row("+ / = : increase step size")
    t.add_row("- / _ : decrease step size")
    t.add_row("b: Blink (one-shot)", "r: Reset")
    t.add_row("[dim]q: Quit[/dim]")
    return Panel(t, title="Controls", border_style="magenta")

def state_panel(lr, ud, sep, limits, rate_hz, keyexpr, endpoints, blinking: bool, steps=None) -> Panel:
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
    if steps:
        t.add_row(f"[yellow]Step size:[/yellow] LR={steps['lr']}  UD={steps['ud']}  SEP={steps['sep']}")
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

    # Step increments and limits
    steps = {
        "lr":  clamp(args.step_lr,  args.step_min, args.step_max),
        "ud":  clamp(args.step_ud,  args.step_min, args.step_max),
        "sep": clamp(args.step_sep, args.step_min, args.step_max),
    }

    # Blink animation state (UI only)
    BLINK_MS = max(80, int(args.blink_ms))  # don't let it get too tiny
    blink_active = False
    blink_t0 = 0.0

    limits = {"lr": (args.lr_min, args.lr_max),
              "ud": (args.ud_min, args.ud_max),
              "sep": (args.sep_min, args.sep_max)}

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

                # --- Handle all pending input
                for key in reader.get_all_keys():
                    if key == "QUIT":
                        raise KeyboardInterrupt
                    elif key == "BLINK":
                        pub.put(lr, ud, sep, blink=True)
                        blink_active = True
                        blink_t0 = now
                        next_pub = now + interval
                    elif key == "RESET":
                        lr, ud, sep = 0, 0, limits["sep"][1]
                    elif key == "LEFT":
                        lr = clamp(lr - steps["lr"], *limits["lr"])
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
                    elif key == "STEP_INC":
                        steps["lr"]  = clamp(steps["lr"]  + 1, args.step_min, args.step_max)
                        steps["ud"]  = clamp(steps["ud"]  + 1, args.step_min, args.step_max)
                        steps["sep"] = clamp(steps["sep"] + 1, args.step_min, args.step_max)
                    elif key == "STEP_DEC":
                        steps["lr"]  = clamp(steps["lr"]  - 1, args.step_min, args.step_max)
                        steps["ud"]  = clamp(steps["ud"]  - 1, args.step_min, args.step_max)
                        steps["sep"] = clamp(steps["sep"] - 1, args.step_min, args.step_max)

                # --- Periodic publish
                if now >= next_pub:
                    pub.put(lr, ud, sep, blink=False)
                    next_pub = now + interval

                # --- Blink UI animation
                vis_sep = sep
                if blink_active:
                    t_ms = (now - blink_t0) * 1000.0
                    if t_ms >= BLINK_MS:
                        blink_active = False
                    else:
                        phase = t_ms / BLINK_MS
                        frac = 1.0 - (phase / 0.5) if phase < 0.5 else (phase - 0.5) / 0.5
                        blink_sep = int(args.sep_max * frac)
                        vis_sep = min(sep, blink_sep)

                # --- Layout & render (Eye below both columns)
                term_w = console.size.width
                term_h = console.size.height

                controls = controls_panel()
                state = state_panel(
                    lr, ud, sep, limits,
                    args.rate_hz, args.keyexpr, args.endpoint,
                    blinking=blink_active, steps=steps
                )

                # Eye spans the full width on the bottom row.
                # Estimate a reasonable drawing size; Rich will clip gracefully if needed.
                eye_w = max(30, term_w - 4)
                # Leave ~10–12 rows for the top row & borders; rest goes to the eye.
                eye_h = max(12, term_h - 12)
                eye_panel = None

                if not args.no_eye:
                    eye_panel = Panel(
                        render_eye(eye_w, eye_h, lr, ud, vis_sep, args.aspect),
                        title="Eye", border_style="green"
                    )

                layout = Layout()
                layout.split(
                    Layout(name="top", ratio=1),
                    Layout(name="eye", ratio=2),
                )
                layout["top"].split_row(
                    Layout(controls, name="controls", ratio=1),
                    Layout(state,    name="state",    ratio=1),
                )
                layout["eye"].update(eye_panel or Panel(Text("Eye disabled (--no-eye)"), title="Eye"))

                live.update(layout)
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
