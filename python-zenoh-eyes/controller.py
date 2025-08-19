#!/usr/bin/env python3
"""Electric Eyes — Rich TUI with Zenoh + FlatBuffers publisher."""

import argparse
import json

# import msvcrt
import os
import time
from math import cos, pi, sqrt
from typing import List, Optional

import flatbuffers
import zenoh
from blessed import Terminal
from rich.console import Console
from rich.layout import Layout
from rich.live import Live
from rich.panel import Panel
from rich.style import Style
from rich.table import Table
from rich.text import Text

from electric_eyes import EyeCommand


# ------------------ CLI ----------------------
def parse_args() -> argparse.Namespace:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description="Electric Eyes (Rich TUI)")
    # Limits
    parser.add_argument("--lr-min", type=int, default=-90)
    parser.add_argument("--lr-max", type=int, default=90)
    parser.add_argument("--ud-min", type=int, default=-90)
    parser.add_argument("--ud-max", type=int, default=90)
    parser.add_argument("--sep-min", type=int, default=0)
    parser.add_argument("--sep-max", type=int, default=90)
    # Step increment defaults & bounds
    parser.add_argument("--step-lr", type=int, default=2)
    parser.add_argument("--step-ud", type=int, default=2)
    parser.add_argument("--step-sep", type=int, default=2)
    parser.add_argument("--step-min", type=int, default=1, help="Minimum step size")
    parser.add_argument("--step-max", type=int, default=10, help="Maximum step size")
    # Initial values
    parser.add_argument("--lr0", type=int, default=0)
    parser.add_argument("--ud0", type=int, default=0)
    parser.add_argument("--sep0", type=int, default=90)
    # Zenoh
    parser.add_argument("--keyexpr", default="robot/eye_command")
    parser.add_argument(
        "--endpoint",
        action="append",
        help="Zenoh endpoint, e.g. tcp/192.168.68.136:7447 (repeatable)",
    )
    parser.add_argument("--rate-hz", type=float, default=30.0, help="publish rate (Hz)")
    # UI
    parser.add_argument(
        "--no-eye",
        action="store_true",
        help="Run without the Eye panel (controls + state only)",
    )
    parser.add_argument(
        "--blink-ms",
        type=int,
        default=350,
        help="UI blink animation duration in milliseconds",
    )
    parser.add_argument(
        "--aspect",
        type=float,
        default=1.5,
        help=(
            "Vertical compensation factor (rows are taller). 1.0=no compensation; "
            "Increasing means more circular, decreasing means more oblong."
        ),
    )
    parser.add_argument(
        "--rim",
        type=int,
        default=1,
        help="Limbal ring thickness in cells (0 disables)",
    )
    return parser.parse_args()


# ------------------ Utils --------------------
def clamp(val: int, lo: int, hi: int) -> int:
    """Clamp integer value to [lo, hi]."""
    return max(lo, min(hi, val))


def build_eye_cmd(lr: int, ud: int, sep: int, blink: bool) -> bytes:
    """Build FlatBuffers payload for EyeCommand."""
    builder = flatbuffers.Builder(0)
    EyeCommand.EyeCommandStart(builder)
    EyeCommand.EyeCommandAddLookLr(builder, int(lr))
    EyeCommand.EyeCommandAddLookUd(builder, int(ud))
    EyeCommand.EyeCommandAddEyeSep(builder, int(sep))
    EyeCommand.EyeCommandAddBlink(builder, bool(blink))
    eye = EyeCommand.EyeCommandEnd(builder)
    builder.Finish(eye)
    return builder.Output()


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

    def __init__(self) -> None:
        self.win = os.name == "nt"
        self._blessed_term = None
        self._blessed_cbreak_ctx = None

        self._blessed_term = Terminal()
        # enter cbreak (raw-ish) mode so inkey() is instantaneous
        self._blessed_cbreak_ctx = self._blessed_term.cbreak()
        self._blessed_cbreak_ctx.__enter__()

        # Buffer to assemble ESC sequences across reads
        self._buf = ""

    def close(self) -> None:
        """Restore terminal modes for fallback; exit blessed cbreak if used."""
        try:
            if self._blessed_cbreak_ctx:
                self._blessed_cbreak_ctx.__exit__(None, None, None)
        except Exception:  # pylint: disable=broad-except
            pass

    @staticmethod
    def _map_action_from_char(ch: str) -> Optional[str]:
        mapping = {
            "q": "QUIT",
            "Q": "QUIT",
            "b": "BLINK",
            "B": "BLINK",
            "r": "RESET",
            "R": "RESET",
            # eyelids on w/s
            "w": "SEP_UP",
            "W": "SEP_UP",
            "s": "SEP_DOWN",
            "S": "SEP_DOWN",
            # step size on +/- and =
            "+": "STEP_INC",
            "=": "STEP_INC",
            "-": "STEP_DEC",
            "_": "STEP_DEC",
            # movement (vim)
            "h": "LEFT",
            "l": "RIGHT",
            "k": "UP",
            "j": "DOWN",
        }
        return mapping.get(ch)

    @staticmethod
    def _map_action_from_name(name: str) -> Optional[str]:
        # blessed names like 'KEY_LEFT', 'KEY_RIGHT', etc.
        if name in ("KEY_LEFT",):
            return "LEFT"
        if name in ("KEY_RIGHT",):
            return "RIGHT"
        if name in ("KEY_UP",):
            return "UP"
        if name in ("KEY_DOWN",):
            return "DOWN"
        return None

    @staticmethod
    def _map_esc_sequence(seq: str) -> Optional[str]:
        # CSI/SS3 arrow keys end with A/B/C/D; ensure we only return arrows
        if not seq or seq[0] != "\x1b":
            return None
        final = seq[-1]
        return {"A": "UP", "B": "DOWN", "C": "RIGHT", "D": "LEFT"}.get(final)

    def get_all_keys(self) -> List[str]:
        """Drain all pending keys and return a list of mapped actions."""
        actions: List[str] = []

        term = self._blessed_term
        while True:
            key = term.inkey(timeout=0)
            if not key:
                break
            if key.is_sequence:
                name = getattr(key, "name", "") or ""
                act = self._map_action_from_name(name) or self._map_esc_sequence(str(key))
            else:
                act = self._map_action_from_char(str(key))
            if act:
                actions.append(act)
        return actions


# ------------------ Eye renderer --------------
S_SCLERA = Style(bgcolor="white")
S_IRIS = Style(bgcolor="#1e90ff")
S_IRIS_LIGHT = Style(bgcolor="#5ec8ff")
S_IRIS_DARK = Style(bgcolor="#0b5ed7")
S_PUPIL = Style(bgcolor="black")
S_VEIN = Style(color="red", bgcolor="white")


def render_eye(
    width: int,
    height: int,
    lr: int,
    ud: int,
    sep: int,
    aspect: float = 0.55,
    rim: int = 1,
) -> Text:
    """
    Circular eye with optical compensation where:
      - Sclera (white) stays visible.
      - Iris can move to the edge at extreme LR/UD.
      - A limbal ring (dark ring) is drawn along the IRIS boundary (not the sclera edge).
      - Eyelids occlude along rounded arcs in compensated space.
    """
    # pylint: disable=too-many-locals, too-many-branches, too-many-statements

    width = max(32, width)
    height = max(12, height)
    txt = Text(no_wrap=True)

    cx, cy = width // 2, height // 2

    # ---- Fixed circle with vertical optical compensation ----
    aspect = max(0.01, float(aspect))
    half_w = (width - 2) // 2
    half_h = (height - 2) // 2
    radius = max(6, min(half_w, int(half_h * aspect)))  # radius in compensated metric
    r2 = radius * radius

    # Eyelid aperture (0..90) -> [0..1]
    aperture = max(0.0, min(1.0, sep / 90.0))

    # ---- Base sizes from radius ----
    iris_base = max(2, int(radius * 0.50))
    pupil_base = max(1, int(iris_base * 0.38))

    # Perspective foreshortening
    s_h = 0.35 + 0.65 * cos(abs(lr) * pi / 180.0)
    s_v = 0.35 + 0.65 * cos(abs(ud) * pi / 180.0)

    # Iris/pupil ellipse semi-axes (x in pixels, y in compensated units)
    iris_rx = max(1, int(iris_base * s_h))
    iris_ry = max(1, int(iris_base * s_v))
    pupil_rx = max(1, int(pupil_base * s_h))
    pupil_ry = max(1, int(pupil_base * s_v))

    # ----- Displace iris so it can reach the edge at extremes -----
    ux_raw = lr / 90.0  # LR>0 (left) moves center to screen-left
    uy_raw = ud / 90.0  # UD>0 (up) moves center up (compensated)
    magnitude = sqrt(ux_raw * ux_raw + uy_raw * uy_raw)
    if magnitude > 0:
        ux = ux_raw / magnitude
        uy = uy_raw / magnitude
        # Support function of ellipse in direction (ux,uy) in compensated space:
        h_ellipse = sqrt((iris_rx * ux) ** 2 + (iris_ry * uy) ** 2)
        d_max = max(0.0, radius - h_ellipse)  # tangent to circle at extremes
        dx_c = (min(1.0, magnitude) * d_max) * ux
        dy_c = (min(1.0, magnitude) * d_max) * uy
    else:
        dx_c = dy_c = 0.0

    iris_cx = cx + int(round(dx_c))
    iris_cy = cy - int(round(dy_c / aspect))  # convert compensated back to pixels

    # Veins stick to eyeball texture (match iris displacement)
    tshift_x = int(round(dx_c))
    tshift_y = -int(round(dy_c / aspect))

    # Lighting vector (near/far)
    d_lr = max(-1.0, min(1.0, -lr / 90.0))
    d_ud = max(-1.0, min(1.0, ud / 90.0))

    # Precompute normalized rim thickness (as fraction of iris radius)
    mean_axis = max(1.0, (iris_rx + iris_ry) / 2.0)
    rim_norm = max(0.0, float(rim)) / mean_axis  # 0 disables

    for y in range(height):
        row = Text(no_wrap=True)
        ey = y - cy
        eyc = ey * aspect

        # Quick skip: completely closed or outside circle vertically
        if aperture <= 0.0 or abs(eyc) > radius:
            txt.append(" " * width)
            if y != height - 1:
                txt.append("\n")
            continue

        for x in range(width):
            ex = x - cx
            ey = y - cy
            eyc = ey * aspect

            # Circle boundary
            if (ex * ex + eyc * eyc) > r2:
                row.append(" ")
                continue

            # Rounded eyelids per-x in compensated space
            inner = r2 - ex * ex
            if inner <= 0:
                row.append(" ")
                continue
            y_cap_c = aperture * sqrt(inner)
            if abs(eyc) > y_cap_c:
                row.append(" ")
                continue

            # Iris/pupil test (ellipse in compensated space)
            dx_i = x - iris_cx
            dy_i_c = (y - iris_cy) * aspect

            # Pupil first
            up = dx_i / max(1, pupil_rx)
            vp = dy_i_c / max(1, pupil_ry)
            if (up * up + vp * vp) <= 1.0:
                row.append(" ", style=S_PUPIL)
                continue

            # Iris
            u = dx_i / max(1, iris_rx)
            v = dy_i_c / max(1, iris_ry)
            r2_iris = u * u + v * v
            if r2_iris <= 1.0:
                # --- Limbal ring near IRIS boundary ---
                if rim_norm > 0.0:
                    q = sqrt(max(0.0, r2_iris))  # 0..1 inside ellipse
                    if q >= (1.0 - rim_norm):
                        row.append(" ", style=S_IRIS_DARK)
                        continue

                # Iris shading: near/far lighting
                dot = (u * d_lr) + (v * (-d_ud))
                if dot > 0.25:
                    style = S_IRIS_LIGHT
                elif dot < -0.25:
                    style = S_IRIS_DARK
                else:
                    style = S_IRIS
                row.append(" ", style=style)
            else:
                # Sclera with moving veins
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
    """Controls panel."""
    table = Table.grid(padding=(0, 1))
    table.add_row("[bold]Controls[/bold]")
    table.add_row("←/h: LR-", "→/l: LR+")
    table.add_row("↑/k: UD+", "↓/j: UD−")
    table.add_row("w: SEP +", "s: SEP −")
    table.add_row("+ / = : increase step size")
    table.add_row("- / _ : decrease step size")
    table.add_row("b: Blink (one-shot)", "r: Reset")
    table.add_row("[dim]q: Quit[/dim]")
    return Panel(table, title="Controls", border_style="magenta")


def state_panel(
    lr: int,
    ud: int,
    sep: int,
    limits: dict,
    rate_hz: float,
    keyexpr: str,
    endpoints: Optional[List[str]],
    blinking: bool,
    steps: Optional[dict] = None,
) -> Panel:
    """State panel showing bars, blink flag, rates, endpoints and step sizes."""

    def progress_bar(val: int, lo: int, hi: int, width: int = 20) -> str:
        span = hi - lo
        ratio = 0.0 if span == 0 else (val - lo) / span
        ratio = max(0.0, min(1.0, ratio))
        filled = int(ratio * width)
        s = "[" + "#" * filled + "-" * (width - filled) + "]"
        return f"{s} {val:>4}"

    eps = ", ".join(endpoints) if endpoints else "(discovery)"
    table = Table.grid(padding=(0, 1))
    table.add_row(f"LR  {progress_bar(lr, limits['lr'][0],  limits['lr'][1])}")
    table.add_row(f"UD  {progress_bar(ud, limits['ud'][0],  limits['ud'][1])}")
    table.add_row(f"SEP {progress_bar(sep, limits['sep'][0], limits['sep'][1])}")
    table.add_row(f"[dim]Blink:[/dim] {'[bold green]YES[/bold green]' if blinking else '[dim]no[/dim]'}")
    if steps:
        table.add_row(f"[yellow]Step size:[/yellow] LR={steps['lr']}  UD={steps['ud']}  SEP={steps['sep']}")
    table.add_row(f"[dim]Rate:[/dim] {rate_hz:.1f} Hz")
    table.add_row(f"[dim]Keyexpr:[/dim] {keyexpr}")
    table.add_row(f"[dim]Endpoints:[/dim] {eps}")
    return Panel(table, title="State", border_style="cyan")


# ------------------ Zenoh wrapper -------------
class ZPub:
    """Tiny zenoh publisher wrapper."""

    def __init__(self, keyexpr: str, endpoints: Optional[List[str]]) -> None:
        self.keyexpr = keyexpr
        self.endpoints = endpoints or []
        self.session = None
        self.pub = None

    def open(self) -> None:
        """Open zenoh session and declare publisher."""
        conf = zenoh.Config()  # type: ignore[attr-defined]  # pylint: disable=no-member
        if self.endpoints:
            conf.insert_json5("connect/endpoints", json.dumps(self.endpoints))  # type: ignore[attr-defined]
        self.session = zenoh.open(conf)  # type: ignore[attr-defined]  # pylint: disable=no-member
        self.pub = self.session.declare_publisher(self.keyexpr)  # type: ignore[attr-defined]

    def close(self) -> None:
        """Close publisher and session."""
        try:
            if self.pub:
                self.pub.close()
        finally:
            if self.session:
                self.session.close()

    def put(self, lr: int, ud: int, sep: int, blink: bool = False) -> None:
        """Publish one EyeCommand payload."""
        if self.pub:
            self.pub.put(build_eye_cmd(lr, ud, sep, blink))  # type: ignore[attr-defined]


# ------------------ Main loop -----------------
def main() -> None:
    """Entry point."""
    args = parse_args()
    console = Console()
    reader = KeyReader()

    # State
    lr = clamp(args.lr0, args.lr_min, args.lr_max)
    ud = clamp(args.ud0, args.ud_min, args.ud_max)
    sep = clamp(args.sep0, args.sep_min, args.sep_max)

    # Step increments and limits
    steps = {
        "lr": clamp(args.step_lr, args.step_min, args.step_max),
        "ud": clamp(args.step_ud, args.step_min, args.step_max),
        "sep": clamp(args.step_sep, args.step_min, args.step_max),
    }

    # Blink animation state (UI only)
    blink_ms = max(80, int(args.blink_ms))
    blink_active = False
    blink_t0 = 0.0

    limits = {
        "lr": (args.lr_min, args.lr_max),
        "ud": (args.ud_min, args.ud_max),
        "sep": (args.sep_min, args.sep_max),
    }

    pub = ZPub(args.keyexpr, args.endpoint)
    try:
        pub.open()
    except Exception as exc:  # pylint: disable=broad-except
        console.print(f"[red]Zenoh open failed:[/red] {exc}")
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
                    if key == "BLINK":
                        pub.put(lr, ud, sep, blink=True)
                        blink_active = True
                        blink_t0 = now
                        next_pub = now + interval
                        continue
                    if key == "RESET":
                        lr, ud, sep = 0, 0, limits["sep"][1]
                        continue
                    if key == "LEFT":
                        lr = clamp(lr - steps["lr"], *limits["lr"])
                        continue
                    if key == "RIGHT":
                        lr = clamp(lr + steps["lr"], *limits["lr"])
                        continue
                    if key == "UP":
                        ud = clamp(ud + steps["ud"], *limits["ud"])
                        continue
                    if key == "DOWN":
                        ud = clamp(ud - steps["ud"], *limits["ud"])
                        continue
                    if key == "SEP_UP":
                        sep = clamp(sep + steps["sep"], *limits["sep"])
                        continue
                    if key == "SEP_DOWN":
                        sep = clamp(sep - steps["sep"], *limits["sep"])
                        continue
                    if key == "STEP_INC":
                        steps["lr"] = clamp(steps["lr"] + 1, args.step_min, args.step_max)
                        steps["ud"] = clamp(steps["ud"] + 1, args.step_min, args.step_max)
                        steps["sep"] = clamp(steps["sep"] + 1, args.step_min, args.step_max)
                        continue
                    if key == "STEP_DEC":
                        steps["lr"] = clamp(steps["lr"] - 1, args.step_min, args.step_max)
                        steps["ud"] = clamp(steps["ud"] - 1, args.step_min, args.step_max)
                        steps["sep"] = clamp(steps["sep"] - 1, args.step_min, args.step_max)
                        continue

                # --- Periodic publish
                if now >= next_pub:
                    pub.put(lr, ud, sep, blink=False)
                    next_pub = now + interval

                # --- Blink UI animation
                vis_sep = sep
                if blink_active:
                    t_ms = (now - blink_t0) * 1000.0
                    if t_ms >= blink_ms:
                        blink_active = False
                    else:
                        phase = t_ms / blink_ms
                        frac = 1.0 - (phase / 0.5) if phase < 0.5 else (phase - 0.5) / 0.5
                        blink_sep = int(args.sep_max * frac)
                        vis_sep = min(sep, blink_sep)

                # --- Layout & render (Eye below both columns)
                term_w = console.size.width
                term_h = console.size.height

                controls = controls_panel()
                state = state_panel(
                    lr,
                    ud,
                    sep,
                    limits,
                    args.rate_hz,
                    args.keyexpr,
                    args.endpoint,
                    blinking=blink_active,
                    steps=steps,
                )

                # Eye spans the full width on the bottom row.
                eye_w = max(30, term_w - 4)
                eye_h = max(12, term_h - 12)
                eye_panel = None
                if not args.no_eye:
                    eye_panel = Panel(
                        render_eye(eye_w, eye_h, lr, ud, vis_sep, args.aspect, args.rim),
                        title="Eye",
                        border_style="green",
                    )

                layout = Layout()
                layout.split(Layout(name="top", ratio=1), Layout(name="eye", ratio=2))
                layout["top"].split_row(
                    Layout(controls, name="controls", ratio=1),
                    Layout(state, name="state", ratio=1),
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
        except Exception:  # pylint: disable=broad-except
            pass


if __name__ == "__main__":
    main()
