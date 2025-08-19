#!/usr/bin/env python3
# Electric Eyes — Textual TUI with YAML config, Zenoh + FlatBuffers publisher
# LR/UD simultaneous control (arrows & Vim), SEP on +/-; blink one-shot.

import json
from pathlib import Path
from dataclasses import dataclass, asdict, field, fields
from typing import List, Optional

import yaml
import flatbuffers
import zenoh
from zenoh import Config

from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Horizontal, Vertical, Grid
from textual.widgets import Header, Footer, Static, Button, Label, Input
from textual.screen import ModalScreen
from textual.reactive import reactive

# ---- FlatBuffers (generated with `flatc --python`) ----
from electric_eyes import EyeCommand  # EyeCommand.EyeCommandStart/Add*/End

# -----------------------------------------------------------------------------
# Config (YAML)
# -----------------------------------------------------------------------------

DEFAULT_CFG_PATH = Path("~/.electric_eyes.yml").expanduser()

@dataclass
class Limits:
    lr_min: int = -90
    lr_max: int = 90
    ud_min: int = -90
    ud_max: int = 90
    sep_min: int = 0
    sep_max: int = 90

@dataclass
class Steps:
    step_lr: int = 2
    step_ud: int = 2
    step_sep: int = 2

@dataclass
class Initial:
    lr0: int = 0
    ud0: int = 0
    sep0: int = 90

@dataclass
class Net:
    keyexpr: str = "robot/eye_command"
    endpoints: List[str] = field(default_factory=list)  # e.g., ["tcp/192.168.68.136:7447"]
    rate_hz: float = 10.0

@dataclass
class AppConfig:
    limits: Limits = field(default_factory=Limits)
    steps: Steps = field(default_factory=Steps)
    initial: Initial = field(default_factory=Initial)
    net: Net = field(default_factory=Net)

def clamp(v: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, v))

def _merge_dataclass(dc_cls, src: dict):
    """
    Build an instance of dc_cls, taking values from src when present,
    otherwise falling back to dc_cls() defaults.
    """
    base = dc_cls()  # default instance
    kwargs = {}
    if not isinstance(src, dict):
        # Nothing to merge; return defaults
        return base
    for f in fields(dc_cls):
        kwargs[f.name] = src.get(f.name, getattr(base, f.name))
    return dc_cls(**kwargs)

def load_config(path: Path) -> AppConfig:
    if not path.exists():
        cfg = AppConfig()
        save_config(path, cfg)
        return cfg
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    limits  = _merge_dataclass(Limits,  data.get("limits", {}))
    steps   = _merge_dataclass(Steps,   data.get("steps", {}))
    initial = _merge_dataclass(Initial, data.get("initial", {}))
    net     = _merge_dataclass(Net,     data.get("net", {}))

    return AppConfig(limits=limits, steps=steps, initial=initial, net=net)

def save_config(path: Path, cfg: AppConfig) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(asdict(cfg), f, sort_keys=False)

# -----------------------------------------------------------------------------
# Zenoh + FlatBuffers
# -----------------------------------------------------------------------------

def build_eye_cmd(lr: int, ud: int, sep: int, blink: bool) -> bytes:
    b = flatbuffers.Builder(0)
    EyeCommand.EyeCommandStart(b)
    EyeCommand.EyeCommandAddLookLr(b, int(lr))
    EyeCommand.EyeCommandAddLookUd(b, int(ud))
    EyeCommand.EyeCommandAddEyeSep(b, int(sep))
    EyeCommand.EyeCommandAddBlink(b, bool(blink))
    eye = EyeCommand.EyeCommandEnd(b)
    b.Finish(eye)
    return b.Output()

class ZenohPub:
    def __init__(self, cfg: Net):
        self.cfg = cfg
        self.session = None
        self.pub = None
    def open(self):
        conf = Config()
        if self.cfg.endpoints:
            conf.insert_json5("connect/endpoints", json.dumps(self.cfg.endpoints))
        self.session = zenoh.open(conf)
        self.pub = self.session.declare_publisher(self.cfg.keyexpr)
    def close(self):
        try:
            if self.pub: self.pub.close()
        finally:
            if self.session: self.session.close()
    def put(self, lr: int, ud: int, sep: int, blink: bool = False):
        if self.pub:
            self.pub.put(build_eye_cmd(lr, ud, sep, blink))

# -----------------------------------------------------------------------------
# Widgets
# -----------------------------------------------------------------------------

class EyeView(Static):
    """ASCII eye: positive LR is LEFT, positive UD is UP; SEP sets eyelid aperture."""
    lr: int = reactive(0)
    ud: int = reactive(0)
    sep: int = reactive(90)

    def render(self) -> str:
        width = max(24, self.size.width - 2)
        height = max(9, self.size.height - 2)
        w = width - 2
        h = height - 2
        rows = [[" "] * width for _ in range(height)]
        for x in range(width):
            rows[0][x] = rows[-1][x] = "─"
        for y in range(height):
            rows[y][0] = rows[y][-1] = "│"
        rows[0][0] = rows[0][-1] = rows[-1][0] = rows[-1][-1] = "┼"

        cx, cy = width // 2, height // 2
        rx, ry = max(6, w // 3), max(3, h // 3)

        def in_eye(x, y):
            return ((x - cx) ** 2) / (rx * rx) + ((y - cy) ** 2) / (ry * ry) <= 1.0

        aperture = max(0.0, min(1.0, self.sep / 90.0))
        lid_half = int(ry * (1.0 - aperture))
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                if in_eye(x, y):
                    rows[y][x] = "·"
        for x in range(1, width - 1):
            for off in range(lid_half):
                yt = cy - ry + off
                yb = cy + ry - off
                if 1 <= yt < height - 1: rows[yt][x] = "▔"
                if 1 <= yb < height - 1: rows[yb][x] = "▁"

        # Map signs: LR>0 = LEFT, UD>0 = UP
        px = cx - int((self.lr / 90.0) * rx * 0.6)   # NOTE the minus to flip LR
        py = cy - int((self.ud / 90.0) * ry * 0.6)
        py = max(cy - ry + lid_half + 1, min(cy + ry - lid_half - 1, py))
        px = max(cx - rx + 1, min(cx + rx - 1, px))
        if 1 <= py < height - 1 and 1 <= px < width - 1:
            rows[py][px] = "●"

        legend = f" LR:{self.lr:+3d}  UD:{self.ud:+3d}  SEP:{self.sep:3d}"
        for i, ch in enumerate(legend[:width-2]):
            rows[1][1 + i] = ch
        return "\n".join("".join(r) for r in rows)

class SettingsModal(ModalScreen[AppConfig]):
    def __init__(self, config: AppConfig, path: Path):
        super().__init__()
        self.config = config
        self.path = path
    def compose(self) -> ComposeResult:
        yield Static("Settings", classes="title")
        with Grid(classes="form"):
            yield Label("LR min"); self.lr_min = Input(str(self.config.limits.lr_min))
            yield Label("LR max"); self.lr_max = Input(str(self.config.limits.lr_max))
            yield Label("UD min"); self.ud_min = Input(str(self.config.limits.ud_min))
            yield Label("UD max"); self.ud_max = Input(str(self.config.limits.ud_max))
            yield Label("SEP min"); self.sep_min = Input(str(self.config.limits.sep_min))
            yield Label("SEP max"); self.sep_max = Input(str(self.config.limits.sep_max))
            yield Label("step LR"); self.step_lr = Input(str(self.config.steps.step_lr))
            yield Label("step UD"); self.step_ud = Input(str(self.config.steps.step_ud))
            yield Label("step SEP"); self.step_sep = Input(str(self.config.steps.step_sep))
            yield Label("LR0"); self.lr0 = Input(str(self.config.initial.lr0))
            yield Label("UD0"); self.ud0 = Input(str(self.config.initial.ud0))
            yield Label("SEP0"); self.sep0 = Input(str(self.config.initial.sep0))
            yield Label("Keyexpr"); self.keyexpr = Input(self.config.net.keyexpr)
            yield Label("Endpoints (comma sep)"); self.endpoints = Input(",".join(self.config.net.endpoints))
            yield Label("Rate Hz"); self.rate = Input(str(self.config.net.rate_hz))
        with Horizontal(classes="buttons"):
            yield Button("Save", id="save", variant="success")
            yield Button("Cancel", id="cancel", variant="warning")
    def on_button_pressed(self, ev: Button.Pressed) -> None:
        if ev.button.id == "cancel":
            self.dismiss(None); return
        try:
            cfg = AppConfig(
                limits=Limits(
                    lr_min=int(self.lr_min.value), lr_max=int(self.lr_max.value),
                    ud_min=int(self.ud_min.value), ud_max=int(self.ud_max.value),
                    sep_min=int(self.sep_min.value), sep_max=int(self.sep_max.value),
                ),
                steps=Steps(
                    step_lr=int(self.step_lr.value),
                    step_ud=int(self.step_ud.value),
                    step_sep=int(self.step_sep.value),
                ),
                initial=Initial(
                    lr0=int(self.lr0.value), ud0=int(self.ud0.value), sep0=int(self.sep0.value),
                ),
                net=Net(
                    keyexpr=self.keyexpr.value.strip() or "robot/eye_command",
                    endpoints=[e.strip() for e in self.endpoints.value.split(",") if e.strip()],
                    rate_hz=float(self.rate.value),
                ),
            )
        except Exception as e:
            self.app.push_screen(InfoModal(f"Invalid settings: {e}")); return
        save_config(self.path, cfg)
        self.dismiss(cfg)

class InfoModal(ModalScreen[None]):
    def __init__(self, message: str):
        super().__init__(); self.message = message
    def compose(self) -> ComposeResult:
        yield Static(self.message, classes="info"); yield Button("OK", id="ok", variant="primary")
    def on_button_pressed(self, ev: Button.Pressed) -> None:
        self.dismiss(None)

# -----------------------------------------------------------------------------
# Main App
# -----------------------------------------------------------------------------

class EyesApp(App):
    CSS = """
    Screen { layout: vertical; }
    .title { content-align: center middle; height: 3; border: round $accent; }
    .form { grid-size: 2; grid-gutter: 1 2; padding: 1 2; }
    .buttons { height: 3; align: center middle; padding: 1; content-align: center middle; }
    #topbar { height: 3; }
    #main { height: 1fr; }
    #controls, #state { width: 34; border: round $secondary; padding: 1; }
    #eye { border: round $accent; padding: 1; height: 1fr; }
    Button { width: 100%; margin: 0 0 1 0; }
    """

    BINDINGS = [
        Binding("q", "quit", "Quit"),
        Binding("b", "blink", "Blink"),
        Binding("r", "reset", "Reset"),
        # LR (left is positive, right is negative)
        Binding("left", "lr_inc", "LR+"),
        Binding("h", "lr_inc", show=False),
        Binding("right", "lr_dec", "LR-"),
        Binding("l", "lr_dec", show=False),
        # UD
        Binding("up", "ud_inc", "UD+"),
        Binding("k", "ud_inc", show=False),
        Binding("down", "ud_dec", "UD-"),
        Binding("j", "ud_dec", show=False),
        # SEP
        Binding("-", "sep_dec", "SEP-"),
        Binding("+", "sep_inc", show=False),
        Binding("=", "sep_inc", show=False),
        Binding("s", "settings", "Settings"),
    ]

    lr: int = reactive(0)
    ud: int = reactive(0)
    sep: int = reactive(90)
    blink_once: bool = reactive(False)

    def __init__(self, cfg_path: Path = DEFAULT_CFG_PATH):
        super().__init__()
        self.cfg_path = cfg_path
        self.cfg = load_config(cfg_path)
        L, I = self.cfg.limits, self.cfg.initial
        self.lr = clamp(I.lr0, L.lr_min, L.lr_max)
        self.ud = clamp(I.ud0, L.ud_min, L.ud_max)
        self.sep = clamp(I.sep0, L.sep_min, L.sep_max)
        self.pub = ZenohPub(self.cfg.net)
        self._pub_timer = None
        self._anim_timer = None

    def compose(self) -> ComposeResult:
        yield Header()
        with Horizontal(id="topbar"):
            yield Label("Electric Eyes — arrows/Vim to steer, +/- eyelid, b blink, r reset, s settings, q quit")
        with Horizontal(id="main"):
            with Vertical(id="controls"):
                yield Label("LR (left is +)")
                yield Button("LR +  (← / h)", id="lr_inc")
                yield Button("LR −  (→ / l)", id="lr_dec")
                yield Label("UD")
                yield Button("UD +  (↑ / k)", id="ud_inc")
                yield Button("UD −  (↓ / j)", id="ud_dec")
                yield Label("Eyelid SEP")
                yield Button("SEP −  (-)", id="sep_dec")
                yield Button("SEP +  (+/=)", id="sep_inc")
                yield Label("Actions")
                yield Button("Blink (b)", id="blink")
                yield Button("Reset (r)", id="reset")
                yield Button("Settings (s)", id="settings")
            self.eye = EyeView(id="eye")
            yield self.eye
            with Vertical(id="state"):
                self.lbl_vals = Label("")
                self.lbl_net  = Label("")
                yield Label("Current")
                yield self.lbl_vals
                yield Label("Network")
                yield self.lbl_net
        yield Footer()

    def on_mount(self) -> None:
        try:
            self.pub.open()
        except Exception as e:
            self.push_screen(InfoModal(f"Zenoh open failed: {e}"))
        self._reschedule_publish()
        self._anim_timer = self.set_interval(1/30, self._tick_animation)
        self._refresh_labels()
        self._sync_eye_widget()

    def on_unmount(self) -> None:
        try: self.pub.close()
        except Exception: pass

    # Rendering & publish
    def _tick_animation(self) -> None:
        self._sync_eye_widget()

    def _sync_eye_widget(self) -> None:
        self.eye.lr = self.lr
        self.eye.ud = self.ud
        self.eye.sep = self.sep

    def _refresh_labels(self) -> None:
        L, S, net = self.cfg.limits, self.cfg.steps, self.cfg.net
        self.lbl_vals.update(
            f"LR {self.lr:+3d}  [{L.lr_min},{L.lr_max}]  step {S.step_lr}\n"
            f"UD {self.ud:+3d}  [{L.ud_min},{L.ud_max}]  step {S.step_ud}\n"
            f"SEP {self.sep:3d}  [{L.sep_min},{L.sep_max}] step {S.step_sep}\n"
        )
        eps = ", ".join(net.endpoints) if net.endpoints else "(discovery)"
        self.lbl_net.update(f"Key: {net.keyexpr}\nEndpoints: {eps}\nRate: {net.rate_hz:.1f} Hz")

    def _reschedule_publish(self) -> None:
        if self._pub_timer: self._pub_timer.stop()
        hz = max(0.1, float(self.cfg.net.rate_hz))
        self._pub_timer = self.set_interval(1.0 / hz, self._tick_publish)

    def _tick_publish(self) -> None:
        blink = self.blink_once
        try:
            self.pub.put(self.lr, self.ud, self.sep, blink=blink)
        except Exception as e:
            self.push_screen(InfoModal(f"Publish failed: {e}"))
        finally:
            self.blink_once = False

    # Actions (keys)
    def action_quit(self) -> None: self.exit()
    def action_blink(self) -> None: self.blink_once = True
    def action_reset(self) -> None:
        L = self.cfg.limits
        self.lr = clamp(0, L.lr_min, L.lr_max)
        self.ud = clamp(0, L.ud_min, L.ud_max)
        self.sep = clamp(L.sep_max, L.sep_min, L.sep_max)
        self._refresh_labels(); self._sync_eye_widget()

    # LR/UD step changes — note LR+ is LEFT
    def action_lr_inc(self) -> None:
        L, S = self.cfg.limits, self.cfg.steps
        self.lr = clamp(self.lr + S.step_lr, L.lr_min, L.lr_max)
        self._refresh_labels(); self._sync_eye_widget()

    def action_lr_dec(self) -> None:
        L, S = self.cfg.limits, self.cfg.steps
        self.lr = clamp(self.lr - S.step_lr, L.lr_min, L.lr_max)
        self._refresh_labels(); self._sync_eye_widget()

    def action_ud_inc(self) -> None:
        L, S = self.cfg.limits, self.cfg.steps
        self.ud = clamp(self.ud + S.step_ud, L.ud_min, L.ud_max)
        self._refresh_labels(); self._sync_eye_widget()

    def action_ud_dec(self) -> None:
        L, S = self.cfg.limits, self.cfg.steps
        self.ud = clamp(self.ud - S.step_ud, L.ud_min, L.ud_max)
        self._refresh_labels(); self._sync_eye_widget()

    # SEP on +/- keys
    def action_sep_inc(self) -> None:
        L, S = self.cfg.limits, self.cfg.steps
        self.sep = clamp(self.sep + S.step_sep, L.sep_min, L.sep_max)
        self._refresh_labels(); self._sync_eye_widget()

    def action_sep_dec(self) -> None:
        L, S = self.cfg.limits, self.cfg.steps
        self.sep = clamp(self.sep - S.step_sep, L.sep_min, L.sep_max)
        self._refresh_labels(); self._sync_eye_widget()

    # Mouse buttons
    def on_button_pressed(self, ev: Button.Pressed) -> None:
        bid = ev.button.id
        if   bid == "lr_inc":  self.action_lr_inc()
        elif bid == "lr_dec":  self.action_lr_dec()
        elif bid == "ud_inc":  self.action_ud_inc()
        elif bid == "ud_dec":  self.action_ud_dec()
        elif bid == "sep_inc": self.action_sep_inc()
        elif bid == "sep_dec": self.action_sep_dec()
        elif bid == "blink":   self.action_blink()
        elif bid == "reset":   self.action_reset()
        elif bid == "settings": self.action_settings()

    def action_settings(self) -> None:
        self.push_screen(SettingsModal(self.cfg, DEFAULT_CFG_PATH), self._on_settings_applied)

    def _on_settings_applied(self, cfg: Optional[AppConfig]) -> None:
        if not cfg: return
        self.cfg = cfg
        L = self.cfg.limits
        self.lr = clamp(self.lr, L.lr_min, L.lr_max)
        self.ud = clamp(self.ud, L.ud_min, L.ud_max)
        self.sep = clamp(self.sep, L.sep_min, L.sep_max)
        self._refresh_labels(); self._sync_eye_widget()
        try: self.pub.close()
        except Exception: pass
        self.pub = ZenohPub(self.cfg.net)
        try: self.pub.open()
        except Exception as e: self.push_screen(InfoModal(f"Zenoh reopen failed: {e}"))
        self._reschedule_publish()

if __name__ == "__main__":
    EyesApp().run()
