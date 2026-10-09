#!/usr/bin/env python3
"""
Package Sorting System - PC Dashboard (conveyor view)
-----------------------------------------------------
Reads the UART text already printed by the STM32 firmware (115200 8N1) and
draws it as a live conveyor animation + dashboard.  No firmware change needed.

    pip install pyserial
    python sorter_gui.py

Press "Demo" to try it without a board.
"""
import datetime
import math
import queue
import random
import re
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError:          # GUI still opens (Demo mode works)
    serial = None
    list_ports = None

# ----------------------------------------------------------------------------
# Model : parses firmware text lines -> state (no tkinter, easy to test)
# ----------------------------------------------------------------------------
SIZES = ("S", "M", "L")
ACTIVE = ("RUNNING", "WAIT_LDR", "SETTLE", "EVALUATE", "ROUTE")
STAGES = ("IR", "LDR", "Settle", "Evaluate", "Route")


ROUTE_CLEAR_S = 3.7      # 0.4 s routing + 3.0 s clear timeout + margin (firmware constants)


class Model:
    def __init__(self):
        # package counters: bumped once per IR event and NEVER reset, so the conveyor
        # can tell "a new package arrived" from "the same package, another line came in"
        self.pkg_seq = 0
        self.ldr_seq = 0
        self.size_seq = 0
        self.dec_seq = 0
        self.reset_all()

    def reset_all(self):
        self.state = "IDLE"
        self.prev_state = "RUNNING"
        self.report_seq = 0           # incremented once per completed report
        self.state_since = time.monotonic()
        self.remaining_at = time.monotonic()   # when self.remaining was last valid
        self.stage = 0                      # number of completed pipeline steps
        self.target = {k: 0 for k in SIZES}
        self.time_limit = 0
        self.remaining = None
        self.accepted = {k: 0 for k in SIZES}
        self.rejected = {k: 0 for k in SIZES}
        self.errors = {"lost": 0, "unexpected": 0, "stuck": 0}
        self.last_size = None
        self.last_decision = None
        self.history = []                   # [(size, "ACCEPT"/"REJECT")]
        self.fault_msg = ""
        self.report = {}
        self.in_report = False

    def _clear_run(self):
        self.accepted = {k: 0 for k in SIZES}
        self.rejected = {k: 0 for k in SIZES}
        self.errors = {"lost": 0, "unexpected": 0, "stuck": 0}
        self.history = []
        self.report = {}
        self.remaining = None
        self.stage = 0
        self.last_size = None
        self.last_decision = None
        self.fault_msg = ""
        self.fault_kind = None        # "lost" | "unexpected" | "stuck"

    def _set(self, st):
        if st == "PAUSED" and self.state != "PAUSED":
            self.prev_state = self.state
        now = time.monotonic()
        if self.remaining is not None and self.state in ACTIVE:
            # bank the time spent in the state we are leaving, so the countdown
            # keeps running across IR -> LDR -> EVALUATE -> ROUTE transitions
            self.remaining = max(0.0, self.remaining - (now - self.remaining_at))
        if st != self.state:
            self.state_since = now                # drives the countdown pop-ups
        self.remaining_at = now
        self.state = st
        if st not in ACTIVE:
            self.stage = 0

    def remaining_now(self, now=None):
        """Seconds left, ticked locally between the board's 1 Hz updates.

        The firmware only prints the remaining time while it is in RUNNING, and
        its clock stops while paused or in fault, so ours only runs in ACTIVE.
        """
        if self.remaining is None:
            return None
        if self.state not in ACTIVE:
            return max(0.0, self.remaining)
        now = time.monotonic() if now is None else now
        return max(0.0, self.remaining - (now - self.remaining_at))

    def infer_idle(self, now=None):
        """The board prints nothing when a package clears the LDR. After the decision it
        is back in RUNNING within ROUTE (0.4 s) + clear timeout (3 s); a fault would have
        printed a line by then. So a silent ROUTE older than that means RUNNING again."""
        now = time.monotonic() if now is None else now
        if self.state == "ROUTE" and now - self.state_since >= ROUTE_CLEAR_S:
            self._set("RUNNING")
            self.stage = 0
            return True
        return False

    # returns a log tag for colouring
    def feed(self, ln):
        s = ln.strip()
        if not s:
            return None

        if "SIMULATION READY" in s:
            self._set("IDLE")
            return "sys"

        if "[CONFIG] Loaded" in s:
            self._clear_run()
            self._set("READY")
            return "ok"
        m = re.match(r"S:\s*(\d+)\s*\|\s*M:\s*(\d+)\s*\|\s*L:\s*(\d+)", s)
        if m:
            self.target = dict(zip(SIZES, map(int, m.groups())))
            return "ok"
        m = re.match(r"Max Time:\s*(\d+)\s*s", s)
        if m:
            self.time_limit = int(m.group(1))
            return "ok"

        if "Sorting Started" in s:
            self._clear_run()
            self._set("RUNNING")
            return "ok"
        m = re.search(r"\[TIME\] Remaining:\s*(\d+)", s)
        if m:
            self.remaining = float(m.group(1))
            self.remaining_at = time.monotonic()
            if self.state in ACTIVE and self.state != "RUNNING":
                self._set("RUNNING")           # the board only prints this line in RUNNING
                self.stage = 0
            return "time"
        if "Operating time expired" in s:
            self.remaining = 0.0
            self.remaining_at = time.monotonic()
            self._set("REPORT")
            return "warn"

        if "[SENSOR] IR" in s:
            self.pkg_seq += 1
            self._set("WAIT_LDR")
            self.stage = 1
            return "sensor"
        if "[SENSOR] LDR" in s:
            self.ldr_seq = self.pkg_seq
            self._set("SETTLE")
            self.stage = 2
            return "sensor"

        m = re.search(r"Size Measured:\s*\[\s*([SML])\s*\]", s)
        if m:
            self.last_size = m.group(1)
            self.size_seq = self.pkg_seq
            self._set("EVALUATE")
            self.stage = 3
            return "eval"
        m = re.search(r"Decision:\s*(ACCEPT|REJECT)", s)
        if m:
            self.last_decision = m.group(1)
            self.dec_seq = self.pkg_seq
            self._set("ROUTE")
            self.stage = 4
            if self.last_size:
                self.history.append((self.last_size, self.last_decision))
                self.history = self.history[-24:]
            return "accept" if m.group(1) == "ACCEPT" else "reject"
        m = re.match(r"Quota:\s*(\d+)\s*\|\s*(Accepted|Rejected):\s*(\d+)", s)
        if m and self.last_size:
            d = self.accepted if m.group(2) == "Accepted" else self.rejected
            d[self.last_size] = int(m.group(3))
            return "accept" if m.group(2) == "Accepted" else "reject"

        if "[FAULT]" in s:
            self.fault_msg = s.replace("[FAULT]", "").strip()
            if "missed LDR" in s:
                self.errors["lost"] += 1
                self.fault_kind = "lost"
            elif "Unexpected LDR" in s:
                self.errors["unexpected"] += 1
                self.fault_kind = "unexpected"
            elif "STUCK" in s.upper():
                self.errors["stuck"] += 1
                self.fault_kind = "stuck"
            self._set("FAULT")
            return "fault"
        if "[EMERGENCY]" in s:
            self._set("EMERGENCY")
            return "emerg"

        if "System PAUSED" in s:
            self._set("PAUSED")
            return "warn"
        if "System RESUMED" in s:
            self._set(self.prev_state if self.prev_state in ACTIVE else "RUNNING")
            return "ok"
        if "Resuming from fault" in s:
            self._set("RUNNING")
            return "ok"
        if ("timeout reached" in s or "target package counts reached" in s
                or "RESET pressed" in s):
            self._set("REPORT")
            return "warn"
        if "System Reset -> IDLE" in s:
            self._set("IDLE")
            return "sys"

        # ---- summary report block ----
        if "SUMMARY REPORT" in s:
            self.in_report = True
            self.report = {}
            self._set("REPORT")
            return "report"
        m = re.match(r"ACCEPTED\s*->\s*S:\s*(\d+)/(\d+)\s*\|\s*M:\s*(\d+)/(\d+)\s*\|\s*L:\s*(\d+)/(\d+)", s)
        if m:
            v = list(map(int, m.groups()))
            for i, k in enumerate(SIZES):
                self.accepted[k], self.target[k] = v[2 * i], v[2 * i + 1]
            return "report"
        m = re.match(r"REJECTED\s*->\s*S:\s*(\d+)\s*\|\s*M:\s*(\d+)\s*\|\s*L:\s*(\d+)", s)
        if m:
            self.rejected = dict(zip(SIZES, map(int, m.groups())))
            return "report"
        m = re.match(r"ERRORS\s*->\s*Object Lost:\s*(\d+)\s*\|\s*Unexpected Object:\s*(\d+)\s*\|\s*Object Stuck:\s*(\d+)", s)
        if m:
            self.errors = dict(zip(("lost", "unexpected", "stuck"), map(int, m.groups())))
            return "report"
        for key, label in (("input", r"TOTAL INPUT"), ("total_target", r"TOTAL TARGET"),
                           ("elapsed", r"ELAPSED TIME"), ("completion", r"TARGET COMPLETION"),
                           ("efficiency", r"SYSTEM EFFICIENCY"), ("reject_rate", r"REJECT RATE"),
                           ("loss", r"SYSTEM LOSS RATE")):
            m = re.match(label + r"\s*->\s*([\d.]+)", s)
            if m:
                self.report[key] = m.group(1)
                if key == "loss":
                    self.in_report = False
                    self.report_seq += 1          # a complete report has arrived
                return "report"
        if self.in_report and s.startswith(("=", "-")):
            return "report"
        return "dim" if s.startswith("=") else "sys" if s.startswith("[SYS]") else None


# ----------------------------------------------------------------------------
# Demo script (mimics firmware text so the GUI can be tried without a board)
# ----------------------------------------------------------------------------
def demo_script(S, M, L, T):
    tgt = {"S": S, "M": M, "L": L}
    acc = {k: 0 for k in SIZES}
    rej = {k: 0 for k in SIZES}
    err_lost = 0
    elapsed = 0
    bar = "=" * 40
    yield 0.2, bar
    yield 0.0, "PACKAGE SORTING SYSTEM SIMULATION READY"
    yield 0.0, bar
    yield 0.6, "[CONFIG] Loaded"
    yield 0.0, f"S: {S} | M: {M} | L: {L}"
    yield 0.0, f"Max Time: {T} s"
    yield 0.0, "[SYS] Press START button (PB4) to begin"
    yield 1.5, "[SYS] Sorting Started!"
    count = 0
    while count < 40:
        done = all(acc[k] >= tgt[k] for k in SIZES)
        if done or (T > 0 and elapsed >= T):
            break
        count += 1
        if T > 0:
            yield 0.4, f"[TIME] Remaining: {T - elapsed} s"
        yield 0.8, "[SENSOR] IR : Entry detected. Waiting for LDR sensor."
        if count == 3 and err_lost == 0:
            err_lost = 1
            yield 1.2, "[FAULT] IR triggered, but package missed LDR sensor!"
            yield 0.0, "[SYS] System paused. Press PAUSE (PB5) to resume or RESET (PB3)."
            yield 2.0, "[SYS] Resuming from fault. Sorting reactivated."
            elapsed += 4
            continue
        yield 0.7, "[SENSOR] LDR : Package arrived at sorting point."
        size = random.choice(SIZES)
        yield 0.5, f"[EVAL] Size Measured: [ {size} ]"
        if acc[size] < tgt[size]:
            yield 0.2, "[EVAL] Decision: ACCEPT -> Passing through (Servo Normal)."
            yield 0.0, f"Quota: {tgt[size]} | Accepted: {acc[size] + 1}"
            acc[size] += 1
        else:
            yield 0.2, "[EVAL] Decision: REJECT -> Actuating Servo to discard."
            yield 0.0, f"Quota: {tgt[size]} | Rejected: {rej[size] + 1}"
            rej[size] += 1
        elapsed += 3
        yield 0.9, ""
    if all(acc[k] >= tgt[k] for k in SIZES):
        yield 0.3, "[SYS] All target package counts reached successfully!"
    else:
        yield 0.3, "[TIME] Operating time expired!"
    ta, tr = sum(acc.values()), sum(rej.values())
    tt, te = sum(tgt.values()), err_lost
    ti = ta + tr + te

    def pct(a, b):
        sc = (a * 10000) // b if b else 0
        return f"{sc // 100}.{sc % 100:02d} %"
    comp = (ta * 10000) // tt if tt else 0
    loss = max(0, 10000 - comp)
    yield 0.4, bar
    yield 0.0, "SUMMARY REPORT"
    yield 0.0, bar
    yield 0.0, "ACCEPTED -> S: %d/%d | M: %d/%d | L: %d/%d" % (acc["S"], S, acc["M"], M, acc["L"], L)
    yield 0.0, "REJECTED -> S: %d | M: %d | L: %d" % (rej["S"], rej["M"], rej["L"])
    yield 0.0, "ERRORS   -> Object Lost: %d | Unexpected Object: 0 | Object Stuck: 0 | Total Errors: %d" % (err_lost, te)
    yield 0.0, "-" * 40
    yield 0.0, f"TOTAL INPUT  -> {ti}"
    yield 0.0, f"TOTAL TARGET -> {tt}"
    yield 0.0, f"ELAPSED TIME -> {elapsed} s"
    yield 0.0, "-" * 40
    yield 0.0, f"TARGET COMPLETION -> {pct(ta, tt)}"
    yield 0.0, f"SYSTEM EFFICIENCY -> {pct(ta, ti)}"
    yield 0.0, f"REJECT RATE       -> {pct(tr, ti)}"
    yield 0.0, f"SYSTEM LOSS RATE  -> {loss // 100}.{loss % 100:02d} %"
    yield 0.0, bar
    yield 1.0, "[SYS] System Reset -> IDLE. Waiting for configuration."


# ----------------------------------------------------------------------------
# Theme
# ----------------------------------------------------------------------------
BG, CARD, BORDER = "#f5f6f8", "#ffffff", "#e9ebef"
TXT, MUTED, FAINT = "#15191f", "#6b7280", "#a3abb6"
BELT, BELT_EDGE, STRIPE = "#dcdfe5", "#c7ccd4", "#f0f2f5"
SIZE_COLOR = {"S": "#f87171", "M": "#fbbf24", "L": "#34d399"}
SIZE_SOFT = {"S": "#fee2e2", "M": "#fef3c7", "L": "#d1fae5"}
IR_C, IR_SOFT = "#38bdf8", "#e0f2fe"
LDR_C, LDR_SOFT = "#a78bfa", "#ede9fe"
ACCENT, ACCENT_SOFT = "#6366f1", "#eef2ff"
REJ, REJ_SOFT = "#fb7185", "#ffe4e6"
STATE_STYLE = {
    "IDLE":      ("#7c8899", "IDLE", "Waiting for configuration"),
    "READY":     ("#4f6ef7", "READY", "Press START (PB4) on the board"),
    "RUNNING":   ("#22c55e", "RUNNING", "Waiting for a package"),
    "WAIT_LDR":  ("#14b8a6", "WAIT LDR", "Package on the belt"),
    "SETTLE":    ("#14b8a6", "SETTLING", "Package at the sorting point"),
    "EVALUATE":  ("#14b8a6", "EVALUATING", "Measuring size"),
    "ROUTE":     ("#14b8a6", "ROUTING", "Servo moving the package"),
    "PAUSED":    ("#f59e0b", "PAUSED", "Press PAUSE (PB5) to resume"),
    "FAULT":     ("#ef4444", "FAULT", ""),
    "EMERGENCY": ("#991b1b", "EMERGENCY", "All operations halted"),
    "REPORT":    ("#8b5cf6", "REPORT", "Summary report"),
}
LOG_TAGS = {
    "sys": MUTED, "dim": FAINT, "time": FAINT, "ok": "#15803d", "sensor": "#0e7490",
    "eval": "#4338ca", "accept": "#16a34a", "reject": "#ea580c", "warn": "#b45309",
    "fault": "#dc2626", "emerg": "#991b1b", "report": "#6d28d9", "tx": ACCENT, "plain": TXT,
}
PKG_SIZE = {"S": 16, "M": 21, "L": 26, None: 18}

DROP_ANIM_S = 0.9             # how long the "package landed in a bin" effect runs


# Firmware timeouts (final.c) that the GUI counts down in a pop-up
PAUSE_TIMEOUT_S = 10          # PAUSE_TIMEOUT_MS
FAULT_TIMEOUT_S = 10          # FAULT_TIMEOUT_MS

FAULT_INFO = {
    "lost":       ("PACKAGE LOST", "between the IR and LDR sensors",
                   "IR saw it enter, but it never reached the LDR."),
    "unexpected": ("UNEXPECTED PACKAGE", "at the LDR sensor",
                   "The LDR triggered without the IR sensor seeing anything first."),
    "stuck":      ("PACKAGE STUCK", "in the sorting zone",
                   "The package never cleared the LDR after being routed."),
    None:         ("FAULT", "on the belt", ""),
}


def fault_zone(g, kind):
    """(x_start, x_end) of the belt segment a fault belongs to."""
    if kind == "lost":
        return g.ir_x, g.ldr_x
    if kind == "unexpected":
        return g.ldr_x - 46, g.ldr_x + 46
    if kind == "stuck":
        return g.ldr_x, g.gate_x
    return g.x0, g.gate_x


# ----------------------------------------------------------------------------
# Conveyor view : geometry is pure maths, so it can be unit-tested headlessly
# ----------------------------------------------------------------------------
class Geometry:
    """Computes every coordinate of the conveyor from the canvas size."""

    def __init__(self, w=1000, h=240):
        self.resize(w, h)

    def resize(self, w, h):
        self.w, self.h = max(w, 620), max(h, 240)
        self.belt_y = self.h * 0.40
        self.belt_t = 24                       # belt thickness
        self.x0 = 66                           # belt start (entry)
        self.x1 = self.w - 28                  # belt end
        span = self.x1 - self.x0
        self.bin_top = self.belt_y + 52
        self.bin_h = max(60, self.h - self.bin_top - 14)

        # 4 bins (reject + S/M/L) share the right-hand part of the belt
        gap = 12
        self.bin_w = min(92.0, (span * 0.60 - 3 * gap) / 4)
        right = self.x1 - 6
        self.bin_x = {k: right - self.bin_w * (3 - i) - gap * (2 - i)   # S, M, L
                      for i, k in enumerate(SIZES)}
        self.rej_x = self.bin_x["S"] - gap - self.bin_w
        self.gate_x = self.rej_x + self.bin_w / 2
        left = self.gate_x - self.x0
        self.ir_x = self.x0 + left * 0.26
        self.ldr_x = self.x0 + left * 0.64

    def point(self, name):
        """Waypoint name -> (x, y) on the canvas."""
        by = self.belt_y
        if name == "entry":
            return (self.x0 - 30, by)
        if name == "ir":
            return (self.ir_x, by)
        if name == "ldr":
            return (self.ldr_x, by)
        if name == "gate":
            return (self.gate_x, by)
        if name == "rej_in":
            return (self.rej_x + self.bin_w / 2, self.bin_top + self.bin_h * 0.52)
        if name.startswith("bin_"):
            return (self.bin_x[name[4]] + self.bin_w / 2, by)
        if name.startswith("drop_"):
            return (self.bin_x[name[5]] + self.bin_w / 2, self.bin_top + self.bin_h * 0.52)
        raise KeyError(name)


def route_for(decision, size):
    """The waypoints a package follows once the servo has decided."""
    if decision == "REJECT":
        return ["gate", "rej_in"]
    return ["gate", f"bin_{size}", f"drop_{size}"]


def speed_scale(g, x):
    """The belt crawls through the zones where a package can get stuck."""
    if g.ldr_x <= x <= g.gate_x:               # measuring / sorting zone
        return 0.55
    if x > g.gate_x:                           # dropping into a bin
        return 0.70
    return 1.0


def step_towards(x, y, tx, ty, dt=1 / 30, scale=1.0):
    """Ease-out move; returns (x, y, arrived)."""
    dx, dy = tx - x, ty - y
    dist = math.hypot(dx, dy)
    if dist < 1.5:
        return tx, ty, True
    speed = max(130.0, dist * 3.2) * scale     # px per second
    stepd = min(dist, speed * dt)
    return x + dx / dist * stepd, y + dy / dist * stepd, False


class ConveyorView:
    """Belt, sensors, servo gate, bins and the travelling package."""

    def __init__(self, parent, model):
        self.model = model
        self.g = Geometry()
        self.canvas = tk.Canvas(parent, bg=CARD, highlightthickness=0, height=240)
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", self._on_resize)
        self.stripe = 0.0
        self.roller = 0.0
        self.pkg = None            # dict(x, y, size, decision, path, mode, seen)
        self.flash = 0.0
        self.drops = []            # landing animations: dict(bin, t)
        self.seen_seq = 0          # last model.pkg_seq we already spawned a package for

    # -- events ------------------------------------------------------------
    def _on_resize(self, e):
        old_w, old_h = self.g.w, self.g.h
        self.g.resize(e.width, e.height)
        if self.pkg:               # keep the package roughly where it was
            self.pkg["x"] *= self.g.w / old_w
            self.pkg["y"] *= self.g.h / old_h

    def _land_now(self):
        """Finish the package on screen (if it has been decided) and clear it."""
        p = self.pkg
        if p and p.get("decision"):
            self.drops.append({"bin": "rej" if p["decision"] == "REJECT" else p["size"], "t": 0.0})
        self.pkg = None

    def sync(self):
        """Pull new state out of the model. A package is created ONLY by a new IR event
        (model.pkg_seq changes); any other line (e.g. [TIME] Remaining) never spawns one."""
        m = self.model
        if m.state in ("EMERGENCY", "IDLE", "READY", "REPORT"):
            self._land_now()
            return
        if m.pkg_seq != self.seen_seq:                    # a new package entered
            self.seen_seq = m.pkg_seq
            self._land_now()                              # previous one still in flight lands now
            x, y = self.g.point("entry")
            self.pkg = {"x": x, "y": y, "size": None, "decision": None, "path": ["ir"],
                        "mode": "run", "seen": 1, "seq": m.pkg_seq}
        p = self.pkg
        if p is None or p.get("seq") != m.pkg_seq:
            return
        if m.state == "FAULT":
            p["mode"] = "fault"
            return
        if p["mode"] == "fault":                          # fault is over: drop the stuck package
            self.pkg = None
            return
        prog = (4 if m.dec_seq == m.pkg_seq else 3 if m.size_seq == m.pkg_seq
                else 2 if m.ldr_seq == m.pkg_seq else 1)
        if prog >= 2 > p["seen"]:
            p["path"], p["seen"] = ["ldr"], 2
        if prog >= 3:
            p["size"] = m.last_size
            p["seen"] = max(p["seen"], 3)
        if prog >= 4 > p["seen"] and m.last_decision:
            p["decision"] = m.last_decision
            p["path"] = route_for(m.last_decision, m.last_size or "M")
            p["seen"] = 4

    def belt_is_moving(self):
        """The belt only runs while a package is actually travelling on it."""
        p = self.pkg
        return bool(p and p["mode"] == "run" and self.model.state in ACTIVE)

    def tick(self, dt=1 / 30):
        if self.belt_is_moving():
            self.stripe = (self.stripe + 120 * dt) % 30
            self.roller = (self.roller + 260 * dt) % 360
        self.flash = (self.flash + dt * (1.6 if self.model.state == "EMERGENCY" else 1.0)) % 1.0
        p = self.pkg
        if p and p["mode"] == "run" and p["path"] and self.model.state in ACTIVE:
            tx, ty = self.g.point(p["path"][0])
            rush = 1.0 + 0.45 * max(0, len(p["path"]) - 1)   # behind? hurry a little
            p["x"], p["y"], arrived = step_towards(
                p["x"], p["y"], tx, ty, dt, speed_scale(self.g, p["x"]) * rush)
            if arrived:
                p["path"].pop(0)
                if not p["path"] and p["decision"]:
                    p["mode"] = "done"
                    self.drops.append({"bin": "rej" if p["decision"] == "REJECT"
                                       else p["size"], "t": 0.0})
        elif p and p["mode"] == "done":
            self.pkg = None
        for d in self.drops:
            d["t"] += dt
        self.drops = [d for d in self.drops if d["t"] < DROP_ANIM_S]
        self.draw()

    # -- drawing -----------------------------------------------------------
    def _round(self, x0, y0, x1, y1, r, **kw):
        r = max(1.0, min(r, (x1 - x0) / 2, (y1 - y0) / 2))
        pts = [x0 + r, y0, x1 - r, y0, x1, y0, x1, y0 + r, x1, y1 - r, x1, y1,
               x1 - r, y1, x0 + r, y1, x0, y1, x0, y1 - r, x0, y0 + r, x0, y0]
        return self.canvas.create_polygon(pts, smooth=True, **kw)

    def draw(self):
        c, g, m = self.canvas, self.g, self.model
        c.delete("all")
        by, t = g.belt_y, g.belt_t
        dim = m.state == "EMERGENCY"
        running = self.belt_is_moving()

        # belt body + rollers
        self._round(g.x0, by - t / 2, g.x1, by + t / 2, t / 2,
                    fill="#eceef1" if dim else BELT, outline=BELT_EDGE)
        if running:
            x = g.x0 + 10 + self.stripe
            while x < g.x1 - 8:
                c.create_line(x, by - t / 2 + 5, x - 10, by + t / 2 - 5,
                              fill=STRIPE, width=4)
                x += 30
        for rx in (g.x0 + t / 2 - 2, g.x1 - t / 2 + 2):
            c.create_oval(rx - t / 2, by - t / 2, rx + t / 2, by + t / 2,
                          fill="#f7f8fa", outline=BELT_EDGE, width=2)
            a = math.radians(self.roller if running else 0)
            c.create_line(rx - (t / 2 - 6) * math.cos(a), by - (t / 2 - 6) * math.sin(a),
                          rx + (t / 2 - 6) * math.cos(a), by + (t / 2 - 6) * math.sin(a),
                          fill=BELT_EDGE, width=3)
        c.create_line(g.x0, by + t / 2 + 9, g.x1, by + t / 2 + 9, fill=BORDER, width=2)

        # entry
        c.create_text(g.x0 - 34, by - 26, text="IN", fill=FAINT, font=("Segoe UI", 9, "bold"))
        c.create_line(g.x0 - 46, by, g.x0 - 10, by, fill=FAINT, width=3,
                      arrow="last", arrowshape=(10, 12, 5))
        if m.state in ACTIVE and self.pkg is None and not dim:
            c.create_text((g.x0 + g.gate_x) / 2, by, text="waiting for a package …",
                          fill=FAINT, font=("Segoe UI", 9))

        # sensors
        self._sensor(g.ir_x, by, "IR", m.stage >= 1 and m.state in ACTIVE, IR_C, IR_SOFT)
        self._sensor(g.ldr_x, by, "LDR", m.stage >= 2 and m.state in ACTIVE, LDR_C, LDR_SOFT)

        # servo gate
        rejecting = bool(self.pkg and self.pkg.get("decision") == "REJECT")
        col = REJ if rejecting else FAINT
        gx, gy = g.gate_x, by - t / 2 - 28
        c.create_line(gx, gy + 6, gx, by - t / 2 - 2, fill=BORDER, width=3)
        ang = math.radians(58 if rejecting else 0)
        ln = 32
        c.create_line(gx, gy, gx + ln * math.cos(ang), gy + ln * math.sin(ang),
                      fill=col, width=8, capstyle="round")
        c.create_oval(gx - 8, gy - 8, gx + 8, gy + 8, fill=REJ_SOFT if rejecting else CARD,
                      outline=col, width=3)
        c.create_text(gx, gy - 20, text="SERVO", fill=col, font=("Segoe UI", 9, "bold"))

        # bins
        for k in SIZES:
            self._bin(g.bin_x[k], g.bin_top, g.bin_w, g.bin_h, SIZE_COLOR[k], SIZE_SOFT[k],
                      k, f"{m.accepted[k]}/{m.target[k]}", "ACCEPT", key=k)
        self._bin(g.rej_x, g.bin_top, g.bin_w, g.bin_h, REJ, REJ_SOFT,
                  "✕", str(sum(m.rejected.values())), "REJECT", key="rej")

        if m.state == "FAULT":
            self._fault_zone()
        if self.pkg:
            self._package(self.pkg)
        if dim:
            self._emergency()

    def _fault_zone(self):
        """Marks WHERE on the belt the fault happened."""
        c, g, m = self.canvas, self.g, self.model
        x0, x1 = fault_zone(g, m.fault_kind)
        by, t = g.belt_y, g.belt_t
        on = self.flash < 0.5
        edge = "#ef4444" if on else "#fca5a5"
        self._round(x0 - 7, by - t / 2 - 12, x1 + 7, by + t / 2 + 12, 12,
                    fill="#fff1f2" if on else CARD, outline=edge, width=3)
        for x in range(int(x0), int(x1), 16):       # hazard hatching
            c.create_line(x, by + t / 2 + 12, x + 10, by - t / 2 - 12,
                          fill="#fecdd3", width=2)
        title, where, _ = FAULT_INFO.get(m.fault_kind, FAULT_INFO[None])
        mx, ty = (x0 + x1) / 2, by + t / 2 + 24
        c.create_text(mx, ty, text="▲", fill=edge, font=("Segoe UI", 10, "bold"))
        c.create_text(mx, ty + 13, text=f"{title} {where}", fill="#b91c1c",
                      font=("Segoe UI", 9, "bold"))
        if on:                                      # blinking warning lamp
            c.create_oval(mx - 11, by - t / 2 - 40, mx + 11, by - t / 2 - 18,
                          fill="#fecaca", outline="#ef4444", width=2)
            c.create_text(mx, by - t / 2 - 29, text="!", fill="#b91c1c",
                          font=("Segoe UI", 11, "bold"))

    def _emergency(self):
        """Full-width flashing alarm overlay."""
        c, g = self.canvas, self.g
        on = self.flash < 0.5
        band = "#ef4444" if on else "#fca5a5"
        for y in (6, g.h - 14):                     # hazard bars top and bottom
            c.create_rectangle(0, y, g.w, y + 8, fill=band, outline="")
        for x in range(-40, int(g.w) + 40, 34):     # diagonal hazard stripes
            c.create_line(x, g.h, x + 46, 0, fill="#fee2e2", width=7)
        w2, y = g.w / 2, g.belt_y
        self._round(w2 - 175, y - 36, w2 + 175, y + 36, 15,
                    fill="#b91c1c" if on else "#ef4444", outline="#7f1d1d", width=3)
        c.create_text(w2 - 134, y, text="⚠", fill="#fff7ed", font=("Segoe UI", 24, "bold"))
        c.create_text(w2 + 14, y - 10, text="EMERGENCY STOP", fill="#ffffff",
                      font=("Segoe UI", 17, "bold"))
        c.create_text(w2 + 14, y + 13, text="all operations halted · press RESET (PB3)",
                      fill="#fee2e2", font=("Segoe UI", 8))

    def _sensor(self, x, by, name, on, color, soft):
        c, t = self.canvas, self.g.belt_t
        top = by - t / 2 - 26
        c.create_line(x, top + 14, x, by - t / 2 - 2, fill=color if on else BORDER, width=3)
        if on:                                            # soft glow ring
            c.create_oval(x - 17, top - 17, x + 17, top + 17, fill=soft, outline="")
        c.create_oval(x - 11, top - 11, x + 11, top + 11,
                      fill=color if on else CARD, outline=color if on else BORDER, width=3)
        if on:
            c.create_oval(x - 5, top - 7, x + 1, top - 1, fill="#ffffff", outline="")
        c.create_text(x, top - 26, text=name, fill=color if on else FAINT,
                      font=("Segoe UI", 9, "bold"))

    def _drop_phase(self, key):
        """0..1 progress of the landing effect for a bin, or None."""
        for d in self.drops:
            if d["bin"] == key:
                return min(1.0, d["t"] / DROP_ANIM_S)
        return None

    def _bin(self, x, y, w, h, color, soft, label, count, kind, key=None):
        c = self.canvas
        ph = self._drop_phase(key)
        if ph is not None:
            # the bin dips, springs back and flashes as the package lands in it
            bounce = math.sin(ph * math.pi) * 7 * (1 - ph)
            y += bounce
            ring = 1.0 - ph
            self._round(x - 6 * ring, y - 6 * ring, x + w + 6 * ring, y + h + 6 * ring,
                        12, fill="", outline=color, width=max(1, int(4 * ring) + 1))
            c.create_text(x + w / 2, y - 16 - 26 * ph, text="+1", fill=color,
                          font=("Segoe UI", 13 if ph < 0.5 else 11, "bold"))
        c.create_line(x + w / 2, y - 38, x + w / 2, y - 5, fill=BORDER, width=2, dash=(4, 4))
        self._round(x, y, x + w, y + h, 10, fill=soft, outline=color, width=2)
        self._round(x + 5, y + h - 10, x + w - 5, y + h - 4, 3, fill=color, outline="")
        c.create_oval(x + w / 2 - 12, y + 5, x + w / 2 + 12, y + 29, fill=color, outline="")
        c.create_text(x + w / 2, y + 17, text=label, fill="#ffffff", font=("Segoe UI", 11, "bold"))
        c.create_text(x + w / 2, y + 42, text=count, fill=TXT, font=("Consolas", 13, "bold"))
        c.create_text(x + w / 2, y + h - 15, text=kind, fill=MUTED, font=("Segoe UI", 7, "bold"))

    def _package(self, p):
        c = self.canvas
        col = SIZE_COLOR.get(p["size"], "#cbd5e1")
        if p["mode"] == "fault":
            col = "#ef4444" if self.flash < 0.5 else "#fecaca"
        s = PKG_SIZE.get(p["size"], PKG_SIZE[None])
        x, y = p["x"], p["y"]
        sx = sy = s
        if p["path"] and p["path"][0].startswith(("drop_", "rej")):     # falling: squash
            sx, sy = s * 1.12, s * 0.88
        c.create_oval(x - s * 0.9, y + sy - 3, x + s * 0.9, y + sy + 5,
                      fill="#e5e7eb", outline="")                       # soft shadow
        self._round(x - sx, y - sy, x + sx, y + sy, 8, fill=col, outline="")
        self._round(x - sx + 4, y - sy + 4, x + sx - 4, y - sy + 9, 3,
                    fill="#ffffff", outline="")                          # highlight
        c.create_line(x - sx, y, x + sx, y, fill="#ffffff", width=3)     # tape
        c.create_line(x, y - sy, x, y + sy, fill="#ffffff", width=3)
        lbl = "!" if p["mode"] == "fault" else (p["size"] or "?")
        c.create_text(x, y - sy - 12, text=lbl, fill=col, font=("Segoe UI", 10, "bold"))


# ----------------------------------------------------------------------------
# Alert pop-ups : paused, fault, emergency, time running out
# ----------------------------------------------------------------------------
def alert_spec(m, now):
    """What alert (if any) the current model state calls for.

    Returns (key, kind, title, message, detail, seconds_left) or None.
    seconds_left is None when the state has no timeout attached.
    """
    el = max(0.0, now - m.state_since)
    if m.state == "EMERGENCY":
        return ("emergency", "emergency", "EMERGENCY STOP",
                "All operations halted.",
                "Release the emergency button, then press RESET (PB3).", None)
    if m.state == "FAULT":
        title, where, why = FAULT_INFO.get(m.fault_kind, FAULT_INFO[None])
        return ("fault", "fault", title, f"{title.title()} {where}.", why,
                max(0.0, FAULT_TIMEOUT_S - el))
    if m.state == "PAUSED":
        return ("paused", "paused", "SYSTEM PAUSED",
                "Press PAUSE (PB5) to resume, or RESET (PB3) to stop.",
                "", max(0.0, PAUSE_TIMEOUT_S - el))
    return None


ALERT_COLORS = {
    "emergency": ("#b91c1c", "#ef4444", "#fee2e2"),
    "fault":     ("#dc2626", "#f87171", "#fee2e2"),
    "paused":    ("#d97706", "#fbbf24", "#fef3c7"),
}


class AlertDialog(tk.Toplevel):
    """A small banner window that flashes and can count a timeout down."""

    def __init__(self, master, spec):
        super().__init__(master)
        key, kind, title, msg, detail, secs = spec
        self.kind = kind
        self.dark, self.light, self.soft = ALERT_COLORS[kind]
        self.phase = 0.0
        self.overrideredirect(False)
        self.title(title.title())
        self.configure(bg=self.dark)
        self.resizable(False, False)
        self.transient(master)
        self.attributes("-topmost", True)

        self.head = tk.Frame(self, bg=self.dark)
        self.head.pack(fill="x")
        icon = {"emergency": "⚠", "fault": "⚠", "paused": "⏸"}[kind]
        self.icon_lbl = tk.Label(self.head, text=icon, bg=self.dark, fg="white",
                                 font=("Segoe UI", 30, "bold"))
        self.icon_lbl.pack(side="left", padx=(22, 14), pady=16)
        txt = tk.Frame(self.head, bg=self.dark)
        txt.pack(side="left", pady=16)
        self.title_lbl = tk.Label(txt, text=title, bg=self.dark, fg="white",
                                  font=("Segoe UI", 19, "bold"))
        self.title_lbl.pack(anchor="w")
        self.msg_lbl = tk.Label(txt, text=msg, bg=self.dark, fg="#ffffff",
                                font=("Segoe UI", 10))
        self.msg_lbl.pack(anchor="w")

        body = tk.Frame(self, bg=CARD)
        body.pack(fill="both", expand=True)
        if detail:
            tk.Label(body, text=detail, bg=CARD, fg=MUTED, font=("Segoe UI", 10),
                     wraplength=380, justify="left").pack(anchor="w", padx=22, pady=(14, 0))
        self.count_lbl = None
        if secs is not None:
            self.count_lbl = tk.Label(body, text="", bg=CARD, fg=self.dark,
                                      font=("Consolas", 30, "bold"))
            self.count_lbl.pack(anchor="w", padx=22, pady=(10, 0))
            self.count_sub = tk.Label(body, text="", bg=CARD, fg=FAINT,
                                      font=("Segoe UI", 9))
            self.count_sub.pack(anchor="w", padx=22)
            self.bar_bg = tk.Frame(body, bg=self.soft, height=8)
            self.bar_bg.pack(fill="x", padx=22, pady=(8, 0))
            self.bar = tk.Frame(self.bar_bg, bg=self.dark, height=8)
            self.bar.place(relx=0, rely=0, relwidth=1, relheight=1)
            self.total = max(secs, 0.001)
        ttk.Button(body, text="Hide", command=self.destroy).pack(anchor="e", padx=22, pady=14)
        self.bind("<Escape>", lambda e: self.destroy())

        self.update_idletasks()
        try:
            x = master.winfo_rootx() + (master.winfo_width() - self.winfo_width()) // 2
            y = master.winfo_rooty() + 90
            self.geometry(f"+{max(x, 0)}+{max(y, 0)}")
        except tk.TclError:
            pass
        self.lift()

    def refresh(self, spec, dt):
        """Called ~30x a second by the app while the alert is on screen."""
        _, _, _, _, _, secs = spec
        self.phase = (self.phase + dt * (2.2 if self.kind in ("emergency", "fault") else 1.0)) % 1.0
        col = self.dark if self.phase < 0.5 else self.light
        for w in (self, self.head, self.icon_lbl, self.title_lbl, self.msg_lbl):
            w.configure(bg=col)
        self.icon_lbl.configure(fg="white")
        self.title_lbl.master.configure(bg=col)
        if self.count_lbl is not None and secs is not None:
            self.count_lbl.configure(text=f"{secs:4.1f} s")
            self.count_sub.configure(text="until the board stops and prints its report")
            self.bar.place(relwidth=max(0.0, min(1.0, secs / self.total)))


# ----------------------------------------------------------------------------
# Summary report pop-up
# ----------------------------------------------------------------------------
def report_rows(m):
    """Builds the lines shown in the pop-up. Pure data, so it can be tested."""
    rows = [("ACCEPTED", None)]
    for k in SIZES:
        hit = m.accepted[k] >= m.target[k] and m.target[k] > 0
        rows.append((f"Size {k}", f"{m.accepted[k]} / {m.target[k]}", SIZE_COLOR[k], hit))
    rows.append(("REJECTED", None))
    for k in SIZES:
        rows.append((f"Size {k}", str(m.rejected[k]), REJ, None))
    rows.append(("ERRORS", None))
    for lab, key in (("Object Lost", "lost"), ("Unexpected Object", "unexpected"),
                     ("Object Stuck", "stuck")):
        rows.append((lab, str(m.errors[key]), MUTED, None))
    rows.append(("TOTALS", None))
    r = m.report
    tot_in = sum(m.accepted.values()) + sum(m.rejected.values()) + sum(m.errors.values())
    rows.append(("Total Input", r.get("input", str(tot_in)), TXT, None))
    rows.append(("Total Target", r.get("total_target", str(sum(m.target.values()))), TXT, None))
    rows.append(("Elapsed Time", r.get("elapsed", "-") + " s", TXT, None))
    rows.append(("RATES", None))
    for lab, key in (("Target Completion", "completion"), ("System Efficiency", "efficiency"),
                     ("Reject Rate", "reject_rate"), ("System Loss Rate", "loss")):
        rows.append((lab, (r[key] + " %") if key in r else "-", TXT, None))
    return rows


class ReportDialog(tk.Toplevel):
    """Shown automatically when the board finishes printing its summary."""

    def __init__(self, master, model, on_save):
        super().__init__(master)
        self.title("Summary Report")
        self.configure(bg=CARD)
        self.resizable(False, False)
        self.transient(master)

        comp = model.report.get("completion")
        done = comp is not None and float(comp) >= 100.0
        head_col = "#22c55e" if done else "#f59e0b"
        head = tk.Frame(self, bg=head_col)
        head.pack(fill="x")
        tk.Label(head, text="SUMMARY REPORT", bg=head_col, fg="white",
                 font=("Segoe UI", 9, "bold")).pack(anchor="w", padx=24, pady=(16, 0))
        tk.Label(head, text=("ALL TARGETS REACHED" if done else "RUN ENDED"), bg=head_col,
                 fg="white", font=("Segoe UI", 21, "bold")).pack(anchor="w", padx=24)
        tk.Label(head, text=f"Target completion   {comp or '-'} %", bg=head_col, fg="white",
                 font=("Segoe UI", 11)).pack(anchor="w", padx=24, pady=(0, 16))

        body = tk.Frame(self, bg=CARD)
        body.pack(fill="both", expand=True, padx=24, pady=16)
        body.columnconfigure(1, weight=1)
        r = 0
        for row in report_rows(model):
            if row[1] is None:
                tk.Label(body, text=row[0], bg=CARD, fg=FAINT, font=("Segoe UI", 8, "bold")).grid(
                    row=r, column=0, sticky="w", pady=(14 if r else 0, 2))
            else:
                lab, val, col, hit = row
                tk.Label(body, text=lab, bg=CARD, fg=MUTED,
                         font=("Segoe UI", 10)).grid(row=r, column=0, sticky="w", pady=2)
                tk.Label(body, text=val, bg=CARD, fg=col, font=("Consolas", 13, "bold")).grid(
                    row=r, column=1, sticky="e", padx=(48, 10))
                if hit is not None:
                    tk.Label(body, text="✓" if hit else "✕", bg=CARD,
                             fg="#16a34a" if hit else "#ef4444",
                             font=("Segoe UI", 12, "bold")).grid(row=r, column=2, sticky="e")
            r += 1

        btns = tk.Frame(self, bg=CARD)
        btns.pack(fill="x", padx=24, pady=(0, 18))
        ttk.Button(btns, text="Save log", command=on_save).pack(side="left")
        ttk.Button(btns, text="Close", command=self.destroy).pack(side="right")

        self.bind("<Escape>", lambda e: self.destroy())
        self.update_idletasks()
        try:                        # centre over the main window
            x = master.winfo_rootx() + (master.winfo_width() - self.winfo_width()) // 2
            y = master.winfo_rooty() + (master.winfo_height() - self.winfo_height()) // 3
            self.geometry(f"+{max(x, 0)}+{max(y, 0)}")
        except tk.TclError:
            pass
        self.lift()
        self.focus_force()


# ----------------------------------------------------------------------------
# App
# ----------------------------------------------------------------------------
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Package Sorting System - Dashboard")
        self.geometry("980x820")
        self.minsize(880, 660)
        self.configure(bg=BG)
        self.model = Model()
        self.q = queue.Queue()
        self.ser = None
        self.stop_evt = threading.Event()
        self.demo_iter = None
        self.demo_job = None
        self.report_shown = 0          # last model.report_seq already popped up
        self.report_win = None
        self.alert_win = None          # paused / fault / emergency / time pop-up
        self.alert_key = None
        self.alert_muted = set()       # keys the user closed by hand
        self.banner_flash = 0.0
        self.clock_text = None
        self._style()
        self._build()
        self._refresh_ports()
        self._refresh_ui()
        self.after(50, self._poll)
        self.after(33, self._animate)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # -- widgets -----------------------------------------------------------
    def _style(self):
        st = ttk.Style(self)
        try:
            st.theme_use("clam")
        except tk.TclError:
            pass
        for k, col in SIZE_COLOR.items():
            st.configure(f"{k}.Horizontal.TProgressbar", troughcolor="#eef0f3", background=col,
                         bordercolor="#eef0f3", lightcolor=col, darkcolor=col, thickness=8)
        st.configure("TButton", padding=(12, 5))
        st.configure("Go.TButton", padding=(18, 8), font=("Segoe UI", 10, "bold"))

    def _card(self, parent, title=None):
        f = tk.Frame(parent, bg=CARD, highlightthickness=1, highlightbackground=BORDER)
        if title:
            tk.Label(f, text=title, bg=CARD, fg=FAINT,
                     font=("Segoe UI", 8, "bold")).pack(anchor="w", padx=12, pady=(7, 0))
        return f

    def _stepper(self, parent, key, label, color, soft, default, hi):
        """One compact pastel pill: label, value and -/+ buttons, all in a row."""
        box = tk.Frame(parent, bg=soft, highlightthickness=1, highlightbackground=color)
        tk.Label(box, text=label, bg=soft, fg=color,
                 font=("Segoe UI", 9, "bold")).pack(side="left", padx=(8, 4), pady=4)
        var = tk.IntVar(value=default)
        self.cfg_vars[key] = var

        def bump(d):
            try:
                v = var.get()
            except tk.TclError:
                v = 0
            var.set(max(0, min(hi, v + d)))

        def mini(txt, d):
            return tk.Button(box, text=txt, width=2, relief="flat", bg=CARD, fg=color,
                             activebackground=color, activeforeground="white", bd=0,
                             font=("Segoe UI", 9, "bold"), cursor="hand2",
                             command=lambda: bump(d))
        mini("−", -1).pack(side="left")
        tk.Entry(box, textvariable=var, width=3, justify="center",
                 relief="flat", bg=soft, fg=TXT, font=("Consolas", 13, "bold"),
                 highlightthickness=0, bd=0).pack(side="left", padx=1, fill="x", expand=True)
        mini("+", 1).pack(side="left", padx=(0, 6))
        return box

    def _build(self):
        pad = dict(padx=12)

        bar = tk.Frame(self, bg=BG)
        bar.pack(fill="x", pady=(9, 4), **pad)
        tk.Label(bar, text="Port", bg=BG, fg=MUTED).pack(side="left")
        self.port_var = tk.StringVar()
        self.port_cb = ttk.Combobox(bar, textvariable=self.port_var, width=30, state="readonly")
        self.port_cb.pack(side="left", padx=5)
        ttk.Button(bar, text="Refresh", command=self._refresh_ports).pack(side="left")
        tk.Label(bar, text="Baud", bg=BG, fg=MUTED).pack(side="left", padx=(10, 3))
        self.baud_var = tk.StringVar(value="115200")
        ttk.Combobox(bar, textvariable=self.baud_var, width=8,
                     values=("9600", "57600", "115200", "230400")).pack(side="left")
        self.conn_btn = ttk.Button(bar, text="Connect", command=self._toggle_connect)
        self.conn_btn.pack(side="left", padx=8)
        self.demo_btn = ttk.Button(bar, text="Demo", command=self._toggle_demo)
        self.demo_btn.pack(side="left")
        self.popup_var = tk.BooleanVar(value=True)
        tk.Checkbutton(bar, text="Pop-up report", variable=self.popup_var, bg=BG, fg=MUTED,
                       activebackground=BG, selectcolor=CARD).pack(side="left", padx=(8, 0))
        ttk.Button(bar, text="Show report", command=self._show_report).pack(side="left", padx=4)
        self.link_lbl = tk.Label(bar, text="● Disconnected", bg=BG, fg=FAINT)
        self.link_lbl.pack(side="right")

        # banner
        self.banner = tk.Frame(self, bg="#7c8899")
        self.banner.pack(fill="x", pady=4, **pad)
        left = tk.Frame(self.banner, bg="#7c8899")
        left.pack(side="left", padx=16, pady=10)
        self.state_lbl = tk.Label(left, text="IDLE", font=("Segoe UI", 21, "bold"),
                                  fg="white", bg="#7c8899")
        self.state_lbl.pack(anchor="w")
        self.sub_lbl = tk.Label(left, text="", font=("Segoe UI", 10), fg="white", bg="#7c8899")
        self.sub_lbl.pack(anchor="w")
        right = tk.Frame(self.banner, bg="#7c8899")
        right.pack(side="right", padx=16)
        self.time_cap = tk.Label(right, text="TIME REMAINING", font=("Segoe UI", 8, "bold"),
                                 fg="white", bg="#7c8899")
        self.time_cap.pack(anchor="e")
        self.time_lbl = tk.Label(right, text="--", font=("Consolas", 23, "bold"),
                                 fg="white", bg="#7c8899")
        self.time_lbl.pack(anchor="e")
        self._banner_widgets = [self.banner, left, right, self.state_lbl, self.sub_lbl,
                                self.time_cap, self.time_lbl]

        # conveyor
        conv = self._card(self, "CONVEYOR")
        conv.pack(fill="x", pady=4, **pad)
        holder = tk.Frame(conv, bg=CARD)
        holder.pack(fill="both", expand=True, padx=8, pady=(3, 4))
        self.conveyor = ConveyorView(holder, self.model)
        self.pkg_lbl = tk.Label(conv, text="", bg=CARD, fg=MUTED, font=("Segoe UI", 9))
        self.pkg_lbl.pack(anchor="w", padx=12, pady=(0, 7))

        # size cards
        row = tk.Frame(self, bg=BG)
        row.pack(fill="x", pady=4, **pad)
        self.cards = {}
        for i, k in enumerate(SIZES):
            row.columnconfigure(i, weight=1, uniform="c")
            c = self._card(row)
            c.grid(row=0, column=i, sticky="nsew", padx=(0 if i == 0 else 8, 0))
            tk.Frame(c, bg=SIZE_COLOR[k], height=4).pack(fill="x")
            head = tk.Frame(c, bg=CARD)
            head.pack(fill="x", padx=12, pady=(6, 0))
            dot = tk.Canvas(head, width=16, height=16, bg=CARD, highlightthickness=0)
            dot.create_oval(1, 1, 15, 15, fill=SIZE_COLOR[k], outline="")
            dot.create_text(8, 8, text=k, fill="white", font=("Segoe UI", 7, "bold"))
            dot.pack(side="left")
            tk.Label(head, text=f"SIZE {k}", bg=CARD, fg=FAINT,
                     font=("Segoe UI", 8, "bold")).pack(side="left", padx=5)
            big = tk.Label(c, text="0 / 0", bg=CARD, fg=TXT, font=("Consolas", 19, "bold"))
            big.pack(anchor="w", padx=12)
            pb = ttk.Progressbar(c, style=f"{k}.Horizontal.TProgressbar", maximum=1, value=0)
            pb.pack(fill="x", padx=12, pady=4)
            rj = tk.Label(c, text="Rejected: 0", bg=CARD, fg="#ea580c", font=("Segoe UI", 8))
            rj.pack(anchor="w", padx=12, pady=(0, 8))
            self.cards[k] = (big, pb, rj)

        # middle
        mid = tk.Frame(self, bg=BG)
        mid.pack(fill="x", pady=4, **pad)
        mid.columnconfigure(0, weight=3, uniform="m")
        mid.columnconfigure(1, weight=2, uniform="m")

        lf = tk.Frame(mid, bg=BG)
        lf.grid(row=0, column=0, sticky="nsew")
        cfg = self._card(lf, "CONFIGURATION  ·  the board accepts it in IDLE only")
        cfg.pack(fill="x")
        cr = tk.Frame(cfg, bg=CARD)
        cr.pack(fill="x", padx=12, pady=(6, 10))
        self.cfg_vars = {}
        specs = (("S", "S", SIZE_COLOR["S"], SIZE_SOFT["S"], 2, 99),
                 ("M", "M", SIZE_COLOR["M"], SIZE_SOFT["M"], 2, 99),
                 ("L", "L", SIZE_COLOR["L"], SIZE_SOFT["L"], 2, 99),
                 ("T (s)", "T", ACCENT, ACCENT_SOFT, 30, 9999))
        for i, (key, lab, col, soft, dv, hi) in enumerate(specs):   # one row, like v3
            cr.columnconfigure(i, weight=1, uniform="cfg")           # stretch with the window
            self._stepper(cr, key, lab, col, soft, dv, hi).grid(
                row=0, column=i, sticky="ew", padx=(0 if i == 0 else 6, 0))
        self.send_btn = tk.Button(cr, text="Send Config  ➜", bg=ACCENT, fg="white",
                                  activebackground="#4f46e5", activeforeground="white",
                                  relief="flat", bd=0, cursor="hand2", padx=12, pady=6,
                                  font=("Segoe UI", 9, "bold"), command=self._send_config)
        self.send_btn.grid(row=0, column=len(specs), sticky="ew", padx=(8, 0))
        self.send_btn.bind("<Enter>", lambda e: self.send_btn.configure(bg="#4f46e5"))
        self.send_btn.bind("<Leave>", lambda e: self.send_btn.configure(bg=ACCENT))

        hist = self._card(lf, "HISTORY  ·  latest right, ✕ = rejected")
        hist.pack(fill="x", pady=(6, 0))
        self.hist = tk.Canvas(hist, height=32, bg=CARD, highlightthickness=0)
        self.hist.pack(fill="x", padx=12, pady=(3, 8))

        rc = self._card(mid, "ERRORS & REPORT")
        rc.grid(row=0, column=1, sticky="nsew", padx=(8, 0))
        grid = tk.Frame(rc, bg=CARD)
        grid.pack(fill="both", expand=True, padx=12, pady=(4, 8))
        grid.columnconfigure(1, weight=1)
        self.rep_vars = {}
        items = (("Object Lost", "lost"), ("Unexpected Object", "unexpected"),
                 ("Object Stuck", "stuck"), ("Total Input", "input"),
                 ("Total Target", "total_target"))
        for r, (lab, key) in enumerate(items):
            tk.Label(grid, text=lab, bg=CARD, fg=MUTED,
                     font=("Segoe UI", 9)).grid(row=r, column=0, sticky="w", pady=1)
            v = tk.StringVar(value="-")
            tk.Label(grid, textvariable=v, bg=CARD, fg=TXT,
                     font=("Consolas", 10, "bold")).grid(row=r, column=1, sticky="e")
            self.rep_vars[key] = v

        # log
        lg = self._card(self, "LOG")
        lg.pack(fill="both", expand=True, pady=(4, 12), **pad)
        tb = tk.Frame(lg, bg=CARD)
        tb.place(relx=1.0, x=-10, y=3, anchor="ne")
        self.auto_var = tk.BooleanVar(value=True)
        tk.Checkbutton(tb, text="Auto-scroll", variable=self.auto_var, bg=CARD, fg=MUTED,
                       activebackground=CARD, selectcolor=CARD).pack(side="left")
        ttk.Button(tb, text="Clear", width=6,
                   command=lambda: self.log.delete("1.0", "end")).pack(side="left", padx=3)
        ttk.Button(tb, text="Save", width=6, command=self._save_log).pack(side="left")
        wrap = tk.Frame(lg, bg=CARD)
        wrap.pack(fill="both", expand=True, padx=10, pady=(26, 10))
        self.log = tk.Text(wrap, height=16, wrap="word", font=("Consolas", 9),
                           bg="#fafbfc", relief="flat", padx=8, pady=6,
                           highlightthickness=1, highlightbackground=BORDER)
        sb = ttk.Scrollbar(wrap, command=self.log.yview)
        self.log.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self.log.pack(side="left", fill="both", expand=True)
        for tag, col in LOG_TAGS.items():
            self.log.tag_configure(tag, foreground=col)
        for tag in ("fault", "emerg"):
            self.log.tag_configure(tag, foreground=LOG_TAGS[tag], font=("Consolas", 9, "bold"))

    # -- serial ------------------------------------------------------------
    def _refresh_ports(self):
        ports = []
        if list_ports is not None:
            ports = [f"{p.device} - {p.description}" for p in list_ports.comports()]
        self.port_cb["values"] = ports
        if ports and not self.port_var.get():
            stl = [p for p in ports if "STM" in p.upper() or "ST-LINK" in p.upper()]
            self.port_var.set((stl or ports)[0])

    def _toggle_connect(self):
        if self.ser is not None:
            self._disconnect()
            return
        if serial is None:
            messagebox.showerror("pyserial missing", "Install it with:\n\npip install pyserial")
            return
        port = self.port_var.get().split(" - ")[0].strip()
        if not port:
            messagebox.showwarning("No port", "Select a COM port first.")
            return
        try:
            self.ser = serial.Serial(port, int(self.baud_var.get()), timeout=0.1)
        except Exception as e:                                   # noqa: BLE001
            messagebox.showerror("Cannot open port", str(e))
            self.ser = None
            return
        self._stop_demo()
        self.model.reset_all()
        self.report_shown = self.model.report_seq
        self.stop_evt.clear()
        threading.Thread(target=self._reader, daemon=True).start()
        self.conn_btn.configure(text="Disconnect")
        self.link_lbl.configure(text=f"● Connected {port}", fg="#16a34a")
        self._log("Connected. Press the board RESET (black button) to resync the dashboard.", "sys")

    def _disconnect(self):
        self.stop_evt.set()
        try:
            if self.ser:
                self.ser.close()
        except Exception:                                        # noqa: BLE001
            pass
        self.ser = None
        self.conn_btn.configure(text="Connect")
        self.link_lbl.configure(text="● Disconnected", fg=FAINT)

    def _reader(self):
        buf = b""
        ser = self.ser
        while not self.stop_evt.is_set():
            try:
                data = ser.read(ser.in_waiting or 1)
            except Exception as e:                               # noqa: BLE001
                self.q.put(("__err__", str(e)))
                return
            if not data:
                continue
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self.q.put(("rx", line.decode("utf-8", "replace").strip("\r")))

    def _cfg(self):
        out = []
        for k in ("S", "M", "L", "T (s)"):
            try:
                out.append(max(0, self.cfg_vars[k].get()))
            except tk.TclError:                                  # empty / non-numeric entry
                self.cfg_vars[k].set(0)
                out.append(0)
        return out

    def _send_config(self):
        s, m, l, t = self._cfg()
        if self.demo_iter is not None:
            messagebox.showinfo("Demo running", "Stop the demo first.")
            return
        if self.ser is None:
            messagebox.showwarning("Not connected", "Connect to the board first (or use Demo).")
            return
        if self.model.state != "IDLE" and not messagebox.askyesno(
                "Board not in IDLE", "The board only accepts a config in IDLE.\nSend anyway?"):
            return
        text = f"S{s},M{m},L{l},T{t}"
        self.ser.write((text + "\n").encode("ascii"))
        self._log("TX> " + text, "tx")

    # -- demo --------------------------------------------------------------
    def _toggle_demo(self):
        if self.demo_iter is not None:
            self._stop_demo()
            return
        if self.ser is not None:
            self._disconnect()
        s, m, l, t = self._cfg()
        self.model.reset_all()
        self.report_shown = self.model.report_seq
        self.demo_iter = demo_script(s, m, l, t)
        self.demo_btn.configure(text="Stop demo")
        self.link_lbl.configure(text="● Demo mode", fg=ACCENT)
        self._demo_tick()

    def _stop_demo(self):
        if self.demo_job:
            self.after_cancel(self.demo_job)
        self.demo_job = None
        self.demo_iter = None
        self.demo_btn.configure(text="Demo")
        if self.ser is None:
            self.link_lbl.configure(text="● Disconnected", fg=FAINT)

    def _demo_tick(self):
        """Each tuple from the script is (delay-before-this-line, line)."""
        try:
            delay, line = next(self.demo_iter)
        except StopIteration:
            self._stop_demo()
            return

        def emit():
            self.q.put(("rx", line))
            self._demo_tick()
        self.demo_job = self.after(int(max(delay, 0.02) * 1000), emit)

    # -- loops -------------------------------------------------------------
    def _log(self, text, tag="plain"):
        ts = datetime.datetime.now().strftime("%H:%M:%S")
        self.log.insert("end", f"{ts}  {text}\n", tag)
        if int(self.log.index("end-1c").split(".")[0]) > 3000:
            self.log.delete("1.0", "500.0")
        if self.auto_var.get():
            self.log.see("end")

    def _save_log(self):
        p = filedialog.asksaveasfilename(
            defaultextension=".txt", filetypes=[("Text", "*.txt")],
            initialfile=f"sorter_log_{datetime.datetime.now():%Y%m%d_%H%M%S}.txt")
        if p:
            with open(p, "w", encoding="utf-8") as f:
                f.write(self.log.get("1.0", "end"))

    def _show_report(self):
        if not self.model.report:
            messagebox.showinfo("No report yet",
                                "The board has not sent a summary report in this session yet.")
            return
        if self.report_win is not None and self.report_win.winfo_exists():
            self.report_win.destroy()
        self.report_win = ReportDialog(self, self.model, self._save_log)

    def _poll(self):
        changed = False
        try:
            while True:
                kind, text = self.q.get_nowait()
                if kind == "__err__":
                    self._log("Serial error: " + text, "fault")
                    self._disconnect()
                    continue
                if not text.strip():
                    continue
                tag = self.model.feed(text)
                self._log(text, tag or "plain")
                changed = True
        except queue.Empty:
            pass
        if changed:
            self.conveyor.sync()
            self._refresh_ui()
            if self.model.report_seq > self.report_shown:
                self.report_shown = self.model.report_seq
                if self.popup_var.get():
                    self._show_report()
        self.after(50, self._poll)

    def _update_alerts(self, dt):
        spec = alert_spec(self.model, time.monotonic())
        key = spec[0] if spec else None
        if key != self.alert_key:                      # state changed -> swap pop-up
            if self.alert_win is not None and self.alert_win.winfo_exists():
                self.alert_win.destroy()
            self.alert_win = None
            self.alert_key = key
            self.alert_muted.discard(key)              # a new event un-mutes it
            if spec and key not in self.alert_muted:
                self.alert_win = AlertDialog(self, spec)
        if spec and self.alert_win is not None:
            if self.alert_win.winfo_exists():
                self.alert_win.refresh(spec, dt)
            else:                                      # user pressed Hide / Esc
                self.alert_muted.add(key)
                self.alert_win = None

    def _tick_clock(self):
        """Counts the remaining time down second by second between board updates."""
        m = self.model
        left = m.remaining_now()
        if left is not None:
            text = f"{math.ceil(left - 1e-6)} s"
        elif m.time_limit and m.state == "READY":
            text = f"{m.time_limit} s"
        else:
            text = "--"
        if text != self.clock_text:
            self.clock_text = text
            self.time_lbl.configure(text=text)

    def _flash_banner(self, dt):
        """Pulses the state banner red while the system is in alarm."""
        m = self.model
        if m.state not in ("EMERGENCY", "FAULT"):
            return
        self.banner_flash = (self.banner_flash + dt * 2.2) % 1.0
        hot = "#7f1d1d" if m.state == "EMERGENCY" else "#dc2626"
        col = hot if self.banner_flash < 0.5 else ("#ef4444" if m.state == "EMERGENCY" else "#fca5a5")
        for w in self._banner_widgets:
            w.configure(bg=col)

    def _animate(self):
        dt = 1 / 30
        try:
            self.conveyor.tick(dt)
            if self.model.infer_idle():
                self._refresh_ui()
            self._tick_clock()
            self._update_alerts(dt)
            self._flash_banner(dt)
        except tk.TclError:
            return
        self.after(33, self._animate)

    def _refresh_ui(self):
        m = self.model
        color, title, sub = STATE_STYLE.get(m.state, STATE_STYLE["IDLE"])
        if m.state == "FAULT":
            sub = m.fault_msg
        for w in self._banner_widgets:
            w.configure(bg=color)
        self.state_lbl.configure(text=title)
        self.sub_lbl.configure(text=sub)
        self._tick_clock()
        self.time_cap.configure(text="TIME LIMIT" if m.state == "READY" else "TIME REMAINING")
        self.pkg_lbl.configure(
            text=f"Current package   ·   Size: {m.last_size or '-'}   ·   Decision: {m.last_decision or '-'}")

        for k in SIZES:
            big, pb, rj = self.cards[k]
            big.configure(text=f"{m.accepted[k]} / {m.target[k]}")
            pb.configure(maximum=max(m.target[k], 1), value=min(m.accepted[k], max(m.target[k], 1)))
            rj.configure(text=f"Rejected: {m.rejected[k]}")

        self.hist.delete("all")
        for i, (sz, dec) in enumerate(m.history):
            x = 3 + i * 27
            self.hist.create_oval(x, 4, x + 24, 28, fill=SIZE_COLOR[sz], outline="")
            self.hist.create_text(x + 12, 16, text=sz, fill="white", font=("Segoe UI", 9, "bold"))
            if dec == "REJECT":
                self.hist.create_line(x + 5, 9, x + 19, 23, fill="#ffffff", width=3)
                self.hist.create_line(x + 19, 9, x + 5, 23, fill="#ffffff", width=3)

        for k in ("lost", "unexpected", "stuck"):
            self.rep_vars[k].set(str(m.errors[k]))
        total_in = sum(m.accepted.values()) + sum(m.rejected.values()) + sum(m.errors.values())
        rp = m.report
        self.rep_vars["input"].set(rp.get("input", str(total_in) if total_in else "-"))
        self.rep_vars["total_target"].set(
            rp.get("total_target", str(sum(m.target.values())) if sum(m.target.values()) else "-"))
        # the remaining figures (elapsed time and the rates) live in the report pop-up

    def _on_close(self):
        if self.alert_win is not None and self.alert_win.winfo_exists():
            self.alert_win.destroy()
        self._stop_demo()
        self._disconnect()
        self.destroy()


if __name__ == "__main__":
    App().mainloop()