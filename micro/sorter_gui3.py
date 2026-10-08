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


class Model:
    def __init__(self):
        self.reset_all()

    def reset_all(self):
        self.state = "IDLE"
        self.prev_state = "RUNNING"
        self.report_seq = 0           # incremented once per completed report
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

    def _set(self, st):
        if st == "PAUSED" and self.state != "PAUSED":
            self.prev_state = self.state
        self.state = st
        if st not in ACTIVE:
            self.stage = 0

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
            self.remaining = int(m.group(1))
            return "time"
        if "Operating time expired" in s:
            self.remaining = 0
            self._set("REPORT")
            return "warn"

        if "[SENSOR] IR" in s:
            self._set("WAIT_LDR")
            self.stage = 1
            return "sensor"
        if "[SENSOR] LDR" in s:
            self._set("SETTLE")
            self.stage = 2
            return "sensor"

        m = re.search(r"Size Measured:\s*\[\s*([SML])\s*\]", s)
        if m:
            self.last_size = m.group(1)
            self._set("EVALUATE")
            self.stage = 3
            return "eval"
        m = re.search(r"Decision:\s*(ACCEPT|REJECT)", s)
        if m:
            self.last_decision = m.group(1)
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
            elif "Unexpected LDR" in s:
                self.errors["unexpected"] += 1
            elif "STUCK" in s.upper():
                self.errors["stuck"] += 1
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
BG, CARD, BORDER = "#f6f7f9", "#ffffff", "#e8eaed"
TXT, MUTED, FAINT = "#14181f", "#6b7280", "#9aa3af"
BELT, BELT_EDGE, STRIPE = "#d9dde3", "#c4cad2", "#eef0f3"
SIZE_COLOR = {"S": "#ef4444", "M": "#f59e0b", "L": "#22c55e"}
ACCENT, REJ = "#2563eb", "#fb7185"
STATE_STYLE = {
    "IDLE":      ("#64748b", "IDLE", "Waiting for configuration"),
    "READY":     ("#2563eb", "READY", "Press START (PB4) on the board"),
    "RUNNING":   ("#16a34a", "RUNNING", "Waiting for a package"),
    "WAIT_LDR":  ("#0d9488", "WAIT LDR", "Package on the belt"),
    "SETTLE":    ("#0d9488", "SETTLING", "Package at the sorting point"),
    "EVALUATE":  ("#0d9488", "EVALUATING", "Measuring size"),
    "ROUTE":     ("#0d9488", "ROUTING", "Servo moving the package"),
    "PAUSED":    ("#d97706", "PAUSED", "Press PAUSE (PB5) to resume"),
    "FAULT":     ("#dc2626", "FAULT", ""),
    "EMERGENCY": ("#7f1d1d", "EMERGENCY", "All operations halted"),
    "REPORT":    ("#7c3aed", "REPORT", "Summary report"),
}
LOG_TAGS = {
    "sys": MUTED, "dim": FAINT, "time": FAINT, "ok": "#15803d", "sensor": "#0e7490",
    "eval": "#1d4ed8", "accept": "#16a34a", "reject": "#ea580c", "warn": "#b45309",
    "fault": "#dc2626", "emerg": "#991b1b", "report": "#6d28d9", "tx": ACCENT, "plain": TXT,
}


# ----------------------------------------------------------------------------
# Conveyor view : geometry is pure maths, so it can be unit-tested headlessly
# ----------------------------------------------------------------------------
class Geometry:
    """Computes every coordinate of the conveyor from the canvas size."""

    def __init__(self, w=980, h=250):
        self.resize(w, h)

    def resize(self, w, h):
        self.w, self.h = max(w, 560), max(h, 200)
        self.belt_y = self.h * 0.40
        self.x0 = 54                       # belt start (entry)
        self.x1 = self.w - 26              # belt end
        span = self.x1 - self.x0
        self.bin_top = self.belt_y + 46
        self.bin_h = max(44, self.h - self.bin_top - 24)

        # 4 bins (reject + S/M/L) share the right-hand half; the belt's left
        # half holds the entry, IR and LDR stations.
        gap = 10
        bw = min(84.0, (span * 0.58 - 3 * gap) / 4)
        self.bin_w = bw
        right = self.x1 - 6
        self.bin_x = {k: right - bw * (3 - i) - gap * (2 - i)     # S, M, L left->right
                      for i, k in enumerate(SIZES)}
        self.rej_x = self.bin_x["S"] - gap - bw                   # reject sits left of S
        self.gate_x = self.rej_x + bw / 2                         # servo above reject bin
        left = self.gate_x - self.x0
        self.ir_x = self.x0 + left * 0.26
        self.ldr_x = self.x0 + left * 0.66

    def point(self, name):
        """Waypoint name -> (x, y) on the canvas."""
        by = self.belt_y
        if name == "entry":
            return (self.x0 - 24, by)
        if name == "ir":
            return (self.ir_x, by)
        if name == "ldr":
            return (self.ldr_x, by)
        if name == "gate":
            return (self.gate_x, by)
        if name == "rej_in":
            return (self.rej_x + self.bin_w / 2, self.bin_top + self.bin_h * 0.55)
        if name.startswith("bin_"):
            k = name[4]
            return (self.bin_x[k] + self.bin_w / 2, by)
        if name.startswith("drop_"):
            k = name[5]
            return (self.bin_x[k] + self.bin_w / 2, self.bin_top + self.bin_h * 0.55)
        raise KeyError(name)


def route_for(decision, size):
    """The waypoints a package follows once the servo has decided."""
    if decision == "REJECT":
        return ["gate", "rej_in"]
    return ["gate", f"bin_{size}", f"drop_{size}"]


def step_towards(x, y, tx, ty, dt=1 / 30):
    """Ease-out move; returns (x, y, arrived)."""
    dx, dy = tx - x, ty - y
    dist = math.hypot(dx, dy)
    if dist < 1.5:
        return tx, ty, True
    speed = max(150.0, dist * 4.0)          # px per second
    stepd = min(dist, speed * dt)
    return x + dx / dist * stepd, y + dy / dist * stepd, False


class ConveyorView:
    """Draws the belt, sensors, servo gate, bins and the travelling package."""

    def __init__(self, parent, model):
        self.model = model
        self.g = Geometry()
        self.canvas = tk.Canvas(parent, bg=CARD, highlightthickness=0, height=250)
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", self._on_resize)
        self.stripe = 0.0
        self.pkg = None          # dict(x, y, size, decision, path, mode, seen)
        self.flash = 0.0

    # -- events ------------------------------------------------------------
    def _on_resize(self, e):
        old_w, old_h = self.g.w, self.g.h
        self.g.resize(e.width, e.height)
        if self.pkg:             # keep the package roughly where it was
            self.pkg["x"] *= self.g.w / old_w
            self.pkg["y"] *= self.g.h / old_h

    def sync(self):
        """Pull new state out of the model and update the package path."""
        m = self.model
        if m.state in ("EMERGENCY", "IDLE", "READY") or (m.state == "REPORT" and self.pkg
                                                         and self.pkg["mode"] != "run"):
            self.pkg = None
            return
        if m.stage >= 1 and self.pkg is None:
            x, y = self.g.point("entry")
            self.pkg = {"x": x, "y": y, "size": None, "decision": None,
                        "path": ["ir"], "mode": "run", "seen": 1}
        p = self.pkg
        if p is None:
            return
        if m.state == "FAULT":
            p["mode"] = "fault"
            return
        if p["mode"] == "fault" and m.state != "FAULT":
            self.pkg = None
            return
        if m.stage >= 2 > p["seen"]:
            p["path"], p["seen"] = ["ldr"], 2
        if m.stage >= 3:
            p["size"] = m.last_size
            p["seen"] = max(p["seen"], 3)
        if m.stage >= 4 > p["seen"] and m.last_decision:
            p["decision"] = m.last_decision
            p["path"] = route_for(m.last_decision, m.last_size or "M")
            p["seen"] = 4

    def tick(self, dt=1 / 30):
        m = self.model
        if m.state in ACTIVE:
            self.stripe = (self.stripe + 110 * dt) % 26
        self.flash = (self.flash + dt) % 1.0
        p = self.pkg
        if p and p["mode"] == "run" and p["path"]:
            tx, ty = self.g.point(p["path"][0])
            p["x"], p["y"], arrived = step_towards(p["x"], p["y"], tx, ty, dt)
            if arrived:
                p["path"].pop(0)
                if not p["path"] and p["decision"]:
                    p["mode"] = "done"
        elif p and p["mode"] == "done":
            self.pkg = None
        self.draw()

    # -- drawing -----------------------------------------------------------
    def _round(self, x0, y0, x1, y1, r, **kw):
        r = max(1, min(r, (x1 - x0) / 2, (y1 - y0) / 2))
        pts = [x0 + r, y0, x1 - r, y0, x1, y0, x1, y0 + r, x1, y1 - r, x1, y1,
               x1 - r, y1, x0 + r, y1, x0, y1, x0, y1 - r, x0, y0 + r, x0, y0]
        return self.canvas.create_polygon(pts, smooth=True, **kw)

    def draw(self):
        c, g, m = self.canvas, self.g, self.model
        c.delete("all")
        by, h = g.belt_y, 22
        dim = m.state == "EMERGENCY"

        # belt
        self._round(g.x0, by - h / 2, g.x1, by + h / 2, 10,
                    fill="#eceef1" if dim else BELT, outline=BELT_EDGE)
        x = g.x0 + 8 + self.stripe
        while x < g.x1 - 6:
            c.create_line(x, by - h / 2 + 4, x - 8, by + h / 2 - 4, fill=STRIPE, width=3)
            x += 26
        c.create_line(g.x0, by + h / 2 + 7, g.x1, by + h / 2 + 7, fill=BORDER)

        # entry arrow
        c.create_text(g.x0 - 20, by - 24, text="IN", fill=FAINT, font=("Segoe UI", 8, "bold"))
        c.create_line(g.x0 - 34, by, g.x0 - 6, by, fill=FAINT, width=2, arrow="last")

        # sensors
        self._sensor(g.ir_x, by, "IR", m.stage >= 1 and m.state in ACTIVE, "#0ea5e9")
        self._sensor(g.ldr_x, by, "LDR", m.stage >= 2 and m.state in ACTIVE, "#8b5cf6")

        # servo gate
        rejecting = bool(self.pkg and self.pkg.get("decision") == "REJECT")
        gx = g.gate_x
        c.create_line(gx, by - 34, gx, by - h / 2 - 2, fill=BORDER, width=2)
        ang = math.radians(55 if rejecting else 0)
        ln = 26
        c.create_line(gx, by - 30, gx + ln * math.cos(ang), by - 30 + ln * math.sin(ang),
                      fill=REJ if rejecting else FAINT, width=5, capstyle="round")
        c.create_oval(gx - 4, by - 34, gx + 4, by - 26, fill=CARD,
                      outline=REJ if rejecting else FAINT, width=2)
        c.create_text(gx, by - 46, text="SERVO", fill=FAINT, font=("Segoe UI", 8, "bold"))

        # bins
        for k in SIZES:
            self._bin(g.bin_x[k], g.bin_top, g.bin_w, g.bin_h, SIZE_COLOR[k], k,
                      f"{m.accepted[k]}/{m.target[k]}", "ACCEPT")
        self._bin(g.rej_x, g.bin_top, g.bin_w, g.bin_h, REJ, "✕",
                  str(sum(m.rejected.values())), "REJECT")

        # package
        p = self.pkg
        if p:
            col = SIZE_COLOR.get(p["size"], "#94a3b8")
            if p["mode"] == "fault":
                col = "#dc2626" if self.flash < 0.5 else "#fca5a5"
            s = {"S": 11, "M": 14, "L": 18}.get(p["size"], 13)
            x, y = p["x"], p["y"]
            self._round(x - s, y - s, x + s, y + s, 5, fill=col, outline="")
            c.create_line(x - s, y, x + s, y, fill="#ffffff", width=2)
            lbl = "!" if p["mode"] == "fault" else (p["size"] or "?")
            c.create_text(x, y - s - 11, text=lbl, fill=col, font=("Segoe UI", 10, "bold"))

        if dim:
            c.create_text(g.w / 2, by, text="EMERGENCY STOP", fill="#b91c1c",
                          font=("Segoe UI", 20, "bold"))

    def _sensor(self, x, by, name, on, color):
        c = self.canvas
        c.create_line(x, by - 30, x, by - 14, fill=color if on else BORDER, width=2)
        c.create_oval(x - 6, by - 38, x + 6, by - 26,
                      fill=color if on else CARD, outline=color if on else BORDER, width=2)
        c.create_text(x, by - 50, text=name, fill=color if on else FAINT,
                      font=("Segoe UI", 8, "bold"))

    def _bin(self, x, y, w, h, color, label, count, kind):
        c = self.canvas
        c.create_line(x + w / 2, y - 36, x + w / 2, y - 4, fill=BORDER, width=1, dash=(3, 3))
        self._round(x, y, x + w, y + h, 8, fill=CARD, outline=color, width=2)
        c.create_rectangle(x + 2, y + h - 7, x + w - 2, y + h - 2, fill=color, outline="")
        c.create_text(x + w / 2, y + 15, text=label, fill=color, font=("Segoe UI", 12, "bold"))
        c.create_text(x + w / 2, y + h / 2 + 8, text=count, fill=TXT, font=("Consolas", 13, "bold"))
        c.create_text(x + w / 2, y + h - 16, text=kind, fill=FAINT, font=("Segoe UI", 7, "bold"))


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
        head_col = "#16a34a" if done else "#d97706"
        head = tk.Frame(self, bg=head_col)
        head.pack(fill="x")
        tk.Label(head, text="SUMMARY REPORT", bg=head_col, fg="white",
                 font=("Segoe UI", 9, "bold")).pack(anchor="w", padx=20, pady=(14, 0))
        tk.Label(head, text=("ALL TARGETS REACHED" if done else "RUN ENDED"),
                 bg=head_col, fg="white", font=("Segoe UI", 20, "bold")).pack(anchor="w", padx=20)
        tk.Label(head, text=f"Target completion  {comp or '-'} %", bg=head_col, fg="white",
                 font=("Segoe UI", 10)).pack(anchor="w", padx=20, pady=(0, 14))

        body = tk.Frame(self, bg=CARD)
        body.pack(fill="both", expand=True, padx=20, pady=14)
        body.columnconfigure(1, weight=1)
        r = 0
        for row in report_rows(model):
            if row[1] is None:
                if r:
                    tk.Frame(body, bg=BORDER, height=1).grid(row=r, column=0, columnspan=3,
                                                             sticky="ew", pady=7)
                    r += 1
                tk.Label(body, text=row[0], bg=CARD, fg=FAINT,
                         font=("Segoe UI", 8, "bold")).grid(row=r, column=0, sticky="w")
            else:
                lab, val, col, hit = row
                tk.Label(body, text=lab, bg=CARD, fg=MUTED).grid(row=r, column=0, sticky="w", pady=1)
                tk.Label(body, text=val, bg=CARD, fg=col, font=("Consolas", 12, "bold")).grid(
                    row=r, column=1, sticky="e", padx=(40, 8))
                if hit is not None:
                    tk.Label(body, text="✓" if hit else "✕", bg=CARD,
                             fg="#16a34a" if hit else "#dc2626",
                             font=("Segoe UI", 11, "bold")).grid(row=r, column=2, sticky="e")
            r += 1

        btns = tk.Frame(self, bg=CARD)
        btns.pack(fill="x", padx=20, pady=(0, 16))
        ttk.Button(btns, text="Save log", command=on_save).pack(side="left")
        ttk.Button(btns, text="Close", command=self.destroy).pack(side="right")

        self.bind("<Escape>", lambda e: self.destroy())
        self.update_idletasks()
        try:                      # centre over the main window
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
        self.geometry("1040x930")
        self.minsize(900, 760)
        self.configure(bg=BG)
        self.model = Model()
        self.q = queue.Queue()
        self.ser = None
        self.stop_evt = threading.Event()
        self.demo_iter = None
        self.demo_job = None
        self.report_shown = 0          # last model.report_seq already popped up
        self.report_win = None
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
            st.configure(f"{k}.Horizontal.TProgressbar", troughcolor="#edeff2", background=col,
                         bordercolor="#edeff2", lightcolor=col, darkcolor=col, thickness=7)
        st.configure("TButton", padding=(10, 4))

    def _card(self, parent, title=None):
        f = tk.Frame(parent, bg=CARD, highlightthickness=1, highlightbackground=BORDER)
        if title:
            tk.Label(f, text=title, bg=CARD, fg=FAINT,
                     font=("Segoe UI", 8, "bold")).pack(anchor="w", padx=14, pady=(9, 0))
        return f

    def _build(self):
        pad = dict(padx=14)

        bar = tk.Frame(self, bg=BG)
        bar.pack(fill="x", pady=(12, 6), **pad)
        tk.Label(bar, text="Port", bg=BG, fg=MUTED).pack(side="left")
        self.port_var = tk.StringVar()
        self.port_cb = ttk.Combobox(bar, textvariable=self.port_var, width=32, state="readonly")
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
        self.banner = tk.Frame(self, bg="#64748b")
        self.banner.pack(fill="x", pady=6, **pad)
        left = tk.Frame(self.banner, bg="#64748b")
        left.pack(side="left", padx=20, pady=14)
        self.state_lbl = tk.Label(left, text="IDLE", font=("Segoe UI", 26, "bold"), fg="white", bg="#64748b")
        self.state_lbl.pack(anchor="w")
        self.sub_lbl = tk.Label(left, text="", font=("Segoe UI", 10), fg="white", bg="#64748b")
        self.sub_lbl.pack(anchor="w")
        right = tk.Frame(self.banner, bg="#64748b")
        right.pack(side="right", padx=20)
        self.time_cap = tk.Label(right, text="TIME REMAINING", font=("Segoe UI", 8, "bold"), fg="white", bg="#64748b")
        self.time_cap.pack(anchor="e")
        self.time_lbl = tk.Label(right, text="--", font=("Consolas", 28, "bold"), fg="white", bg="#64748b")
        self.time_lbl.pack(anchor="e")
        self._banner_widgets = [self.banner, left, right, self.state_lbl, self.sub_lbl,
                                self.time_cap, self.time_lbl]

        # conveyor
        conv = self._card(self, "CONVEYOR")
        conv.pack(fill="x", pady=6, **pad)
        holder = tk.Frame(conv, bg=CARD)
        holder.pack(fill="both", expand=True, padx=10, pady=(4, 10))
        self.conveyor = ConveyorView(holder, self.model)
        self.pkg_lbl = tk.Label(conv, text="", bg=CARD, fg=MUTED, font=("Segoe UI", 9))
        self.pkg_lbl.pack(anchor="w", padx=14, pady=(0, 8))

        # size cards
        row = tk.Frame(self, bg=BG)
        row.pack(fill="x", pady=6, **pad)
        self.cards = {}
        for i, k in enumerate(SIZES):
            row.columnconfigure(i, weight=1, uniform="c")
            c = self._card(row)
            c.grid(row=0, column=i, sticky="nsew", padx=(0 if i == 0 else 8, 0))
            tk.Frame(c, bg=SIZE_COLOR[k], height=4).pack(fill="x")
            tk.Label(c, text=f"SIZE {k}", bg=CARD, fg=FAINT,
                     font=("Segoe UI", 8, "bold")).pack(anchor="w", padx=14, pady=(7, 0))
            big = tk.Label(c, text="0 / 0", bg=CARD, fg=TXT, font=("Consolas", 24, "bold"))
            big.pack(anchor="w", padx=14)
            pb = ttk.Progressbar(c, style=f"{k}.Horizontal.TProgressbar", maximum=1, value=0)
            pb.pack(fill="x", padx=14, pady=5)
            rj = tk.Label(c, text="Rejected: 0", bg=CARD, fg="#ea580c", font=("Segoe UI", 9))
            rj.pack(anchor="w", padx=14, pady=(0, 11))
            self.cards[k] = (big, pb, rj)

        # middle
        mid = tk.Frame(self, bg=BG)
        mid.pack(fill="x", pady=6, **pad)
        mid.columnconfigure(0, weight=1, uniform="m")
        mid.columnconfigure(1, weight=1, uniform="m")

        lf = tk.Frame(mid, bg=BG)
        lf.grid(row=0, column=0, sticky="nsew")
        cfg = self._card(lf, "CONFIGURATION  (board accepts it in IDLE only)")
        cfg.pack(fill="x")
        cr = tk.Frame(cfg, bg=CARD)
        cr.pack(fill="x", padx=14, pady=9)
        self.cfg_vars = {}
        for lab, default, hi in (("S", 2, 99), ("M", 2, 99), ("L", 2, 99), ("T (s)", 30, 9999)):
            tk.Label(cr, text=lab, bg=CARD, fg=MUTED).pack(side="left", padx=(0, 3))
            v = tk.IntVar(value=default)
            ttk.Spinbox(cr, from_=0, to=hi, width=5, textvariable=v).pack(side="left", padx=(0, 9))
            self.cfg_vars[lab] = v
        ttk.Button(cr, text="Send Config", command=self._send_config).pack(side="left")

        hist = self._card(lf, "HISTORY  (latest right, ✕ = rejected)")
        hist.pack(fill="x", pady=(8, 0))
        self.hist = tk.Canvas(hist, height=34, bg=CARD, highlightthickness=0)
        self.hist.pack(fill="x", padx=14, pady=(4, 10))

        rc = self._card(mid, "ERRORS & REPORT")
        rc.grid(row=0, column=1, sticky="nsew", padx=(8, 0))
        grid = tk.Frame(rc, bg=CARD)
        grid.pack(fill="both", expand=True, padx=14, pady=9)
        grid.columnconfigure(1, weight=1)
        self.rep_vars = {}
        items = (("Object Lost", "lost"), ("Unexpected Object", "unexpected"), ("Object Stuck", "stuck"),
                 ("Total Input", "input"), ("Total Target", "total_target"), ("Elapsed", "elapsed"),
                 ("Target Completion", "completion"), ("System Efficiency", "efficiency"),
                 ("Reject Rate", "reject_rate"), ("System Loss Rate", "loss"))
        for i, (lab, key) in enumerate(items):
            r = i + (1 if i >= 3 else 0) + (1 if i >= 6 else 0)
            tk.Label(grid, text=lab, bg=CARD, fg=MUTED).grid(row=r, column=0, sticky="w", pady=1)
            v = tk.StringVar(value="-")
            tk.Label(grid, textvariable=v, bg=CARD, fg=TXT,
                     font=("Consolas", 11, "bold")).grid(row=r, column=1, sticky="e", padx=(20, 0))
            self.rep_vars[key] = v
        tk.Frame(grid, bg=BORDER, height=1).grid(row=4, column=0, columnspan=2, sticky="ew", pady=4)
        tk.Frame(grid, bg=BORDER, height=1).grid(row=8, column=0, columnspan=2, sticky="ew", pady=4)

        # log
        lg = self._card(self, "LOG")
        lg.pack(fill="both", expand=True, pady=(6, 14), **pad)
        tb = tk.Frame(lg, bg=CARD)
        tb.place(relx=1.0, x=-12, y=4, anchor="ne")
        self.auto_var = tk.BooleanVar(value=True)
        tk.Checkbutton(tb, text="Auto-scroll", variable=self.auto_var, bg=CARD,
                       fg=MUTED, activebackground=CARD).pack(side="left")
        ttk.Button(tb, text="Clear", width=6,
                   command=lambda: self.log.delete("1.0", "end")).pack(side="left", padx=3)
        ttk.Button(tb, text="Save", width=6, command=self._save_log).pack(side="left")
        wrap = tk.Frame(lg, bg=CARD)
        wrap.pack(fill="both", expand=True, padx=10, pady=(28, 10))
        self.log = tk.Text(wrap, height=8, wrap="word", font=("Consolas", 10),
                           bg="#fafbfc", relief="flat", padx=8, pady=6)
        sb = ttk.Scrollbar(wrap, command=self.log.yview)
        self.log.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self.log.pack(side="left", fill="both", expand=True)
        for tag, col in LOG_TAGS.items():
            self.log.tag_configure(tag, foreground=col)
        for tag in ("fault", "emerg"):
            self.log.tag_configure(tag, foreground=LOG_TAGS[tag], font=("Consolas", 10, "bold"))

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

    def _send_config(self):
        try:
            s, m, l, t = (self.cfg_vars[k].get() for k in ("S", "M", "L", "T (s)"))
        except tk.TclError:
            messagebox.showwarning("Invalid", "Config values must be integers.")
            return
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
        s, m, l, t = (self.cfg_vars[k].get() for k in ("S", "M", "L", "T (s)"))
        self.model.reset_all()
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

    def _show_report(self):
        if not self.model.report:
            messagebox.showinfo("No report yet",
                                "The board has not sent a summary report in this session yet.")
            return
        if self.report_win is not None and self.report_win.winfo_exists():
            self.report_win.destroy()
        self.report_win = ReportDialog(self, self.model, self._save_log)

    def _animate(self):
        try:
            self.conveyor.tick()
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
        if m.remaining is not None:
            self.time_lbl.configure(text=f"{m.remaining} s")
        else:
            self.time_lbl.configure(text=f"{m.time_limit} s" if m.time_limit and m.state == "READY" else "--")
        self.time_cap.configure(text="TIME LIMIT" if m.state == "READY" else "TIME REMAINING")
        self.pkg_lbl.configure(
            text=f"Current package   Size: {m.last_size or '-'}    Decision: {m.last_decision or '-'}")

        for k in SIZES:
            big, pb, rj = self.cards[k]
            big.configure(text=f"{m.accepted[k]} / {m.target[k]}")
            pb.configure(maximum=max(m.target[k], 1), value=min(m.accepted[k], max(m.target[k], 1)))
            rj.configure(text=f"Rejected: {m.rejected[k]}")

        self.hist.delete("all")
        for i, (sz, dec) in enumerate(m.history):
            x = 2 + i * 26
            self.hist.create_rectangle(x, 6, x + 22, 28, fill=SIZE_COLOR[sz], outline="")
            self.hist.create_text(x + 11, 17, text=sz, fill="white", font=("Segoe UI", 9, "bold"))
            if dec == "REJECT":
                self.hist.create_line(x + 2, 8, x + 20, 26, fill="#111827", width=2)
                self.hist.create_line(x + 20, 8, x + 2, 26, fill="#111827", width=2)

        for k in ("lost", "unexpected", "stuck"):
            self.rep_vars[k].set(str(m.errors[k]))
        total_in = sum(m.accepted.values()) + sum(m.rejected.values()) + sum(m.errors.values())
        rp = m.report
        self.rep_vars["input"].set(rp.get("input", str(total_in) if total_in else "-"))
        self.rep_vars["total_target"].set(
            rp.get("total_target", str(sum(m.target.values())) if sum(m.target.values()) else "-"))
        self.rep_vars["elapsed"].set(rp["elapsed"] + " s" if "elapsed" in rp else "-")
        for k in ("completion", "efficiency", "reject_rate", "loss"):
            self.rep_vars[k].set(rp[k] + " %" if k in rp else "-")

    def _on_close(self):
        self._stop_demo()
        self._disconnect()
        self.destroy()


if __name__ == "__main__":
    App().mainloop()