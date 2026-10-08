#!/usr/bin/env python3
"""
Package Sorting System - PC Dashboard
-------------------------------------
Reads the UART text already printed by the STM32 firmware (115200 8N1) and
shows it as a live dashboard.  No firmware change is required.

    pip install pyserial
    python sorter_gui.py

Use the "Demo" button to try the GUI without a board.
"""
import datetime
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
# GUI
# ----------------------------------------------------------------------------
BG, CARD, BORDER, TXT, MUTED = "#f3f4f6", "#ffffff", "#e5e7eb", "#111827", "#6b7280"
SIZE_COLOR = {"S": "#dc2626", "M": "#eab308", "L": "#16a34a"}
STATE_STYLE = {
    "IDLE": ("#6b7280", "IDLE", "Waiting for configuration"),
    "READY": ("#2563eb", "READY", "Press START (PB4) on the board"),
    "RUNNING": ("#16a34a", "RUNNING", "Waiting for a package (IR sensor)"),
    "WAIT_LDR": ("#0d9488", "WAIT LDR", "IR triggered - waiting for LDR"),
    "SETTLE": ("#0d9488", "SETTLING", "Package at sorting point"),
    "EVALUATE": ("#0d9488", "EVALUATING", "Measuring size"),
    "ROUTE": ("#0d9488", "ROUTING", "Servo moving package"),
    "PAUSED": ("#d97706", "PAUSED", "Press PAUSE (PB5) to resume"),
    "FAULT": ("#dc2626", "FAULT", ""),
    "EMERGENCY": ("#7f1d1d", "EMERGENCY", "All operations halted"),
    "REPORT": ("#7c3aed", "REPORT", "Summary report"),
}
LOG_TAGS = {
    "sys": "#6b7280", "dim": "#9ca3af", "time": "#9ca3af", "ok": "#15803d",
    "sensor": "#0e7490", "eval": "#1d4ed8", "accept": "#16a34a", "reject": "#ea580c",
    "warn": "#b45309", "fault": "#dc2626", "emerg": "#991b1b", "report": "#6d28d9",
    "tx": "#2563eb", "plain": "#111827",
}


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Package Sorting System - Dashboard")
        self.geometry("1000x880")
        self.minsize(920, 740)
        self.configure(bg=BG)
        self.model = Model()
        self.q = queue.Queue()
        self.ser = None
        self.stop_evt = threading.Event()
        self.demo_iter = None
        self.demo_job = None
        self._style()
        self._build()
        self._refresh_ports()
        self._refresh_ui()
        self.after(50, self._poll)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---- widgets ---------------------------------------------------------
    def _style(self):
        st = ttk.Style(self)
        st.theme_use("clam")
        for k, c in SIZE_COLOR.items():
            st.configure(f"{k}.Horizontal.TProgressbar", troughcolor="#e5e7eb",
                         background=c, bordercolor=BORDER, lightcolor=c, darkcolor=c)
        st.configure("TCombobox", padding=3)

    def _card(self, parent, title=None):
        f = tk.Frame(parent, bg=CARD, highlightthickness=1, highlightbackground=BORDER)
        if title:
            tk.Label(f, text=title, bg=CARD, fg=MUTED, font=("Segoe UI", 9, "bold")).pack(anchor="w", padx=12, pady=(8, 0))
        return f

    def _build(self):
        pad = dict(padx=12)
        # connection bar
        bar = tk.Frame(self, bg=BG)
        bar.pack(fill="x", pady=(10, 4), **pad)
        tk.Label(bar, text="Port", bg=BG).pack(side="left")
        self.port_var = tk.StringVar()
        self.port_cb = ttk.Combobox(bar, textvariable=self.port_var, width=34, state="readonly")
        self.port_cb.pack(side="left", padx=4)
        ttk.Button(bar, text="Refresh", command=self._refresh_ports).pack(side="left")
        tk.Label(bar, text="Baud", bg=BG).pack(side="left", padx=(10, 2))
        self.baud_var = tk.StringVar(value="115200")
        ttk.Combobox(bar, textvariable=self.baud_var, width=8,
                     values=("9600", "57600", "115200", "230400")).pack(side="left")
        self.conn_btn = ttk.Button(bar, text="Connect", command=self._toggle_connect)
        self.conn_btn.pack(side="left", padx=8)
        self.demo_btn = ttk.Button(bar, text="Demo", command=self._toggle_demo)
        self.demo_btn.pack(side="left")
        self.link_lbl = tk.Label(bar, text="● Disconnected", bg=BG, fg=MUTED)
        self.link_lbl.pack(side="right")

        # state banner
        self.banner = tk.Frame(self, bg="#6b7280")
        self.banner.pack(fill="x", pady=4, **pad)
        left = tk.Frame(self.banner, bg="#6b7280")
        left.pack(side="left", padx=18, pady=12)
        self.state_lbl = tk.Label(left, text="IDLE", font=("Segoe UI", 28, "bold"), fg="white", bg="#6b7280")
        self.state_lbl.pack(anchor="w")
        self.sub_lbl = tk.Label(left, text="", font=("Segoe UI", 11), fg="white", bg="#6b7280")
        self.sub_lbl.pack(anchor="w")
        right = tk.Frame(self.banner, bg="#6b7280")
        right.pack(side="right", padx=18)
        self.time_cap = tk.Label(right, text="TIME REMAINING", font=("Segoe UI", 9, "bold"), fg="white", bg="#6b7280")
        self.time_cap.pack(anchor="e")
        self.time_lbl = tk.Label(right, text="--", font=("Consolas", 30, "bold"), fg="white", bg="#6b7280")
        self.time_lbl.pack(anchor="e")
        self._banner_widgets = [self.banner, left, right, self.state_lbl, self.sub_lbl, self.time_cap, self.time_lbl]

        # size cards
        row = tk.Frame(self, bg=BG)
        row.pack(fill="x", pady=4, **pad)
        self.cards = {}
        for i, k in enumerate(SIZES):
            row.columnconfigure(i, weight=1, uniform="c")
            c = self._card(row)
            c.grid(row=0, column=i, sticky="nsew", padx=(0 if i == 0 else 6, 0))
            tk.Frame(c, bg=SIZE_COLOR[k], height=5).pack(fill="x")
            tk.Label(c, text=f"SIZE {k}", bg=CARD, fg=MUTED, font=("Segoe UI", 9, "bold")).pack(anchor="w", padx=12, pady=(6, 0))
            big = tk.Label(c, text="0 / 0", bg=CARD, fg=TXT, font=("Consolas", 26, "bold"))
            big.pack(anchor="w", padx=12)
            pb = ttk.Progressbar(c, style=f"{k}.Horizontal.TProgressbar", maximum=1, value=0)
            pb.pack(fill="x", padx=12, pady=4)
            rj = tk.Label(c, text="Rejected: 0", bg=CARD, fg="#ea580c", font=("Segoe UI", 10))
            rj.pack(anchor="w", padx=12, pady=(0, 10))
            self.cards[k] = (big, pb, rj)

        # middle row
        mid = tk.Frame(self, bg=BG)
        mid.pack(fill="x", pady=4, **pad)
        mid.columnconfigure(0, weight=1, uniform="m")
        mid.columnconfigure(1, weight=1, uniform="m")

        lf = tk.Frame(mid, bg=BG)
        lf.grid(row=0, column=0, sticky="nsew")
        cfg = self._card(lf, "CONFIGURATION  (accepted by board in IDLE only)")
        cfg.pack(fill="x")
        cr = tk.Frame(cfg, bg=CARD)
        cr.pack(fill="x", padx=12, pady=8)
        self.cfg_vars = {}
        for lab, default, hi in (("S", 2, 99), ("M", 2, 99), ("L", 2, 99), ("T (s)", 30, 9999)):
            tk.Label(cr, text=lab, bg=CARD).pack(side="left", padx=(0, 2))
            v = tk.IntVar(value=default)
            ttk.Spinbox(cr, from_=0, to=hi, width=5, textvariable=v).pack(side="left", padx=(0, 8))
            self.cfg_vars[lab] = v
        ttk.Button(cr, text="Send Config", command=self._send_config).pack(side="left", padx=4)

        pipe = self._card(lf, "CURRENT PACKAGE")
        pipe.pack(fill="x", pady=(6, 0))
        chips = tk.Frame(pipe, bg=CARD)
        chips.pack(fill="x", padx=12, pady=(6, 4))
        self.chips = []
        for name in STAGES:
            lb = tk.Label(chips, text=name, bg="#e5e7eb", fg=MUTED, width=9, pady=4, font=("Segoe UI", 9, "bold"))
            lb.pack(side="left", padx=(0, 4))
            self.chips.append(lb)
        self.pkg_lbl = tk.Label(pipe, text="Size: -    Decision: -", bg=CARD, fg=TXT, font=("Segoe UI", 11))
        self.pkg_lbl.pack(anchor="w", padx=12)
        tk.Label(pipe, text="History (latest right)", bg=CARD, fg=MUTED, font=("Segoe UI", 8)).pack(anchor="w", padx=12, pady=(6, 0))
        self.hist = tk.Canvas(pipe, height=30, bg=CARD, highlightthickness=0)
        self.hist.pack(fill="x", padx=12, pady=(0, 10))

        rc = self._card(mid, "ERRORS & REPORT")
        rc.grid(row=0, column=1, sticky="nsew", padx=(6, 0))
        grid = tk.Frame(rc, bg=CARD)
        grid.pack(fill="both", expand=True, padx=12, pady=8)
        self.rep_vars = {}
        items = (("Object Lost", "lost"), ("Unexpected Object", "unexpected"), ("Object Stuck", "stuck"),
                 ("Total Input", "input"), ("Total Target", "total_target"), ("Elapsed", "elapsed"),
                 ("Target Completion", "completion"), ("System Efficiency", "efficiency"),
                 ("Reject Rate", "reject_rate"), ("System Loss Rate", "loss"))
        for i, (lab, key) in enumerate(items):
            r = i + (1 if i >= 3 else 0) + (1 if i >= 6 else 0)
            tk.Label(grid, text=lab, bg=CARD, fg=MUTED).grid(row=r, column=0, sticky="w", pady=1)
            v = tk.StringVar(value="-")
            tk.Label(grid, textvariable=v, bg=CARD, fg=TXT, font=("Consolas", 11, "bold")).grid(row=r, column=1, sticky="e", padx=(20, 0))
            grid.columnconfigure(1, weight=1)
            self.rep_vars[key] = v
        tk.Frame(grid, bg=BORDER, height=1).grid(row=4, column=0, columnspan=2, sticky="ew", pady=3)
        tk.Frame(grid, bg=BORDER, height=1).grid(row=8, column=0, columnspan=2, sticky="ew", pady=3)

        # log
        lg = self._card(self, "LOG")
        lg.pack(fill="both", expand=True, pady=(4, 12), **pad)
        tb = tk.Frame(lg, bg=CARD)
        tb.place(relx=1.0, x=-10, y=4, anchor="ne")
        self.auto_var = tk.BooleanVar(value=True)
        tk.Checkbutton(tb, text="Auto-scroll", variable=self.auto_var, bg=CARD).pack(side="left")
        ttk.Button(tb, text="Clear", width=6, command=lambda: self.log.delete("1.0", "end")).pack(side="left", padx=2)
        ttk.Button(tb, text="Save", width=6, command=self._save_log).pack(side="left")
        wrap = tk.Frame(lg, bg=CARD)
        wrap.pack(fill="both", expand=True, padx=8, pady=(26, 8))
        self.log = tk.Text(wrap, height=10, wrap="word", font=("Consolas", 10), bg="#fafafa", relief="flat")
        sb = ttk.Scrollbar(wrap, command=self.log.yview)
        self.log.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self.log.pack(side="left", fill="both", expand=True)
        for tag, col in LOG_TAGS.items():
            self.log.tag_configure(tag, foreground=col)
        self.log.tag_configure("emerg", foreground=LOG_TAGS["emerg"], font=("Consolas", 10, "bold"))
        self.log.tag_configure("fault", foreground=LOG_TAGS["fault"], font=("Consolas", 10, "bold"))

    # ---- serial ----------------------------------------------------------
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
        except Exception as e:       # noqa: BLE001
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
        except Exception:            # noqa: BLE001
            pass
        self.ser = None
        self.conn_btn.configure(text="Connect")
        self.link_lbl.configure(text="● Disconnected", fg=MUTED)

    def _reader(self):
        buf = b""
        ser = self.ser
        while not self.stop_evt.is_set():
            try:
                data = ser.read(ser.in_waiting or 1)
            except Exception as e:   # noqa: BLE001
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
        text = f"S{s},M{m},L{l},T{t}"
        if self.demo_iter is not None:
            messagebox.showinfo("Demo", "Stop the demo first, or use it as is (uses these values).")
            return
        if self.ser is None:
            messagebox.showwarning("Not connected", "Connect to the board first (or use Demo).")
            return
        if self.model.state != "IDLE" and not messagebox.askyesno(
                "Board not in IDLE", "The board only accepts a config in IDLE.\nSend anyway?"):
            return
        self.ser.write((text + "\n").encode("ascii"))
        self._log("TX> " + text, "tx")

    # ---- demo ------------------------------------------------------------
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
        self.link_lbl.configure(text="● Demo mode", fg="#2563eb")
        self._demo_tick()

    def _stop_demo(self):
        if self.demo_job:
            self.after_cancel(self.demo_job)
        self.demo_job = None
        self.demo_iter = None
        self.demo_btn.configure(text="Demo")
        if self.ser is None:
            self.link_lbl.configure(text="● Disconnected", fg=MUTED)

    def _demo_tick(self):
        """Schedule the next demo line (each tuple is: delay-before, line)."""
        try:
            delay, line = next(self.demo_iter)
        except StopIteration:
            self._stop_demo()
            return

        def emit():
            self.q.put(("rx", line))
            self._demo_tick()
        self.demo_job = self.after(int(max(delay, 0.02) * 1000), emit)

    # ---- log / refresh ---------------------------------------------------
    def _log(self, text, tag="plain"):
        ts = datetime.datetime.now().strftime("%H:%M:%S")
        self.log.insert("end", f"{ts}  {text}\n", tag)
        if int(self.log.index("end-1c").split(".")[0]) > 3000:
            self.log.delete("1.0", "500.0")
        if self.auto_var.get():
            self.log.see("end")

    def _save_log(self):
        p = filedialog.asksaveasfilename(defaultextension=".txt", filetypes=[("Text", "*.txt")],
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
            self._refresh_ui()
        self.after(50, self._poll)

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

        for k in SIZES:
            big, pb, rj = self.cards[k]
            big.configure(text=f"{m.accepted[k]} / {m.target[k]}")
            pb.configure(maximum=max(m.target[k], 1), value=min(m.accepted[k], max(m.target[k], 1)))
            rj.configure(text=f"Rejected: {m.rejected[k]}")

        for i, lb in enumerate(self.chips):
            if m.state not in ACTIVE or m.state == "RUNNING" and m.stage == 0:
                lb.configure(bg="#e5e7eb", fg=MUTED)
            elif i < m.stage:
                lb.configure(bg="#16a34a", fg="white")
            elif i == m.stage:
                lb.configure(bg="#f59e0b", fg="white")
            else:
                lb.configure(bg="#e5e7eb", fg=MUTED)
        self.pkg_lbl.configure(text=f"Size: {m.last_size or '-'}    Decision: {m.last_decision or '-'}")

        self.hist.delete("all")
        for i, (sz, dec) in enumerate(m.history):
            x = 2 + i * 24
            self.hist.create_rectangle(x, 4, x + 20, 26, fill=SIZE_COLOR[sz], outline="")
            self.hist.create_text(x + 10, 15, text=sz, fill="white", font=("Segoe UI", 9, "bold"))
            if dec == "REJECT":
                self.hist.create_line(x, 4, x + 20, 26, fill="black", width=2)
                self.hist.create_line(x + 20, 4, x, 26, fill="black", width=2)

        for k in ("lost", "unexpected", "stuck"):
            self.rep_vars[k].set(str(m.errors[k]))
        total_in = sum(m.accepted.values()) + sum(m.rejected.values()) + sum(m.errors.values())
        rp = m.report
        self.rep_vars["input"].set(rp.get("input", str(total_in) if total_in else "-"))
        self.rep_vars["total_target"].set(rp.get("total_target", str(sum(m.target.values())) if sum(m.target.values()) else "-"))
        self.rep_vars["elapsed"].set(rp["elapsed"] + " s" if "elapsed" in rp else "-")
        for k in ("completion", "efficiency", "reject_rate", "loss"):
            self.rep_vars[k].set(rp[k] + " %" if k in rp else "-")

    def _on_close(self):
        self._stop_demo()
        self._disconnect()
        self.destroy()


if __name__ == "__main__":
    App().mainloop()
