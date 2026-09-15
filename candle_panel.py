#!/usr/bin/env python3
"""
candle_panel.py

A bench remote for the candelabra rig. Drives the transmitter's serial line
directly, and optionally holds a second port open on a par node so the colour
trims can be turned while you watch.

    pip3 install pyserial
    python3 candle_panel.py

Nothing here talks MIDI. It writes the same F-lines candle_bridge.py writes,
so the two cannot share a port — close the bridge before opening the
transmitter here, or you will get "resource busy".

Layout, left to right:
    fixture and preset      what to address and what it does
    sliders                 master, speed, depth, warmth, fade, hue, sat, boost
    palettes                the octave colours as buttons
    node tuning             ww / wa / gt / tr, over the air by default

The trims ride fixture codes 90-93 through the transmitter, so the par node
does not need its own cable. Untick "over the air" to drive them down a
direct USB line to the node instead.
"""

import json
import os
import queue
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.exit("pip3 install pyserial")


BAUD = 115200
SEND_HZ = 25                      # coalesce slider spam to this rate
PRESET_FILE = os.path.expanduser("~/.candle_panel_presets.json")


# ---------------------------------------------------------------- fixtures

FIXTURES = [
    ("F0   both candelabras", 0),
    ("F1   candelabra A", 1),
    ("F2   candelabra B", 2),
    ("F40  all pars", 40),
    ("F41  par 1", 41),
    ("F42  par 2", 42),
    ("F43  par 3", 43),
    ("F44  par 4", 44),
    ("F45  pars 1+2", 45),
    ("F46  pars 3+4", 46),
    ("F47  inside 1+4", 47),
    ("F48  outside 2+3", 48),
]

PRESETS = [("blackout", 0), ("steady", 1), ("flicker", 2), ("gust", 3),
           ("ember", 4), ("pulse", 5), ("strobe", 6)]

# name -> (hue, sat, warmth, master_ceiling, boost)
PALETTES = [
    ("deep ember",   20, 234, 255,  27,   0),
    ("candle flame",  0,   0,   0, 255,   0),
    ("red",         255, 255,   0, 255,   0),
    ("orange",       17, 255,   0, 255,   0),
    ("yellow",       34, 255,   0, 255,   0),
    ("green",        73, 242,   0, 255,   0),
    ("cyan",        125, 236,   0, 255,   0),
    ("blue",        160, 255,   0, 255,   0),
    ("violet",      186, 252,   0, 255,   0),
    ("magenta",     223, 255,   0, 255,   0),
    ("full white",    0,   0,   0, 255, 255),
]

SLIDERS = [
    ("master", "M", 0, 255, 255),
    ("speed",  "S", 0, 255, 170),
    ("depth",  "D", 0, 255, 220),
    ("warmth", "W", 0, 255,   0),
    ("fade",   "T", 0, 255,   0),
    ("hue",    "H", 0, 255,   0),
    ("sat",    "C", 0, 255,   0),
    ("boost",  "B", 0, 255,   0),
]

# The trims as they were before any of this existed: white die at full, no
# amber, no output trim. Useful as an A/B — it is what the par looked like
# when it read stark and white beside a candle.
UNTRIMMED = {"ww": 1.00, "wa": 0.00, "gt": 0.88, "tf": 1.00,
             "tc": 1.00, "dg": 1.00}

# label, serial key, over-the-air fixture code, lo, hi, default
TUNING = [
    ("ww  white die share",  "ww", 90, 0.0, 1.0, 0.621),
    ("wa  amber share",      "wa", 91, 0.0, 1.0, 0.974),
    ("gt  green trim",       "gt", 92, 0.5, 1.0, 0.88),
    ("tf  flame trim",       "tf", 93, 0.0, 1.0, 0.293),
    ("tc  colour trim",      "tc", 94, 0.0, 1.0, 1.000),
    ("dg  dimmer curve",     "dg", 95, 0.3, 1.0, 0.469),
]


# ---------------------------------------------------------------- serial

class Link:
    """One serial port, opened in a worker thread so the UI never blocks."""

    def __init__(self, log):
        self.ser = None
        self.log = log
        self.q = queue.Queue()
        self.stop = False
        threading.Thread(target=self._pump, daemon=True).start()

    def open(self, port):
        self.close()
        try:
            self.ser = serial.Serial(port, BAUD, timeout=0)
            self.log(f"opened {port}, waiting for boot")
            time.sleep(2)               # ESP32 resets when the port opens
            self.log(f"ready on {port}")
            return True
        except Exception as e:
            self.ser = None
            self.log(f"could not open {port}: {e}")
            return False

    def close(self):
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def send(self, line):
        self.q.put(line)

    def _pump(self):
        while not self.stop:
            try:
                line = self.q.get(timeout=0.1)
            except queue.Empty:
                continue
            if not self.ser:
                continue
            try:
                self.ser.write((line + "\n").encode())
                self.log("> " + line)
            except Exception as e:
                self.log(f"write failed: {e}")
                self.ser = None

    def read_pending(self):
        """Whatever the board has said since last time. Called from the UI."""
        if not self.ser:
            return ""
        try:
            n = self.ser.in_waiting
            if n:
                return self.ser.read(n).decode(errors="replace")
        except Exception:
            self.ser = None
        return ""


def ports():
    return [p.device for p in serial.tools.list_ports.comports()]


# ---------------------------------------------------------------- app

class Panel:

    def __init__(self, root):
        self.root = root
        root.title("candelabra panel")

        self.logq = queue.Queue()
        self.tx = Link(self.logq.put)
        self.node = Link(self.logq.put)

        self.fixture = tk.IntVar(value=0)
        self.preset = tk.IntVar(value=1)
        self.vals = {k: tk.IntVar(value=d) for k, _, _, _, d in SLIDERS}
        # A palette's master ceiling is a MULTIPLIER on top of the master
        # fader, exactly as it is in candle_bridge.py — not a replacement for
        # it. Deep ember is inherently dim wherever it is played, and the
        # fader keeps whatever you set it to.
        self.pmaster = tk.IntVar(value=255)
        self.tune = {k: tk.DoubleVar(value=d) for _, k, _, _, _, d in TUNING}
        # Trims can go over the air through the transmitter, which means the
        # node does not need its own cable. Fixture codes 90-93 carry the
        # value in the master field and light nothing.
        self.tune_ota = tk.BooleanVar(value=True)
        self.mirror = tk.BooleanVar(value=True)

        self.pending = False
        self.last_send = 0.0

        self._build()
        self.root.after(60, self._tick)

    # ------------------------------------------------------------ layout

    def _build(self):
        pad = dict(padx=6, pady=3)

        # Everything lives inside a scrolling canvas, because the full panel is
        # taller than a laptop screen and a cut-off window hides controls with
        # no hint that they are there.
        self.root.geometry("980x760")
        self.root.minsize(820, 420)

        outer = ttk.Frame(self.root)
        outer.pack(fill="both", expand=True)

        canvas = tk.Canvas(outer, highlightthickness=0)
        vbar = ttk.Scrollbar(outer, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=vbar.set)
        vbar.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)

        body = ttk.Frame(canvas)
        win = canvas.create_window((0, 0), window=body, anchor="nw")
        body.bind("<Configure>",
                  lambda _e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind("<Configure>",
                    lambda e: canvas.itemconfigure(win, width=e.width))

        def wheel(e):
            canvas.yview_scroll(-1 * (e.delta if abs(e.delta) < 20
                                      else e.delta // 120), "units")
        canvas.bind_all("<MouseWheel>", wheel)
        canvas.bind_all("<Button-4>", lambda _e: canvas.yview_scroll(-1, "units"))
        canvas.bind_all("<Button-5>", lambda _e: canvas.yview_scroll(1, "units"))

        root = body

        # ---- ports
        bar = ttk.LabelFrame(root, text="serial")
        bar.grid(row=0, column=0, columnspan=3, sticky="ew", **pad)

        ttk.Label(bar, text="transmitter").grid(row=0, column=0, sticky="w", padx=4)
        self.tx_port = ttk.Combobox(bar, values=ports(), width=28)
        self.tx_port.grid(row=0, column=1, padx=4, pady=3)
        ttk.Button(bar, text="open",
                   command=lambda: self.tx.open(self.tx_port.get())
                   ).grid(row=0, column=2, padx=4)

        ttk.Label(bar, text="par node").grid(row=1, column=0, sticky="w", padx=4)
        self.nd_port = ttk.Combobox(bar, values=ports(), width=28)
        self.nd_port.grid(row=1, column=1, padx=4, pady=3)
        ttk.Button(bar, text="open",
                   command=lambda: self.node.open(self.nd_port.get())
                   ).grid(row=1, column=2, padx=4)
        ttk.Button(bar, text="rescan", command=self._rescan
                   ).grid(row=1, column=3, padx=4)

        # ---- fixture + preset
        left = ttk.Frame(root)
        left.grid(row=1, column=0, sticky="n", **pad)

        fx = ttk.LabelFrame(left, text="fixture")
        fx.pack(fill="x")
        for label, code in FIXTURES:
            ttk.Radiobutton(fx, text=label, value=code, variable=self.fixture,
                            command=self.mark).pack(anchor="w", padx=4)

        ps = ttk.LabelFrame(left, text="preset")
        ps.pack(fill="x", pady=(8, 0))
        for label, code in PRESETS:
            ttk.Radiobutton(ps, text=label, value=code, variable=self.preset,
                            command=self.mark).pack(anchor="w", padx=4)

        # ---- sliders
        mid = ttk.LabelFrame(root, text="parameters")
        mid.grid(row=1, column=1, sticky="n", **pad)

        for r, (name, key, lo, hi, default) in enumerate(SLIDERS):
            ttk.Label(mid, text=name, width=7).grid(row=r, column=0, sticky="w", padx=4)
            v = self.vals[name]
            ttk.Scale(mid, from_=lo, to=hi, variable=v, length=200,
                      command=lambda _e, vv=v: self.mark(snap=vv)
                      ).grid(row=r, column=1, padx=4, pady=2)
            e = ttk.Entry(mid, textvariable=v, width=5, justify="right")
            e.grid(row=r, column=2, padx=(2, 4))
            # Type a number and press return, or just click away.
            e.bind("<Return>",   lambda _e, vv=v, a=lo, b=hi: self.typed(vv, a, b))
            e.bind("<FocusOut>", lambda _e, vv=v, a=lo, b=hi: self.typed(vv, a, b))

        ttk.Label(mid, text="ceiling").grid(row=len(SLIDERS), column=0,
                                           sticky="w", padx=4)
        ttk.Label(mid, textvariable=self.pmaster, width=4
                  ).grid(row=len(SLIDERS), column=2)
        ttk.Label(mid, text="palette multiplier on master, 255 = none",
                  foreground="#777").grid(row=len(SLIDERS), column=1, sticky="w")

        ttk.Checkbutton(mid, text="mirror to F40 as well  (match test)",
                        variable=self.mirror
                        ).grid(row=len(SLIDERS) + 1, column=0, columnspan=3,
                               sticky="w", padx=4, pady=(6, 2))

        bb = ttk.Frame(mid)
        bb.grid(row=len(SLIDERS) + 2, column=0, columnspan=3, sticky="w", padx=4)
        ttk.Button(bb, text="defaults", command=self.defaults).pack(side="left", padx=2)
        ttk.Button(bb, text="blackout", command=self.blackout).pack(side="left", padx=2)
        ttk.Button(bb, text="resend", command=self.send_now).pack(side="left", padx=2)

        # ---- palettes
        right = ttk.Frame(root)
        right.grid(row=1, column=2, sticky="n", **pad)

        pl = ttk.LabelFrame(right, text="palette")
        pl.pack(fill="x")
        for name, h, s, w, m, b in PALETTES:
            ttk.Button(pl, text=name, width=16,
                       command=lambda h=h, s=s, w=w, m=m, b=b:
                           self.palette(h, s, w, m, b)
                       ).pack(padx=4, pady=1)

        tn = ttk.LabelFrame(right, text="par node tuning")
        tn.pack(fill="x", pady=(8, 0))
        ttk.Checkbutton(tn, text="over the air  (no node cable needed)",
                        variable=self.tune_ota
                        ).grid(row=0, column=0, columnspan=2, sticky="w", padx=4)
        for r, (label, key, code, lo, hi, default) in enumerate(TUNING):
            ttk.Label(tn, text=label).grid(row=r * 2 + 1, column=0,
                                          sticky="w", padx=4)
            v = self.tune[key]
            ttk.Scale(tn, from_=lo, to=hi, variable=v, length=140,
                      command=lambda _e, k=key, c=code: self.tune_send(k, c)
                      ).grid(row=r * 2 + 2, column=0, padx=4)
            e = ttk.Entry(tn, textvariable=v, width=6, justify="right")
            e.grid(row=r * 2 + 2, column=1, padx=(2, 4))
            e.bind("<Return>",   lambda _e, k=key, c=code: self.tune_send(k, c))
            e.bind("<FocusOut>", lambda _e, k=key, c=code: self.tune_send(k, c))
        tb = ttk.Frame(tn)
        tb.grid(row=len(TUNING) * 2 + 1, column=0, columnspan=2,
                sticky="w", padx=4, pady=4)
        ttk.Button(tb, text="resend all", command=self.tune_all).pack(side="left")
        ttk.Button(tb, text="copy values", command=self.copy_tuning
                   ).pack(side="left", padx=4)
        tb2 = ttk.Frame(tn)
        tb2.grid(row=len(TUNING) * 2 + 2, column=0, columnspan=2,
                 sticky="w", padx=4, pady=(0, 4))
        ttk.Button(tb2, text="defaults", command=self.tune_defaults
                   ).pack(side="left")
        ttk.Button(tb2, text="no trim", command=self.tune_untrimmed
                   ).pack(side="left", padx=4)

        # ---- presets
        pr = ttk.LabelFrame(root, text="presets")
        pr.grid(row=2, column=0, columnspan=3, sticky="ew", **pad)

        self.preset_name = ttk.Entry(pr, width=24)
        self.preset_name.grid(row=0, column=0, padx=4, pady=4)
        ttk.Button(pr, text="save", command=self.preset_save
                   ).grid(row=0, column=1, padx=2)
        self.preset_pick = ttk.Combobox(pr, width=24, state="readonly")
        self.preset_pick.grid(row=0, column=2, padx=8)
        ttk.Button(pr, text="load", command=self.preset_load
                   ).grid(row=0, column=3, padx=2)
        ttk.Button(pr, text="delete", command=self.preset_delete
                   ).grid(row=0, column=4, padx=2)
        ttk.Label(pr, text=f"stored in {PRESET_FILE}", foreground="#777"
                  ).grid(row=1, column=0, columnspan=5, sticky="w", padx=6)
        self._preset_refresh()

        # ---- raw + log
        bot = ttk.LabelFrame(root, text="raw line and log")
        bot.grid(row=3, column=0, columnspan=3, sticky="ew", **pad)

        self.raw = ttk.Entry(bot, width=60)
        self.raw.grid(row=0, column=0, padx=4, pady=4, sticky="w")
        self.raw.bind("<Return>", lambda _e: self.send_raw())
        ttk.Button(bot, text="to transmitter", command=self.send_raw
                   ).grid(row=0, column=1, padx=2)
        ttk.Button(bot, text="to node",
                   command=lambda: self.send_raw(node=True)
                   ).grid(row=0, column=2, padx=2)

        self.logbox = tk.Text(bot, height=7, width=92, font=("Courier", 9))
        self.logbox.grid(row=1, column=0, columnspan=3, padx=4, pady=4)

    def _rescan(self):
        self.tx_port["values"] = ports()
        self.nd_port["values"] = ports()
        self.logq.put("rescanned ports")

    # ------------------------------------------------------------ sending

    def mark(self, snap=None):
        if snap is not None:
            snap.set(int(round(snap.get())))     # sliders land on integers
        self.pending = True

    def master_out(self):
        return int(round(self.vals["master"].get() * self.pmaster.get() / 255))

    def line(self, fixture):
        v = self.vals
        return (f"F{fixture} P{self.preset.get()} M{self.master_out()} "
                f"S{v['speed'].get()} D{v['depth'].get()} W{v['warmth'].get()} "
                f"T{v['fade'].get()} H{v['hue'].get()} C{v['sat'].get()} "
                f"B{v['boost'].get()}")

    def send_now(self):
        fx = self.fixture.get()
        self.tx.send(self.line(fx))
        # The match test: same look on the candles and the pars at once, so
        # any difference is the par's colour trims and nothing else.
        if self.mirror.get() and fx < 40:
            self.tx.send(self.line(40))
        self.pending = False
        self.last_send = time.time()

    def send_raw(self, node=False):
        s = self.raw.get().strip()
        if not s:
            return
        (self.node if node else self.tx).send(s)
        self.raw.delete(0, "end")

    def tune_send(self, key, code):
        v = self.tune[key]
        v.set(round(v.get(), 3))
        if self.tune_ota.get():
            # Master carries the value, 0-255 mapped to 0.0-1.0. The node
            # sets the variable and returns without touching any par.
            self.tx.send(f"F{code} M{int(round(v.get() * 255))}")
        else:
            self.node.send(f"{key} {v.get():.3f}")

    def typed(self, var, lo, hi):
        """A number was typed into an entry box. Clamp it and send."""
        try:
            v = float(var.get())
        except Exception:
            return                       # mid-edit or nonsense, leave it
        var.set(int(round(max(lo, min(hi, v)))))
        self.pending = True

    def copy_tuning(self):
        vals = "  ".join(f"{k} {self.tune[k].get():.3f}"
                         for _l, k, _c, _lo, _hi, _d in TUNING)
        self.root.clipboard_clear()
        self.root.clipboard_append(vals)
        self.logq.put("copied: " + vals)

    # ---- presets ----------------------------------------------------

    def _presets(self):
        try:
            with open(PRESET_FILE) as f:
                return json.load(f)
        except Exception:
            return {}

    def _preset_refresh(self):
        names = sorted(self._presets())
        self.preset_pick["values"] = names
        if names and not self.preset_pick.get():
            self.preset_pick.set(names[0])

    def preset_save(self):
        name = self.preset_name.get().strip()
        if not name:
            self.logq.put("name the preset first")
            return
        data = self._presets()
        data[name] = {
            "fixture": self.fixture.get(),
            "preset":  self.preset.get(),
            "vals":    {k: self.vals[k].get() for k in self.vals},
            "pmaster": self.pmaster.get(),
            "tune":    {k: self.tune[k].get() for k in self.tune},
        }
        try:
            with open(PRESET_FILE, "w") as f:
                json.dump(data, f, indent=2, sort_keys=True)
        except Exception as e:
            self.logq.put(f"save failed: {e}")
            return
        self._preset_refresh()
        self.preset_pick.set(name)
        self.logq.put(f"saved '{name}'")

    def preset_load(self):
        name = self.preset_pick.get()
        d = self._presets().get(name)
        if not d:
            return
        self.fixture.set(d.get("fixture", 0))
        self.preset.set(d.get("preset", 1))
        self.pmaster.set(d.get("pmaster", 255))
        for k, v in d.get("vals", {}).items():
            if k in self.vals:
                self.vals[k].set(v)
        for k, v in d.get("tune", {}).items():
            if k in self.tune:
                self.tune[k].set(v)
        self.preset_name.delete(0, "end")
        self.preset_name.insert(0, name)
        self.tune_all()          # push the trims to the node
        self.mark()              # and the look to the fixtures
        self.logq.put(f"loaded '{name}'")

    def preset_delete(self):
        name = self.preset_pick.get()
        data = self._presets()
        if name in data:
            del data[name]
            with open(PRESET_FILE, "w") as f:
                json.dump(data, f, indent=2, sort_keys=True)
            self.preset_pick.set("")
            self._preset_refresh()
            self.logq.put(f"deleted '{name}'")

    def tune_defaults(self):
        """Back to whatever is compiled into the firmware."""
        for _l, k, _c, _lo, _hi, d in TUNING:
            self.tune[k].set(d)
        self.tune_all()
        self.logq.put("trims -> firmware defaults")

    def tune_untrimmed(self):
        """Back to the original behaviour: full white die, no amber, no output
        trim. The A/B for judging whether the trims are helping."""
        for k, v in UNTRIMMED.items():
            self.tune[k].set(v)
        self.tune_all()
        self.logq.put("trims -> untrimmed (original)")

    def tune_all(self):
        """Restate every trim. Useful after the node reboots, since nothing
        here is stored on the board."""
        for _label, key, code, _lo, _hi, _d in TUNING:
            self.tune_send(key, code)

    def palette(self, h, s, w, m, b):
        # Every palette sets every field it owns, so clicking one never
        # inherits a leftover from the last.
        self.vals["hue"].set(h)
        self.vals["sat"].set(s)
        self.vals["warmth"].set(w)
        self.vals["boost"].set(b)
        self.pmaster.set(m)
        self.mark()

    def defaults(self):
        for name, _k, _lo, _hi, d in SLIDERS:
            self.vals[name].set(d)
        self.pmaster.set(255)
        self.preset.set(2)
        self.mark()

    def blackout(self):
        self.preset.set(0)
        self.vals["master"].set(0)
        self.mark()

    # ------------------------------------------------------------ loop

    def _tick(self):
        now = time.time()
        if self.pending and (now - self.last_send) >= (1.0 / SEND_HZ):
            self.send_now()

        for link in (self.tx, self.node):
            txt = link.read_pending()
            if txt:
                for ln in txt.splitlines():
                    if ln.strip():
                        self.logq.put("< " + ln.strip())

        while True:
            try:
                self.logbox.insert("end", self.logq.get_nowait() + "\n")
            except queue.Empty:
                break
            self.logbox.see("end")

        self.root.after(40, self._tick)


def main():
    root = tk.Tk()
    Panel(root)
    root.mainloop()


if __name__ == "__main__":
    main()
