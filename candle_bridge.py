#!/usr/bin/env python3
"""
candle_bridge.py

Ableton -> IAC virtual MIDI port -> this script -> USB serial -> transmitter ESP32

Built for a pre-rendered set: automation lanes fire CC far faster than the
serial link can carry, so changes are coalesced and sent at a fixed rate
rather than one message per MIDI event.

    pip3 install mido python-rtmidi pyserial

    python3 candle_bridge.py                 # auto-detect, or prompts
    python3 candle_bridge.py --list          # show ports and exit
    python3 candle_bridge.py --midi "IAC Driver Bus 1" --serial /dev/cu.usbserial-0001


MAPPING
-------

MIDI channel picks what the message addresses:
    ch 1   both candelabras
    ch 2   candelabra A          ch 3   candelabra B
    ch 4   A candle 1            ch 7   B candle 1
    ch 5   A candle 2            ch 8   B candle 2
    ch 6   A candle 3            ch 9   B candle 3

Each target keeps its own held notes and its own parameters. A group
command and a per-candle command simply overwrite whatever they address —
last received wins, with no priority and no modal state. In practice that
means: end a channel 1 note before a per-candle section starts, and don't
restart it until those notes have released.

Notes pick the preset AND the colour. They are GATED, not latched: the
look is active for as long as the note is held, and releasing it returns
to blackout. Draw a note as long as you want the effect to last.
Overlapping notes stack, and releasing one falls back to whichever is
still held. Pass --latch if you'd rather notes stick until the next note.

    C  blackout    D  steady    E  flicker    F  gust
    G  ember       A  pulse     B  strobe

    The OCTAVE picks the colour palette — see PALETTES below. Black keys
    do nothing, so the piano roll stays readable.

    Note VELOCITY scales CC 14 for that trigger. CC 14 is the master fader,
    velocity is the level of the note on top of it, and the two multiply.
    So CC 14 at 0 stays dark no matter how hard the note is struck.

    A palette entry may also carry its own master ceiling as a fourth value,
    which multiplies in on top. That is how C-2 ember stays dim wherever it
    is played.

    CC numbers to avoid: 0 and 32 (bank select), 1 (mod wheel - DAWs reset
    it), 6 and 38 (data entry), 7 (volume), 10 and 11 (pan, expression),
    96-101 (RPN/NRPN), 120-127 (channel mode). 14, 15 and 20-31 are
    undefined in the spec and safe.

    Because every note-on carries a complete state, locating anywhere in
    the set lands somewhere defined. This is the reason colour lives on
    notes rather than only on envelopes: Live only emits a CC when its
    value changes, but a note-on always transmits.

CCs set parameters (0-127, scaled to 0-255):
    CC 14  master brightness
    CC 2   speed
    CC 3   depth
    CC 4   warmth
    CC 5   crossfade time into the next preset
    CC 12  hue OFFSET from the note's palette colour. 64 is the centre
           and changes nothing; 0 and 127 are half a wheel either way.
    CC 15  white boost. 0 is the flame as always, 127 lights all four
           dies at full. The only route to maximum output.
    CC 13  saturation OFFSET from the palette. 64 is the centre;
           127 is fully saturated hue. Ride this to bloom a flame into
           stage colour without changing preset.

Nothing held  ->  blackout, while the set is running.

TRANSPORT
---------
If Ableton is sending MIDI clock to the same port, stopping or pausing the
set drops the candles to an idle gusting flicker rather than blackout, and
starting again hands control back to the notes. Enable this in Ableton:
Settings > Link/MIDI > the IAC output row > turn Sync ON.

Without clock enabled, the same thing happens after a few seconds of total
MIDI silence, just less promptly.

There is a third, separate mechanism: if this script or the transmitter
stops entirely, the candelabras fall back to their own idle flicker from
the receiver firmware. All three failure modes land on a lit candle.
"""

import argparse
import sys
import time

try:
    import mido
except ImportError:
    sys.exit("mido not installed.  pip3 install mido python-rtmidi")

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.exit("pyserial not installed.  pip3 install pyserial")


# ------------------------------------------------------------------ config

BAUD          = 115200
SEND_INTERVAL = 0.02    # 50 messages/sec ceiling. Automation can fire far
                        # faster than that; we coalesce and send the latest.
HEARTBEAT     = 0.5     # restate current values this often

# What to show when the set is paused or stopped, or when Ableton has been
# quiet for a while. Matches the receiver's own fallback, so a pause and a
# dropped link look the same.
IDLE_PRESET = 3         # gust
IDLE_MASTER = 255
IDLE_SPEED  = 170
IDLE_DEPTH  = 220
IDLE_WARMTH = 0
IDLE_SAT    = 0         # idle is always real candle colour
IDLE_FADE   = 80        # 0.8s ease into the idle look
IDLE_AFTER  = 5.0       # seconds of total MIDI silence before idling anyway

# ---------------------------------------------------------------- note map
#
# C major scale = the seven presets. Same shape in every octave.
#   C blackout   D steady   E flicker   F gust   G ember   A pulse   B strobe
#
# The OCTAVE picks a colour palette. Transpose a phrase up an octave and
# the same lighting recolours. Black keys are unused, so the piano roll
# reads at a glance.
#
# Every note-on transmits a complete state — preset AND colour — so
# locating anywhere in the set lands somewhere defined. CC 12 and 13 still
# modulate hue and saturation on top of whatever a note established.

PRESET_NAMES = ["blackout", "steady", "flicker", "gust",
                "ember", "pulse", "strobe"]

# semitone offset within an octave -> preset index
SCALE = {0: 0, 2: 1, 4: 2, 5: 3, 7: 4, 9: 5, 11: 6}

# Octave number (Ableton naming, where note 60 = C3) -> (hue, sat, warmth).
# hue 0-255 round the wheel in six sectors: 0 red, 43 yellow, 85 green,
# 128 cyan, 170 blue, 213 magenta.
# sat 0 = real candle flame, 255 = fully saturated colour.
# EDIT THIS TABLE. It is the whole palette.
PALETTES = {
    #  hue  sat  warmth [ master ]
    #  master is optional, defaults to 255. Use it when an octave should
    #  always play dim no matter how hard the note is struck.
    -2: ( 20, 234, 255,  27),   # C-2  deep ember
    -1: (  0,   0,   0),   # C-1  HOME - true candle flame
     0: (255, 255,   0),   # C0   red      (hue 255 wraps to 0)
     1: ( 17, 255,   0),   # C1   orange
     2: ( 34, 255,   0),   # C2   yellow
     3: ( 73, 242,   0),   # C3   green
     4: (125, 236,   0),   # C4   cyan
     5: (160, 255,   0),   # C5   blue
     6: (186, 252,   0),   # C6   violet
     7: (223, 255,   0),   # C7   magenta
     8: (  0,   0,   0, 255, 255),   # C8   full white - all four dies at max
}
DEFAULT_PALETTE = (0, 0, 0, 255, 0)

def note_to_look(note):
    """MIDI note -> (preset, hue, sat, warmth, pmaster, boost), or None if
    the note is not in C major. Palette entries may be 3 to 5 long. A missing
    fourth value means no master ceiling (255); a missing fifth means no
    boost (0)."""
    semitone = note % 12
    if semitone not in SCALE:
        return None
    octave = (note // 12) - 2          # note 60 -> octave 3
    entry  = PALETTES.get(octave, DEFAULT_PALETTE)
    hue, sat, warmth = entry[0], entry[1], entry[2]
    pmaster = entry[3] if len(entry) > 3 else 255
    boost   = entry[4] if len(entry) > 4 else 0
    return SCALE[semitone], hue, sat, warmth, pmaster, boost

# Master lived on CC 1 and kept getting zeroed at transport start. CC 1 is
# Mod Wheel, the most reset-happy controller there is - DAWs zero it,
# controllers emit it on connect, and clips inherit it. CC 14 is undefined
# in the spec, so nothing touches it unasked.
CC_MASTER = 14
CC_SPEED, CC_DEPTH, CC_WARMTH, CC_FADE = 2, 3, 4, 5
CC_BOOST = 15      # also undefined in the spec
# Hue and saturation used to sit on CC 6 and 7. Don't put them back there:
# CC 6 is Data Entry MSB and CC 7 is Channel Volume, both reserved, and Live
# swallows them instead of passing them through. Every other CC worked while
# those two silently did nothing.
#
# Steer clear of: 0 and 32 (bank select), 6 and 38 (data entry), 7 (volume),
# 10 and 11 (pan, expression), 96-101 (RPN/NRPN), 120-127 (channel mode).
# 12 and 13 are Effect Control 1 and 2, ordinary controllers that pass
# through cleanly. If you need more later, 20-31 are undefined outright.
CC_HUE, CC_SAT = 12, 13

# Which State attribute each CC writes to. Drives both the dispatch and the
# no-change check, so the two can never drift apart.
# CC_HUE and CC_SAT are handled separately, as signed offsets.
CC_FIELD = {
    CC_MASTER: "level",
    CC_SPEED:  "speed",
    CC_DEPTH:  "depth",
    CC_WARMTH: "warmth",
    CC_FADE:   "fade",
    CC_BOOST:  "boost",
}


def scale(v):
    """MIDI 0-127 -> 0-255."""
    return min(255, int(v * 255 / 127))


# ------------------------------------------------------------------ targets

# MIDI channel (0-based, as mido reports it) -> fixture code in the packet.
#   ch 1  both candelabras          ch 4-6  fixture A, candles 1-3
#   ch 2  fixture A                 ch 7-9  fixture B, candles 1-3
#   ch 3  fixture B
# Each target keeps its own held notes and its own parameters, so a group
# command and a per-candle command simply overwrite whatever they address.
# Last received wins.
CHAN_TARGET = {0: 0, 1: 1, 2: 2,
               3: 11, 4: 12, 5: 13,
               6: 21, 7: 22, 8: 23}

TARGET_NAMES = {0: "all", 1: "A", 2: "B",
                11: "A1", 12: "A2", 13: "A3",
                21: "B1", 22: "B2", 23: "B3"}

# Addresses nothing. The receivers stamp their watchdog before checking the
# fixture code, so this proves the link is alive without changing any look.
HEARTBEAT_LINE = b"F99\n" 


# ------------------------------------------------------------------ state

class State:
    def __init__(self, fixture=0):
        self.fixture = fixture
        self.preset  = 0      # blackout until a note is held
        # Master is two independent things multiplied together:
        #   level  CC 1, the master fader for this target
        #   vel    the velocity of the note currently sounding
        # Velocity used to overwrite master outright, which made a note-on
        # flash at full before the next CC 14 pulled it back down.
        self.level   = 255
        self.vel     = 255
        # Ceiling carried by the palette entry, so an octave can be
        # inherently dim. 255 means no ceiling.
        self.pmaster = 255
        self.speed   = 170
        self.depth   = 220
        self.warmth  = 0
        self.fade    = 0
        # Colour is the note's palette entry plus a signed offset from the
        # CC. The CC is bipolar: 64 is the centre and means "leave the
        # palette alone", below pushes one way, above the other. That way a
        # hue envelope moves away from whatever colour the note established,
        # in either direction, instead of replacing it outright.
        self.pal_hue = 0
        self.pal_sat = 0      # 0 = candle flame, 255 = saturated colour
        self.hue_off = 0      # -128..126, wraps round the wheel
        self.sat_off = 0      # -256..252, clamped
        # Nothing in the normal colour path lights all four dies at once, so
        # boost blends toward that. 0 is the flame as always, 255 is maximum
        # output. Carried as a fifth palette value or ridden on CC 15.
        self.boost   = 0
        self.dirty   = True   # something changed since last send

        # note -> velocity, in the order they were pressed
        self.held    = {}

        self.idling  = False

    def go_idle(self):
        if self.idling:
            return
        self.idling = True
        self.preset = IDLE_PRESET
        self.level   = IDLE_MASTER
        self.vel     = 255
        self.pmaster = 255
        self.speed  = IDLE_SPEED
        self.depth  = IDLE_DEPTH
        self.warmth = IDLE_WARMTH
        self.pal_sat = IDLE_SAT
        self.sat_off = 0
        self.hue_off = 0
        self.boost   = 0
        self.fade   = IDLE_FADE
        self.dirty  = True

    def resolve(self, latch):
        """Preset and colour follow the most recently pressed note still
        held. Nothing held means blackout, unless we're latching."""
        self.idling = False
        if self.held:
            note = list(self.held)[-1]
            look = note_to_look(note)
            if look:
                (self.preset, self.pal_hue, self.pal_sat,
                 self.warmth, self.pmaster, self.boost) = look
                self.vel = scale(self.held[note])
        elif not latch:
            self.preset = 0        # blackout
        self.dirty = True

    @property
    def master(self):
        return (self.level * self.vel * self.pmaster) // (255 * 255)

    @property
    def hue(self):
        return (self.pal_hue + self.hue_off) % 256

    @property
    def sat(self):
        return max(0, min(255, self.pal_sat + self.sat_off))

    def line(self):
        return (f"F{self.fixture} P{self.preset} M{self.master} "
                f"S{self.speed} D{self.depth} W{self.warmth} "
                f"T{self.fade} H{self.hue} C{self.sat} B{self.boost}\n")


# ------------------------------------------------------------------ ports

def pick_midi(name):
    ports = mido.get_input_names()
    if not ports:
        sys.exit("No MIDI inputs found.\n"
                 "On macOS: Audio MIDI Setup > Window > Show MIDI Studio,\n"
                 "double-click IAC Driver, tick 'Device is online'.")
    if name:
        for p in ports:
            if name.lower() in p.lower():
                return p
        sys.exit(f"No MIDI input matching {name!r}.  Found: {ports}")
    for p in ports:
        if "iac" in p.lower():
            return p
    print("MIDI inputs:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p}")
    return ports[int(input("choose: "))]


def pick_serial(name):
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        sys.exit("No serial ports found. Is the transmitter plugged in?")
    if name:
        return name
    # ESP32 boards show up as usbserial / SLAB / wchusb / COM
    for p in ports:
        d = p.device.lower()
        if any(k in d for k in ("usbserial", "slab", "wchusb", "usbmodem")):
            return p.device
    print("Serial ports:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device}  {p.description}")
    return ports[int(input("choose: "))].device


# ------------------------------------------------------------------ main

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--midi",   help="MIDI input port name (substring ok)")
    ap.add_argument("--serial", help="serial device path")
    ap.add_argument("--list",   action="store_true")
    ap.add_argument("--quiet",  action="store_true", help="don't echo commands")
    ap.add_argument("--latch",  action="store_true",
                    help="notes stick until the next note, instead of "
                         "returning to blackout on note-off")
    args = ap.parse_args()

    if args.list:
        print("MIDI inputs:")
        for p in mido.get_input_names():
            print("  ", p)
        print("Serial ports:")
        for p in serial.tools.list_ports.comports():
            print("  ", p.device, "-", p.description)
        return

    midi_name = pick_midi(args.midi)
    ser_name  = pick_serial(args.serial)

    print(f"MIDI   : {midi_name}")
    print(f"serial : {ser_name}")

    ser = serial.Serial(ser_name, BAUD, timeout=0)
    time.sleep(2)          # ESP32 resets when the port opens; let it boot

    states = {code: State(code) for code in CHAN_TARGET.values()}
    st_all = states[0]                 # the F0 target, used for heartbeats
    send_order = list(states.values())
    last_send = 0.0
    last_midi = time.time()
    playing   = None       # None = unknown (no clock), True/False from transport

    print("bridge running.  ctrl-C to stop.\n")

    with mido.open_input(midi_name) as port:
        try:
            while True:
                # ---- drain every MIDI message waiting, keep only the result
                for msg in port.iter_pending():

                    if msg.type != "clock":
                        last_midi = time.time()

                    # ---- transport
                    if msg.type == "stop":
                        playing = False
                        for s in states.values():
                            s.held.clear()
                        # F0 addresses every candle, so one idle line resets
                        # anything a per-candle command left lit.
                        st_all.go_idle()
                        if not args.quiet:
                            print("  transport stop -> idle")
                        continue

                    if msg.type in ("start", "continue"):
                        playing = True
                        for s in states.values():
                            s.held.clear()
                        st_all.resolve(args.latch)
                        # Live only emits a CC when its value CHANGES, so
                        # locating into a flat stretch of envelope sends
                        # nothing. Restate everything we have so the fixtures
                        # are at least in a known state on every start.
                        st_all.dirty = True
                        last_send = 0.0        # force an immediate send
                        if not args.quiet:
                            print(f"  transport {msg.type} -> notes, resending state")
                        continue

                    if msg.type == "clock":
                        playing = True     # clock only ticks while running
                        continue

                    # The channel picks which target this message addresses.
                    # Anything outside ch 1-9 is ignored entirely.
                    if msg.type in ("note_on", "note_off", "control_change"):
                        target = CHAN_TARGET.get(msg.channel)
                        if target is None:
                            continue
                        st = states[target]

                    if msg.type == "note_on" and msg.velocity > 0:
                        look = note_to_look(msg.note)
                        if look:
                            st.held.pop(msg.note, None)   # re-press moves it
                            st.held[msg.note] = msg.velocity
                            st.resolve(args.latch)
                            if not args.quiet:
                                oct_ = (msg.note // 12) - 2
                                print(f"  on  [{TARGET_NAMES[target]}] "
                                      f"{msg.note} -> "
                                      f"{PRESET_NAMES[look[0]]} oct{oct_} "
                                      f"H{st.hue} C{st.sat} @ {st.master}")

                    elif (msg.type == "note_off" or
                          (msg.type == "note_on" and msg.velocity == 0)):
                        if msg.note in st.held:
                            del st.held[msg.note]
                            st.resolve(args.latch)
                            if not args.quiet:
                                print(f"  off [{TARGET_NAMES[target]}] "
                                      f"{msg.note} -> "
                                      f"{PRESET_NAMES[st.preset]}")

                    elif msg.type == "control_change":
                        if msg.control in (CC_HUE, CC_SAT):
                            # Bipolar. 64 is the centre and leaves the note's
                            # palette colour untouched.
                            if msg.control == CC_HUE:
                                field, val = "hue_off", (msg.value - 64) * 2
                            else:
                                field, val = "sat_off", (msg.value - 64) * 4
                        else:
                            field = CC_FIELD.get(msg.control)
                            if field is None:
                                continue
                            val = scale(msg.value)

                        # A CC that repeats its current value changes nothing,
                        # and Live emits plenty of those. Sending them anyway
                        # floods the serial link and buries real changes in
                        # the log.
                        if getattr(st, field) == val:
                            continue
                        setattr(st, field, val)
                        st.dirty = True

                # ---- nothing from Ableton for a while: idle.
                # A held note fires one note-on and then nothing, so held
                # notes must count as activity. And if clock is running the
                # set is playing, whatever else is quiet.
                any_held = any(s.held for s in states.values())
                if (not any_held and playing is not True
                        and time.time() - last_midi > IDLE_AFTER
                        and not st_all.idling):
                    st_all.go_idle()
                    if not args.quiet:
                        print("  midi silent -> idle")

                # ---- send at a fixed rate, or heartbeat if nothing changed
                now = time.time()
                dirty = [s for s in send_order if s.dirty]

                if dirty and (now - last_send) >= SEND_INTERVAL:
                    # Every dirty target gets its own line. There are at most
                    # nine, and only the ones that actually changed are sent.
                    for s in dirty:
                        ser.write(s.line().encode())
                        if not args.quiet:
                            print("  " + s.line().strip())
                        s.dirty = False
                    last_send = now

                elif (now - last_send) >= HEARTBEAT:
                    # Liveness only. F99 matches no fixture, so the packet is
                    # broadcast and both receivers stamp their watchdog, but
                    # nothing is overwritten. Restating F0 here would stomp
                    # any per-candle look twice a second.
                    ser.write(HEARTBEAT_LINE)
                    last_send = now

                time.sleep(0.002)

        except KeyboardInterrupt:
            print("\nblackout, closing")
            st_all.preset, st_all.level, st_all.fade = 0, 0, 30
            ser.write(st_all.line().encode())
            time.sleep(0.2)
            ser.close()


if __name__ == "__main__":
    main()
