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
    ch 1   both candelabras      F0
    ch 2   candelabra A          F1
    ch 3   candelabra B          F2
    ch 4   all four pars         F40
    ch 5   pars 1 and 2          F45
    ch 6   pars 3 and 4          F46
    ch 7   pars 1 and 4, inside  F47
    ch 8   pars 2 and 3, outside F48

Each target keeps its own held notes and its own parameters. Two commands
that address the same fixture simply overwrite each other — last received
wins, with no priority and no modal state. Channels 4 and 7 both reach par
1, so end one before the other starts.

LINK MODE. Holding one particular black key on a par channel makes that
group follow a candelabra exactly, for as long as it is held:

    ch 4 follows ch 1, 2 or 3 — whichever last changed
    ch 5 follows ch 1 or 2       ch 7 follows ch 1
    ch 6 follows ch 1 or 3       ch 8 follows ch 1

Listing ch 1 alongside the specific source is what makes a link still work
when both candelabras are driven together rather than separately.

Release and the group snaps back to whatever its own channel had become in
the meantime — its state never stopped tracking underneath. That makes the
release a handoff rather than a reset.

To link permanently instead, with no key to play, put the par codes in
LINK_ALWAYS below, or pass --link for all four. Their own channels then do
nothing, which is the trade.

F0 addresses the candles ONLY. If it also reached the pars, channel 1 would
drive them directly and a link note on channel 4 would mean nothing. F40 is
the par equivalent wherever "everything" is wanted.

The idle look — transport stopped, or five seconds of MIDI silence — is the
C-1 candle flame with a gust on it. It reads its colour from the palette, so
retuning the flame retunes idle as well.

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
    CC 5   crossfade time, both directions
    CC 20  crossfade IN only — the attack
    CC 21  crossfade OUT only — the release.
           Per-channel, so the candles and the pars fade independently.
           A channel you have not automated sits at the default, not at
           whatever some other channel happened to be doing. Global by default — see GLOBAL_CC — so one envelope
           covers every channel. Targets start at DEFAULT_FADE, so a
           channel with no envelope still eases rather than snapping.
    CC 12  hue OFFSET from the note's palette colour. 64 is the centre.
           Offsets reset to centre whenever a target falls to blackout and
           on transport start — see RESET_OFFSETS_ON_BLACKOUT
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
# Idle takes its COLOUR from a palette entry rather than carrying its own
# copy, so tuning the flame tunes the idle look with it. Movement — preset,
# master, speed, depth, fade — stays separate below, because idle wants to
# drift more than a played note does.
IDLE_OCTAVE = -1        # C-1, the candle flame
IDLE_FADE   = 80        # 0.8s ease into the idle look

# Every target starts here, so a channel with no CC 5 envelope still eases
# rather than snapping. Units of 10ms — 15 is 150ms, enough to take the edge
# off a note release without smearing a cue. 0 restores the old hard cut.
DEFAULT_FADE = 15

# CCs that apply to EVERY target, whichever channel they arrive on. Without
# this, an envelope drawn on channel 1 sets the fade for F0 alone and the
# other channels keep their defaults — which is the usual reason one channel
# eases and another snaps.
#   {CC_FADE}                       one fade envelope drives everything
#   {CC_FADE, CC_SPEED, CC_DEPTH}   share the movement, keep master separate
#   set()                           strictly per-channel, as before
# EMPTY on purpose. A global CC overwrites every target, which is fine until
# you want the candles and the pars fading differently — then it is a wall.
#
# The problem globals were added to solve was a channel that had never been
# sent a CC sitting on its birth default while the rest of the rig moved on.
# NOTE_REFRESHES_CC below handles that properly: an untouched channel inherits
# the newest value seen anywhere, and a channel with its own value keeps it.
# That gives inheritance without taking independence away.
#
# Put a CC number in here only if you want it truly locked across every
# target, with no way to differ.
GLOBAL_CC = set()

# Hue and saturation offsets are CCs, so they persist until something moves
# them — including across a gap where nothing is held. Leave a sweep sitting
# at 90 and every note after it is shifted a quarter turn, until a later clip
# happens to send CC 12 again.
#
# With this on, a target that falls to blackout clears its offsets, so the
# next note starts from its palette colour.
#
# OFF by default, because the reset loses more than it fixes. Live only emits
# a CC when its value CHANGES, so an offset held deliberately across a section
# — a whole part sitting a quarter turn round the wheel, with gaps between the
# notes — would be thrown away at the first gap and not restated until the
# envelope moved again. Sticky offsets are the lesser problem: end a sweep at
# 64 and it behaves.
# Two separate switches, because the two moments carry different risk.
#
# ON_START is safe and on. Locating into a section inherits whatever offset
# was last sent, which is never what you want when you jump around a set.
#
# ON_BLACKOUT is off, and should probably stay off. MIDI carries no clip
# boundary, so "a new clip with no CC 12 automation" and "a gap in the notes
# inside the clip I am already in" look identical on the wire. Resetting at
# blackout catches the first and destroys the second — an offset you set
# deliberately over a part would vanish the moment the notes stopped, and
# Live would not restate a flat envelope to bring it back.
#
# The reliable fix for clip-to-clip bleed is a CC 12 and CC 13 point at the
# START of each clip. candelabra_template.mid seeds both at 64 for exactly
# this reason — build clips from it and the problem does not arise.
RESET_OFFSETS_ON_START    = True
RESET_OFFSETS_ON_BLACKOUT = False

# CCs are per-target, which means a channel that has never been sent one keeps
# whatever it was born with — while the channel you HAVE been driving sits at
# something else entirely. Play a note on the quiet one and it comes out at a
# stale value nobody chose.
#
# With this on, a note-on resets any CC that target has never been sent back
# to its resting default. A target that HAS received a CC is left alone — an
# explicit value always wins.
#
# This also cleans up after go_idle, which writes speed, depth and warmth
# directly without any CC involved. Without the reset those idle values sit
# there afterwards and a channel plays at numbers nobody chose.
NOTE_REFRESHES_CC = True

# Where an unset CC lands.
#   False  its resting default — predictable, and what the log will show
#   True   the newest value seen for that CC on ANY channel
#
# Inheritance sounds helpful and reads as a haunting: a value you set on one
# channel turns up on another you never touched, and the log gives you no
# clue where it came from.
INHERIT_UNSET_CC = False

# The most recent value seen for each CC, on any channel.
cc_seen = {}

# Where a CC rests when nothing has ever set it. Matches State.__init__.
CC_RESTING = {"level": 255, "speed": 170, "depth": 220,
              "warmth": 0, "fade": 15, "fade_out": 15, "boost": 0}
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
    # C-1  HOME - candle flame. Tuned by eye rather than derived: hue 56
    # sits between yellow and green, blended halfway into the blackbody
    # curve, with a little boost to lift the whole thing. Note the sat of
    # 133 — the flame is no longer a low-saturation look, which matters for
    # the par's trim curve. See trimCurve in par_node.ino.
    -1: ( 56, 133,   0, 255,  14),
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

# Fades are asymmetric in practice: a release usually wants longer than an
# attack. Three CCs, in order of specificity —
#
#   CC 5    both directions at once, the general one
#   CC 20   the attack only, into a lit preset
#   CC 21   the release only, out to blackout
#
# CC 5 writes both fields, so send it alone for symmetric fades, or send it
# first and then override one side. The packet still carries a single T; the
# bridge picks which of the two to send based on where the target is heading,
# so none of this needs a firmware change.
CC_FADE_IN  = 20
CC_FADE_OUT = 21
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
    CC_FADE_IN:  "fade",
    CC_FADE_OUT: "fade_out",
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
CHAN_TARGET = {0: 0,  1: 1,  2: 2,
               3: 40, 4: 45, 5: 46, 6: 47, 7: 48}

TARGET_NAMES = {0: "all", 1: "A", 2: "B",
                40: "pars", 45: "pars12", 46: "pars34",
                47: "inside", 48: "outside"}

# F40 is the par equivalent of F0 — needed anywhere the code idles or blacks
# out "everything", since F0 no longer reaches the pars.
PAR_ALL = 40

# ---- link mode
# One black key, held on a par channel, makes that group follow a candelabra
# target until it is released. Matched by pitch class, so any octave works
# and it plays the same wherever your hand happens to be.
LINK_NOTE_CLASS = 1        # 1 = C#, 3 = D#, 6 = F#, 8 = G#, 10 = A#

# par fixture code -> the targets it follows while the key is held.
# Several sources per group is allowed: channel 4 mirrors whichever of the
# three candelabra channels last changed, so it works whether you are driving
# both together on ch 1 or A and B separately on ch 2 and ch 3.
# Every group lists ch 1 as well as its specific source, so a link still does
# something when both candelabras are being driven together from ch 1 rather
# than separately from ch 2 and ch 3.
LINK_SOURCE = {40: (0, 1, 2),      # all pars   <- ch 1, 2 or 3
               45: (0, 1),         # pars 1+2   <- ch 1 or 2
               46: (0, 2),         # pars 3+4   <- ch 1 or 3
               47: (0,),           # inside     <- ch 1
               48: (0,)}           # outside    <- ch 1

# par fixture code -> set of link notes currently held for it
link_held = {}

# Groups that are linked permanently, with no key held. Put a par code in
# here and it follows its LINK_SOURCE for the whole session — every note,
# every CC, on channels 1, 2 and 3, with nothing to play and nothing to
# forget. Its own channel then does nothing, which is the trade.
#
#   {40}          all four pars shadow the candelabras
#   {45, 46}      each pair shadows its own candelabra
#   set()         off, the C# key is the only way in
#
# --link on the command line is the same as {40}.
LINK_ALWAYS = set()

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
        self.fade     = DEFAULT_FADE     # into a lit preset
        self.fade_out = DEFAULT_FADE     # out to blackout
        # go_idle borrows `fade` for its slow ease. Stash the real value so
        # resolve() can hand it back.
        self._fade_held = DEFAULT_FADE
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

        # CC numbers this target has actually been sent, as opposed to the
        # ones it is merely sitting on. See NOTE_REFRESHES_CC.
        self.cc_explicit = set()

        # note -> velocity, in the order they were pressed
        self.held    = {}

        self.idling  = False

    def go_idle(self):
        if self.idling:
            return
        self.idling = True
        self._fade_held = self.fade
        self.preset = IDLE_PRESET
        self.level   = IDLE_MASTER
        self.vel     = 255
        self.pmaster = 255
        self.speed  = IDLE_SPEED
        self.depth  = IDLE_DEPTH
        # Colour straight from the palette. pal_hue used to be left at
        # whatever was last played, which did not show while idle sat at
        # saturation 0 but would the moment it did not.
        entry = PALETTES[IDLE_OCTAVE]
        self.pal_hue = entry[0]
        self.pal_sat = entry[1]
        self.warmth  = entry[2]
        self.boost   = entry[4] if len(entry) > 4 else 0
        self.sat_off = 0
        self.hue_off = 0
        self.fade   = IDLE_FADE
        self.dirty  = True

    def resolve(self, latch):
        """Preset and colour follow the most recently pressed note still
        held. Nothing held means blackout, unless we're latching."""
        if self.idling:
            # Coming out of the idle look. Put the fade back, or this target
            # keeps an 800ms release for the rest of the session while every
            # other one sits at DEFAULT_FADE — which is why channel 1 used to
            # ease and channels 2 and 3 snapped.
            self.fade = self._fade_held
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
            if RESET_OFFSETS_ON_BLACKOUT:
                # Back to the palette's own colour, so the next note is not
                # wearing the last phrase's offset.
                self.hue_off = 0
                self.sat_off = 0
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

    def line_as(self, fixture):
        """This state's values under some other fixture code. Link mode uses
        it to send a candelabra's look to a par group without either State
        having to know about the other."""
        # One T on the wire, chosen by direction. Heading to blackout is a
        # release; anything else is an attack.
        t = self.fade_out if self.preset == 0 else self.fade
        return (f"F{fixture} P{self.preset} M{self.master} "
                f"S{self.speed} D{self.depth} W{self.warmth} "
                f"T{t} H{self.hue} C{self.sat} B{self.boost}\n")

    def line(self):
        return self.line_as(self.fixture)


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
    ap.add_argument("--link",   action="store_true",
                    help="link all four pars to the candelabras permanently, "
                         "as though the C# key were always held")
    ap.add_argument("--latch",  action="store_true",
                    help="notes stick until the next note, instead of "
                         "returning to blackout on note-off")
    args = ap.parse_args()
    if args.link:
        LINK_ALWAYS.add(40)

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

    if LINK_ALWAYS:
        names = ", ".join(TARGET_NAMES[t] for t in sorted(LINK_ALWAYS))
        print(f"permanent link: {names}")
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
                        # F0 no longer reaches the pars, so they need their own.
                        states[PAR_ALL].go_idle()
                        link_held.clear()
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
                        # Same for the pars. Clearing link_held means a link
                        # can never survive a locate that skipped its note-off.
                        link_held.clear()
                        # Locating into the middle of a set inherits whatever
                        # offsets were last sent. Centre them so a clip starts
                        # from its palette colours.
                        # Every claim drops too. A CC sent an hour ago on some
                        # other clip should not still own a channel — without
                        # this, one stray value marks that target explicit for
                        # the life of the process and the refresh skips it
                        # forever, with nothing in the log to say why.
                        for s_ in states.values():
                            s_.cc_explicit.clear()
                        if RESET_OFFSETS_ON_START:
                            for s_ in states.values():
                                s_.hue_off = 0
                                s_.sat_off = 0
                        states[PAR_ALL].resolve(args.latch)
                        states[PAR_ALL].dirty = True
                        last_send = 0.0        # force an immediate send
                        if not args.quiet:
                            print(f"  transport {msg.type} -> notes, resending state")
                        continue

                    if msg.type == "clock":
                        playing = True     # clock only ticks while running
                        continue

                    # The channel picks which target this message addresses.
                    # Anything outside ch 1-8 is ignored entirely.
                    if msg.type in ("note_on", "note_off", "control_change"):
                        target = CHAN_TARGET.get(msg.channel)
                        if target is None:
                            continue
                        st = states[target]

                    if msg.type == "note_on" and msg.velocity > 0:
                        # Before anything else, pull in any CC this target
                        # has never been sent. Without it a channel plays at
                        # whatever it was born with while the rest of the rig
                        # has moved on.
                        if NOTE_REFRESHES_CC:
                            for num, fld in CC_FIELD.items():
                                if num in st.cc_explicit:
                                    continue
                                v = (cc_seen.get(num, CC_RESTING.get(fld))
                                     if INHERIT_UNSET_CC
                                     else CC_RESTING.get(fld))
                                if v is not None and getattr(st, fld) != v:
                                    setattr(st, fld, v)
                                    st.dirty = True

                        look = note_to_look(msg.note)

                        # The link key. Black, so note_to_look ignores it and
                        # it can never also be a preset.
                        if ((msg.note % 12) == LINK_NOTE_CLASS
                                and target in LINK_SOURCE):
                            link_held.setdefault(target, set()).add(msg.note)
                            if not args.quiet:
                                src = "/".join(TARGET_NAMES[x]
                                               for x in LINK_SOURCE[target])
                                print(f"  link[{TARGET_NAMES[target]}] on "
                                      f"-> follows {src}")

                        if look:
                            st.held.pop(msg.note, None)   # re-press moves it
                            st.held[msg.note] = msg.velocity
                            st.resolve(args.latch)
                            if not args.quiet:
                                oct_ = (msg.note // 12) - 2
                                print(f"  on  [{TARGET_NAMES[target]}] "
                                      f"{msg.note} -> "
                                      f"{PRESET_NAMES[look[0]]} oct{oct_} "
                                      f"H{st.hue} C{st.sat} @ {st.master}"
                                      # master is three things multiplied, so
                                      # a zero tells you nothing on its own.
                                      f"  (lvl {st.level} vel {st.vel} "
                                      f"ceil {st.pmaster})")

                    elif (msg.type == "note_off" or
                          (msg.type == "note_on" and msg.velocity == 0)):
                        held_link = link_held.get(target)
                        if held_link and msg.note in held_link:
                            held_link.discard(msg.note)
                            # Snap back to this group's own look on release.
                            st.dirty = True
                            if not args.quiet:
                                print(f"  link[{TARGET_NAMES[target]}] off")

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
                                fields, val = ["hue_off"], (msg.value - 64) * 2
                            else:
                                fields, val = ["sat_off"], (msg.value - 64) * 4
                        elif msg.control == CC_FADE:
                            # The general one. Writes both directions, and
                            # claims 20 and 21 too so a later note-on refresh
                            # does not quietly undo it.
                            fields, val = ["fade", "fade_out"], scale(msg.value)
                        else:
                            f = CC_FIELD.get(msg.control)
                            if f is None:
                                continue
                            fields, val = [f], scale(msg.value)

                        # A CC that repeats its current value changes nothing,
                        # and Live emits plenty of those. Sending them anyway
                        # floods the serial link and buries real changes in
                        # the log.
                        # A global CC lands on every target at once, so one
                        # envelope can drive the whole rig.
                        cc_seen[msg.control] = val
                        claims = ({CC_FADE, CC_FADE_IN, CC_FADE_OUT}
                                  if msg.control == CC_FADE else {msg.control})
                        if not args.quiet:
                            # Which CC, from where, to what. Without this an
                            # unexpected value in an outgoing line gives no
                            # clue whether a clip sent it or it was inherited.
                            print(f"  cc  [{TARGET_NAMES[target]}] "
                                  f"{msg.control} = {msg.value} "
                                  f"-> {'/'.join(fields)} {val}")
                        for tgt in (states.values() if msg.control in GLOBAL_CC
                                    else (st,)):
                            tgt.cc_explicit |= claims
                            for field in fields:
                                if getattr(tgt, field) == val:
                                    continue
                                setattr(tgt, field, val)
                                tgt.dirty = True

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

                # Which par groups are currently following a candelabra, and
                # whether that source has anything new to say. Sampled before
                # the loop below, which clears the dirty flags.
                linked = LINK_ALWAYS | {t for t, n in link_held.items() if n}

                # For each linked group, which of its sources has something
                # new to say this pass. A group can list several; if more than
                # one changed in the same window the last one listed wins,
                # rather than sending two lines for the same fixture.
                hot_src = {}
                for t in linked:
                    hot = [src for src in LINK_SOURCE[t] if states[src].dirty]
                    if hot:
                        hot_src[t] = hot[-1]

                if (dirty or linked) and (now - last_send) >= SEND_INTERVAL:
                    # Every dirty target gets its own line, and only the ones
                    # that actually changed are sent.
                    for s in dirty:
                        # A linked group is driven from its source just below,
                        # so skip its own line or the two would fight.
                        if s.fixture in linked:
                            s.dirty = False
                            continue
                        ser.write(s.line().encode())
                        if not args.quiet:
                            print("  " + s.line().strip())
                        s.dirty = False

                    for t in sorted(linked):
                        src = hot_src.get(t)
                        if src is None:
                            continue        # no source changed, nothing to say
                        line = states[src].line_as(t)
                        ser.write(line.encode())
                        if not args.quiet:
                            print("  " + line.strip() + "   (link)")

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
            pa = states[PAR_ALL]
            pa.preset, pa.level, pa.fade = 0, 0, 30
            ser.write(pa.line().encode())
            time.sleep(0.2)
            ser.close()


if __name__ == "__main__":
    main()
