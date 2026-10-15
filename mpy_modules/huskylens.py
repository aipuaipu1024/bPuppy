# huskylens.py -- HuskyLens (Er Ha) UART1 debug tool for bPuppy
#
# ASCII ONLY (Bluetooth upload drops non-ASCII bytes).
#
# WIRING (HuskyLens 4-pin header -> bPuppy):
#   HuskyLens TX  -> GPIO5  (UART1 RX)
#   HuskyLens RX  <- GPIO4  (UART1 TX)
#   HuskyLens VCC -> 5V     (HuskyLens draws ~300mA)
#   GND common
# NOTE: GPIO4/5 are shared with the camera SCCB. If the camera was
# initialized, run() releases it automatically before starting.
#
# USAGE (REPL, after copying this file to the board root):
#   import huskylens
#   huskylens.run()             # request every 100 ms
#   huskylens.run(200)          # every 200 ms
#   huskylens.run(100, 115200)  # match HuskyLens menu baud rate
#   Ctrl-C stops.
#
# CLOSED LOOP (robot walks and steers to keep the target at screen center):
#   import huskylens
#   huskylens.on()       # robot stands up, walks, and centers the target
#   huskylens.off()      # stop the loop and the robot
#   huskylens.read()     # (loop on?, last x, last err deg, last w px)
#   huskylens.set(kp=0.5)          # tune; current values in huskylens.loop_cfg
#   huskylens.ERR_SIGN = -1        # flip if it steers AWAY from the target
#
#   !! on() makes the robot move -- clear the floor first.
#   !! Do NOT run at the same time as heading_anchor's loop (both write turn).
#
# DISTANCE (block width as a ruler -- near = wide, D = K / w):
#   # put the target at a known distance, fully in view:
#   huskylens.dist_cal(1.0)        # target exactly 1 m away, wait 3 s
#   huskylens.dist()               # -> meters (same object & pose!)
#
# PROTOCOL (DFRobot UART frame: 55 AA 11 + len + cmd + data + checksum):
#   request : 55 AA 11 00 20 30          ("give me the latest result")
#   reply   : 0x29 INFO  = blocks(2) learned_ids(2) frame_no(2) rsvd(4)
#             0x2A BLOCK = x(2) y(2) w(2) h(2) id(2), one frame per target
#   all 16-bit little-endian; checksum = sum(all frame bytes) & 0xFF

import bpuppy_motion as m
import bpuppy_uart
import heading_follow as flw    # frozen module: turns err() into set_turn()
import time

HALF_FOV_DEG = 30.0    # horizontal half field-of-view. Calibrate: put a target
                       # at a known angle, read x, solve angle=(x-160)/160*HFOV.
                       # Only scales the loop gain (see closed loop notes).

X_CENTER = 160.0       # screen x that counts as "straight ahead" (0..320)

# Steers AWAY from the target instead of toward it? -> set to -1.0 in REPL
# (runtime, not saved). Same role as heading_follow.TURN_SIGN: physical sign.
ERR_SIGN = 1.0

# Distance-scheduled gain: effective kp = loop_cfg['kp'] * _gain_scale().
# Closer target -> higher gain; 3.0x at <= GAIN_NEAR_M, 1.0x at >= GAIN_FAR_M,
# linear in between. Needs dist_cal() -- without K the scale stays 1.0.
GAIN_NEAR_M = 0.10
GAIN_FAR_M  = 0.50
GAIN_NEAR_X = 3.0

# Speed schedule by distance (approach control): far -> fast, close -> stop.
# At SPD_NEAR_M and closer: 0 (stand). From SPD_NEAR_M to SPD_MID_M: SPD_MID
# (hold the lowest gear). SPD_MID at SPD_MID_M -> SPD_FAR at SPD_FAR_M, linear.
# **Never crawls**: the only transition under 1 is the straight 1 -> 0 jump at
# SPD_NEAR_M. Speed 0 makes GO fade out and stand; a later rise kicks it back
# into walking (the documented 0 -> N resume path).
# Needs dist_cal() -- without K the speed is left untouched.
SPD_NEAR_M = 0.07
SPD_MID_M  = 0.10
SPD_FAR_M  = 0.50
SPD_MID    = 1.0
SPD_FAR    = 7.0
SPD_WRITE_STEP = 0.3    # write set_speed only when the scheduled value moved
                        # at least this much: distance jitter otherwise nudges
                        # it every tick and each call logs to UART0

# Loop defaults -- same names/meaning as heading_anchor.cfg (passed straight
# into flw.start()). Actual tick = period_ms sleep + ~40 ms UART round trip.
_LOOP_DEF = {
    'kp':        1.6,   # fraction of the error taken per gait cycle. Above the
                        # 1.0 "one cycle settles it" rule of thumb -- the error
                        # is scaled by the GUESSED HALF_FOV_DEG, so this number
                        # is not comparable to heading_anchor's kp.
    'ki':        0.0,   # integral; only needed for a moving target
    'deadband':  2.0,   # deg; a still target jitters only ~0.6 deg
    'period_ms': 100,   # ~7 Hz actual; loop bandwidth is only ~0.8 Hz anyway
    'i_limit':   0.3,
}
loop_cfg = {}
loop_cfg.update(_LOOP_DEF)

_on     = False        # closed loop running
_last_x = None         # last valid screen x from a BLOCK frame
_last_w = None         # last valid block width, px ("distance ruler")
_last_e = None         # last error fed to the loop (deg)
_spd_set = None        # last speed written via set_speed (skip no-op writes:
                       # every call logs "Speed: ... (omega=...)" to UART0)
_lost_n  = 0           # consecutive target-less ticks (see LOST_STOP_N)

LOST_STOP_N = 3        # stop only after this many consecutive ticks without a
                       # target: a single dropped frame (shaky camera while
                       # walking) must not abort the gait -- speed 0 starts the
                       # GO fade-out and each toggle causes walk/stall jerking
# d[m] * w[px] (D = K / w). Preloaded with the 2026-10-15 three-point fit
# (7/20/50 cm -> w 139/41/15) for THIS target; a different object needs its
# own dist_cal(known_distance_m).
_DIST_K = 9.53

_REQ = bytes([0x55, 0xAA, 0x11, 0x00, 0x20, 0x30])


def _init(baud):
    # GPIO4/5 shared with camera SCCB -- release camera if it holds them.
    try:
        import bpuppy_camera
        if bpuppy_camera.is_ready():
            bpuppy_camera.deinit()
            print("[hl] camera deinit (GPIO4/5 -> UART1)")
    except Exception:
        pass
    bpuppy_uart.u1_stop()              # u1_init is not idempotent: stop first so
    bpuppy_uart.u1_init(4, 5, baud)    # a new baud rate actually takes effect


def _read_reply():
    """Send one request, collect reply bytes until the line goes quiet."""
    bpuppy_uart.u1_send(_REQ)
    raw = b''
    t0 = time.ticks_ms()
    last = t0
    while True:
        d = bpuppy_uart.u1_read(128)
        now = time.ticks_ms()
        if d:
            raw += d
            last = now
        else:
            quiet = time.ticks_diff(now, last)
            if raw and quiet > 15:      # got something, 15 ms silence = done
                break
            if not raw and quiet > 60:  # nothing arrived within 60 ms
                break
        if time.ticks_diff(now, t0) > 250:
            break
        time.sleep_ms(2)
    return raw


def _angle(x):
    """Screen x (0..320) -> angle vs straight ahead (deg)."""
    return (x - 160) / 160.0 * HALF_FOV_DEG


def _block_xw(raw):
    """Silent parse: (x, w) of the first valid BLOCK frame, or None.

    No prints -- this one runs inside the control loop thread.
    """
    n = len(raw)
    i = 0
    while True:
        i = raw.find(b'\x55\xaa\x11', i)
        if i < 0 or n - i < 6:
            return None
        ln = raw[i + 3]
        if n - i < 6 + ln:
            return None
        fr = raw[i:i + 6 + ln]
        if fr[4] == 0x2A and ln == 10:
            csum = 0
            for b in fr[:-1]:
                csum = (csum + b) & 0xFF
            if csum != fr[-1]:
                return None            # bad checksum: distrust the reply
            d = fr[5:15]
            return (d[1] << 8 | d[0], d[5] << 8 | d[4])
        i += 6 + ln


def _parse(raw):
    """Print every valid frame found in raw. Returns True if any frame OK."""
    n = len(raw)
    i = 0
    ok_any = False
    while True:
        i = raw.find(b'\x55\xaa\x11', i)
        if i < 0 or n - i < 6:
            break
        ln = raw[i + 3]
        if n - i < 6 + ln:
            print("[hl] partial frame (%d B), dropped" % (n - i))
            break
        fr = raw[i:i + 6 + ln]
        csum = 0
        for b in fr[:-1]:
            csum = (csum + b) & 0xFF
        ok = (csum == fr[-1])
        cmd = fr[4]
        d = fr[5:5 + ln]
        if cmd == 0x29 and ln == 10:
            blocks = d[1] << 8 | d[0]
            ids = d[3] << 8 | d[2]
            fno = d[5] << 8 | d[4]
            print("[hl] INFO  blocks=%d learned=%d frame=%d  csum=%s"
                  % (blocks, ids, fno, "OK" if ok else "BAD"))
            ok_any = ok_any or ok
        elif cmd == 0x2A and ln == 10:
            x = d[1] << 8 | d[0]
            y = d[3] << 8 | d[2]
            w = d[5] << 8 | d[4]
            h = d[7] << 8 | d[6]
            bid = d[9] << 8 | d[8]
            print("[hl] BLOCK x=%d y=%d w=%d h=%d id=%d  angle=%+.1f deg  csum=%s"
                  % (x, y, w, h, bid, _angle(x), "OK" if ok else "BAD"))
            ok_any = ok_any or ok
        else:
            print("[hl] frame cmd=0x%02x len=%d (unknown)" % (cmd, ln))
        i += 6 + ln
    return ok_any


# ---------------------------------------------------------------------------
# closed loop -- keep the target at screen center (drives set_turn)
# ---------------------------------------------------------------------------
# err = ERR_SIGN * (x - X_CENTER) / 160 * HALF_FOV_DEG      [deg]
#
# Sign check (negative feedback): target right of center -> x > 160 -> err > 0
# -> heading_follow outputs turn > 0 (= robot turns right, see the TURN_SIGN
# notes there) -> camera pans right -> target moves left on screen -> x falls
# back to 160. If the robot instead runs away from the target, ERR_SIGN is the
# single line to flip.
#
# Aiming at X_CENTER *on screen* (not at a physical angle) means a mounting
# misalignment does NOT bias the result -- HALF_FOV_DEG only scales the gain.

def _dist_of(w):
    """Distance in m for a block width w, or None when K is unknown / w
    is too small to trust."""
    if _DIST_K is None or w is None or w < 5:
        return None
    return _DIST_K / w


def _gain_scale():
    """Distance-scheduled kp multiplier (see GAIN_* constants). 1.0 when the
    distance is unknown (no dist_cal / no target)."""
    d = _dist_of(_last_w)
    if d is None:
        return 1.0
    if d <= GAIN_NEAR_M:
        return GAIN_NEAR_X
    if d >= GAIN_FAR_M:
        return 1.0
    return 1.0 + (GAIN_NEAR_X - 1.0) * (GAIN_FAR_M - d) / (GAIN_FAR_M - GAIN_NEAR_M)


def _speed_for(d):
    """Scheduled walk speed for distance d (see SPD_* constants).
    Never crawls: the 0..1 ramp is replaced by a hard 1 -> 0 at SPD_NEAR_M."""
    if d <= SPD_NEAR_M:
        return 0.0
    if d < SPD_MID_M:
        return SPD_MID                 # lowest gear, not a ramp down to 0
    if d < SPD_FAR_M:
        return SPD_MID + (SPD_FAR - SPD_MID) * (d - SPD_MID_M) / (SPD_FAR_M - SPD_MID_M)
    return SPD_FAR


def err():
    """Loop input: current error (deg). None = no data this tick (walk straight).
    The returned error is scaled by the distance gain (turn = kp * err).
    Internal -- called by heading_follow every tick."""
    global _last_x, _last_w, _last_e, _spd_set, _lost_n
    if not _on:
        return None
    r = _block_xw(_read_reply())
    if r is None:
        # No target (or no reply). heading_follow sees None and lets the turn
        # decay to 0; the speed goes to 0 -- but only after LOST_STOP_N ticks,
        # so one hiccup frame does not break the walk.
        _last_x = None
        _last_w = None
        _last_e = None
        _lost_n += 1
        if _lost_n >= LOST_STOP_N and _spd_set != 0.0:
            _spd_set = 0.0
            m.set_speed(0.0)
        return None
    _lost_n = 0
    _last_x, _last_w = r
    # Revive after a stop: speed 0 fades the stride out into GAIT_STOP, and a
    # plain set_speed() cannot pull it back (motion_task.cpp: the speed
    # follower only runs in a non-static gait). So while a target is visible,
    # re-arm 'go' whenever the gait is not a moving one (0=stop, 4=repos).
    # Once it walks again this fires no more. To pause deliberately, off().
    if m.get_params()[6] not in (1, 2, 3):
        m.set_gait('go')
    d = _dist_of(_last_w)
    if d is not None:
        sv = _speed_for(d)
        if _spd_set is None or abs(sv - _spd_set) >= SPD_WRITE_STEP:
            _spd_set = sv
            m.set_speed(sv)
    _last_e = ERR_SIGN * (_last_x - X_CENTER) / 160.0 * HALF_FOV_DEG
    return _last_e * _gain_scale()


def on(baud=9600):
    """Start steering so the target sits at screen center. Returns at once.

    If the robot is standing still this also starts the walk ('go') -- turn
    does nothing in a static pose. An already running gait is left untouched.

    !! The robot WILL move -- clear the floor first.
    !! Keep heading_anchor's loop off while this runs (both write turn).
    """
    global _on, _spd_set, _lost_n
    _init(baud)
    _on = True
    _spd_set = None                # first tick always writes the scheduled speed
    _lost_n = 0
    try:
        if m.get_params()[6] not in (1, 2, 3):   # 0=stop 4=repos -> start walking
            m.set_gait('go')
    except Exception:
        pass
    flw.start(err, **loop_cfg)


def off():
    """Stop: end the loop, release the turn, and stop the robot ('stop')."""
    global _on
    _on = False
    flw.stop()
    try:
        m.set_gait('stop')
    except Exception:
        pass


def read():
    """(loop running?, last x, last err deg, last w px).
    None where no data yet. w doubles as the distance ruler (see dist_cal)."""
    return (_on, _last_x, _last_e, _last_w)


def dist_cal(d_m, secs=3):
    """Calibrate the distance ruler: put the target at d_m meters and keep it
    fully in view. Samples w for secs seconds, stores K = d_m * w_median.
    Returns K [m*px], or None if no frame arrived.

    Valid only for the SAME object & pose -- turning the object or partial
    occlusion changes w without the distance changing.
    """
    global _DIST_K
    if not _on:
        _init(9600)
    ws = []
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < secs * 1000:
        if _on:                 # loop thread owns the UART -- sample its feed
            if _last_w is not None:
                ws.append(_last_w)
        else:
            r = _block_xw(_read_reply())
            if r is not None:
                ws.append(r[1])
        time.sleep_ms(20)
    if not ws:
        print("[hl] dist_cal: no target seen")
        return None
    ws.sort()
    wmed = ws[len(ws) // 2]
    _DIST_K = d_m * wmed
    print("[hl] dist_cal: w_med=%d n=%d  K=%.1f m*px" % (wmed, len(ws), _DIST_K))
    return _DIST_K


def dist():
    """Distance in meters of the target in view (K / w), or None when not
    calibrated / nothing in view. Polls the lens directly when the loop is
    off; when it is running, the loop thread's latest frame is reused."""
    global _last_x, _last_w
    if _DIST_K is None:
        return None
    if not _on:
        r = _block_xw(_read_reply())
        if r is None:
            return None
        _last_x, _last_w = r
    if _last_w is None or _last_w < 5:      # tiny box = unreliable center
        return None
    return _DIST_K / _last_w


def set(**kw):
    """Tune loop gains (same names as heading_anchor). No args = restore
    defaults. Takes effect immediately when the loop is running."""
    if not kw:
        loop_cfg.clear()
        loop_cfg.update(_LOOP_DEF)
    else:
        for k in kw:
            if k not in _LOOP_DEF:
                raise ValueError('no such param: %s' % k)
            loop_cfg[k] = float(kw[k])
    if _on:
        flw.start(err, **loop_cfg)


def watch(period_ms=300, ticks=None):
    """Roll-print d / err / speed / turn while the loop runs.

    d      distance in m (K/w)     err   target angle, deg
    spd    scheduled speed         turn  turn sent by the loop
    Ctrl-C stops **watching only** -- the loop keeps running (off() to stop).
    ticks=N prints N lines and returns (handy for a quick look).
    """
    print("[hl] watch:  d(m)   err(deg)  spd   turn")
    n = 0
    try:
        while ticks is None or n < ticks:
            n += 1
            p = m.get_params()
            d = dist()
            e = read()[2]
            print("[hl] d=%s err=%s spd=%.1f turn=%+.1f"
                  % (('%.2f' % d) if d is not None else '--',
                     ('%+.1f' % e) if e is not None else '--',
                     p[0], p[5]))
            time.sleep_ms(period_ms)
    except KeyboardInterrupt:
        print("[hl] watch stopped (loop still running -- off() to stop it)")


def run(period_ms=100, baud=9600):
    """Poll HuskyLens every period_ms; print raw hex + parsed result.
    Ctrl-C stops. baud must match the value set in the HuskyLens menu.
    Not usable while the closed loop is on (both own the UART)."""
    if _on:
        print("[hl] closed loop is running -- call off() first")
        return
    _init(baud)
    print("[hl] UART1 @%d baud, period %d ms (Ctrl-C to stop)" % (baud, period_ms))
    n = 0
    try:
        while True:
            t0 = time.ticks_ms()
            raw = _read_reply()
            n += 1
            if raw:
                print("[hl] #%d RX %d B: %s" % (n, len(raw), raw.hex()))
                if not _parse(raw):
                    print("[hl] no valid frame")
            else:
                print("[hl] #%d no reply" % n)
            dt = time.ticks_diff(time.ticks_ms(), t0)
            if dt < period_ms:
                time.sleep_ms(period_ms - dt)
    except KeyboardInterrupt:
        print("[hl] stopped")
