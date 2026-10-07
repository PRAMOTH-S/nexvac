"""Raspberry Pi health, read straight from /proc, /sys and vcgencmd.

Nothing here imports rclpy or aiohttp: it is plain stdlib so it can be run on
its own (`ros2 run nexva_web pi_health`) on a robot that is misbehaving, and
so it can be unit-tested on a laptop. The bridge keeps one HealthMonitor and
calls `snapshot()` from its thread executor - `vcgencmd` is a subprocess and
has no business running on the event loop.

Every section answers with {'ok': bool} and, when not ok, a 'why' string. That
is deliberate: this development machine is x86 and has no vcgencmd and no
Pi thermal zone, and the honest answer there is "unavailable", never a
plausible-looking number. A fabricated 45 C is worse than a blank, because the
whole point of this page is to tell you whether to believe the hardware.

The one metric that cannot be got anywhere else is vcgencmd's throttle word.
A Pi that browns out under motor load caps its own clock and keeps running,
so the symptom is "the robot got slow and the control loop started missing
deadlines" - which looks exactly like a software bug until you read bit 0.
"""

import os
import re
import socket
import subprocess
import threading
import time

# vcgencmd lives outside a normal PATH on Raspberry Pi OS, and absolutely
# nowhere on anything else. Looked up by absolute path so that a $PATH entry
# cannot decide what this runs.
VCGENCMD_PATHS = ('/usr/bin/vcgencmd', '/opt/vc/bin/vcgencmd')

# vcgencmd get_throttled bits. The low four are "right now", and the same four
# shifted up 16 are "has happened at least once since boot". The since-boot
# ones are the valuable half: a brownout during a turn is over in milliseconds
# and no poll will ever catch it live.
THROTTLE_BITS = (
    (0, 'under-voltage'),
    (1, 'ARM frequency capped'),
    (2, 'currently throttled'),
    (3, 'soft temperature limit'),
)
THROTTLE_SINCE_BOOT_SHIFT = 16

# How long each reading stays good for. The page polls at 2 s; these stop that
# poll from re-running the expensive sources every time, and keep the CPU
# delta window long enough to mean something.
CPU_MIN_WINDOW_S = 0.4
TEMP_TTL_S = 2.0
THROTTLE_TTL_S = 5.0
MEM_TTL_S = 2.0
DISK_TTL_S = 20.0

# Thermal zone types that are the actual SoC, best first. Everything else
# (wifi chip, NVMe, battery) is real but is not the number we want to show.
PREFERRED_ZONES = ('cpu-thermal', 'soc-thermal', 'x86_pkg_temp', 'cpu_thermal',
                   'acpitz', 'coretemp')

_MEMINFO_RE = re.compile(r'^(\w+):\s+(\d+)\s*kB', re.M)


def _read(path):
    """File contents, or None. Every source here is allowed to not exist."""
    try:
        with open(path) as fh:
            return fh.read()
    except OSError:
        return None


def _vcgencmd_path():
    for path in VCGENCMD_PATHS:
        if os.path.isfile(path) and os.access(path, os.X_OK):
            return path
    return None


def vcgencmd(*args, timeout=2.0):
    """Run vcgencmd, or return None if this is not a Pi.

    List form, no shell, absolute path. A missing binary is the normal case on
    anything but a Pi and must never raise - this is called from the page's
    2 s poll, and one exception there would blank the whole card.
    """
    exe = _vcgencmd_path()
    if exe is None:
        return None
    try:
        out = subprocess.run([exe, *args], capture_output=True, text=True,
                             timeout=timeout, check=False)
    except (OSError, subprocess.SubprocessError):
        return None
    if out.returncode != 0:
        return None
    return out.stdout.strip()


def decode_throttled(word):
    """0x50005 -> what is wrong now and what has been wrong since boot.

    `word` is the integer from `vcgencmd get_throttled`. Returns plain words,
    because "0x50005" on a dashboard is a number nobody reads twice.
    """
    now = [label for bit, label in THROTTLE_BITS if word & (1 << bit)]
    since = [label for bit, label in THROTTLE_BITS
             if word & (1 << (bit + THROTTLE_SINCE_BOOT_SHIFT))]
    return {
        'ok': True,
        'raw': '0x%x' % word,
        'now': now,
        'since_boot': since,
        'healthy': not now and not since,
    }


def throttling():
    """The Pi's throttle word, decoded. Unavailable off a Pi."""
    out = vcgencmd('get_throttled')
    if out is None:
        return {'ok': False,
                'why': 'vcgencmd not present - not a Raspberry Pi, or the '
                       'VideoCore tools are not installed'}
    # "throttled=0x0"
    _, _, value = out.partition('=')
    try:
        word = int(value.strip(), 0)
    except ValueError:
        return {'ok': False, 'why': 'could not parse vcgencmd output %r' % out}
    return decode_throttled(word)


def thermal_zones():
    """Every readable /sys thermal zone as (type, celsius)."""
    zones = []
    for i in range(32):                      # more than any board has
        base = '/sys/class/thermal/thermal_zone%d' % i
        raw = _read(base + '/temp')
        if raw is None:
            continue
        try:
            value = float(raw.strip())
        except ValueError:
            continue
        # Nearly every driver reports millidegrees, a few report degrees.
        # 200 C is not a temperature a running board has, so it is the unit.
        celsius = value / 1000.0 if abs(value) > 200 else value
        kind = (_read(base + '/type') or '').strip() or ('zone%d' % i)
        zones.append({'type': kind, 'celsius': round(celsius, 1)})
    return zones


def temperature():
    """SoC temperature in °C.

    /sys/class/thermal is tried first: it needs no privileges and exists on
    every Linux, Pi or not. vcgencmd measure_temp is the fallback and is
    Pi-only, so on this dev box the sysfs path is the only one that answers.
    """
    zones = thermal_zones()
    if zones:
        chosen = None
        for want in PREFERRED_ZONES:
            # A board can expose several zones of the same type (this dev box
            # has two 'acpitz'). The hot one is the die; picking the first
            # would quietly report the chassis sensor instead.
            same = [z for z in zones if z['type'] == want]
            if same:
                chosen = max(same, key=lambda z: z['celsius'])
                break
        if chosen is None:
            # No name we recognise: the hottest zone is the safest guess for
            # "is this board cooking", which is the question being asked.
            chosen = max(zones, key=lambda z: z['celsius'])
        return {'ok': True, 'celsius': chosen['celsius'],
                'source': '/sys/class/thermal (%s)' % chosen['type'],
                'zones': zones}

    out = vcgencmd('measure_temp')           # "temp=48.9'C"
    if out is not None:
        match = re.search(r'([-\d.]+)', out)
        if match:
            return {'ok': True, 'celsius': round(float(match.group(1)), 1),
                    'source': 'vcgencmd measure_temp', 'zones': []}

    return {'ok': False,
            'why': 'no readable thermal zone and no vcgencmd', 'zones': []}


def _cpu_times():
    """Per-CPU jiffy counters from /proc/stat: {'cpu': [...], 'cpu0': [...]}."""
    raw = _read('/proc/stat')
    if raw is None:
        return None
    out = {}
    for line in raw.splitlines():
        if not line.startswith('cpu'):
            break                            # the cpu lines come first
        parts = line.split()
        try:
            out[parts[0]] = [int(v) for v in parts[1:]]
        except ValueError:
            continue
    return out or None


def _busy_pct(before, after):
    """Busy share between two /proc/stat rows, as a percentage."""
    total = sum(after) - sum(before)
    if total <= 0:
        return None
    # fields 3 (idle) and 4 (iowait) are the not-working ones. iowait counts
    # as idle here: the core is available, it is the SD card that is slow.
    idle = (after[3] - before[3]) + (after[4] - before[4])
    return round(max(0.0, min(100.0, 100.0 * (total - idle) / total)), 1)


def cpu_clock():
    """Current and maximum core clock in MHz.

    Current well below maximum under load is the other face of throttling, and
    unlike get_throttled this works on any board. cpufreq first: the Pi's
    /proc/cpuinfo has no "cpu MHz" line at all, where x86's does.
    """
    cur, maximum = [], []
    for i in range(64):
        base = '/sys/devices/system/cpu/cpu%d/cpufreq/' % i
        raw = _read(base + 'scaling_cur_freq')
        if raw is None:
            continue
        try:
            cur.append(float(raw.strip()) / 1000.0)      # kHz -> MHz
        except ValueError:
            pass
        top = _read(base + 'cpuinfo_max_freq')
        if top:
            try:
                maximum.append(float(top.strip()) / 1000.0)
            except ValueError:
                pass
    if not cur:
        raw = _read('/proc/cpuinfo') or ''
        cur = [float(m) for m in re.findall(r'cpu MHz\s*:\s*([\d.]+)', raw)]
    if not cur:
        return {'ok': False, 'why': 'no cpufreq and no "cpu MHz" in /proc/cpuinfo'}
    return {'ok': True,
            'mhz': round(sum(cur) / len(cur)),
            'max_mhz': round(max(maximum)) if maximum else None}


def load_average():
    try:
        one, five, fifteen = os.getloadavg()
    except OSError:
        return {'ok': False, 'why': 'load average not available'}
    return {'ok': True, 'avg': [round(one, 2), round(five, 2),
                                round(fifteen, 2)]}


def memory():
    """Total / used / available / swap, in bytes, from /proc/meminfo."""
    raw = _read('/proc/meminfo')
    if raw is None:
        return {'ok': False, 'why': '/proc/meminfo not readable'}
    kb = {k: int(v) for k, v in _MEMINFO_RE.findall(raw)}
    if 'MemTotal' not in kb:
        return {'ok': False, 'why': '/proc/meminfo had no MemTotal'}
    total = kb['MemTotal'] * 1024
    # MemAvailable, not MemFree: page cache is free for the taking, and
    # MemFree alone makes every healthy Linux box look like it is out of RAM.
    available = kb.get('MemAvailable', kb.get('MemFree', 0)) * 1024
    swap_total = kb.get('SwapTotal', 0) * 1024
    swap_free = kb.get('SwapFree', 0) * 1024
    return {
        'ok': True,
        'total': total,
        'available': available,
        'used': max(0, total - available),
        'pct': round(100.0 * (total - available) / total, 1) if total else None,
        'swap_total': swap_total,
        'swap_used': max(0, swap_total - swap_free),
    }


def disk(path=None):
    """Space on the filesystem holding the workspace."""
    # This module lives inside the install tree, which lives in the workspace,
    # which is the filesystem that fills up when maps and bags pile up.
    path = path or os.path.dirname(os.path.abspath(__file__))
    try:
        st = os.statvfs(path)
    except OSError as exc:
        return {'ok': False, 'why': 'statvfs(%s) failed: %s' % (path, exc)}
    total = st.f_blocks * st.f_frsize
    # f_bavail, not f_bfree: the root-reserved blocks are not ours to use.
    free = st.f_bavail * st.f_frsize
    used = total - st.f_bfree * st.f_frsize
    return {'ok': True, 'path': path, 'total': total, 'free': free,
            'used': used,
            'pct': round(100.0 * used / total, 1) if total else None}


def uptime():
    raw = _read('/proc/uptime')
    if raw is None:
        return {'ok': False, 'why': '/proc/uptime not readable'}
    try:
        return {'ok': True, 'seconds': float(raw.split()[0])}
    except (ValueError, IndexError):
        return {'ok': False, 'why': 'could not parse /proc/uptime'}


def format_uptime(seconds):
    seconds = int(seconds)
    days, rem = divmod(seconds, 86400)
    hours, rem = divmod(rem, 3600)
    minutes = rem // 60
    if days:
        return '%dd %dh %dm' % (days, hours, minutes)
    if hours:
        return '%dh %dm' % (hours, minutes)
    return '%dm' % minutes


class HealthMonitor:
    """One sampler, held by the bridge for the life of the process.

    CPU utilisation is the reason this is a class and not a function: it is a
    difference between two /proc/stat reads, so somebody has to remember the
    previous one. The first sample is taken in __init__ so that the first
    page load already has a window to measure against rather than showing a
    blank for two seconds.
    """

    def __init__(self):
        self.hostname = socket.gethostname()
        self._cpu_prev = _cpu_times()
        self._cpu_prev_at = time.monotonic()
        self._cache = {}                     # key -> (expires_at, value)
        # snapshot() can arrive from two threads (the HTTP poll and the
        # WebSocket command); the CPU baseline and the cache are shared state.
        self._lock = threading.Lock()

    def _cached(self, key, ttl, fn):
        now = time.monotonic()
        hit = self._cache.get(key)
        if hit is not None and hit[0] > now:
            return hit[1]
        value = fn()
        self._cache[key] = (now + ttl, value)
        return value

    def cpu(self):
        """Total and per-core busy percentage since the previous call."""
        now = time.monotonic()
        current = _cpu_times()
        if current is None:
            return {'ok': False, 'why': '/proc/stat not readable'}
        previous, elapsed = self._cpu_prev, now - self._cpu_prev_at
        if previous is None or elapsed < CPU_MIN_WINDOW_S:
            # Too soon to tell them apart - two reads a millisecond apart give
            # a number made of rounding noise. Keep the old baseline so the
            # next call still has a real window, and say nothing this time.
            if previous is None:
                self._cpu_prev, self._cpu_prev_at = current, now
            return {'ok': False, 'why': 'waiting for a second /proc/stat sample',
                    'cores': max(0, len(current) - 1)}
        self._cpu_prev, self._cpu_prev_at = current, now

        per_core = []
        for i in range(len(current) - 1):
            key = 'cpu%d' % i
            if key in current and key in previous:
                per_core.append(_busy_pct(previous[key], current[key]))
        total = (_busy_pct(previous['cpu'], current['cpu'])
                 if 'cpu' in previous and 'cpu' in current else None)
        # A per-core entry can legitimately be None: a core whose counters did
        # not move at all over the window is parked, and 0% would be a claim
        # the kernel did not make. The page renders those as a dash.
        out = {'ok': total is not None, 'total': total, 'per_core': per_core,
               'cores': len(per_core), 'window_s': round(elapsed, 2),
               'clock': cpu_clock(), 'load': load_average()}
        if total is None:
            out['why'] = '/proc/stat counters did not move'
        return out

    def snapshot(self):
        """Everything, as one JSON-safe dict. Never raises."""
        with self._lock:
            return self._snapshot()

    def _snapshot(self):
        return {
            'hostname': self.hostname,
            'time': time.time(),
            'cpu': self.cpu(),
            'temperature': self._cached('temp', TEMP_TTL_S, temperature),
            'throttling': self._cached('throttle', THROTTLE_TTL_S, throttling),
            'memory': self._cached('mem', MEM_TTL_S, memory),
            'disk': self._cached('disk', DISK_TTL_S, disk),
            'uptime': uptime(),
        }


def main(argv=None):
    """Print one snapshot. For when the page is the thing that is broken."""
    import json
    monitor = HealthMonitor()
    time.sleep(CPU_MIN_WINDOW_S + 0.1)       # give the CPU delta a window
    print(json.dumps(monitor.snapshot(), indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
