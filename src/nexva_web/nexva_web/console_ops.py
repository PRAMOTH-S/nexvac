"""An allowlisted command console. No shell, ever.

This web UI is unauthenticated and binds 0.0.0.0, so anything on the same
wifi can reach it - and it drives real motors. A "run this command" endpoint
here would be remote code execution as the robot's user, with wheels. So
there is no such endpoint: there is a fixed table of operations, each one a
list-form argv with its arguments substituted in only after they match a
strict pattern, and every one of them read-only.

The rules, in order of how much they matter:

  1. Nothing in this file ever goes near a shell. subprocess is called with a
     list, shell=False, and the executable comes from EXECUTABLES below, not
     from whatever the caller typed.
  2. Arguments are matched against ARG_RE - ^[A-Za-z0-9_/]{1,200}$ - which is
     an allowlist of characters, not a denylist of bad ones. ';', '&&', '|',
     '$(', backticks, spaces, '..' and a leading '-' all fail it, and so does
     anything nobody has thought of yet. A denylist would be a list of the
     attacks we happened to remember.
  3. Nothing writes. No `ros2 topic pub`, no `ros2 run`, no `ros2 launch`, no
     `ros2 lifecycle set`, no `ros2 param set`, no systemctl, no reboot, no
     sudo, no rm. Those are not missing because they are hard; they are
     missing because this page is reachable by strangers.
  4. Everything has a hard timeout and truncated output. `ros2 topic hz` and
     `ros2 topic echo` never return on their own, and a page that can start
     unbounded processes is a denial of service against the robot.

A leading '-' failing ARG_RE matters more than it looks: without it, a topic
argument of '--all' or '-f' would turn a read into whatever flag the tool
happens to have.
"""

import glob
import os
import re
import shutil
import signal
import subprocess
import threading
import time

# Topic / node / service / parameter names. Allowlist of characters.
ARG_RE = re.compile(r'^[A-Za-z0-9_/]{1,200}$')

# Only these may ever be exec'd. The op table is checked against this at
# import, so a careless edit to the table fails loudly at startup rather than
# quietly widening what the page can run.
EXECUTABLES = ('ros2', 'df', 'free', 'uptime', 'vcgencmd', 'i2cdetect', 'ls')

MAX_OUTPUT_CHARS = 20000
DEFAULT_TIMEOUT_S = 5.0

# Worth knowing: the first `ros2 ...` here starts the ros2 CLI daemon, which
# outlives the command. That is the CLI's normal behaviour and not something
# the timeout should chase - killing it would only make the next call slower.

# Two at a time. The bridge runs these in its thread pool, which is the same
# pool an e-stop goes through: a browser holding a fistful of `ros2 topic hz`
# open must not be able to delay the robot stopping.
MAX_CONCURRENT = 2
_slots = threading.BoundedSemaphore(MAX_CONCURRENT)


class ConsoleError(Exception):
    """A request that was refused before anything ran."""


def _devices():
    """Build the `ls -l` for whatever serial devices exist, without a shell.

    `ls -l /dev/esp /dev/ttyUSB*` is the thing you actually type when the base
    has stopped answering, but the '*' is a shell feature and there is no
    shell here. Python expands it and `ls` is handed real paths.
    """
    paths = sorted(set(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
                       + glob.glob('/dev/esp*')))
    if not paths:
        raise ConsoleError('no /dev/esp, /dev/ttyUSB* or /dev/ttyACM* exists '
                           '- the base is not plugged in (or udev has not '
                           'made the symlink)')
    return ['ls', '-l', *paths]


# Each entry: argv with '{0}', '{1}' where the operator's arguments go.
# 'args' names them, for the page's input labels. 'timeout' overrides the
# default. 'build' replaces argv entirely for the one op that needs globbing.
OPS = {
    'topic_list': {
        'label': 'ros2 topic list',
        'argv': ['ros2', 'topic', 'list', '-t'],
        'args': [],
        'help': 'every topic, with its type',
    },
    'topic_info': {
        'label': 'ros2 topic info <topic>',
        'argv': ['ros2', 'topic', 'info', '{0}', '--verbose'],
        'args': ['topic'],
        'help': 'who publishes and subscribes it, and with what QoS',
    },
    'topic_hz': {
        'label': 'ros2 topic hz <topic>',
        'argv': ['ros2', 'topic', 'hz', '{0}'],
        'args': ['topic'],
        'timeout': 6.0,
        # hz never exits. It is stopped by the timeout, and whatever it
        # printed by then is the answer - which is why partial output is kept
        # on a timeout instead of thrown away.
        'help': 'publish rate, measured for ~6 s then stopped',
    },
    'topic_echo': {
        'label': 'ros2 topic echo <topic> --once',
        'argv': ['ros2', 'topic', 'echo', '{0}', '--once'],
        'args': ['topic'],
        'timeout': 6.0,
        'help': 'one message (times out if nothing is published)',
    },
    'node_list': {
        'label': 'ros2 node list',
        'argv': ['ros2', 'node', 'list'],
        'args': [],
        'help': 'every node currently alive',
    },
    'node_info': {
        'label': 'ros2 node info <node>',
        'argv': ['ros2', 'node', 'info', '{0}'],
        'args': ['node'],
        'help': 'a node’s topics, services and actions',
    },
    'param_list': {
        'label': 'ros2 param list <node>',
        'argv': ['ros2', 'param', 'list', '{0}'],
        'args': ['node'],
        'help': 'every parameter a node has',
    },
    'param_get': {
        'label': 'ros2 param get <node> <param>',
        'argv': ['ros2', 'param', 'get', '{0}', '{1}'],
        'args': ['node', 'parameter'],
        'help': 'read one parameter (reading only - there is no set)',
    },
    'service_list': {
        'label': 'ros2 service list',
        'argv': ['ros2', 'service', 'list', '-t'],
        'args': [],
        'help': 'every service, with its type',
    },
    'df': {
        'label': 'df -h',
        'argv': ['df', '-h'],
        'args': [],
        'help': 'disk space',
    },
    'free': {
        'label': 'free -m',
        'argv': ['free', '-m'],
        'args': [],
        'help': 'memory and swap',
    },
    'uptime': {
        'label': 'uptime',
        'argv': ['uptime'],
        'args': [],
        'help': 'uptime and load average',
    },
    'measure_temp': {
        'label': 'vcgencmd measure_temp',
        'argv': ['vcgencmd', 'measure_temp'],
        'args': [],
        'help': 'SoC temperature (Raspberry Pi only)',
    },
    'get_throttled': {
        'label': 'vcgencmd get_throttled',
        'argv': ['vcgencmd', 'get_throttled'],
        'args': [],
        'help': 'raw throttle word (Raspberry Pi only; decoded on this page)',
    },
    'i2cdetect': {
        'label': 'i2cdetect -y 1',
        'argv': ['i2cdetect', '-y', '1'],
        'args': [],
        'timeout': 8.0,
        'help': 'what is answering on the I2C bus (the IMU should be there)',
    },
    'devices': {
        'label': 'ls -l /dev/esp /dev/ttyUSB*',
        'build': _devices,
        'args': [],
        'help': 'the base’s serial device and who owns it',
    },
}


def _audit_table():
    """Fail at import if the table has grown something it should not have.

    A table is only as safe as the last edit to it, and the edit that widens
    this one will be made in a hurry on a robot that is misbehaving. Better
    that the bridge refuses to start than that it starts with a hole.
    """
    for name, spec in OPS.items():
        argv = spec.get('argv')
        if argv is None:                     # 'build' ops construct their own
            continue
        if argv[0] not in EXECUTABLES:
            raise AssertionError('%s runs %r, which is not permitted'
                                 % (name, argv[0]))
        holes = {p for p in argv if p.startswith('{') and p.endswith('}')}
        if len(holes) != len(spec['args']):
            raise AssertionError('%s: argv has %d placeholders but %d named '
                                 'arguments' % (name, len(holes),
                                                len(spec['args'])))


_audit_table()


def validate_arg(value, what='argument'):
    """Return `value` if it is a legal name, else raise ConsoleError.

    The message says what was wrong rather than just "invalid": an operator
    who pasted a topic with a trailing space should be told that, not left
    guessing that the console is broken.
    """
    if value is None:
        raise ConsoleError('%s is required' % what)
    if not isinstance(value, str):
        raise ConsoleError('%s must be text' % what)
    if not value:
        raise ConsoleError('%s is empty' % what)
    if len(value) > 200:
        raise ConsoleError('%s is too long (200 characters max)' % what)
    if not ARG_RE.match(value):
        raise ConsoleError(
            '%s %r is not allowed - only letters, digits, _ and / '
            '(no spaces, no "..", no leading "-", no shell characters)'
            % (what, value))
    return value


def describe():
    """Return the table in the shape the page's dropdown needs.

    'preview' is the argv with the placeholders left in, so the page can show
    the exact command before it runs it. The page showing something other than
    what the server will run would make the preview a lie, so it is built from
    the same list the server executes.
    """
    out = []
    for name, spec in sorted(OPS.items()):
        if 'build' in spec:
            preview = [spec['label']]
        else:
            preview = list(spec['argv'])
        out.append({
            'op': name,
            'label': spec['label'],
            'help': spec['help'],
            'args': list(spec['args']),
            'preview': preview,
            'timeout': spec.get('timeout', DEFAULT_TIMEOUT_S),
        })
    return out


def build_argv(op, args=None):
    """Resolve one operation to the exact argv, or refuse.

    Separate from run() so a test can assert on what *would* have run without
    a ROS graph, a Pi, or any process at all.
    """
    spec = OPS.get(op)
    if spec is None:
        raise ConsoleError(
            'operation %r is not on the allowlist. This console runs a fixed '
            'set of read-only commands and nothing else.' % (op,))

    args = list(args or [])
    wanted = spec['args']
    if len(args) != len(wanted):
        raise ConsoleError('%s needs %d argument(s): %s'
                           % (spec['label'], len(wanted),
                              ', '.join(wanted) or 'none'))
    clean = [validate_arg(a, w) for a, w in zip(args, wanted)]

    if 'build' in spec:
        argv = spec['build']()
    else:
        argv = [p.format(*clean) if '{' in p else p for p in spec['argv']]

    # Belt and braces. If a future edit to OPS ever lets an argument reach the
    # front of the list, this is what stops it being an arbitrary binary.
    if argv[0] not in EXECUTABLES:
        raise ConsoleError('internal: %r is not a permitted binary' % argv[0])
    return argv


def run_op(op, args=None):
    """Run one allowlisted operation. Returns a dict; never raises.

    Refusals come back as ok=False with a reason, because the page shows them
    in the same place as output and an exception there would just read
    "command failed".
    """
    try:
        argv = build_argv(op, args)
    except ConsoleError as exc:
        return {'ok': False, 'op': op, 'refused': True, 'error': str(exc)}

    if shutil.which(argv[0]) is None:
        return {'ok': False, 'op': op, 'cmd': argv, 'refused': False,
                'error': '%s is not installed on this machine' % argv[0]}

    timeout = OPS[op].get('timeout', DEFAULT_TIMEOUT_S)
    if not _slots.acquire(blocking=False):
        return {'ok': False, 'op': op, 'cmd': argv, 'refused': True,
                'error': 'two console commands are already running - wait for '
                         'one to finish'}
    started = time.monotonic()
    timed_out = False
    try:
        # start_new_session so the timeout can kill the whole group: `ros2
        # topic hz` is a python process that may have spawned its own, and
        # killing only the parent would leave those subscribed forever.
        proc = subprocess.Popen(argv, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True,
                                start_new_session=True)
        try:
            out, _ = proc.communicate(timeout=timeout)
        except subprocess.TimeoutExpired:
            timed_out = True
            _kill_group(proc)
            out, _ = proc.communicate()
            # Not an error for hz/echo: being stopped by the clock is how
            # those are meant to end, and what they printed first is the
            # answer. So the partial output is kept.
    except OSError as exc:
        return {'ok': False, 'op': op, 'cmd': argv, 'error': str(exc)}
    finally:
        _slots.release()

    out = out or ''
    truncated = len(out) > MAX_OUTPUT_CHARS
    if truncated:
        out = out[:MAX_OUTPUT_CHARS] + '\n... [truncated]'

    return {
        'ok': timed_out or proc.returncode == 0,
        'op': op,
        'cmd': argv,
        'output': out,
        'exit_code': proc.returncode,
        'timed_out': timed_out,
        'truncated': truncated,
        'seconds': round(time.monotonic() - started, 2),
    }


def _kill_group(proc):
    for sig in (signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(os.getpgid(proc.pid), sig)
        except (OSError, ProcessLookupError):
            return
        try:
            proc.wait(timeout=1.0)
            return
        except subprocess.TimeoutExpired:
            continue
