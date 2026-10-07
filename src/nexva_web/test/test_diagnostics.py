"""Tests for the diagnostics collector and the console allowlist.

Runs under pytest, and also standalone (`python3 test/test_diagnostics.py`)
so it can be run on the robot without pytest installed. Neither module under
test imports rclpy or aiohttp, which is the point: the console allowlist is
the security boundary of this package and it must be testable on a laptop,
on a Pi, and in CI, without a ROS graph.
"""

import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from nexva_web import console_ops                              # noqa: E402
from nexva_web import pi_health                                # noqa: E402


# --------------------------------------------------------------- allowlist

# Every one of these must be refused before anything is executed. The shell
# metacharacters are the obvious half; '..' (path traversal), a bare space
# (argument splitting) and a leading '-' (turning a value into a flag) are
# the half that gets forgotten.
BAD_ARGS = [
    '/scan; rm -rf /',
    '/scan && reboot',
    '/scan | tee /tmp/x',
    '/scan`id`',
    '/scan$(id)',
    '/scan $(id)',
    'two words',
    '../../etc/passwd',
    '/scan/../../../etc/shadow',
    '-f',
    '--all',
    '-rf',
    '/scan\nreboot',
    '/scan\x00',
    "/scan'",
    '/scan"',
    '/scan>out',
    '/scan<in',
    '/scan*',
    '/scan~',
    '/scan#',
    '',
    'x' * 201,
]

GOOD_ARGS = ['/scan', 'scan', '/amcl', 'use_sim_time', '/a_b/c_d',
             '/frontier_explorer/status_json', 'A1', 'x' * 200]


def test_bad_arguments_are_refused():
    for bad in BAD_ARGS:
        try:
            console_ops.validate_arg(bad, 'topic')
        except console_ops.ConsoleError:
            continue
        raise AssertionError('validate_arg accepted %r' % (bad,))


def test_bad_arguments_never_build_an_argv():
    """The refusal has to happen before the argv exists, not after."""
    for bad in BAD_ARGS:
        try:
            argv = console_ops.build_argv('topic_info', [bad])
        except console_ops.ConsoleError:
            continue
        raise AssertionError('built %r from %r' % (argv, bad))


def test_non_string_arguments_are_refused():
    for bad in (None, 5, ['/scan'], {'topic': '/scan'}, True):
        try:
            console_ops.validate_arg(bad, 'topic')
        except console_ops.ConsoleError:
            continue
        raise AssertionError('validate_arg accepted %r' % (bad,))


def test_good_arguments_are_accepted():
    for good in GOOD_ARGS:
        assert console_ops.validate_arg(good, 'topic') == good


def test_unknown_operation_is_refused():
    for op in ('rm', 'sh', 'ros2 topic pub', 'topic_pub', '', None,
               'topic_info; rm -rf /', '../console_ops'):
        try:
            console_ops.build_argv(op, ['/scan'])
        except console_ops.ConsoleError:
            continue
        raise AssertionError('build_argv accepted operation %r' % (op,))


def test_run_op_refuses_without_executing():
    result = console_ops.run_op('rm', [])
    assert result['ok'] is False
    assert result['refused'] is True
    assert 'cmd' not in result            # nothing was even assembled

    result = console_ops.run_op('topic_info', ['/scan; id'])
    assert result['ok'] is False
    assert result['refused'] is True
    assert 'cmd' not in result


def test_wrong_argument_count_is_refused():
    for op, args in [('topic_info', []), ('topic_info', ['/a', '/b']),
                     ('param_get', ['/amcl']), ('node_list', ['/amcl']),
                     ('df', ['-h'])]:
        try:
            console_ops.build_argv(op, args)
        except console_ops.ConsoleError:
            continue
        raise AssertionError('%s accepted %r' % (op, args))


def test_argv_is_built_as_a_list_with_a_permitted_binary():
    assert console_ops.build_argv('param_get', ['/amcl', 'use_sim_time']) == \
        ['ros2', 'param', 'get', '/amcl', 'use_sim_time']
    assert console_ops.build_argv('topic_hz', ['/scan']) == \
        ['ros2', 'topic', 'hz', '/scan']
    assert console_ops.build_argv('df', []) == ['df', '-h']

    for op, spec in console_ops.OPS.items():
        if 'build' in spec:
            continue
        args = ['/x'] * len(spec['args'])
        argv = console_ops.build_argv(op, args)
        assert isinstance(argv, list), op
        assert argv[0] in console_ops.EXECUTABLES, op
        # Nothing assembled here may contain a shell metacharacter, even in
        # the fixed parts of the table.
        for part in argv:
            assert not set(part) & set(';&|<>`$()\n\\"\''), (op, part)


def test_nothing_that_writes_is_on_the_allowlist():
    """The table is the boundary; this is the assertion that guards it."""
    banned = ('pub', 'run', 'launch', 'lifecycle', 'set', 'kill', 'remove',
              'delete', 'write', 'load', 'save', 'reboot', 'shutdown')
    for op, spec in console_ops.OPS.items():
        argv = spec.get('argv') or []
        for part in argv:
            assert part not in banned, '%s runs %r' % (op, part)
        assert 'sudo' not in argv, op
        assert 'rm' not in argv, op
        assert 'systemctl' not in argv, op


def test_describe_matches_what_will_run():
    for entry in console_ops.describe():
        spec = console_ops.OPS[entry['op']]
        if 'build' in spec:
            continue
        # The page shows `preview`; the server runs `argv`. If they ever come
        # apart, the page is lying about what it is about to do.
        assert entry['preview'] == spec['argv'], entry['op']
        assert entry['timeout'] > 0


def test_a_real_allowlisted_command_runs():
    result = console_ops.run_op('uptime', [])
    if 'uptime is not installed' in (result.get('error') or ''):
        return                                   # fine, nothing to prove
    assert result['ok'] is True, result
    assert result['cmd'] == ['uptime']
    assert 'load average' in result['output']


# ------------------------------------------------------------ throttling

def test_throttle_bitmask_decoding():
    assert pi_health.decode_throttled(0x0) == {
        'ok': True, 'raw': '0x0', 'now': [], 'since_boot': [],
        'healthy': True}

    now_only = pi_health.decode_throttled(0x1)
    assert now_only['now'] == ['under-voltage']
    assert now_only['since_boot'] == []
    assert now_only['healthy'] is False

    # The classic reading off a robot that dipped under load and recovered:
    # nothing wrong now, under-voltage and capped frequency since boot.
    past = pi_health.decode_throttled(0x50000)
    assert past['now'] == []
    assert past['since_boot'] == ['under-voltage', 'currently throttled']
    assert past['healthy'] is False

    both = pi_health.decode_throttled(0x50005)
    assert both['now'] == ['under-voltage', 'currently throttled']
    assert both['since_boot'] == ['under-voltage', 'currently throttled']
    assert both['raw'] == '0x50005'

    allbits = pi_health.decode_throttled(0xF000F)
    assert allbits['now'] == ['under-voltage', 'ARM frequency capped',
                              'currently throttled', 'soft temperature limit']
    assert allbits['since_boot'] == allbits['now']

    assert pi_health.decode_throttled(0x8)['now'] == ['soft temperature limit']
    assert pi_health.decode_throttled(0x80000)['since_boot'] == \
        ['soft temperature limit']


def _without_vcgencmd(fn):
    """Run fn with every vcgencmd path pointed at nothing - i.e. not a Pi."""
    saved = pi_health.VCGENCMD_PATHS
    pi_health.VCGENCMD_PATHS = ('/nonexistent/vcgencmd',)
    try:
        return fn()
    finally:
        pi_health.VCGENCMD_PATHS = saved


def test_missing_vcgencmd_reports_unavailable_and_does_not_raise():
    assert _without_vcgencmd(lambda: pi_health.vcgencmd('get_throttled')) is None

    result = _without_vcgencmd(pi_health.throttling)
    assert result['ok'] is False
    assert 'vcgencmd' in result['why']
    # The thing that must never happen: a missing source inventing a value.
    assert 'now' not in result
    assert 'since_boot' not in result
    assert 'raw' not in result


def test_missing_temperature_sources_report_unavailable():
    saved = pi_health.thermal_zones
    pi_health.thermal_zones = lambda: []
    try:
        result = _without_vcgencmd(pi_health.temperature)
    finally:
        pi_health.thermal_zones = saved
    assert result['ok'] is False
    assert 'celsius' not in result
    assert result['why']


def test_unparseable_vcgencmd_output_is_not_guessed():
    saved = pi_health.vcgencmd
    pi_health.vcgencmd = lambda *a, **k: 'throttled=banana'
    try:
        result = pi_health.throttling()
    finally:
        pi_health.vcgencmd = saved
    assert result['ok'] is False
    assert 'parse' in result['why']


# ----------------------------------------------------------------- health

def test_sections_all_answer_ok_or_why():
    monitor = pi_health.HealthMonitor()
    monitor._cpu_prev_at -= 10.0            # pretend a real window has passed
    snapshot = monitor.snapshot()
    for key in ('cpu', 'temperature', 'throttling', 'memory', 'disk',
                'uptime'):
        section = snapshot[key]
        assert isinstance(section, dict), key
        assert 'ok' in section, key
        if not section['ok']:
            assert section.get('why'), '%s is not ok but says no why' % key
    assert snapshot['hostname']


def test_snapshot_is_json_serializable():
    """It goes out over a WebSocket; one odd value would drop every client."""
    monitor = pi_health.HealthMonitor()
    json.loads(json.dumps(monitor.snapshot()))


def test_snapshot_never_raises_when_every_source_is_missing():
    saved = (pi_health._read, pi_health.VCGENCMD_PATHS, pi_health.thermal_zones)
    pi_health._read = lambda path: None             # nothing is readable
    pi_health.VCGENCMD_PATHS = ('/nonexistent/vcgencmd',)
    pi_health.thermal_zones = lambda: []
    try:
        monitor = pi_health.HealthMonitor()
        snapshot = monitor.snapshot()
    finally:
        (pi_health._read, pi_health.VCGENCMD_PATHS,
         pi_health.thermal_zones) = saved
    for key in ('cpu', 'temperature', 'throttling', 'memory', 'uptime'):
        assert snapshot[key]['ok'] is False, key
        assert snapshot[key]['why'], key
    json.dumps(snapshot)


def test_cpu_needs_two_samples_before_it_claims_a_number():
    monitor = pi_health.HealthMonitor()
    first = monitor.cpu()                   # same instant as __init__'s sample
    if first['ok']:
        raise AssertionError('reported CPU load from a zero-length window')
    assert 'sample' in first['why']

    monitor._cpu_prev_at -= 10.0
    second = monitor.cpu()
    if second['ok']:
        assert 0.0 <= second['total'] <= 100.0
        # A parked core reports None rather than a made-up 0%.
        assert all(v is None or 0.0 <= v <= 100.0 for v in second['per_core'])
        assert second['cores'] == len(second['per_core'])


def test_busy_percentage_arithmetic():
    #            user nice sys idle iowait irq softirq steal
    before = [100, 0, 50, 1000, 0, 0, 0, 0]
    after = [200, 0, 50, 1000, 0, 0, 0, 0]      # 100 busy jiffies, 0 idle
    assert pi_health._busy_pct(before, after) == 100.0
    after = [100, 0, 50, 1100, 0, 0, 0, 0]      # 100 idle jiffies, 0 busy
    assert pi_health._busy_pct(before, after) == 0.0
    after = [150, 0, 50, 1050, 0, 0, 0, 0]
    assert pi_health._busy_pct(before, after) == 50.0
    assert pi_health._busy_pct(before, before) is None   # no elapsed time


def test_memory_and_disk_and_uptime_read_this_machine():
    mem = pi_health.memory()
    if mem['ok']:
        assert mem['total'] > 0
        assert 0 <= mem['used'] <= mem['total']
        assert mem['available'] <= mem['total']
        assert 0.0 <= mem['pct'] <= 100.0

    disk = pi_health.disk()
    assert disk['ok'] is True
    assert disk['total'] > 0
    assert 0 <= disk['free'] <= disk['total']

    up = pi_health.uptime()
    if up['ok']:
        assert up['seconds'] > 0


def test_disk_on_a_missing_path_degrades():
    result = pi_health.disk('/no/such/path/anywhere')
    assert result['ok'] is False
    assert 'why' in result


def test_format_uptime():
    assert pi_health.format_uptime(59) == '0m'
    assert pi_health.format_uptime(3600) == '1h 0m'
    assert pi_health.format_uptime(90061) == '1d 1h 1m'


if __name__ == '__main__':
    failures = 0
    for name, fn in sorted(globals().items()):
        if not name.startswith('test_') or not callable(fn):
            continue
        try:
            fn()
            print('ok    %s' % name)
        except Exception as exc:                            # noqa: BLE001
            failures += 1
            print('FAIL  %s: %s' % (name, exc))
    print('\n%d failed' % failures if failures else '\nall passed')
    sys.exit(1 if failures else 0)
