"""Bridge logic against a stub node and stub sockets - no ROS graph, no aiohttp.

Covers what keeps the page responsive: events are batched and coalesced, a
stuck client cannot hold up the others, a slow command cannot hold up STOP,
the map endpoints never wait for a map_server, and errors name the command
that caused them. Runs under pytest or standalone (python3 test/test_web_bridge.py).
"""

import asyncio
import json
import math
import os
import sys
import tempfile
import threading
import time
import types

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _stubs                                                   # noqa: E402

_stubs.install_aiohttp_stub()

from nexva_web import web_bridge                                # noqa: E402
from nexva_web import waypoints as wp_mod                       # noqa: E402


class StubNode:

    def __init__(self):
        self.on_event = None
        self.calls = []
        self.waypoints = types.SimpleNamespace(as_dict=lambda: {'items': []})
        self.errors = []

    def get_logger(self):
        return types.SimpleNamespace(
            error=lambda m: self.errors.append(m), info=lambda m: None,
            warn=lambda m: None)

    def state(self):
        return {'type': 'state'}

    def recent_logs(self, since=0, limit=400):
        return []

    def teleop_stop(self):
        self.calls.append('teleop_stop')

    def estop(self):
        self.calls.append('estop')
        return True


def make_bridge(node=None, clients=(), pools=None):
    loop = asyncio.get_running_loop()
    node = node or StubNode()
    bridge = web_bridge.Bridge(node, loop, health=None, pools=pools)
    for ws in clients:
        bridge.clients.add(ws)
    return bridge, node


def emit_from_thread(node, events, gap_every=0, gap=0.0):
    def run():
        for i, ev in enumerate(events):
            node.on_event(ev)
            if gap_every and i % gap_every == gap_every - 1:
                time.sleep(gap)
    t = threading.Thread(target=run)
    t.start()
    return t


# --------------------------------------------------------------- batching

def test_events_are_batched_at_five_hz_and_nothing_is_lost():
    async def main():
        a, b = _stubs.FakeWS(), _stubs.FakeWS()
        bridge, node = make_bridge(clients=(a, b))
        events = []
        for i in range(1, 601):
            events.append({'type': 'log', 'seq': i, 'msg': 'line %d' % i})
            events.append({'type': 'pose', 'x': float(i), 'y': 0.0, 'yaw': 0.0})
            events.append({'type': 'pose', 'x': float(i) + 0.5, 'y': 0.0,
                           'yaw': 0.0})
        t0 = time.monotonic()
        # 1800 events over ~0.6 s from a ROS-style foreign thread
        t = emit_from_thread(node, events, gap_every=60, gap=0.02)
        # join off the loop: the loop must stay free to flush while it runs
        await asyncio.get_running_loop().run_in_executor(None, t.join)
        await asyncio.sleep(0.5)                     # let the last flush run
        elapsed = time.monotonic() - t0
        for ws in (a, b):
            frames = [m for m in ws.sent if m['type'] == 'batch']
            assert len(frames) == len(ws.sent)       # nothing sent unbatched
            # at most 5 frames/second (+1 for the leading edge)
            assert len(frames) <= math.ceil(elapsed * 5) + 1, \
                (len(frames), elapsed)
            logs = ws.items('log')
            assert [l['seq'] for l in logs] == list(range(1, 601)), 'log lost'
            poses = ws.items('pose')
            # newest wins: one pose per frame at most, and the last one is the
            # last one emitted
            assert len(poses) <= len(frames)
            assert poses[-1]['x'] == 600.5
        # the old path was one thread hop + dump + send per event per client
        assert bridge.flushes <= math.ceil(elapsed * 5) + 1
        return len(a.sent), len(events)
    sent, n = asyncio.run(main())
    assert sent < n / 50, (sent, n)


def test_log_burst_beyond_the_cap_keeps_the_newest_in_order():
    async def main():
        ws = _stubs.FakeWS()
        bridge, node = make_bridge(clients=(ws,))
        cap = web_bridge.APPEND_CAPS['log']
        for i in range(1, cap * 3 + 1):
            bridge._on_ros_event({'type': 'log', 'seq': i})
        await asyncio.sleep(0.3)
        seqs = [l['seq'] for l in ws.items('log')]
        assert seqs == list(range(cap * 2 + 1, cap * 3 + 1))
    asyncio.run(main())


def test_the_two_feedback_shapes_do_not_erase_each_other():
    async def main():
        ws = _stubs.FakeWS()
        bridge, node = make_bridge(clients=(ws,))
        bridge._on_ros_event({'type': 'feedback', 'distance_remaining': 1.0})
        bridge._on_ros_event({'type': 'feedback', 'waypoint_index': 2,
                              'waypoint_name': 'P3', 'total': 4})
        bridge._on_ros_event({'type': 'feedback', 'distance_remaining': 0.5})
        await asyncio.sleep(0.3)
        fb = ws.items('feedback')
        assert len(fb) == 2
        nav = [f for f in fb if 'distance_remaining' in f][0]
        assert nav['distance_remaining'] == 0.5
        assert any(f.get('waypoint_name') == 'P3' for f in fb)
    asyncio.run(main())


def test_results_are_never_collapsed():
    async def main():
        ws = _stubs.FakeWS()
        bridge, _ = make_bridge(clients=(ws,))
        for st in ('ABORTED', 'SUCCEEDED', 'CANCELED'):
            bridge._on_ros_event({'type': 'result', 'status': st})
        bridge._on_ros_event({'type': 'state', 'a': 1})
        bridge._on_ros_event({'type': 'state', 'a': 2})
        await asyncio.sleep(0.3)
        assert [r['status'] for r in ws.items('result')] == \
            ['ABORTED', 'SUCCEEDED', 'CANCELED']
        assert [s['a'] for s in ws.items('state')] == [2]
    asyncio.run(main())


def test_with_no_clients_events_cost_nothing():
    async def main():
        bridge, node = make_bridge()
        for i in range(1000):
            node.on_event({'type': 'log', 'seq': i})
        await asyncio.sleep(0.3)
        assert bridge.flushes == 0
        assert not bridge._latest and not bridge._append
    asyncio.run(main())


def test_a_stuck_client_does_not_delay_the_others_and_is_dropped():
    saved = web_bridge.SEND_TIMEOUT_S
    web_bridge.SEND_TIMEOUT_S = 0.3

    async def main():
        good, stuck = _stubs.FakeWS(), _stubs.FakeWS(stall=True)
        bridge, _ = make_bridge(clients=(good, stuck))
        loop = asyncio.get_running_loop()
        t0 = loop.time()
        bridge._on_ros_event({'type': 'state', 'n': 1})
        await asyncio.sleep(0.15)
        assert good.sent, 'healthy client was held up by the stuck one'
        assert good.sent_at[0] - t0 < 0.15
        await asyncio.sleep(0.5)
        assert stuck not in bridge.clients and stuck.closed
        assert good in bridge.clients
    try:
        asyncio.run(main())
    finally:
        web_bridge.SEND_TIMEOUT_S = saved


# ------------------------------------------------------------------- JSON

def test_dumps_never_emits_nan():
    out = web_bridge.dumps({'a': float('nan'), 'b': [1.0, float('inf')],
                            'c': {'d': -float('inf')}})
    parsed = json.loads(out)                  # JSON.parse would reject NaN
    assert parsed == {'a': None, 'b': [1.0, None], 'c': {'d': None}}
    assert 'NaN' not in out and 'Infinity' not in out
    import numpy as np
    parsed = json.loads(web_bridge.dumps({'x': np.float64('nan'),
                                          'y': np.array([1.0, 2.0])}))
    assert parsed == {'x': None, 'y': [1.0, 2.0]}


# --------------------------------------------------------------- dispatch

def test_errors_name_the_command_that_caused_them():
    async def main():
        ws = _stubs.FakeWS()
        bridge, node = make_bridge(clients=(ws,))

        def refuse(*a):
            raise wp_mod.WaypointError('AMCL is not running')
        node.set_pose_at = refuse
        node.goto_pose = refuse
        await bridge.dispatch(ws, {'cmd': 'set_pose', 'x': 1, 'y': 2,
                                   'yaw': 0})
        await bridge.dispatch(ws, {'cmd': 'goto_pose', 'x': 1, 'y': 2,
                                   'yaw': 0})
        await bridge.dispatch(ws, {'cmd': 'nonsense'})
        errs = [m for m in ws.sent if m['type'] == 'error']
        assert [(e['cmd'], e['msg']) for e in errs[:2]] == \
            [('set_pose', 'AMCL is not running'),
             ('goto_pose', 'AMCL is not running')]
        assert errs[2]['cmd'] == 'nonsense'
    asyncio.run(main())


def test_new_commands_reach_the_node_and_reply():
    async def main():
        ws = _stubs.FakeWS()
        bridge, node = make_bridge(clients=(ws,))
        node.goto_pose = lambda x, y, yaw: node.calls.append(
            ('goto_pose', x, y, yaw)) or 'point (1.00, 2.00)'
        node.tour_points = lambda pts: node.calls.append(
            ('tour_points', pts)) or 'queue of 2 points'
        await bridge.dispatch(ws, {'cmd': 'goto_pose', 'x': 1.0, 'y': 2.0,
                                   'yaw': 0.5})
        await bridge.dispatch(ws, {'cmd': 'tour_points',
                                   'points': [[0, 0], [1, 1, 0.3]]})
        assert ('goto_pose', 1.0, 2.0, 0.5) in node.calls
        assert ('tour_points', [[0, 0], [1, 1, 0.3]]) in node.calls
        acks = [m for m in ws.sent if m['type'] == 'ack']
        assert acks[0]['cmd'] == 'goto_pose' and 'point (1.00' in acks[0]['detail']
        assert acks[1]['cmd'] == 'tour_points' and 'queue of 2' in acks[1]['detail']
    asyncio.run(main())


def test_zone_ack_reports_who_received_it():
    async def main():
        ws = _stubs.FakeWS()
        bridge, node = make_bridge(clients=(ws,))
        node.send_zone = lambda pts: 2
        node.zone_command = lambda c: 1
        await bridge.dispatch(ws, {'cmd': 'zone',
                                   'points': [[0, 0], [1, 0], [1, 1]]})
        await bridge.dispatch(ws, {'cmd': 'zone_cmd', 'command': 'plan drawn'})
        a, b = [m for m in ws.sent if m['type'] == 'ack']
        assert a['listeners'] == 2 and 'delivered to 2' in a['detail']
        assert b['listeners'] == 1
        # and when nobody is there the node raises, so no ack is sent at all
        ws2 = _stubs.FakeWS()

        def nobody(*a):
            raise wp_mod.WaypointError('zone_coverage is not running')
        node.send_zone = nobody
        await bridge.dispatch(ws2, {'cmd': 'zone', 'points': [[0, 0]] * 3})
        assert ws2.sent[0]['type'] == 'error'
        assert ws2.sent[0]['cmd'] == 'zone'
        assert 'zone_coverage is not running' in ws2.sent[0]['msg']
    asyncio.run(main())


def test_set_pose_replies_sent_first_then_the_settle_result():
    async def main():
        ws = _stubs.FakeWS()
        bridge, node = make_bridge(clients=(ws,))
        node.set_pose_at = lambda x, y, yaw, settle: node.calls.append(
            ('set_pose_at', x, y, yaw, settle)) or False
        node.wait_localized = lambda t: (time.sleep(0.3), True)[1]
        t0 = time.monotonic()
        task = asyncio.ensure_future(bridge.dispatch(
            ws, {'cmd': 'set_pose', 'x': 1.5, 'y': -2.0, 'yaw': 0.25}))
        await asyncio.sleep(0.1)
        # the "sent" frame is out before the 0.3 s settle wait has finished
        assert ws.sent and ws.sent[0]['type'] == 'initial_pose'
        assert ws.sent[0]['stage'] == 'sent'
        assert time.monotonic() - t0 < 0.25
        await task
        assert ws.sent[1]['stage'] == 'done' and ws.sent[1]['localized'] is True
        assert node.calls == [('set_pose_at', 1.5, -2.0, 0.25, 0.0)]
    asyncio.run(main())


# ------------------------------------------------- the websocket handler

def _run_ws_handler(script, node, pools=None):
    """Drive web_bridge.ws_handler with scripted client messages."""
    async def main():
        ws = _stubs.FakeWS(script)

        async def prepare(request):
            return None
        ws.prepare = prepare
        saved = web_bridge.web.WebSocketResponse
        web_bridge.web.WebSocketResponse = lambda **kw: ws
        try:
            bridge, _ = make_bridge(node, pools=pools)
            request = types.SimpleNamespace(app={'bridge': bridge})
            await web_bridge.ws_handler(request)
            # the handler cancels what is still in flight when the client
            # leaves; let tasks that finished meanwhile deliver their replies
            await asyncio.sleep(0.05)
        finally:
            web_bridge.web.WebSocketResponse = saved
        return ws
    return asyncio.run(main())


def test_estop_is_not_stuck_behind_a_slow_command_from_the_same_client():
    saved = web_bridge.console_ops.run_op

    def slow_console(op, args):
        time.sleep(1.0)
        return {'ok': True, 'output': 'late'}
    web_bridge.console_ops.run_op = slow_console
    try:
        node = StubNode()
        # console first, STOP 50 ms later; the client then stays connected
        # long enough for the console to finish
        t0 = time.monotonic()
        ws = _run_ws_handler([(0, {'cmd': 'console', 'op': 'uptime'}),
                              (0.05, {'cmd': 'estop'}),
                              (1.3, {'cmd': 'state'})], node)
        order = [m['type'] for m in ws.sent]
        assert 'ack' in order and 'console_result' in order, order
        assert order.index('ack') < order.index('console_result'), order
        assert 'estop' in node.calls
    finally:
        web_bridge.console_ops.run_op = saved


def test_a_new_client_is_seeded_with_state_logs_and_imu():
    node = StubNode()
    node.imu_event = lambda: {'type': 'imu', 'present': False, 'fresh': False}
    ws = _run_ws_handler([], node)
    assert ws.types()[:4] == ['waypoints', 'state', 'log_history', 'imu']


def test_disconnect_zeroes_teleop_without_sleeping_on_the_loop():
    node = StubNode()
    ws = _run_ws_handler([(0, {'cmd': 'teleop', 'vx': 0.1, 'wz': 0})],
                         types.SimpleNamespace(**{}) if False else node)
    assert 'teleop_stop' in node.calls


def test_pools_isolate_safety_from_slow_ros_calls():
    async def main():
        bridge, node = make_bridge()
        loop = asyncio.get_running_loop()
        slow = [loop.run_in_executor(bridge.pools.ros, time.sleep, 0.6)
                for _ in range(8)]                    # saturate the ros pool
        t0 = loop.time()
        await bridge._blocking(node.estop, pool='safety')
        took = loop.time() - t0
        assert took < 0.25, 'estop queued behind slow ROS calls (%.2fs)' % took
        await asyncio.gather(*slow)
    asyncio.run(main())


# ----------------------------------------------------------- map handlers

def test_map_endpoints_never_wait_for_a_map_server():
    seen = []

    class Node:
        def check_map(self, timeout=5.0, wait=True):
            seen.append((timeout, wait))
            return None, 'map_server not running'
    t0 = time.monotonic()
    status, payload = web_bridge._map_meta_blocking(Node())
    status2, body = web_bridge._map_png_blocking(Node())
    assert (status, status2) == (503, 503)
    assert time.monotonic() - t0 < 0.1
    assert seen and all(w is False for (_, w) in seen), seen


def test_map_meta_reads_the_files():
    with tempfile.TemporaryDirectory() as d:
        with open(os.path.join(d, 'm.pgm'), 'wb') as fh:
            fh.write(b'P5\n4 3\n255\n' + bytes(12))
        yml = os.path.join(d, 'm.yaml')
        with open(yml, 'w') as fh:
            fh.write('image: m.pgm\nresolution: 0.05\norigin: [-1.0, -2.0, 0]\n')

        class Node:
            def check_map(self, timeout=5.0, wait=True):
                return True, yml
        status, p = web_bridge._map_meta_blocking(Node())
        assert status == 200
        assert (p['width'], p['height'], p['name']) == (4, 3, 'm')
        assert p['origin'] == [-1.0, -2.0] and p['resolution'] == 0.05


def test_health_snapshot_is_safe_from_two_threads():
    from nexva_web import pi_health
    mon = pi_health.HealthMonitor()
    errs = []

    def hammer():
        try:
            for _ in range(20):
                json.dumps(mon.snapshot())
        except Exception as exc:                            # noqa: BLE001
            errs.append(exc)
    ts = [threading.Thread(target=hammer) for _ in range(4)]
    [t.start() for t in ts]
    [t.join() for t in ts]
    assert not errs


if __name__ == '__main__':
    failures = 0
    for name, fn in sorted(globals().items()):
        if not name.startswith('test_') or not callable(fn):
            continue
        try:
            fn()
            print('ok    %s' % name)
        except Exception as exc:                            # noqa: BLE001
            import traceback
            traceback.print_exc()
            failures += 1
            print('FAIL  %s: %s' % (name, exc))
    print('\n%d failed' % failures if failures else '\nall passed')
    sys.exit(1 if failures else 0)


def test_delete_map_command_replies_with_outcome_and_passes_robot_mode():
    from nexva_web import map_admin
    seen = {}

    def fake_delete(name, source, root, robot_mode):
        seen.update(name=name, mode=robot_mode)
        if name == 'bundled':
            raise map_admin.MapDeleteError('refused: bundled')
        return {'ok': True, 'name': name, 'deleted': ['a.yaml'],
                'registry': 'registry untouched'}

    real = map_admin.delete_map
    map_admin.delete_map = fake_delete

    async def go():
        node = StubNode()
        node.robot_mode = {'mode': 'idle'}
        bridge, _ = make_bridge(node)
        try:
            ok = await bridge.handle(None, {'cmd': 'delete_map', 'name': 'x'})
            bad = await bridge.handle(None, {'cmd': 'delete_map',
                                             'name': 'bundled'})
            return ok, bad
        finally:
            bridge.pools.shutdown()
    try:
        ok, bad = asyncio.run(go())
    finally:
        map_admin.delete_map = real
    assert ok['type'] == 'map_deleted' and ok['ok'] and seen['mode'] == {'mode': 'idle'}
    assert bad == {'type': 'map_deleted', 'ok': False, 'name': 'bundled',
                   'error': 'refused: bundled'}
