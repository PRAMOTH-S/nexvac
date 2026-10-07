"""aiohttp server exposing the waypoint client over WebSocket.

rclpy spins on its own thread; aiohttp owns the main thread's event loop. The
node pushes events through `on_event`, which runs on a ROS callback thread, so
it hands them to the loop rather than touching asyncio state directly. Nothing
here calls rclpy - that all lives in NavClient.

Two rules keep the page responsive on a Pi:

  * Nothing that can block runs on the event loop. Blocking ROS calls, health
    snapshots, console operations and file/image work each go to a dedicated
    worker pool (`Pools`). They used to share asyncio's one default pool, so a
    console command (8 s) or a map probe (5 s) could starve everything else,
    and an `estop` could queue behind them.
  * Events are batched. A /rosout entry or an /amcl_pose tick used to cost one
    thread hop, one JSON dump and one send per client *each*. They are now
    coalesced and flushed at most FLUSH_HZ times a second as one message,
    {"type": "batch", "items": [...]}: log lines are all kept, in order;
    high-rate state (pose, feedback, status...) keeps only the newest.
"""

import argparse
import asyncio
import collections
import concurrent.futures
import itertools
import json
import math
import os
import sys
import threading

import rclpy
from aiohttp import WSMsgType, web
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor

from . import console_ops
from . import pi_health
from . import waypoints as wp_mod
from .nav_client import NavClient

# At most this many batched frames per second reach each browser.
FLUSH_HZ = 5.0
FLUSH_INTERVAL_S = 1.0 / FLUSH_HZ

# Event types that must not be collapsed to "the newest": every one matters.
# Value = how many are kept per flush if a burst exceeds it (oldest dropped;
# the node's own ring still holds the logs, see recent_logs).
APPEND_CAPS = {'log': 400, 'result': 50, 'save_result': 20, 'robot_mode': 30}

# A client that cannot take a frame in this long is stuck (phone asleep, wifi
# gone). Waiting on it would hold up every other client's frame.
SEND_TIMEOUT_S = 3.0

# Commands one client may have running at once.
MAX_INFLIGHT_PER_CLIENT = 16

# Commands handled inline, in arrival order: hot path and cheap (no sleeping).
INLINE_COMMANDS = ('teleop', 'teleop_stop', 'list', 'state', 'logs')


def _json_safe(obj):
    """Last-resort coercion for values json.dumps cannot handle.

    ROS messages carry numpy scalars - float64[] fields deserialize as numpy
    arrays - and one of those reaching the encoder used to raise inside the
    WebSocket send path. That drops the client the instant it connects and
    looks exactly like "the server is down". One odd value must never cost the
    whole connection.
    """
    if hasattr(obj, 'item'):              # numpy scalar -> Python scalar
        try:
            return obj.item()
        except Exception:
            pass
    if hasattr(obj, 'tolist'):            # numpy array -> list
        try:
            return obj.tolist()
        except Exception:
            pass
    return str(obj)


def _scrub(obj):
    """Replace NaN/Infinity (not valid JSON) with None, recursively."""
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None
    if isinstance(obj, dict):
        return {k: _scrub(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_scrub(v) for v in obj]
    if hasattr(obj, 'item') or hasattr(obj, 'tolist'):
        return _scrub(_json_safe(obj))
    return obj


def dumps(obj) -> str:
    """JSON the browser's JSON.parse will accept.

    Python happily writes NaN, which JSON.parse rejects - and the page then
    loses that whole frame (a batch carries many events). allow_nan=False
    turns that into an error here, where it is cleaned up and retried.
    """
    try:
        return json.dumps(obj, default=_json_safe, allow_nan=False)
    except ValueError:
        return json.dumps(_scrub(obj), default=_json_safe, allow_nan=False)


class Pools:
    """Dedicated worker pools, so one kind of slow work cannot starve another.

    ros     blocking ROS calls: pose, goals, zones, mission commands, check_map
    safety  estop / cancel: never queue behind anything slow
    health  pi_health.snapshot (vcgencmd is a subprocess)
    console allowlisted console operations, up to 8 s each
    io      map files, cv2, live-map encoding, the saved-map library
    """

    def __init__(self):
        def mk(n, name):
            return concurrent.futures.ThreadPoolExecutor(
                max_workers=n, thread_name_prefix=name)
        self.ros = mk(4, 'web-ros')
        self.safety = mk(2, 'web-safety')
        self.health = mk(1, 'web-health')
        self.console = mk(2, 'web-console')
        self.io = mk(2, 'web-io')

    def shutdown(self):
        for pool in (self.ros, self.safety, self.health, self.console,
                     self.io):
            pool.shutdown(wait=False, cancel_futures=True)


class Bridge:

    def __init__(self, node, loop, health, pools=None):
        self.node = node
        self.loop = loop
        self.health = health
        self.pools = pools or Pools()
        self.clients = set()

        # Event batching. _on_ros_event is called from ROS threads; everything
        # it touches is under _plock. _armed means a flush is already queued.
        self._plock = threading.Lock()
        self._latest = collections.OrderedDict()    # key -> newest event
        self._append = {}                           # type -> deque[(n, event)]
        self._n = itertools.count()
        self._armed = False
        self._last_flush = float('-inf')
        self.flushes = 0                            # for tests / diagnostics
        node.on_event = self._on_ros_event

    # ROS callback thread -> asyncio loop
    def _on_ros_event(self, event):
        if not self.clients:
            # Nobody to tell. A client that connects later is brought up to
            # date by ws_handler (state, log history, imu), not by replay.
            return
        kind = event.get('type')
        cap = APPEND_CAPS.get(kind)
        with self._plock:
            if cap:
                q = self._append.get(kind)
                if q is None:
                    q = self._append[kind] = collections.deque(maxlen=cap)
                q.append((next(self._n), event))
            else:
                if kind == 'feedback':
                    # navigate_to_pose and follow_waypoints feedback differ in
                    # shape; keep the newest of each rather than letting one
                    # erase the other.
                    key = ('feedback', 'distance_remaining' in event)
                else:
                    key = kind
                self._latest[key] = event
            if self._armed:
                return
            self._armed = True
        try:
            self.loop.call_soon_threadsafe(self._arm_flush)
        except RuntimeError:                  # loop already closed: shutdown
            pass

    def _arm_flush(self):                     # on the loop
        delay = max(0.0, self._last_flush + FLUSH_INTERVAL_S - self.loop.time())
        self.loop.call_later(delay, self._flush)

    def _flush(self):                         # on the loop
        with self._plock:
            appended = sorted(itertools.chain.from_iterable(
                self._append.values()), key=lambda pair: pair[0])
            items = [ev for _, ev in appended] + list(self._latest.values())
            self._append.clear()
            self._latest.clear()
            self._armed = False
        self._last_flush = self.loop.time()
        self.flushes += 1
        if items and self.clients:
            asyncio.ensure_future(self.broadcast({'type': 'batch',
                                                  'items': items}))

    async def _send_text(self, ws, text):
        try:
            await asyncio.wait_for(ws.send_str(text), SEND_TIMEOUT_S)
        except asyncio.TimeoutError:
            # Stuck. Drop it so it stops costing everyone else a wait; the
            # page reconnects and is resynced from scratch.
            self.clients.discard(ws)
            try:
                asyncio.ensure_future(ws.close())
            except Exception:                              # noqa: BLE001
                pass
        except Exception:                                  # noqa: BLE001
            self.clients.discard(ws)

    async def broadcast(self, message):
        clients = list(self.clients)
        if not clients:
            return
        text = dumps(message)
        # Concurrently: one slow client must not delay the rest.
        await asyncio.gather(*(self._send_text(ws, text) for ws in clients))

    async def send(self, ws, message):
        await self._send_text(ws, dumps(message))

    async def _blocking(self, fn, *args, pool='ros'):
        return await self.loop.run_in_executor(
            getattr(self.pools, pool), fn, *args)

    async def handle(self, ws, msg):
        """Dispatch one client command. Returns the reply to send back."""
        cmd = msg.get('cmd')

        if cmd == 'list':
            payload = self.node.waypoints.as_dict()
            payload['type'] = 'waypoints'
            return payload

        if cmd == 'state':
            return self.node.state()

        if cmd == 'check_map':
            ok, detail = await self._blocking(self.node.check_map, 2.0)
            return {'type': 'map_check', 'ok': ok, 'detail': detail}

        if cmd == 'set_initial_pose':
            name = msg.get('waypoint')
            if not msg.get('confirmed'):
                return {'type': 'error', 'cmd': cmd,
                        'msg': 'initial pose needs explicit confirmation that '
                               'the robot is physically at that waypoint'}
            ok = await self._blocking(self.node.set_initial_pose, name)
            return {'type': 'initial_pose', 'waypoint': name, 'localized': ok}

        if cmd == 'set_pose':
            # A spot picked on the map canvas: x, y in map metres, yaw in
            # radians. Checked against the map in the node before it is
            # published - a pose seeded inside a wall fails silently.
            #
            # Two replies. The first goes out as soon as the pose has been
            # published (a fraction of a second, or an error at once if AMCL
            # is not running); the second follows once localization has had
            # time to settle. The person gets an answer immediately instead
            # of after the settle wait.
            await self._blocking(
                self.node.set_pose_at,
                msg.get('x'), msg.get('y'), msg.get('yaw'), 0.0)
            await self.send(ws, {
                'type': 'initial_pose', 'stage': 'sent',
                'waypoint': 'picked on the map',
                'x': float(msg['x']), 'y': float(msg['y']),
                'yaw': float(msg['yaw'])})
            ok = await self._blocking(self.node.wait_localized, 2.0)
            return {'type': 'initial_pose', 'stage': 'done',
                    'waypoint': 'picked on the map', 'localized': ok}

        if cmd == 'goto':
            await self._blocking(self.node.goto, msg.get('waypoint'))
            return None

        if cmd == 'goto_pose':
            # One drag on the map: x, y in map metres, yaw in radians.
            label = await self._blocking(
                self.node.goto_pose,
                msg.get('x'), msg.get('y'), msg.get('yaw'))
            return {'type': 'ack', 'cmd': 'goto_pose',
                    'detail': 'goal sent to Nav2: %s' % label}

        if cmd == 'tour':
            names = msg.get('waypoints') or []
            loops = int(msg.get('loops', 0))
            await self._blocking(self.node.tour, names, loops)
            return None

        if cmd == 'tour_points':
            # [[x, y], [x, y, yaw], ...] dropped on the map; yaw optional.
            label = await self._blocking(
                self.node.tour_points, msg.get('points'))
            return {'type': 'ack', 'cmd': 'tour_points',
                    'detail': '%s sent to follow_waypoints' % label}

        if cmd == 'zone':
            # Rectangle dragged on the map: [[x,y],...] already in map metres.
            # The node raises if nothing is subscribed, so an ack here means
            # somebody actually received it.
            listeners = await self._blocking(
                self.node.send_zone, msg.get('points'))
            return {'type': 'ack', 'cmd': 'zone',
                    'points': len(msg.get('points') or []),
                    'listeners': listeners,
                    'detail': 'zone delivered to %d listener(s)' % listeners}

        if cmd == 'zone_cmd':
            listeners = await self._blocking(
                self.node.zone_command, msg.get('command'))
            return {'type': 'ack', 'cmd': 'zone_cmd',
                    'command': msg.get('command'), 'listeners': listeners,
                    'detail': 'delivered to %d listener(s)' % listeners}

        if cmd == 'teleop':
            # Hot path: the browser sends these continuously while the stick is
            # held. Publishing is done by a fixed-rate timer in the node, so
            # this just records the latest command and answers nothing.
            self.node.teleop(msg.get('vx', 0.0), msg.get('wz', 0.0))
            return None

        if cmd == 'teleop_stop':
            self.node.teleop_stop()
            return {'type': 'ack', 'cmd': 'teleop_stop'}

        if cmd == 'cancel':
            cancelled = await self._blocking(self.node.cancel, pool='safety')
            return {'type': 'ack', 'cmd': 'cancel', 'was_active': cancelled}

        if cmd == 'pid_limits':
            limits = await self._blocking(
                self.node.set_pid_limits,
                msg.get('min_pwm'), msg.get('max_pwm'), msg.get('max_speed'))
            return {'type': 'pid_limits', **limits}

        if cmd == 'pid_limits_reset':
            limits = await self._blocking(self.node.reset_pid_limits)
            return {'type': 'pid_limits', **limits}

        if cmd == 'set_mode':
            # {mode, map, new} -> mode_manager on /set_robot_mode. Only ever
            # sent from a click on the page, never on connect.
            mode = msg.get('mode')
            map_name = msg.get('map') or ''
            await self._blocking(self.node.set_robot_mode, mode, map_name,
                                 bool(msg.get('new')))
            return {'type': 'ack', 'cmd': 'set_mode', 'mode': mode,
                    'map': map_name}

        if cmd == 'save_map':
            # {name?} -> /save_map. The outcome arrives later as a
            # 'save_result' broadcast; this only confirms the request went out.
            name = msg.get('name') or ''
            request_id = await self._blocking(self.node.save_map, name)
            return {'type': 'ack', 'cmd': 'save_map', 'map': name,
                    'id': request_id}

        if cmd == 'delete_map':
            # {name} -> map_admin.delete_map. Resolved against the map
            # library listing, never joined onto a path. Off the loop (file
            # work) on the io pool.
            name = msg.get('name')
            try:
                from . import map_admin
                result = await self._blocking(
                    map_admin.delete_map, name, 'hardware', None,
                    dict(self.node.robot_mode), pool='io')
            except Exception as exc:                        # noqa: BLE001
                return {'type': 'map_deleted', 'ok': False, 'name': name,
                        'error': str(exc) or exc.__class__.__name__}
            return {'type': 'map_deleted', **result}

        if cmd == 'explorer':
            # pause | resume | stop | explore | clean -> frontier_explorer.
            command = msg.get('command')
            await self._blocking(self.node.explorer_command, command)
            return {'type': 'ack', 'cmd': 'explorer', 'command': command}

        if cmd == 'estop':
            await self._blocking(self.node.estop, pool='safety')
            return {'type': 'ack', 'cmd': 'estop'}

        if cmd == 'logs':
            # The diagnostics page asking for what it missed. `since` is the
            # last seq it holds, so a reconnect costs one short message
            # instead of the whole ring.
            return {'type': 'log_history',
                    'items': self.node.recent_logs(since=msg.get('since', 0))}

        if cmd == 'health':
            # Reading /proc is quick; vcgencmd is a subprocess, so the whole
            # snapshot goes through its own pool rather than the event loop.
            snapshot = await self._blocking(self.health.snapshot,
                                            pool='health')
            return {'type': 'health', 'health': snapshot}

        if cmd == 'console':
            # An ALLOWLISTED operation only - see console_ops for why there is
            # no general "run this" command here. Nothing in `msg` reaches a
            # shell, and `op` is a key into a fixed table rather than a
            # command line. Its own pool: an op can run for 8 s.
            result = await self._blocking(
                console_ops.run_op, msg.get('op'), msg.get('args') or [],
                pool='console')
            return {'type': 'console_result', **result}

        return {'type': 'error', 'cmd': cmd,
                'msg': 'unknown command %r' % cmd}

    async def dispatch(self, ws, msg, inflight=None):
        """Run one command and send its reply. Never raises.

        Errors become {'type': 'error', 'cmd': ..., 'msg': ...}; `cmd` lets the
        page put the message next to the control that caused it.
        """
        cmd = msg.get('cmd') if isinstance(msg, dict) else None
        try:
            reply = await self.handle(ws, msg)
        except wp_mod.WaypointError as exc:
            reply = {'type': 'error', 'cmd': cmd, 'msg': str(exc)}
        except Exception as exc:                            # noqa: BLE001
            self.node.get_logger().error('command %r failed: %s' % (cmd, exc))
            reply = {'type': 'error', 'cmd': cmd, 'msg': str(exc)}
        finally:
            if inflight is not None:
                inflight.discard(asyncio.current_task())
        if reply is not None:
            await self.send(ws, reply)


async def ws_handler(request):
    bridge = request.app['bridge']
    ws = web.WebSocketResponse(heartbeat=20)
    await ws.prepare(request)

    # Bring a new client fully up to date before it can do anything. It joins
    # the broadcast only AFTER this: a batched frame that overtook the seed
    # would put log lines newer than the history in front of it, and the page
    # drops anything older than the last line it holds.
    payload = bridge.node.waypoints.as_dict()
    payload['type'] = 'waypoints'
    await bridge.send(ws, payload)
    await bridge.send(ws, bridge.node.state())
    # The log so far, before anything new can arrive. Diagnosing something
    # that already happened is the whole reason the ring exists.
    history = bridge.node.recent_logs()
    await bridge.send(ws, {'type': 'log_history', 'items': history})
    # The IMU card has no event of its own while the board is silent, so a
    # page that opens then needs to be told what the node currently knows.
    imu_event = getattr(bridge.node, 'imu_event', None)
    if imu_event is not None:
        await bridge.send(ws, imu_event())

    bridge.clients.add(ws)
    # Close the gap between the snapshots above and joining the broadcast:
    # whatever changed meanwhile is sent once more, directly.
    last_seq = history[-1]['seq'] if history else 0
    missed = bridge.node.recent_logs(since=last_seq)
    if missed:
        await bridge.send(ws, {'type': 'log_history', 'items': missed})
    await bridge.send(ws, bridge.node.state())

    # Commands run as tasks, not inline: a slow one (a console op, a pose
    # settling) used to hold up every later message from this client, STOP
    # and the joystick included. Only the cheap, order-sensitive hot-path
    # commands are handled in arrival order.
    inflight = set()
    try:
        async for raw in ws:
            if raw.type != WSMsgType.TEXT:
                continue
            try:
                msg = json.loads(raw.data)
            except json.JSONDecodeError:
                await bridge.send(ws, {'type': 'error', 'msg': 'invalid JSON'})
                continue
            if not isinstance(msg, dict):
                await bridge.send(ws, {'type': 'error',
                                       'msg': 'a command must be a JSON object'})
                continue
            if msg.get('cmd') in INLINE_COMMANDS:
                await bridge.dispatch(ws, msg)
            elif len(inflight) >= MAX_INFLIGHT_PER_CLIENT:
                await bridge.send(ws, {'type': 'error', 'cmd': msg.get('cmd'),
                                       'msg': 'too many commands in flight'})
            else:
                inflight.add(asyncio.ensure_future(
                    bridge.dispatch(ws, msg, inflight)))
    finally:
        for task in list(inflight):
            task.cancel()
        bridge.clients.discard(ws)
        # A disconnected client must not leave the robot driving. The node's
        # deadman would catch it 0.4 s later anyway; this is immediate, and
        # teleop_stop no longer sleeps, so it is safe on the loop.
        try:
            bridge.node.teleop_stop()
        except Exception:
            pass
    return ws


async def waypoints_json(request):
    payload = request.app['node'].waypoints.as_dict()
    payload['type'] = 'waypoints'
    return web.json_response(payload, dumps=dumps)


async def state_json(request):
    return web.json_response(request.app['node'].state(), dumps=dumps)


async def plan_json(request):
    """The planned sweep as flat [x, y] pairs, for the map canvas."""
    node = request.app['node']
    return web.json_response(
        {'type': 'plan', 'points': [[round(x, 3), round(y, 3)]
                                    for (x, y) in node.zone_plan]},
        dumps=dumps)


def _run(request, pool, fn, *args):
    """Await a blocking function on one of the bridge's dedicated pools."""
    return asyncio.get_running_loop().run_in_executor(
        getattr(request.app['pools'], pool), fn, *args)


def _map_yaml(node):
    """map_server's yaml path, or None. Never waits for map_server to appear.

    These handlers used to call check_map() - a 5 s wait_for_service when
    there is no map_server (every explore/manual mission) - directly on the
    event loop, which froze every page for that long, health card included.
    """
    ok, detail = node.check_map(timeout=1.0, wait=False)
    if ok and detail and os.path.isfile(str(detail)):
        return str(detail)
    return None


def _map_meta_blocking(node):
    """-> (status, payload dict). File I/O; run it off the loop."""
    import yaml
    path = _map_yaml(node)
    if not path:
        return 503, {'error': 'map_server did not report a file'}
    with open(path) as fh:
        meta = yaml.safe_load(fh)
    img = meta['image']
    if not os.path.isabs(img):
        img = os.path.join(os.path.dirname(path), img)
    with open(img, 'rb') as fh:
        head = fh.read(64)
    # minimal P5 header parse, just for width/height
    parts = head.split()
    w, h = int(parts[1]), int(parts[2])
    return 200, {
        'resolution': float(meta['resolution']),
        'origin': [float(meta['origin'][0]), float(meta['origin'][1])],
        'width': w, 'height': h,
        'name': os.path.splitext(os.path.basename(path))[0],
        '_image': img,
    }


async def map_meta(request):
    """Geometry the browser needs to turn a pixel drag into map metres."""
    node = request.app['node']
    try:
        status, payload = await _run(request, 'io', _map_meta_blocking, node)
    except Exception as exc:                                # noqa: BLE001
        return web.json_response({'error': 'map metadata unreadable: %s' % exc},
                                 status=503)
    image = payload.pop('_image', None)
    if image:
        request.app['map_image_path'] = image
    return web.json_response(payload, status=status)


def _map_png_blocking(node):
    """-> (status, body bytes | text). cv2 decode + encode; run off the loop."""
    import yaml
    path = _map_yaml(node)
    if not path:
        return 503, 'no map'
    with open(path) as fh:
        meta = yaml.safe_load(fh)
    img = meta['image']
    if not os.path.isabs(img):
        img = os.path.join(os.path.dirname(path), img)
    try:
        import cv2
    except ImportError:
        return 503, 'opencv not available'
    pgm = cv2.imread(img, cv2.IMREAD_GRAYSCALE)
    ok_enc, buf = cv2.imencode('.png', pgm)
    if not ok_enc:
        return 500, 'encode failed'
    return 200, buf.tobytes()


async def map_png(request):
    """The occupancy map as a PNG, so the page can draw a zone on top of it."""
    node = request.app['node']
    try:
        status, body = await _run(request, 'io', _map_png_blocking, node)
    except Exception as exc:                                # noqa: BLE001
        return web.Response(status=503, text='map unreadable: %s' % exc)
    if status != 200:
        return web.Response(status=status, text=body)
    return web.Response(body=body, content_type='image/png')


async def maps_json(request):
    """Saved maps from the nexva_explore map library.

    Imported here, not at module level, so the page keeps working before
    that package is built - the mission card then just says so.
    """
    try:
        from nexva_explore import map_library
    except ImportError as exc:
        return web.json_response(
            {'error': 'map library unavailable - is nexva_explore built? (%s)'
                      % exc,
             'source': 'hardware', 'maps': []},
            status=503, dumps=dumps)
    def read_library():
        maps = map_library.list_maps('hardware')
        # The pose each map was saved at, so the page can show where it will
        # resume. Read per request; these are tiny files - but still file
        # reads, one per map, so they are off the loop with the listing.
        try:
            from nexva_explore import pose_store
            maps = [dict(m, pose=pose_store.pose_summary(m['name']))
                    for m in maps]
        except Exception:                                   # noqa: BLE001
            pass
        return list(maps)

    try:
        maps = await _run(request, 'io', read_library)
    except Exception as exc:                                # noqa: BLE001
        return web.json_response(
            {'error': 'map library failed: %s' % exc,
             'source': 'hardware', 'maps': []},
            status=500, dumps=dumps)
    return web.json_response({'source': 'hardware', 'maps': maps},
                             dumps=dumps)


async def coverage_json(request):
    """The explorer's coverage squares, reduced: [{x, y, size, s[, f]}]."""
    node = request.app['node']
    return web.json_response({'blocks': list(node.coverage_blocks)},
                             dumps=dumps)


async def explore_path_json(request):
    node = request.app['node']
    return web.json_response(
        {'points': [[x, y] for (x, y) in node.explore_path]}, dumps=dumps)


async def live_map_json(request):
    """The latest /map the node has seen, whoever published it.

    /api/map_meta and /api/map.png read the file map_server reports, which
    does not exist during an explore mission - there the map is slam_toolbox's
    and it grows. This serves it straight from the topic, run-length encoded.
    Encoding a big grid takes real CPU, so it runs in the executor.
    """
    node = request.app['node']
    encoded = await _run(request, 'io', node.live_map)
    if encoded is None:
        return web.json_response({'error': 'no /map received yet'},
                                 status=503)
    return web.json_response(encoded, dumps=dumps)


async def health_json(request):
    """Raspberry Pi health. Polled by the diagnostics tab while it is open.

    In the executor: vcgencmd is a subprocess, and a Pi under load can take a
    moment to answer. Blocking the loop for that would stall teleop for every
    other client.
    """
    snapshot = await _run(request, 'health', request.app['health'].snapshot)
    return web.json_response(snapshot, dumps=dumps)


async def console_ops_json(request):
    """The console allowlist, so the page can build its dropdown.

    Read-only and static. The page shows the exact argv from here before it
    runs anything, which is only honest because the server builds the command
    from the same table.
    """
    return web.json_response({'ops': console_ops.describe()}, dumps=dumps)


def default_waypoint_file():
    env = os.environ.get('NEXVA_WAYPOINTS')
    if env:
        return env
    return os.path.join(
        get_package_share_directory('nexva_web'), 'config', 'waypoints.yaml')


def main(argv=None):
    ap = argparse.ArgumentParser(prog='web_bridge')
    ap.add_argument('-f', '--file', default=None)
    ap.add_argument('--host', default='0.0.0.0')
    ap.add_argument('--port', type=int, default=8080)
    known, _ = ap.parse_known_args(argv if argv is not None else sys.argv[1:])

    path = known.file or default_waypoint_file()
    try:
        wps = wp_mod.load(path)
    except wp_mod.WaypointError as exc:
        print('error: %s' % exc, file=sys.stderr)
        return 1

    rclpy.init()
    node = NavClient(wps)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    def startup_map_check():
        # In the background: with no map_server (every explore/manual start)
        # this waits its full timeout, and the page should not be held back
        # that long just to log a line.
        ok, detail = node.check_map()
        if ok is True:
            node.get_logger().info('map OK: %s' % detail)
        elif ok is False:
            node.get_logger().error('MAP MISMATCH: %s' % detail)
        else:
            node.get_logger().warn('could not verify map: %s' % detail)

    threading.Thread(target=startup_map_check, daemon=True,
                     name='web-startup-map-check').start()

    app = web.Application()
    app['node'] = node
    app['pools'] = Pools()
    # One sampler for the life of the process: CPU utilisation is a delta
    # between two /proc/stat reads, so something has to hold the previous one.
    app['health'] = pi_health.HealthMonitor()

    async def on_startup(app):
        # Bind the bridge to the loop run_app actually created. aiohttp 3.x
        # dropped run_app's `loop` argument, so the loop has to be captured
        # from inside the running server rather than handed to it.
        app['bridge'] = Bridge(app['node'], asyncio.get_running_loop(),
                               app['health'], app['pools'])

    async def on_cleanup(app):
        app['pools'].shutdown()

    app.on_startup.append(on_startup)
    app.on_cleanup.append(on_cleanup)
    app.router.add_get('/ws', ws_handler)
    app.router.add_get('/api/waypoints', waypoints_json)
    app.router.add_get('/api/state', state_json)
    app.router.add_get('/api/plan', plan_json)
    app.router.add_get('/api/map_meta', map_meta)
    app.router.add_get('/api/map.png', map_png)
    app.router.add_get('/api/maps', maps_json)
    app.router.add_get('/api/coverage', coverage_json)
    app.router.add_get('/api/explore_path', explore_path_json)
    app.router.add_get('/api/live_map', live_map_json)
    app.router.add_get('/api/health', health_json)
    app.router.add_get('/api/console_ops', console_ops_json)

    web_dir = os.path.join(get_package_share_directory('nexva_web'), 'web')
    app.router.add_get('/', lambda r: web.FileResponse(
        os.path.join(web_dir, 'index.html')))
    app.router.add_static('/static/', web_dir)

    node.get_logger().info('web UI on http://%s:%d  (waypoints: %s)'
                           % (known.host, known.port, path))
    try:
        web.run_app(app, host=known.host, port=known.port, print=None)
    except KeyboardInterrupt:
        pass
    finally:
        # Zero the base, but leave any mission running: mode_manager owns
        # mission lifetime so that a clean survives the UI being restarted.
        node.estop(stop_mission=False)
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
