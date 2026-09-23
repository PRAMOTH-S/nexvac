"""aiohttp server exposing the waypoint client over WebSocket.

rclpy spins on its own thread; aiohttp owns the main thread's event loop. The
node pushes events through `on_event`, which runs on a ROS callback thread, so
it hands them to the loop with call_soon_threadsafe rather than touching
asyncio state directly. Nothing here calls rclpy - that all lives in NavClient.

Blocking ROS calls (set_initial_pose sleeps while AMCL settles, estop spends a
second publishing zeros) go through run_in_executor so a slow one cannot stall
the server for every other client.
"""

import argparse
import asyncio
import json
import os
import sys
import threading

import rclpy
from aiohttp import WSMsgType, web
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor

from . import waypoints as wp_mod
from .nav_client import NavClient


class Bridge:

    def __init__(self, node, loop):
        self.node = node
        self.loop = loop
        self.clients = set()
        node.on_event = self._on_ros_event

    # ROS callback thread -> asyncio loop
    def _on_ros_event(self, event):
        self.loop.call_soon_threadsafe(
            lambda: asyncio.ensure_future(self.broadcast(event)))

    async def broadcast(self, message):
        if not self.clients:
            return
        text = json.dumps(message)
        for ws in list(self.clients):
            try:
                await ws.send_str(text)
            except (ConnectionResetError, RuntimeError):
                self.clients.discard(ws)

    async def send(self, ws, message):
        try:
            await ws.send_str(json.dumps(message))
        except (ConnectionResetError, RuntimeError):
            self.clients.discard(ws)

    async def _blocking(self, fn, *args):
        return await self.loop.run_in_executor(None, fn, *args)

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
            ok, detail = await self._blocking(self.node.check_map)
            return {'type': 'map_check', 'ok': ok, 'detail': detail}

        if cmd == 'set_initial_pose':
            name = msg.get('waypoint')
            if not msg.get('confirmed'):
                return {'type': 'error',
                        'msg': 'initial pose needs explicit confirmation that '
                               'the robot is physically at that waypoint'}
            ok = await self._blocking(self.node.set_initial_pose, name)
            return {'type': 'initial_pose', 'waypoint': name, 'localized': ok}

        if cmd == 'goto':
            await self._blocking(self.node.goto, msg.get('waypoint'))
            return None

        if cmd == 'tour':
            names = msg.get('waypoints') or []
            loops = int(msg.get('loops', 0))
            await self._blocking(self.node.tour, names, loops)
            return None

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
            cancelled = await self._blocking(self.node.cancel)
            return {'type': 'ack', 'cmd': 'cancel', 'was_active': cancelled}

        if cmd == 'estop':
            await self._blocking(self.node.estop)
            return {'type': 'ack', 'cmd': 'estop'}

        return {'type': 'error', 'msg': 'unknown command %r' % cmd}


async def ws_handler(request):
    bridge = request.app['bridge']
    ws = web.WebSocketResponse(heartbeat=20)
    await ws.prepare(request)
    bridge.clients.add(ws)

    # Bring a new client fully up to date before it can do anything.
    payload = bridge.node.waypoints.as_dict()
    payload['type'] = 'waypoints'
    await bridge.send(ws, payload)
    await bridge.send(ws, bridge.node.state())

    try:
        async for raw in ws:
            if raw.type != WSMsgType.TEXT:
                continue
            try:
                msg = json.loads(raw.data)
            except json.JSONDecodeError:
                await bridge.send(ws, {'type': 'error', 'msg': 'invalid JSON'})
                continue
            try:
                reply = await bridge.handle(ws, msg)
            except wp_mod.WaypointError as exc:
                reply = {'type': 'error', 'msg': str(exc)}
            except Exception as exc:                        # noqa: BLE001
                bridge.node.get_logger().error('command failed: %s' % exc)
                reply = {'type': 'error', 'msg': str(exc)}
            if reply is not None:
                await bridge.send(ws, reply)
    finally:
        bridge.clients.discard(ws)
        # A disconnected client must not leave the robot driving. The node's
        # deadman would catch it 0.4 s later anyway; this is immediate.
        try:
            bridge.node.teleop_stop()
        except Exception:
            pass
    return ws


async def waypoints_json(request):
    payload = request.app['node'].waypoints.as_dict()
    payload['type'] = 'waypoints'
    return web.json_response(payload)


async def state_json(request):
    return web.json_response(request.app['node'].state())


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

    ok, detail = node.check_map()
    if ok is True:
        node.get_logger().info('map OK: %s' % detail)
    elif ok is False:
        node.get_logger().error('MAP MISMATCH: %s' % detail)
    else:
        node.get_logger().warn('could not verify map: %s' % detail)

    app = web.Application()
    app['node'] = node

    async def on_startup(app):
        # Bind the bridge to the loop run_app actually created. aiohttp 3.x
        # dropped run_app's `loop` argument, so the loop has to be captured
        # from inside the running server rather than handed to it.
        app['bridge'] = Bridge(app['node'], asyncio.get_running_loop())

    app.on_startup.append(on_startup)
    app.router.add_get('/ws', ws_handler)
    app.router.add_get('/api/waypoints', waypoints_json)
    app.router.add_get('/api/state', state_json)

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
        node.estop()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
