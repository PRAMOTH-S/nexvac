"""
Save the map when told to, not before.

Exploring used to write /map to disk on every SLAM update (map_autosaver).
Now the map is saved on demand, by one command, as one consistent snapshot:

    /save_map          std_msgs/String   request
    /save_map_result   std_msgs/String   JSON reply

REQUEST payload: either a bare map name ("kitchen"; empty = the node's
`map_name`), or JSON {"name": "kitchen", "id": "abc", "pose": {"x":..,"y":..,
"yaw":..}}. `pose` is optional: the explorer sends the pose it captured the
moment it stopped; without one this node reads map->base_footprint itself.

One request does, in this order:
  1. .pgm + .yaml        (atomic, same writer the autosaver uses)
  2. map.md registry entry
  3. <name>.pose.yaml    (pose_store.save_pose - the 2D pose AND heading)
  4. slam_toolbox pose graph via /slam_toolbox/serialize_map, when it exists

REPLY: {"ok": bool, "id", "name", "map_yaml", "pose": {x,y,yaw_deg}|null,
        "posegraph": bool, "warnings": [..], "error": str|null}
`ok` means the map files were written. A missing pose or pose graph is a
warning, not a failure - the map is still usable.

WHY A SUBCLASS OF MapAutosaver and not a new writer: the pgm/yaml rendering,
registry update and serialise call are already correct and tested there. This
node runs it with autosave off, so the only thing that changes is WHEN it
writes. It is a separate executable so explore.launch.py contains no
`map_autosaver`: nothing in an explore run writes the map unprompted.

Wall clock only; nothing here reads the ROS clock.
"""

import json
import math
import os

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.time import Time
from slam_toolbox.srv import SerializePoseGraph
from std_msgs.msg import String
import tf2_ros

from nexva_explore import map_library, pose_store
from nexva_explore.map_autosaver import MapAutosaver, SERIALIZE_SERVICE

# How long to wait for slam_toolbox to finish writing its graph.
SERIALIZE_TIMEOUT = 20.0


class MapSaver(MapAutosaver):
    """Saves the latest /map, the pose and the pose graph on request."""

    AUTOSAVE_DEFAULT = False     # this node writes only when asked

    def __init__(self):
        super().__init__('map_saver')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('pose_file', '')
        self.declare_parameter('pose_timeout', 2.0)
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.pose_file = self.get_parameter('pose_file').value or None
        self.pose_timeout = self.get_parameter('pose_timeout').value
        self.default_name = self.map_name

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.result_pub = self.create_publisher(String, '/save_map_result', 10)
        self.create_subscription(String, '/save_map', self.request_cb, 10)
        self.get_logger().info(
            f'map_saver ready: publish a name on /save_map to save into '
            f'{self.map_dir} (autosave={"on" if self.autosave else "off"})')

    # ------------------------------------------------------------ request

    @staticmethod
    def parse_request(text):
        """(name, request_id, pose_or_None) from a bare name or JSON."""
        text = (text or '').strip()
        if text.startswith('{'):
            try:
                body = json.loads(text)
            except ValueError:
                body = None
            if isinstance(body, dict):
                pose = body.get('pose')
                if isinstance(pose, dict):
                    try:
                        pose = (float(pose['x']), float(pose['y']),
                                float(pose['yaw']))
                        if not all(math.isfinite(v) for v in pose):
                            pose = None
                    except (KeyError, TypeError, ValueError):
                        pose = None
                else:
                    pose = None
                return str(body.get('name') or '').strip(), body.get('id'), pose
        return text, None, None

    def request_cb(self, msg):
        name, request_id, pose = self.parse_request(msg.data)
        name = name or self.default_name
        reply = {'ok': False, 'id': request_id, 'name': name, 'map_yaml': None,
                 'pose': None, 'posegraph': False, 'warnings': [], 'error': None}

        cleaned, problem = map_library.valid_name(name)
        if problem:
            reply['error'] = f'bad map name {name!r}: {problem}'
            return self.finish(reply)
        name = reply['name'] = cleaned

        if self.latest_map is None:
            reply['error'] = ('no /map has been received yet - is slam_toolbox '
                              'or map_server running?')
            return self.finish(reply)

        # Point the inherited writers at this name for this request.
        self.map_name = name
        self.pgm_path = os.path.join(self.map_dir, name + '.pgm')
        self.yaml_path = os.path.join(self.map_dir, name + '.yaml')
        self.graph_path = os.path.join(self.map_dir, name)

        try:
            self.write_map(self.latest_map)
        except OSError as exc:
            reply['error'] = f'could not write the map to {self.map_dir}: {exc}'
            return self.finish(reply)

        reply['ok'] = True
        reply['map_yaml'] = self.yaml_path
        self.record_save()

        if pose is None:
            pose = self.tf_pose()
            if pose is None:
                reply['warnings'].append(
                    f'no live {self.map_frame}->{self.base_frame} pose - the '
                    'pose file was NOT written')
        if pose is not None:
            try:
                saved = pose_store.save_pose(
                    pose_store.pose_path(name, self.pose_file), *pose, name,
                    source=self.map_source, frame=self.map_frame)
                reply['pose'] = {'x': saved['x'], 'y': saved['y'],
                                 'yaw_deg': saved['yaw_deg']}
            except (OSError, ValueError) as exc:
                reply['warnings'].append(f'pose file not written: {exc}')

        self.serialise_then_finish(reply)

    def tf_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time())
        except tf2_ros.TransformException:
            return None
        age = (self.get_clock().now()
               - Time.from_msg(tf.header.stamp)).nanoseconds * 1e-9
        if age > self.pose_timeout:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return (t.x, t.y, pose_store.yaw_from_quat(q.z, q.w, q.x, q.y))

    # ---------------------------------------------------------- pose graph

    def serialise_then_finish(self, reply):
        if not self.serialize_client.service_is_ready():
            reply['warnings'].append(
                f'{SERIALIZE_SERVICE} not available - no pose graph written '
                '(normal under Nav2/AMCL)')
            return self.finish(reply)

        request = SerializePoseGraph.Request()
        request.filename = self.graph_path
        future = self.serialize_client.call_async(request)
        state = {'done': False}

        def complete(error=None):
            if state['done']:
                return
            state['done'] = True
            timer.cancel()
            self.destroy_timer(timer)
            if error:
                reply['warnings'].append(f'pose graph not written: {error}')
            else:
                reply['posegraph'] = True
            self.finish(reply)

        def done(fut):
            try:
                result = fut.result()
            except Exception as exc:                        # noqa: BLE001
                return complete(str(exc))
            if result is None or getattr(result, 'result', 0) != 0:
                return complete(f'slam_toolbox returned {result}')
            complete()

        timer = self.create_timer(SERIALIZE_TIMEOUT,
                                  lambda: complete('timed out'))
        future.add_done_callback(done)

    # -------------------------------------------------------------- reply

    def finish(self, reply):
        if reply['ok']:
            self.get_logger().info(
                f'MAP SAVED "{reply["name"]}" -> {reply["map_yaml"]}'
                f' pose={reply["pose"]} posegraph={reply["posegraph"]}')
            for warning in reply['warnings']:
                self.get_logger().warn(f'save "{reply["name"]}": {warning}')
        else:
            self.get_logger().error(f'MAP SAVE FAILED: {reply["error"]}')
        out = String()
        out.data = json.dumps(reply)
        self.result_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MapSaver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
