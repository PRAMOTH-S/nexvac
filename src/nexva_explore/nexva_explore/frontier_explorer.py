"""
Frontier-based autonomous exploration.

Reads the live SLAM occupancy grid, finds the boundaries between mapped free
space and unknown space (frontiers), plans a path to the nearest reachable
one over the grid itself, and drives there. Repeats until no frontiers are
left, at which point the reachable area is fully mapped.
"""

from collections import deque
import json
import math
import time
import uuid

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from nexva_explore.stall_watch import StallWatch
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from slam_toolbox.srv import Pause
from std_msgs.msg import Bool, String
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray

# OccupancyGrid conventions: -1 unknown, 0 free, 100 occupied.
UNKNOWN = -1
FREE_MAX = 25
OCCUPIED_MIN = 65

# Coverage cells are packed into a single int: (x + ORIGIN) * STRIDE + y.
# ORIGIN keeps negative coordinates positive, STRIDE stops x and y colliding.
COVER_ORIGIN = 1 << 20
COVER_STRIDE = 1 << 22


def quantise(value, resolution):
    """
    Round a world coordinate to a coverage cell, halves always going up.

    `round()` and `np.round()` both round halves to EVEN, and a grid cell's
    centre sits exactly on a half whenever the map origin happens to be a
    multiple of the resolution - `x = origin + (col + 0.5) * res`, so
    `x / res` ends in .5 for every column. Banker's rounding then collapses
    adjacent columns onto the same key in pairs: measured at origin 0.0 on a
    0.05 m grid, eight consecutive cells produced only FIVE distinct keys, so
    a cell read as swept because its neighbour had been.

    Latent rather than active - slam_toolbox re-anchors to arbitrary offsets
    and the map in use has origin -5.004479, which is not a multiple of 0.05,
    so nothing ties there. It costs nothing to make it impossible.
    """
    return np.floor(np.asarray(value) / resolution + 0.5).astype(np.int64)


# Map topics are latched. map_server publishes /map exactly once when it
# activates, so a plain volatile subscription that joins later hears nothing
# at all and the robot just sits there. TRANSIENT_LOCAL replays that message.
MAP_QOS = QoSProfile(
    depth=1,
    history=QoSHistoryPolicy.KEEP_LAST,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)

# The same latching, for what this node hands to the web page: the coverage
# squares and the current mode. A browser tab opened mid-run gets the last
# value straight away instead of a blank until the next change.
LATCHED_QOS = MAP_QOS


class BlockState:
    """
    Per-block coverage of the floor.

    How much drivable floor each square holds, and how much of it has been
    swept.

    Worked out once and handed to both the RViz squares and the planner that
    chases them. They each used to compute it separately, which is how the
    robot could drive past a square the screen was still showing red.

    A block with NO drivable floor - one that lies entirely inside a wall or a
    piece of furniture - never appears here at all. It has no cells to count,
    so it is not in `total`, gets no marker, and is not a candidate. That is
    the "ignore the ones inside an obstacle" rule, and it falls out of the
    arithmetic rather than being a special case.
    """

    def __init__(self, rows, cols, swept, flat_block, total, done, origin, span):
        self.rows = rows
        self.cols = cols
        self.swept = swept
        self.flat_block = flat_block
        self.total = total
        self.done = done
        self.origin = origin
        self.span = span

    def fraction(self, index):
        """How much of block `index` has been swept, 0 to 1."""
        if self.total[index] <= 0:
            return 1.0

        return float(self.done[index]) / float(self.total[index])

    def fractions(self):
        """Swept fraction of every block, as an array. Empty blocks read 1."""
        with np.errstate(divide='ignore', invalid='ignore'):
            out = np.where(self.total > 0, self.done / np.maximum(self.total, 1), 1.0)

        return out

    def block_xy(self, index):
        """Block grid coordinates for a block index."""
        origin_x, origin_y = self.origin

        return (int(index // self.span) + origin_x,
                int(index % self.span) + origin_y)

    def index_of(self, block):
        """Block index for grid coordinates, or None if it is off this map."""
        origin_x, origin_y = self.origin
        offset_x = block[0] - origin_x
        offset_y = block[1] - origin_y

        if offset_x < 0 or offset_y < 0 or offset_y >= self.span:
            return None

        index = offset_x * self.span + offset_y

        if index >= self.total.size:
            return None

        return int(index)

    def centre(self, index, block_size):
        """World coordinates of the middle of a block."""
        grid_x, grid_y = self.block_xy(index)

        return ((grid_x + 0.5) * block_size, (grid_y + 0.5) * block_size)


class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')

        # Real footprint, taken from the URDF: a 270 x 270 mm chassis with the
        # wheels tucked in flush, so nothing sticks out past the base. Nexva's
        # body is nearer 300 x 300 mm once the bumper trim is counted, and on
        # real motors the stopping margins are wider than the sim needed.
        self.declare_parameter('footprint_length', 0.30)
        self.declare_parameter('footprint_width', 0.30)
        self.declare_parameter('safety_margin', 0.04)
        self.declare_parameter('stop_distance', 0.18)
        self.declare_parameter('slow_distance', 0.45)

        # Gap left between the robot's side and a wall when planning. The
        # planner inflates by half the robot's width plus this, NOT by the
        # turning circle - inflating by the turning circle would leave a strip
        # along every wall that the robot could never clean.
        self.declare_parameter('wall_clearance', 0.07)

        # Hard limits on what ever reaches the wheels. `max_linear` sits under
        # the ESP32's own 0.30 m/s cap and matches the web teleop; nothing in
        # this file asks for more, but a gate that trusts its callers is not a
        # gate.
        self.declare_parameter('max_linear', 0.26)
        self.declare_parameter('max_angular', 1.5)

        # Sensor staleness. The sim never lost a sensor; the real robot can
        # lose the lidar to a loose USB cable or the pose to a stalled
        # micro-ROS agent, and a node driving on its last snapshot of the
        # room is a node driving blind.
        self.declare_parameter('scan_timeout', 0.5)
        self.declare_parameter('pose_timeout', 2.0)

        # THE LOCALIZATION GATE. Nothing drives until the robot genuinely
        # knows where it is.
        #
        # This replaces the fixed `TimerAction(period=8.0)` that used to hold
        # the explorer back in explore.launch.py. A sleep is a guess about how
        # long slam_toolbox needs; on a cold Pi 5 with a map to load and a
        # first scan to match it is routinely longer, and when the guess is
        # short the explorer starts driving with no pose at all: robot_pose()
        # hands back None, the first plan is nonsense and the operator sees a
        # robot wandering while the map stays empty. A sleep is also wrong in
        # the other direction - when SLAM is ready in 2 s, 6 s are thrown away.
        #
        # So the wait is a measurement, not a guess, and it lives HERE rather
        # than in the shell script: a gate the driving node owns cannot be
        # skipped, cannot race the node it is guarding, and re-arms on every
        # new mission. While it waits the wheels get an explicit zero Twist
        # every tick - never silence, which is indistinguishable from a dead
        # node - and the reason is logged.
        #
        # `localize_timeout` bounds it: past this the run is failed loudly
        # with a diagnosis instead of holding station forever.
        self.declare_parameter('localize_timeout', 60.0)

        # Reversing into a rear sector with NO returns. On this chassis an
        # occluded sector reads exactly like an empty one, so by default it is
        # treated as blocked; set true only after confirming the scanner sees
        # clear over the back of the body.
        self.declare_parameter('allow_blind_reverse', False)

        # How many forced reverse pulses `publish_safe` may issue in a row
        # before it gives up and stops the run, rather than nudging backwards
        # once a second for as long as the node is alive.
        self.declare_parameter('max_forced_reverse', 5)

        # 'explore' maps first then cleans; 'clean' skips straight to cleaning,
        # for running against a map that was saved earlier.
        self.declare_parameter('start_mode', 'explore')

        # When exploration finishes the robot stops, captures its pose, asks
        # map_saver (/save_map) to save map + pose, and only then cleans.
        # map_name is the default name for that save (the launch passes it).
        self.declare_parameter('map_name', '')
        self.declare_parameter('handoff_settle', 1.5)
        self.declare_parameter('handoff_save_timeout', 30.0)

        # Getting unwedged. Nothing may block forever.
        self.declare_parameter('stuck_timeout', 6.0)
        self.declare_parameter('stuck_distance', 0.06)
        self.declare_parameter('recovery_time', 2.5)

        # Backing out is committed to for this long. Without it the robot
        # re-decides every tick: blocked, so reverse; now the way ahead is
        # clear, so drive; blocked again. Measured in a 0.44 m corridor that
        # is 179 direction changes in 40 seconds and no heading change at all.
        self.declare_parameter('backout_time', 1.5)

        # How hard to steer away from the nearest obstacle while doing it, as
        # a fraction of angular_speed. Gentle on purpose: this is an arc while
        # translating, not a spin in place.
        self.declare_parameter('avoid_turn', 0.45)

        # A back-out keeps going while there is still nowhere to turn, up to
        # this long. A corridor narrower than the robot can turn in has to be
        # reversed out of end to end, and 1.5 s of that is not enough - it
        # just puts the robot back where it started, facing the same way.
        self.declare_parameter('backout_max', 8.0)
        self.declare_parameter('max_recovery_attempts', 3)

        # THE PRIMARY RECOVERY: stop, aim, go. The timed blind back-out above
        # is now only the last resort (nowhere clear to point, nowhere to
        # turn). Each step below is bounded, so recovery can never run away.
        #
        # STOP: zero Twist for this long before anything moves. Long enough
        # for the wheels to coast to rest and the lidar to deliver a scan
        # taken while still, not so long it is a pause: the base drops its
        # wheels 500 ms after the last cmd_vel, so ~0.3 s is already "stopped".
        self.declare_parameter('recovery_settle', 0.3)

        # AIM: rotate until within this of the wanted heading. 0.15 rad is
        # 8.6 deg: one tick of rotation at angular_speed is 0.05 rad, so the
        # spin cannot step over the window, and a driven path error that small
        # is swallowed by the follower's own steering within a metre.
        self.declare_parameter('aim_tolerance', 0.15)

        # A rotation (aim, or turn to a clear heading) gives up after this
        # long and decides with what it has. 7 s is a full half-turn at the
        # default angular_speed (6.3 s) plus margin; a spin that has not
        # finished by then is not turning (wheels slipping, pinned).
        self.declare_parameter('aim_timeout', 7.0)

        # GO: drive forward this long once aimed at clear air. 1.0 s at
        # linear_speed 0.15 is 0.15 m, 2.5x the stuck_distance (0.06 m) the
        # progress timer needs to see - enough to be seen to be moving, short
        # enough that the normal follower (with its steering and slow-down)
        # takes over almost at once. It ends early the moment the way ahead
        # closes to stop_distance.
        self.declare_parameter('recovery_go_time', 1.0)

        # Ceiling on stop + aim + turn together (GO and the fallback get their
        # own time on top). Stops two worst-case half-turns stacking into 14 s.
        self.declare_parameter('recovery_budget', 9.0)

        # Crash / wheel-slip detection. The wheels reporting motion that the
        # map does not agree with means the robot is shoved against something
        # and the wheels are spinning on the spot.
        self.declare_parameter('slip_window', 2.5)
        self.declare_parameter('slip_distance', 0.05)
        self.declare_parameter('slip_ratio', 0.20)

        # THE STALL TEST. This is the one that notices a crash.
        #
        # The two tests below it both ask the outside world whether the robot
        # moved, and both are slow or blind in the case that matters. The
        # odometry-versus-map test waits `slip_window` for centimetres of
        # disagreement to accumulate - and it compares against SLAM's pose,
        # which is itself being dragged forward by the same lying odometry, so
        # the two can agree all the way into the wall. The scan test is
        # geometry-dependent: measured, it splits cleanly in a corridor
        # (0.025 m of beam change moving, 0.0000 m pinned) and fails outright
        # in an open room, where most beams run across the direction of travel
        # and hardly change even at full speed.
        #
        # `stall_watch` asks the question properly instead: for each beam it
        # knows how far that range SHOULD change if the wheels are telling the
        # truth, and fits observed against predicted. One number comes out -
        # the fraction of the claimed motion the world actually backs up.
        # Beams across the direction of travel predict nothing and drop out on
        # their own, which is the open-room blind spot gone.
        #
        # The GYRO makes it work while the robot is turning, by saying how far
        # it really turned so the previous scan can be rolled back before the
        # comparison. The wheels cannot be asked for that - during a stall
        # their reported turn is exactly as fake as their reported speed. The
        # gyro also answers a blocked spin outright.
        #
        # Measured, median fit over five scan pairs: genuine driving never
        # came below +0.645 in 119 samples; pinned sat on +0.000. See
        # nexva_explore/stall_watch.py for the full table and for why the
        # accelerometer is deliberately NOT a trigger.
        #
        # Off on Nexva for now: the BNO055 driver exists in nexva_sensor but
        # is not enabled in bringup, and without a gyro the watch cannot
        # de-rotate a scan pair, so every turn would read as a stall.
        self.declare_parameter('use_imu_stall', False)
        self.declare_parameter('stall_window', 0.6)
        self.declare_parameter('stall_confirm', 0.6)
        self.declare_parameter('stall_release', 1.0)
        self.declare_parameter('stall_fit_threshold', 0.25)
        self.declare_parameter('stall_rotation_ratio', 0.25)

        # Fast stall test: wheels turning while the view stays frozen. It
        # spots a crash in a fraction of a second instead of a second and a
        # half, but it is OFF by default. Measured, reacting that early made
        # the map lurch further, not less: holding scans off sooner means SLAM
        # loses the robot for longer and has more to unwind when it resumes.
        # Turn it on only if you re-measure the map afterwards.
        self.declare_parameter('use_scan_stall', False)
        self.declare_parameter('scan_stall_delta', 0.015)
        self.declare_parameter('scan_stall_window', 0.4)

        # While the robot is crashing, its odometry is lying. Feeding that to
        # SLAM as a motion prior is what smears the map and makes it appear to
        # jump. Hold off new scans until the robot is genuinely moving again.
        # Off by default on the real robot until the crash detectors have been
        # measured against it: a false crash here pauses SLAM on live floor.
        self.declare_parameter('protect_map_on_crash', False)
        self.declare_parameter('resume_mapping_after', 3.0)

        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('lookahead', 0.25)
        self.declare_parameter('min_frontier_size', 10)
        self.declare_parameter('replan_period', 2.5)

        # A frontier goal has to be far enough away to be worth driving to.
        # Standing on a frontier yields a zero-length path, and with a 360 deg
        # lidar there is nothing to be gained by spinning on the spot, so the
        # map never changes and the robot deadlocks.
        self.declare_parameter('min_goal_distance', 0.30)

        # Cleaning phase: once there is nothing left to explore, sweep the
        # mapped floor in zig-zag rows this far apart.
        #
        # Both of these are the SWATH, not the chassis. No brush or vacuum
        # head is fitted to Nexva yet, so the swath is the body itself and
        # these sim values stand in. When a cleaning head goes on, set
        # `cleaning_radius` to its real half-width and `row_spacing` to a
        # little under twice that, or the coverage record will claim floor the
        # head never touched.
        self.declare_parameter('row_spacing', 0.22)
        self.declare_parameter('cleaning_radius', 0.135)
        # Two passes, not three: a re-sweep drives only still-uncovered
        # waypoints, so a second pass already has very little to do - the
        # third existed to paper over the edge strips the old row sampling
        # left behind. Anything missed after two is a genuine gap and the
        # block chase is the right tool for it.
        self.declare_parameter('max_cleaning_passes', 2)

        # Spacing of the intermediate waypoints dropped ALONG each row. A row
        # is otherwise just its two ends with the grid BFS joining them -
        # sound on a perfect tracker, but follow_path pulls points within
        # lookahead and bends the heading by avoid_steer, so a 5 m leg beside
        # a wall can bow well off the planned line. A waypoint every metre
        # bounds that bow; 0.0 goes back to endpoints only.
        self.declare_parameter('row_point_spacing', 1.0)

        # Which corner the sweep starts at, and therefore which way it works
        # across the room. 'top' starts at whichever top corner is nearer and
        # zig-zags down; 'bottom', 'left' and 'right' are the same idea from
        # the other three sides. 'auto' picks whichever of the four corners is
        # nearest the robot, which is the shortest drive to the first waypoint
        # but does not always end up where a person expects.
        self.declare_parameter('sweep_start', 'top')

        # Which way the zig-zag rows run: 'x', 'y', or 'auto' to lay them
        # along the longer side of the floor, which is the fewest turns.
        # Only consulted when `sweep_start` is 'auto' - naming a side already
        # fixes the axis, because 'top' means the rows run across x and the
        # sweep works down y.
        self.declare_parameter('sweep_axis', 'auto')

        # How much a corner behind the robot is penalised against one in front
        # of it, in metres per radian of turn. Only ever breaks ties between
        # corners at a similar distance; 0.0 makes the choice pure distance.
        self.declare_parameter('corner_turn_weight', 0.25)

        # Coverage blocks. The floor is divided into squares this big, and a
        # square only counts as done once the robot has actually covered
        # `clean_threshold` of the drivable area inside it - passing through
        # a corner of it is not enough.
        self.declare_parameter('block_size', 0.5)
        self.declare_parameter('clean_threshold', 0.75)

        # How the floor gets covered.
        #
        # 'blocks' drives the coverage squares directly: pick the nearest
        #   square that is still red, go into it, and stay there until it is
        #   green, then pick the next. What is on the screen IS the plan, so
        #   nothing can be left red behind the robot.
        # 'sweep' lays the zig-zag rows first, exactly as before, and then
        #   finishes on blocks rather than stopping - so the tuned row pattern
        #   still does the bulk of the work on open floor, and the block chase
        #   picks up whatever it missed.
        #
        # Blocks with no drivable floor at all - the ones sitting inside a
        # wall or under the furniture - never enter either plan. They have no
        # cells to sweep, so they get no marker and are never a target.
        self.declare_parameter('clean_mode', 'sweep')

        # A block the robot cannot finish is dropped after this many goes at
        # it, so one unreachable corner cannot hold up the whole run.
        self.declare_parameter('block_attempts', 3)

        # Leave a square once it has stopped improving for this long, even if
        # it never reached `clean_threshold`.
        #
        # The last few percent of a square are its edges, and they are far and
        # away the most expensive part: measured without this, one 0.5 m square
        # sat at 68% and took 94 s and 24 recovery attempts to close, against
        # 12 s for the square before it. The robot was arriving at a leftover
        # cell, stopping `goal_tolerance` (0.15 m) short of it, and marking
        # only `cleaning_radius` (0.135 m) around itself - so the cell it had
        # driven to never counted as swept, and it went back for it forever.
        #
        # Bounding the tail is what makes "go in until it is green" finish. A
        # square left at 70% is reported as left at 70%, not silently called
        # done.
        self.declare_parameter('block_stall_timeout', 8.0)
        self.declare_parameter('block_timeout', 40.0)

        # Clearance kept outside the swept turning circle before a rotation is
        # allowed at all.
        self.declare_parameter('turn_margin', 0.03)

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.footprint_length = self.get_parameter('footprint_length').value
        self.footprint_width = self.get_parameter('footprint_width').value
        self.safety_margin = self.get_parameter('safety_margin').value
        self.stop_distance = self.get_parameter('stop_distance').value
        self.slow_distance = self.get_parameter('slow_distance').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.lookahead = self.get_parameter('lookahead').value
        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.replan_period = self.get_parameter('replan_period').value
        self.min_goal_distance = self.get_parameter('min_goal_distance').value
        self.row_spacing = self.get_parameter('row_spacing').value
        self.cleaning_radius = self.get_parameter('cleaning_radius').value
        self.max_cleaning_passes = self.get_parameter('max_cleaning_passes').value
        self.row_point_spacing = self.get_parameter('row_point_spacing').value
        self.sweep_start = self.get_parameter('sweep_start').value
        self.sweep_axis = self.get_parameter('sweep_axis').value
        self.corner_turn_weight = self.get_parameter('corner_turn_weight').value
        self.block_size = self.get_parameter('block_size').value
        self.clean_threshold = self.get_parameter('clean_threshold').value
        self.clean_mode = self.get_parameter('clean_mode').value
        self.block_attempts = self.get_parameter('block_attempts').value
        self.block_stall_timeout = self.get_parameter('block_stall_timeout').value
        self.block_timeout = self.get_parameter('block_timeout').value
        self.turn_margin = self.get_parameter('turn_margin').value
        self.wall_clearance = self.get_parameter('wall_clearance').value
        self.start_mode = self.get_parameter('start_mode').value
        self.map_name = self.get_parameter('map_name').value
        self.handoff_settle = self.get_parameter('handoff_settle').value
        self.handoff_save_timeout = self.get_parameter('handoff_save_timeout').value
        self.stuck_timeout = self.get_parameter('stuck_timeout').value
        self.stuck_distance = self.get_parameter('stuck_distance').value
        self.recovery_time = self.get_parameter('recovery_time').value
        self.backout_time = self.get_parameter('backout_time').value
        self.avoid_turn = self.get_parameter('avoid_turn').value
        self.backout_max = self.get_parameter('backout_max').value
        self.max_recovery_attempts = self.get_parameter('max_recovery_attempts').value
        self.recovery_settle = self.get_parameter('recovery_settle').value
        self.aim_tolerance = self.get_parameter('aim_tolerance').value
        self.aim_timeout = self.get_parameter('aim_timeout').value
        self.recovery_go_time = self.get_parameter('recovery_go_time').value
        self.recovery_budget = self.get_parameter('recovery_budget').value
        self.slip_window = self.get_parameter('slip_window').value
        self.slip_distance = self.get_parameter('slip_distance').value
        self.slip_ratio = self.get_parameter('slip_ratio').value
        self.use_imu_stall = self.get_parameter('use_imu_stall').value
        self.use_scan_stall = self.get_parameter('use_scan_stall').value
        self.scan_stall_delta = self.get_parameter('scan_stall_delta').value
        self.scan_stall_window = self.get_parameter('scan_stall_window').value
        self.protect_map_on_crash = self.get_parameter('protect_map_on_crash').value
        self.resume_mapping_after = self.get_parameter('resume_mapping_after').value
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.scan_timeout = self.get_parameter('scan_timeout').value
        self.pose_timeout = self.get_parameter('pose_timeout').value
        self.localize_timeout = self.get_parameter('localize_timeout').value
        self.allow_blind_reverse = self.get_parameter('allow_blind_reverse').value
        self.max_forced_reverse = self.get_parameter('max_forced_reverse').value

        self.half_length = self.footprint_length / 2.0
        self.half_width = self.footprint_width / 2.0

        # Circle the robot sweeps when it spins in place. Anything closer
        # than this is a collision, whichever way the robot is facing, so it
        # is also what the map gets inflated by when planning.
        self.turn_radius = math.hypot(self.half_length, self.half_width)

        # What the planner keeps clear. Driving alongside a wall only needs
        # half the robot's width, so this is much tighter than the turning
        # circle and lets the sweep reach right up to the skirting board.
        self.plan_radius = self.half_width + self.wall_clearance

        self.map_msg = None
        self.last_pose = None
        self.scan_points = []
        self.scan = None

        # Wall-clock time of the last scan and of the last FRESH pose. Both
        # gate the wheels: in the sim a dead lidar meant a dead sim, on the
        # real robot it means a node driving on a frozen snapshot of the room.
        self.scan_time = None
        self.pose_time = None

        # base_frame <- scan frame, looked up from TF once the URDF is on the
        # air and cached, since the mount does not move. None until then.
        self.scan_offset = None

        # Mission runs explore -> clean -> done, or starts at clean when
        # working from a map that was saved on an earlier run. `~/command`
        # can restart it in either mode at any time; 'stopped' is where a
        # stop command leaves it, and only another command leaves that.
        self.mode = 'clean' if self.start_mode == 'clean' else 'explore'

        # Pause is its own flag rather than a mode. 'done' is absorbing, and
        # the underlying mode has to survive the pause so resume can carry on
        # exactly where it left off.
        self.paused = False
        self.pause_started = 0.0
        self.last_mode_published = None

        # Everything that describes progress through ONE mission lives in
        # here, so a command that starts a new mission resets all of it in
        # one place and nothing can be forgotten.
        self._reset_mission_state()

        # Crash detection state.
        self.odom_pose = None
        self.slip_ref_odom = None
        self.slip_ref_map = None
        self.slip_ref_time = time.monotonic()
        self.collisions = 0
        self.last_crash_time = 0.0
        self.odom_speed = 0.0
        self.scan_ref = None
        self.scan_ref_time = time.monotonic()
        self.imu_stalls = 0

        # Holding SLAM off while the odometry is untrustworthy.
        self.map_frozen = False
        self.want_map_frozen = False
        self.pause_pending = False

        self.get_logger().info(
            f'Footprint {self.footprint_length:.2f} x {self.footprint_width:.2f} m, '
            f'turn radius {self.turn_radius:.2f} m'
        )

        # slam_toolbox publishes /map absolutely, ignoring its namespace
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, MAP_QOS)
        # Sensor QoS, not depth 10: the RPLIDAR driver and the BNO055 driver
        # both publish BEST_EFFORT, and a RELIABLE subscriber to a BEST_EFFORT
        # publisher is incompatible - it never connects and never says so.
        # The profile is also depth 5, which keeps the IMU on the newest
        # samples rather than working through stale history, which is exactly
        # the lag that would have it call a stall after the fact.
        self.create_subscription(
            LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(
            Imu, 'imu', self.imu_callback, qos_profile_sensor_data)

        # Runtime control from the web page: explore, clean, pause, resume,
        # stop. Before this the mode was fixed at launch by `start_mode`.
        self.create_subscription(String, '~/command', self.command_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.save_pub = self.create_publisher(String, '/save_map', 10)
        self.save_result_pub = self.create_publisher(String, '/save_map_result', 10)
        self.create_subscription(
            String, '/save_map_result', self.save_result_callback, 10)
        self.path_pub = self.create_publisher(Path, 'explore_path', 10)
        self.blocks_pub = self.create_publisher(
            MarkerArray, 'coverage_blocks', LATCHED_QOS)
        self.status_pub = self.create_publisher(String, 'robot_status', 10)

        # The same status as a JSON object. The text form has an internal
        # space in `map_pose=(+1.23, +4.56)` and in `stall=no lidar`, so
        # splitting it on whitespace gives the wrong fields.
        self.status_json_pub = self.create_publisher(String, '~/status_json', 10)
        self.mode_pub = self.create_publisher(String, '~/mode', LATCHED_QOS)

        # Tells the saved-map updater to stop folding in scans; slam_toolbox
        # is held off through its own pause service instead.
        self.freeze_pub = self.create_publisher(Bool, 'map_freeze', 10)
        try:
            self.pause_client = self.create_client(
                Pause, 'slam_toolbox/pause_new_measurements')
        except Exception as exc:                       # noqa: BLE001
            # No pause service means no held mapping, not no node.
            self.get_logger().warn(f'slam_toolbox pause client unavailable: {exc}')
            self.pause_client = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_timer(self.replan_period, self.replan)
        self.create_timer(0.1, self.follow_path)
        self.create_timer(0.1, self.service_handoff)
        self.create_timer(0.5, self.check_collision)
        # Its own timer, at 10 Hz. The IMU test is cheap and is the fast one -
        # putting it on the 2 Hz collision timer would have thrown away most
        # of the speed that makes it worth having.
        self.create_timer(0.1, self.check_stalled)
        self.create_timer(1.0, self.publish_status)
        self.create_timer(1.0, self.publish_blocks)

        # One-shot replan, for when `follow_path` needs a fresh plan NOW but
        # must not compute it inside its own 10 Hz tick. Armed by
        # `request_replan`, disarmed again the moment it fires.
        self.replan_kick = self.create_timer(0.01, self.kicked_replan)
        self.replan_kick.cancel()

        self.publish_mode()
        self.get_logger().info('Frontier exploration started')

    def _reset_mission_state(self):
        """
        Forget every trace of the mission so far.

        Called once from `__init__` and again whenever a command starts a
        fresh explore or clean, so the two cannot drift apart. Leaving any of
        this behind is how a 'clean' issued after a finished run inherited a
        full `covered` set and declared itself done on its first replan.
        """
        self.path = []
        self.no_frontier_count = 0
        self.near_only_count = 0

        # What the last replan worked out: the map message it used, the
        # inflated drivable mask and the wavefront from the robot. Lets an
        # arrival route to the next waypoint without redoing all of that.
        self.plan_cache = None

        # Explore->clean handoff (stop, pose, save map). `handoff` is the
        # one in flight, or None; `handoff_finished` stops start_cleaning
        # starting a second one for the same finished exploration.
        self.handoff = None
        self.handoff_finished = False

        self.cleaning_goals = []
        self.clean_regions = 0
        self.clean_corner = None
        self.clean_axis = 'x'
        self.clean_side = 'top'
        self.clean_coverage = 0.0
        self.goal_regions = []
        self.clean_rows = 0
        self.goal_index = 0
        self.cleaning_pass = 0
        self.covered = set()
        self.clean_plan_ready = False
        self.blocks_done = 0
        self.blocks_total = 0

        # Block chasing. `target_block` is the square being worked right now;
        # `dead_blocks` are the ones given up on, so the picker stops offering
        # them and the run cannot stall on a corner it can never reach.
        self.target_block = None
        self.target_tries = 0
        self.dead_blocks = set()
        self.blocks_cleared = 0
        self.chasing_blocks = self.clean_mode == 'blocks'

        # Progress watchdog for the square being worked.
        self.target_since = 0.0
        self.target_best = 0.0
        self.target_gain_time = 0.0

        # Wedge detection. Wall clock, so a stalled sim clock cannot make the
        # robot look "not stuck" forever.
        self.progress_pose = None
        self.progress_time = time.monotonic()
        self.recovery_until = 0.0
        self.recovery_attempts = 0

        # The stop / aim / go recovery. `recovery_phase` is None when no
        # recovery is running, else one of 'stop', 'aim', 'turn', 'go',
        # 'fallback'. `recovery_phase_until` is when the current step times
        # out; `recovery_target` is the world-frame heading being rotated to;
        # `recovery_started` anchors `recovery_budget`; `recovery_turned` is
        # True once the turn-to-clear-heading step has been used, so a
        # recovery rotates to a chosen heading at most once.
        self.recovery_phase = None
        self.recovery_phase_until = 0.0
        self.recovery_target = None
        self.recovery_started = 0.0
        self.recovery_turned = False

        # The localization gate, re-armed for every mission. `localized`
        # flips once - and only once - map -> base_footprint has resolved
        # with a FRESH stamp and a /map has arrived; until then the wheels
        # get zeros. `localize_started` anchors `localize_timeout`.
        self.localized = False
        self.localize_started = time.monotonic()

        # Consecutive ticks where publish_safe refused everything it was
        # given. follow_path runs at 10 Hz, so 10 is one second of a robot
        # being told to move and not moving.
        self.gated_ticks = 0
        self.gated_limit = 10

        # Reverse pulses the watchdog has issued back to back. Bounded by
        # `max_forced_reverse`, because on real motors "nudge backwards once
        # a second forever" is a robot grinding against whatever is behind it.
        self.forced_reverses = 0

        # While this is in the future the robot is backing out and will not
        # change its mind and drive forward again. `backout_started` bounds
        # how long it may keep extending that.
        self.backout_until = 0.0
        self.backout_started = 0.0
        self.blacklist = set()

        # Checks the wheels' claim against what the room actually does.
        self.watch = StallWatch(
            window=self.get_parameter('stall_window').value,
            confirm=self.get_parameter('stall_confirm').value,
            release=self.get_parameter('stall_release').value,
            fit_threshold=self.get_parameter('stall_fit_threshold').value,
            rotation_ratio=self.get_parameter('stall_rotation_ratio').value,
        )

        # Only the RISING edge of a stall is a new crash. The watch stays
        # latched for as long as the robot is pinned, and reporting on every
        # tick of that produced CRASH #1 through #10 in twelve seconds for what
        # was one obstacle - burning all three recovery attempts, twice, on a
        # single wall.
        self.was_stalled = False

    # ------------------------------------------------------------------
    # Runtime control
    # ------------------------------------------------------------------

    def command_callback(self, msg):
        """
        Change what the robot is doing, from the web page or the shell.

        `explore` and `clean` always start a NEW mission - there is no
        "carry on exploring". `pause` holds the wheels with everything else
        intact; `resume` lets go. `stop` drops the plan too and parks the
        node in 'stopped' until the next explore or clean.
        """
        command = msg.data.strip().lower()
        now = time.monotonic()

        if command in ('explore', 'clean'):
            self._reset_mission_state()
            self.paused = False
            self.mode = command
            self.get_logger().info(f'Command {command!r}: starting a fresh mission')

        elif command == 'pause':
            if not self.paused:
                self.paused = True
                self.pause_started = now
                self.cmd_pub.publish(Twist())
                self.get_logger().info('Paused')

        elif command == 'resume':
            if self.paused:
                self.paused = False
                self.unpause_clocks(now - self.pause_started)
                self.get_logger().info('Resumed')

        elif command == 'stop':
            self.paused = False
            self.path = []
            self.recovery_until = 0.0
            self.recovery_phase = None
            self.backout_until = 0.0
            self.handoff = None
            self.mode = 'stopped'
            self.cmd_pub.publish(Twist())
            self.get_logger().info('Stopped - send explore or clean to start again')

        else:
            self.get_logger().warn(
                f'Unknown command {msg.data!r}; want explore, clean, pause, '
                f'resume or stop')
            return

        self.publish_mode()

    def unpause_clocks(self, elapsed):
        """
        Push the mission's wall-clock watchdogs past a pause.

        They all run on `time.monotonic()`, which does not stop when the robot
        does. Without this a minute's pause comes back as "no progress for a
        minute": `is_stuck` fires on the first tick and the square being
        worked is written off by `block_stall_timeout` before the wheels
        have turned.
        """
        self.progress_time += elapsed
        self.target_since += elapsed
        self.target_gain_time += elapsed

        if self.handoff is not None:
            self.handoff['phase_start'] += elapsed

    def reported_mode(self):
        """The mode as the outside world should see it."""
        if self.paused:
            return 'paused'

        if not self.localized and self.mode in ('explore', 'clean'):
            return 'localizing'

        if time.monotonic() < self.recovery_until:
            return 'recovering'

        return self.mode

    def publish_mode(self):
        """Latch the current mode on `~/mode`, only when it actually changes."""
        mode = self.reported_mode()

        if mode == self.last_mode_published:
            return

        self.last_mode_published = mode

        message = String()
        message.data = mode
        self.mode_pub.publish(message)

    # ------------------------------------------------------------------
    # Inputs
    # ------------------------------------------------------------------

    def map_callback(self, msg):
        self.map_msg = msg

    def scan_transform(self, frame):
        """
        Where the scanner sits in `base_frame`, as (cos, sin, x, y, yaw).

        Read from TF rather than typed in, so the URDF stays the one place
        that says where the lidar is. On Nexva it is not where the sim had it:
        base_link is yawed +90 deg under base_footprint and the laser joint
        adds another +90 deg, so the scanner's own x axis points out the BACK
        of the robot. Feed those points in raw and every "ahead", "left" and
        "behind" in this file is silently the opposite way round.

        The mount is fixed, so the first lookup that works is the only one
        needed; until then scans are dropped rather than trusted.
        """
        if self.scan_offset is not None:
            return self.scan_offset

        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, frame, Time())
        except tf2_ros.TransformException as exc:
            self.get_logger().warn(
                f'No transform {self.base_frame} <- {frame} yet ({exc}); '
                f'dropping scans until it appears',
                throttle_duration_sec=5.0,
            )
            return None

        t = tf.transform.translation
        q = tf.transform.rotation

        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        self.scan_offset = (math.cos(yaw), math.sin(yaw), t.x, t.y, yaw)
        self.get_logger().info(
            f'Lidar frame {frame!r} sits at ({t.x:+.3f}, {t.y:+.3f}) m, '
            f'yawed {math.degrees(yaw):+.0f} deg, in {self.base_frame}'
        )

        return self.scan_offset

    def scan_callback(self, scan):
        """
        Cache the full 360 deg sweep as points in the robot's own frame.

        In the sim the lidar sat at the centre of base_link (0, 0, 0.18) with
        x forward, so a return at (range, angle) was already a point in the
        base frame. Here every return goes through the mount transform from
        `scan_transform` first - see there for why it matters.
        """
        offset = self.scan_transform(scan.header.frame_id)

        if offset is None:
            # Better no scan than a mirrored one. With nothing cached the
            # staleness stop in follow_path holds the wheels until TF is up.
            return

        cos_yaw, sin_yaw, shift_x, shift_y, yaw = offset
        points = []

        for i, distance in enumerate(scan.ranges):
            if not math.isfinite(distance):
                continue

            if not (scan.range_min <= distance <= scan.range_max):
                continue

            angle = scan.angle_min + i * scan.angle_increment
            local_x = distance * math.cos(angle)
            local_y = distance * math.sin(angle)
            x = shift_x + cos_yaw * local_x - sin_yaw * local_y
            y = shift_y + sin_yaw * local_x + cos_yaw * local_y

            # Range re-measured from the robot's centre, which is what the
            # turning-circle tests compare against. Identical to the raw
            # range while the scanner sits on the axis, right if it ever
            # does not.
            points.append((x, y, math.hypot(x, y)))

        self.scan_points = points
        self.scan = scan
        self.scan_time = time.monotonic()

        # The stall test lives on consecutive scans, so it is fed here rather
        # than on a timer - a timer would re-compare the same scan with itself
        # whenever it ran faster than the lidar, and an unchanged scan is
        # exactly the stall signature.
        #
        # The watch predicts each beam's range change from the bearing it
        # makes with the robot's direction of travel, so it gets bearings in
        # the ROBOT frame: the raw angle plus the mount yaw. Handed the raw
        # angle on a scanner facing backwards, every prediction has the wrong
        # sign and honest driving fits as a stall.
        self.watch.feed_scan(
            time.monotonic(), scan.ranges, scan.angle_min + yaw,
            scan.angle_increment, scan.range_min, scan.range_max)

    def scan_age(self):
        """Seconds since the last scan arrived, or None if none ever has."""
        if self.scan_time is None:
            return None

        return time.monotonic() - self.scan_time

    def scan_is_stale(self):
        """
        Whether the lidar has gone quiet for longer than `scan_timeout`.

        `scan_points` is a snapshot, and nothing else in the file ages it. A
        lidar that unplugs mid-run would leave the robot driving on the last
        room it saw, with every clearance test answering from a picture that
        stopped being true.
        """
        age = self.scan_age()

        return age is None or age > self.scan_timeout

    def forward_clearance(self):
        """
        Gap between the front edge and the nearest obstacle straight ahead.

        Only returns inside the corridor the robot actually sweeps when
        driving forward - its own width plus a margin - are considered, and
        only those genuinely BEYOND the front edge.

        The `x > half_length` is doing real work. With `x > 0` this counted
        anything alongside the robot's own body: a wall running parallel at
        0.13 m to the side has returns at x near 0 and y = -0.13, which is
        inside the corridor, so it reported a clearance of about
        -half_length - permanently, whatever was actually in front.

        A robot driving within `half_width + safety_margin` of a wall
        therefore believed it was blocked ahead for as long as it stayed
        there, and reversing never helped because the wall stayed alongside.
        In a corner that never resolves: measured, forward_clearance sat at
        -0.132 m while the robot reversed a metre and a half away from the
        wall in front of it.
        """
        # The corridor is the robot's ACTUAL half width, not that plus the
        # safety margin. Widening it sounds safer and is not: a wall running
        # alongside at 0.15 m is one the robot physically clears by 1.5 cm,
        # but a 0.175 m corridor counts it as an obstacle - at every x, for
        # as long as the robot stays beside it. The robot then believes it is
        # blocked ahead with the whole room open in front of it. Measured in
        # a corner facing out: 0.02 m travelled in 60 seconds.
        #
        # Longitudinal safety is `stop_distance`, applied to the clearance
        # this returns. Lateral safety is the planner, which inflates
        # obstacles by `plan_radius` when it lays out a path.
        limit = self.half_width
        best = float('inf')

        for x, y, _ in self.scan_points:
            if x > self.half_length and abs(y) <= limit:
                best = min(best, x - self.half_length)

        return best

    def rear_clearance(self):
        """
        Gap between the back edge and the nearest obstacle straight behind.

        The mirror of `forward_clearance`, with the same corridor and the
        same `x` cut-off, for the same reasons. It exists because reverse
        used to be the one direction nobody checked: of the eleven places
        that back up, one looked behind first. In Gazebo the worst case was
        a clipped wall; on real motors it is a wheel grinding against one.

        A corridor with NO returns in it is not the same as a clear one. The
        scanner cannot see inside its own minimum range, and on this chassis
        an occluded sector reads exactly like an empty one - so by default
        silence means blocked, and only `allow_blind_reverse` says otherwise.
        """
        limit = self.half_width
        best = float('inf')
        seen = False

        for x, y, _ in self.scan_points:
            if x < -self.half_length and abs(y) <= limit:
                seen = True
                best = min(best, -x - self.half_length)

        if not seen and not self.allow_blind_reverse:
            return 0.0

        return best

    def nearest_obstacle(self):
        """Bearing and range of the closest return, in the robot frame."""
        best = None

        for x, y, distance in self.scan_points:
            if best is None or distance < best[1]:
                best = (math.atan2(y, x), distance)

        return best if best is not None else (None, None)

    def avoid_steer(self):
        """
        How hard to turn away from the nearest obstacle, in rad/s.

        The angle does the work, which is what makes this different from
        picking a side and committing to it. An obstacle dead ahead gets the
        full steer; one off to the side gets almost none, because driving past
        it is fine. An obstacle at the stopping distance gets the full steer;
        one at the slow-down distance gets almost none.

        Sign is away from the obstacle. When it is within a few degrees of
        dead ahead the sign is a coin toss, so the side with more room wins -
        otherwise the robot picks a direction that steers it into the wall it
        is already too close to.
        """
        bearing, distance = self.nearest_obstacle()

        if bearing is None:
            return 0.0

        # 1 straight ahead, 0 at 90 degrees and behind. Nothing behind the
        # robot should make it swerve while it is driving forwards.
        head_on = max(0.0, math.cos(bearing))

        # 1 at the stopping distance, fading to 0 by the slow-down distance.
        span = max(1e-3, self.slow_distance - self.stop_distance)
        near = (self.slow_distance - distance) / span
        near = max(0.0, min(1.0, near))

        if abs(bearing) < math.radians(5.0):
            away = 1.0 if self.side_clearance(1) > self.side_clearance(-1) else -1.0
        else:
            away = -1.0 if bearing > 0.0 else 1.0

        return away * self.avoid_turn * self.angular_speed * head_on * near

    def can_turn(self):
        """
        Whether a spin would actually survive `publish_safe`.

        Every place that decides to rotate must ask THIS, not
        `rotation_clearance() > 0`. The gate refuses a spin below
        `turn_margin`, so a branch testing `> 0` can choose to rotate in the
        0 to turn_margin window, have the rotation zeroed, and - having set no
        linear component - publish nothing at all. Measured before this was
        shared: 18 of 40 situations froze, including one with a full metre of
        clear space straight ahead and a single obstacle 0.20 m to the side.
        """
        return self.rotation_clearance() >= self.turn_margin

    def rotation_clearance(self):
        """
        Gap between the circle swept while spinning and the nearest return.

        This is the check the old forward-only cone was missing: turning in
        place sweeps every direction, so a wall beside or behind the robot
        matters just as much as one in front.
        """
        best = float('inf')

        for _, _, distance in self.scan_points:
            best = min(best, distance - self.turn_radius)

        return best

    def odom_callback(self, msg):
        """Track the wheel-odometry pose, which is what RViz drifts on."""
        position = msg.pose.pose.position
        q = msg.pose.pose.orientation

        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        self.odom_pose = (position.x, position.y, yaw)
        self.odom_speed = abs(msg.twist.twist.linear.x)

        # What the wheels CLAIM. The stall watch exists to disagree with it.
        self.watch.feed_wheels(
            msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def imu_callback(self, msg):
        """Give the watch the rotation the wheels cannot be trusted to report."""
        self.watch.feed_gyro(time.monotonic(), msg.angular_velocity.z)

    def check_stalled(self):
        """
        Notice, from the robot's own body, that it has stopped moving.

        This is the answer to "the wheels are spinning but I am not going
        anywhere". It consults the wheels only for the claim it is checking,
        and it needs neither the map nor SLAM - both of which are dragged
        along by the same lying odometry and so can agree with it all the way
        into the wall.
        """
        if self.mode == 'done' or not self.use_imu_stall:
            return

        now = time.monotonic()

        # Nothing to react to during a deliberate escape: reversing away from
        # the wall is the cure, not another symptom.
        if now < self.recovery_until or self.backing_out():
            return

        verdict = self.watch.poll(now)
        rising = verdict.stalled and not self.was_stalled
        self.was_stalled = verdict.stalled

        if not rising:
            return

        self.imu_stalls += 1
        self.report_crash(f'the robot is not moving - {verdict.reason}')

    def check_collision(self):
        """
        Notice when the wheels are turning but the robot is not moving.

        Wheel odometry integrates whatever the wheels do, so driving into a
        wall keeps "moving" the robot in odom even though it is pinned - that
        false motion is what drags the RViz pose off and smears the map. The
        map frame comes from scan matching against the world, so comparing the
        two says whether the robot really travelled. They disagreeing means a
        crash or a slipping wheel.
        """
        if self.mode == 'done':
            return

        self.review_map_freeze()

        # Fast test first: it fires within a fraction of a second, before much
        # phantom odometry has piled up.
        if self.scan_stalled():
            self.report_crash('wheels turning but the view is frozen')
            return

        if self.odom_pose is None or self.last_pose is None:
            return

        now = time.monotonic()

        if self.slip_ref_odom is None:
            self.slip_ref_odom = self.odom_pose
            self.slip_ref_map = self.last_pose
            self.slip_ref_time = now
            return

        if now - self.slip_ref_time < self.slip_window:
            return

        wheels_say = math.dist(self.odom_pose[:2], self.slip_ref_odom[:2])
        world_says = math.dist(self.last_pose[:2], self.slip_ref_map[:2])

        self.slip_ref_odom = self.odom_pose
        self.slip_ref_map = self.last_pose
        self.slip_ref_time = now

        if wheels_say < self.slip_distance:
            return

        if world_says >= self.slip_ratio * wheels_say:
            return

        self.report_crash(
            f'wheels turned {wheels_say:.2f} m but the robot only moved '
            f'{world_says:.2f} m in the map'
        )

    def report_crash(self, reason):
        """React to a crash, however it was spotted."""
        now = time.monotonic()

        # One crash, not one per detector per tick.
        if now - self.last_crash_time < 1.0:
            return

        self.collisions += 1
        self.last_crash_time = now

        self.get_logger().warn(
            f'CRASH #{self.collisions}: {reason} - backing off, mapping held')

        # The odometry is now lying about where the robot is. Anything mapped
        # against it lands in the wrong place, which is what tears the map and
        # makes it lurch. Stop taking new scans until motion is real again.
        self.set_map_frozen(True)

        # Treat it exactly like being wedged: stop pushing and get out.
        if self.last_pose is not None and now >= self.recovery_until:
            self.begin_recovery(self.last_pose[0], self.last_pose[1])

    def scan_stalled(self):
        """
        Report whether the wheels are turning while the view stays frozen.

        This is the fast crash test. The odometry-versus-map comparison has to
        wait for centimetres of disagreement to build up, and every moment
        spent waiting is phantom odometry that SLAM later has to unwind as a
        visible jump. Two scans taken a fraction of a second apart already
        say whether the robot actually moved.
        """
        scan = self.scan
        now = time.monotonic()

        if scan is None or not self.use_scan_stall:
            return False

        ranges = scan.ranges

        if self.scan_ref is None or len(self.scan_ref) != len(ranges):
            self.scan_ref = list(ranges)
            self.scan_ref_time = now
            return False

        if now - self.scan_ref_time < self.scan_stall_window:
            return False

        moved = []
        for previous, current in zip(self.scan_ref, ranges):
            if math.isfinite(previous) and math.isfinite(current):
                if scan.range_min <= previous <= scan.range_max:
                    if scan.range_min <= current <= scan.range_max:
                        moved.append(abs(current - previous))

        self.scan_ref = list(ranges)
        self.scan_ref_time = now

        if len(moved) < 20:
            return False

        # Wheels clearly turning, yet the world looks identical.
        return (sum(moved) / len(moved) < self.scan_stall_delta
                and self.odom_speed > 0.05)

    def review_map_freeze(self):
        """Let mapping resume once the robot is properly moving again."""
        if not self.want_map_frozen:
            return

        now = time.monotonic()

        if now < self.recovery_until:
            return

        if now - self.last_crash_time < self.resume_mapping_after:
            return

        self.get_logger().info('Motion looks real again - mapping resumed')
        self.set_map_frozen(False)

    def set_map_frozen(self, freeze):
        """Hold off or release new scans, in whichever mapper is running."""
        if not self.protect_map_on_crash:
            return

        if self.want_map_frozen == freeze:
            return

        self.want_map_frozen = freeze

        # The saved-map updater watches this topic.
        message = Bool()
        message.data = freeze
        self.freeze_pub.publish(message)

        self.toggle_slam_pause(freeze)

    def toggle_slam_pause(self, freeze):
        """
        Flip slam_toolbox's pause exactly once per state change.

        Its service is a toggle, and the `status` it returns is only "the call
        worked" - it is true whichever way the flip went. So the resulting
        state cannot be read back, and driving towards a target by re-calling
        and re-checking just toggles forever. The state is tracked here.
        """
        if self.map_frozen == freeze:
            self.get_logger().debug(
                f'slam pause: already believe frozen={freeze}, no toggle sent')
            return

        if self.pause_pending:
            self.get_logger().warn('slam pause: a toggle is still in flight')
            return

        if self.pause_client is None or not self.pause_client.service_is_ready():
            # No slam_toolbox here - a saved-map run uses the freeze topic.
            self.get_logger().debug(
                'slam pause: no slam_toolbox, using the freeze topic only')
            return

        # slam_toolbox only logs when it enters pause, never when it leaves,
        # so log both directions here to keep the trail readable.
        self.get_logger().info(f'slam pause: toggling towards frozen={freeze}')

        self.pause_pending = True

        try:
            future = self.pause_client.call_async(Pause.Request())
        except Exception as exc:                       # noqa: BLE001
            # Ready a moment ago, gone now. The freeze topic already went
            # out, so whatever mapper is left is held where it can be.
            self.pause_pending = False
            self.get_logger().warn(f'Could not reach slam_toolbox pause: {exc}')
            return

        future.add_done_callback(
            lambda done, target=freeze: self.pause_done(done, target))

    def pause_done(self, future, target):
        self.pause_pending = False

        try:
            future.result()
        except Exception as exc:                       # noqa: BLE001
            self.get_logger().warn(f'Could not change mapping pause: {exc}')
            return

        self.map_frozen = target
        self.get_logger().debug(f'slam pause: now believe frozen={target}')

    def publish_status(self):
        """
        Report where the robot thinks it is and whether it is crashing.

        Published as plain text on `robot_status` so it can be watched with
        `ros2 topic echo` without any custom message type.
        """
        self.publish_mode()
        self.publish_status_json()

        if self.last_pose is None:
            return

        x, y, yaw = self.last_pose

        drift = 0.0
        if self.odom_pose is not None:
            drift = math.dist(self.odom_pose[:2], (x, y))

        state = 'recovering' if time.monotonic() < self.recovery_until else self.mode

        message = String()
        message.data = (
            f'state={state} '
            f'map_pose=({x:+.2f}, {y:+.2f}) '
            f'heading={math.degrees(yaw):+.0f}deg '
            f'odom_drift={drift:.2f}m '
            f'crashes={self.collisions} '
            f'recoveries={self.recovery_attempts} '
            f'mapping={"held" if self.want_map_frozen else "live"} '
            f'{self.motion_status()} '
            f'blocks={self.blocks_done}/{self.blocks_total} '
            f'{self.block_status()} '
            f'{self.sweep_status()}'
        )
        self.status_pub.publish(message)

    def sweep_status(self):
        """
        How far through the boustrophedon sweep the robot is.

        Written to be read live off a terminal: it has to answer "is it
        sweeping, where has it got to, and how much floor is done" from one
        line of `ros2 topic echo robot_status`, without the logs.
        """
        if self.mode != 'clean':
            return 'sweep=off'

        if self.chasing_blocks:
            # Either clean_mode:='blocks', or the rows are finished and this
            # is the gap-fill. Either way there is no row position to report,
            # only how much floor is down.
            stage = 'gapfill' if self.clean_plan_ready else 'blocks'
            return f'sweep={stage} coverage={self.clean_coverage:.0f}%'

        total = len(self.cleaning_goals)

        if not total:
            return 'sweep=planning'

        return (f'sweep=rows '
                f'region={self.current_region()}/{max(1, self.clean_regions)} '
                f'waypoint={min(self.goal_index + 1, total)}/{total} '
                f'rows={self.clean_rows} '
                f'pass={self.cleaning_pass}/{self.max_cleaning_passes} '
                f'coverage={self.clean_coverage:.0f}%')

    def publish_status_json(self):
        """
        The same report as an object, for the web page.

        Goes out whether or not there is a pose yet: a page that hears
        nothing cannot tell "waiting for TF" from "node is dead".
        """
        now = time.monotonic()

        x = y = heading = drift = None

        if self.last_pose is not None:
            x, y, yaw = self.last_pose
            heading = round(math.degrees(yaw), 1)

            if self.odom_pose is not None:
                drift = round(math.dist(self.odom_pose[:2], (x, y)), 3)

            x = round(x, 3)
            y = round(y, 3)

        scan_age = self.scan_age()
        pose_age = None if self.pose_time is None else now - self.pose_time

        target = None
        if self.target_block is not None:
            target = [int(self.target_block[0]), int(self.target_block[1])]

        coverage = 0.0
        if self.blocks_total > 0:
            coverage = self.blocks_done / self.blocks_total

        status = {
            'mode': ('localizing'
                     if not self.localized and self.mode in ('explore', 'clean')
                     else 'recovering' if now < self.recovery_until
                     else self.mode),
            'localized': self.localized,
            'paused': self.paused,
            'x': x,
            'y': y,
            'heading_deg': heading,
            'odom_drift_m': drift,
            'crashes': self.collisions,
            'recoveries': self.recovery_attempts,
            'mapping': 'held' if self.want_map_frozen else 'live',
            'blocks_done': int(self.blocks_done),
            'blocks_total': int(self.blocks_total),
            'blocks_skipped': len(self.dead_blocks),
            'target': target,
            'coverage': round(coverage, 4),
            'stall': self.motion_status(),
            'scan_age_s': None if scan_age is None else round(scan_age, 2),
            'pose_age_s': None if pose_age is None else round(pose_age, 2),
        }

        message = String()
        message.data = json.dumps(status, separators=(',', ':'))
        self.status_json_pub.publish(message)

    def block_status(self):
        """Which square the robot is working, and what it has written off."""
        if not self.chasing_blocks:
            return 'target=rows'

        if self.target_block is None:
            return f'target=none skipped={len(self.dead_blocks)}'

        return (f'target={self.target_block[0]},{self.target_block[1]} '
                f'skipped={len(self.dead_blocks)}')

    def motion_status(self):
        """
        Summarise what the body says about whether the robot is moving.

        Reported next to the wheel figures on purpose: when they disagree, the
        disagreement is the interesting part and it should be readable with a
        plain `ros2 topic echo robot_status`.
        """
        if not self.use_imu_stall:
            return 'imu=off'

        if not self.watch.has_scan(time.monotonic()):
            return 'stall=no lidar'

        measured = self.watch.fit()

        if measured is None:
            return 'stall=idle'

        # A scene with nothing in it that motion would change - long parallel
        # walls, nothing ahead in range - cannot vouch for the wheels either
        # way, and the watch abstains rather than guessing. Say so: "not
        # watching here" and "watching, all fine" must be distinguishable on
        # the status line.
        information = self.watch.info()

        if information is not None and information < self.watch.min_info:
            return f'stall=degenerate info={information:.0f}/{self.watch.min_info:.0f}'

        return (f'view_backs={measured:+.2f}/{self.watch.fit_threshold:.2f} '
                f'moving={"no" if self.watch.stalled else "yes"} '
                f'stalls={self.imu_stalls}')

    def publish_safe(self, cmd):
        """
        Last gate before the wheels: refuse motion the footprint cannot make.

        Every command goes through here. Individual branches used to police
        themselves and three of them did not - the turn taken when no path is
        planned, the steering correction applied while driving, and the
        forward-and-turning case - which is how the robot could still clip a
        wall with a corner while turning.

        Rotating in place sweeps a circle of `turn_radius` around the centre.
        If anything at all is inside that circle, in ANY direction, the turn
        is refused - a wall behind or beside matters exactly as much as one in
        front. Reverse used to be left alone so an escape was always possible;
        on real motors it gets the same `stop_distance` check as forward, and
        the escape that is always possible is now "stop", not "push".

        Whatever survives is clamped to `max_linear` / `max_angular` on the
        way out. Nothing above asks for more, but a gate that trusts its
        callers is not a gate.
        """
        # Nothing at all from a lidar that has stopped talking. Every test
        # below reads `scan_points`, and a stale snapshot answers every one
        # of them with confidence.
        if self.scan_is_stale():
            self.cmd_pub.publish(Twist())
            return

        # A small margin, not zero. Stopping exactly at the swept circle
        # means the decision lands on the same tick the corner arrives, and
        # measured that way the closest approach while turning was 0.188 m
        # against a 0.191 m circle - no contact, but no room either.
        wanted = (cmd.linear.x, cmd.angular.z)

        # The forward rule runs FIRST. The turn rule below asks whether the
        # robot is still translating, and if it asked before this it could see
        # a forward speed that is about to be taken away - letting a turn
        # through as "an arc" and then zeroing the linear underneath it,
        # leaving exactly the spin in place the turn rule exists to refuse.
        # Measured with the old order: 3720 spins in place allowed through.
        if cmd.linear.x > 0.0 and self.forward_clearance() <= self.stop_distance:
            cmd.linear.x = 0.0

        # And its mirror for reverse, for the same reason. Ten of the eleven
        # branches that back up never look behind first.
        if cmd.linear.x < 0.0 and self.rear_clearance() <= self.stop_distance:
            cmd.linear.x = 0.0

        # And the same refusal when the body says the robot is pinned, whether
        # or not the lidar can see what it is pinned against. This is the
        # direct answer to "it keeps thinking it is going forward": while the
        # IMU feels nothing, forward is not a thing this robot is allowed to
        # keep asking for. Reverse is untouched, so the escape always exists -
        # and if this leaves nothing at all, the gated-ticks watchdog below
        # backs out after a second.
        #
        # The stop_distance rule above cannot cover this. A robot wedged on a
        # chair leg, a cable or a lip in the floor has clear air in front of
        # its lidar and every reason, as far as that rule is concerned, to
        # keep driving into it.
        if cmd.linear.x > 0.0 and self.use_imu_stall and self.watch.stalled:
            cmd.linear.x = 0.0

        if cmd.angular.z != 0.0 and not self.can_turn():
            # A spin in place sweeps the whole turning circle, so it stays
            # refused. A gentle arc while the robot is translating does not -
            # the translation carries the body and the turn only bends the
            # path. Refusing those too is what left the robot with nothing but
            # straight forward and straight back, which is the shuffle: it
            # could never change heading near a wall, so it never got out.
            #
            # Allowed only while translating, only up to the gentle avoidance
            # rate, and only when it turns AWAY from the closest return - so
            # it always increases the clearance that is the reason for the
            # restriction in the first place.
            bearing, _ = self.nearest_obstacle()
            gentle = abs(cmd.angular.z) <= self.avoid_turn * self.angular_speed + 1e-9
            translating = abs(cmd.linear.x) > 1e-6
            away = (bearing is None
                    or abs(bearing) < math.radians(5.0)
                    or (cmd.angular.z > 0.0) != (bearing > 0.0))

            if not (translating and gentle and away):
                cmd.angular.z = 0.0

        # Watchdog: the gate ate a real command and left nothing behind.
        #
        # drive_recovery no longer does this, but any future branch that asks
        # for a pure turn in a tight spot would - and the failure is silent
        # and total. The robot publishes zero, so it never moves, so the
        # progress timer never resets, so it is "stuck" against the same
        # obstacle forever. Reverse is the one thing this gate never refuses,
        # so that is what to fall back on.
        moving = abs(cmd.linear.x) > 1e-6 or abs(cmd.angular.z) > 1e-6
        asked = abs(wanted[0]) > 1e-6 or abs(wanted[1]) > 1e-6

        if asked and not moving:
            self.gated_ticks += 1

            if self.gated_ticks >= self.gated_limit:
                self.forced_reverses += 1

                if self.forced_reverses > self.max_forced_reverse:
                    # Pulsing backwards has not changed anything for as many
                    # goes as it gets. In the sim the next pulse cost
                    # nothing; here it is a motor stalled against a wall.
                    self.get_logger().error(
                        f'{self.forced_reverses - 1} forced reverse pulses '
                        f'and still nothing is allowed through - stopping. '
                        f'Wanted {wanted[0]:+.2f} m/s {wanted[1]:+.2f} rad/s.'
                    )
                    self.path = []
                    self.mode = 'stopped'
                    self.gated_ticks = 0
                    self.forced_reverses = 0
                    self.cmd_pub.publish(Twist())
                    return

                # The pulse goes through the rear check like any other
                # reverse: it is the fallback for a refused FORWARD, and it
                # earns nothing by pushing into what is behind.
                if self.rear_clearance() > self.stop_distance:
                    cmd.linear.x = -0.4 * self.linear_speed
                cmd.angular.z = 0.0
                self.get_logger().warn(
                    f'Every command refused for {self.gated_ticks} ticks '
                    f'({self.gated_ticks / 10.0:.1f} s) - backing out '
                    f'({self.forced_reverses}/{self.max_forced_reverse}). '
                    f'Wanted {wanted[0]:+.2f} m/s {wanted[1]:+.2f} rad/s.',
                    throttle_duration_sec=5.0,
                )
                self.gated_ticks = 0
        else:
            self.gated_ticks = 0
            self.forced_reverses = 0

        cmd.linear.x = max(-self.max_linear, min(self.max_linear, cmd.linear.x))
        cmd.angular.z = max(-self.max_angular, min(self.max_angular, cmd.angular.z))

        self.cmd_pub.publish(cmd)

    def block_state(self, msg, traversable):
        """
        Work out how much of each coverage square has been swept.

        `traversable` is free floor with the obstacles already inflated by the
        driving half-width, so a square that lies inside a wall or under a
        chair contributes no cells and never appears in the result at all.
        """
        rows, cols = np.nonzero(traversable)

        if rows.size == 0:
            return None

        res = msg.info.resolution
        xs = msg.info.origin.position.x + (cols + 0.5) * res
        ys = msg.info.origin.position.y + (rows + 0.5) * res

        block_x = np.floor(xs / self.block_size).astype(np.int64)
        block_y = np.floor(ys / self.block_size).astype(np.int64)

        keys = ((quantise(xs, res) + COVER_ORIGIN) * COVER_STRIDE
                + (quantise(ys, res) + COVER_ORIGIN))

        if self.covered:
            covered = np.fromiter(self.covered, dtype=np.int64,
                                  count=len(self.covered))
            swept = np.isin(keys, covered)
        else:
            swept = np.zeros(keys.shape, dtype=bool)

        origin_x = int(block_x.min())
        origin_y = int(block_y.min())
        span = int(block_y.max()) - origin_y + 1

        flat_block = (block_x - origin_x) * span + (block_y - origin_y)
        total = np.bincount(flat_block)
        done = np.bincount(flat_block, weights=swept.astype(np.float64),
                           minlength=total.size)

        return BlockState(rows, cols, swept, flat_block, total, done,
                          (origin_x, origin_y), span)

    def red_blocks(self, state):
        """Return the blocks that still have floor left to sweep."""
        fractions = state.fractions()
        red = (state.total > 0) & (fractions < self.clean_threshold)

        for block in self.dead_blocks:
            index = state.index_of(block)

            if index is not None:
                red[index] = False

        return np.nonzero(red)[0]

    def advance_blocks(self, msg, traversable, dist, start, h, w):
        """
        Go to whichever square is still red, and stay in it until it is green.

        This is the coverage display driving the robot rather than describing
        it. A zig-zag plan is laid out once and then followed blind, so a row
        the robot was pushed off, or floor that only appeared after the map
        grew, stays red with nothing left to go back for it. Picking the next
        target off the squares themselves closes that: the robot cannot finish
        while anything reachable is still red, because "still red" is exactly
        what it looks for.

        Reachability is the BFS flood from the robot, so a square behind a
        closed door is skipped for the same reason a square inside a wall is -
        there is no cell in it the robot can stand on.
        """
        state = self.block_state(msg, traversable)

        if state is None:
            self.complete_cleaning(msg, traversable)
            return

        flat = state.rows * w + state.cols
        reachable = dist[flat] >= 0

        index = None
        now = time.monotonic()

        # Stay on the current square while it still needs work and is still
        # getting better.
        if self.target_block is not None:
            index = state.index_of(self.target_block)

            if index is None:
                self.target_block = None
            else:
                fraction = state.fraction(index)

                if fraction > self.target_best + 0.01:
                    self.target_best = fraction
                    self.target_gain_time = now

                if fraction >= self.clean_threshold:
                    self.blocks_cleared += 1
                    self.get_logger().info(
                        f'Block {self.target_block} is green '
                        f'({100.0 * fraction:.0f}%) - '
                        f'{self.blocks_cleared} done'
                    )
                    self.target_block = None
                    index = None

                elif (now - self.target_gain_time > self.block_stall_timeout
                        or now - self.target_since > self.block_timeout):
                    # Bounded on purpose - see `block_stall_timeout`. Reported
                    # honestly as left below threshold, not as done.
                    self.get_logger().warn(
                        f'Leaving block {self.target_block} at '
                        f'{100.0 * fraction:.0f}% (wanted '
                        f'{100.0 * self.clean_threshold:.0f}%) - it stopped '
                        f'improving after '
                        f'{now - self.target_since:.0f} s'
                    )
                    self.dead_blocks.add(self.target_block)
                    self.target_block = None
                    index = None

        if index is None:
            index = self.pick_block(state, flat, dist, reachable)

            if index is None:
                self.complete_cleaning(msg, traversable)
                return

            self.target_block = state.block_xy(index)
            self.target_tries = 0
            self.target_since = now
            self.target_gain_time = now
            self.target_best = state.fraction(index)

            centre_x, centre_y = state.centre(index, self.block_size)
            self.get_logger().info(
                f'Block {self.target_block} at ({centre_x:+.2f}, '
                f'{centre_y:+.2f}) is {100.0 * state.fraction(index):.0f}% '
                f'done - driving in'
            )

        goal = self.next_cell_in_block(state, flat, dist, reachable, index)

        if goal is None:
            # Still red, but nothing left in it the robot can reach - the rest
            # of the square is behind something. Give it up rather than
            # circling it forever.
            self.retire_block(state, index, 'no reachable floor left in it')
            return

        self.path = self.build_path(msg, dist, start, goal, w)
        self.publish_path(msg)

        remaining = int(self.red_blocks(state).size)
        self.clean_coverage = self.coverage_percent(msg, traversable)
        self.get_logger().info(
            f'Cleaning block {self.target_block}: '
            f'{100.0 * state.fraction(index):.0f}% of it done, '
            f'{remaining} block(s) still red, '
            f'{self.clean_coverage:.0f}% of the floor swept',
            throttle_duration_sec=2.0,
        )

    def pick_block(self, state, flat, dist, reachable):
        """
        Nearest red block, measured along the floor rather than through walls.

        Distance is the BFS flood, so the square just the other side of a wall
        is correctly far away and the robot works its way round instead of
        driving at it and wedging in the corner.
        """
        red = self.red_blocks(state)

        if red.size == 0:
            return None

        wanted = np.isin(state.flat_block, red)
        candidate = wanted & reachable & ~state.swept

        if not candidate.any():
            return None

        # Nearest unswept cell in any red block; its block becomes the target.
        far = np.iinfo(np.int32).max
        costs = np.where(candidate, dist[flat], far)

        return int(state.flat_block[int(np.argmin(costs))])

    def next_cell_in_block(self, state, flat, dist, reachable, index):
        """
        Aim at the middle of what is left unswept in this square.

        The MIDDLE, not the nearest cell. Driving at the nearest leftover cell
        aims at the edge of the unswept patch, and the robot stops
        `goal_tolerance` short of it while marking only `cleaning_radius`
        around itself - so the cell it drove to does not get swept, it is still
        the nearest one next time round, and the robot goes back for it
        forever. That is what cost 94 s on one square.

        Aiming at the centroid drives the robot THROUGH the patch instead, so
        its swath does the work on the way past.
        """
        mine = (state.flat_block == index) & reachable & ~state.swept

        if not mine.any():
            return None

        rows = state.rows[mine]
        cols = state.cols[mine]

        middle_row = rows.mean()
        middle_col = cols.mean()

        # Nearest unswept cell to that centre - the centroid itself can land on
        # a cell that is already swept, or outside the block entirely when the
        # leftovers straddle an obstacle.
        offsets = (rows - middle_row) ** 2 + (cols - middle_col) ** 2

        return int(flat[mine][int(np.argmin(offsets))])

    def retire_block(self, state, index, why):
        """Give up on a square, once, and say so."""
        block = state.block_xy(index)

        self.dead_blocks.add(block)
        self.target_block = None
        self.target_tries = 0
        self.path = []

        self.get_logger().warn(
            f'Skipping block {block} ({100.0 * state.fraction(index):.0f}% '
            f'done) - {why}'
        )

    def publish_blocks(self):
        """
        Show which squares of floor are done and which still need cleaning.

        A square only turns green once the robot has actually covered
        `clean_threshold` of the drivable floor inside it. Clipping one corner
        of a square is not cleaning it, so a simple "has the robot ever been
        in this square" test would lie.
        """
        if self.map_msg is None:
            return

        msg = self.map_msg
        height = msg.info.height
        width = msg.info.width
        res = msg.info.resolution

        grid = np.array(msg.data, dtype=np.int16).reshape(height, width)
        free = (grid >= 0) & (grid <= FREE_MAX)
        occupied = grid >= OCCUPIED_MIN

        inflate = max(1, int(math.ceil(self.plan_radius / res)))
        drivable = free & ~self.dilate(occupied, inflate)

        # The SAME figures the planner drives on, so a square cannot be red on
        # the screen while the robot has already written it off, or green while
        # the robot is still working it.
        state = self.block_state(msg, drivable)

        if state is None:
            return

        total = state.total
        done = state.done

        # The counts are kept whether or not anyone is watching. They used to
        # fall out of building the markers, so with no RViz attached the
        # status read blocks=0/0 for the entire run.
        live = np.nonzero(total)[0]
        self.blocks_total = int(live.size)
        self.blocks_done = int(np.count_nonzero(
            done[live] / total[live] >= self.clean_threshold))

        if self.blocks_pub.get_subscription_count() == 0:
            return

        markers = MarkerArray()
        now = self.get_clock().now().to_msg()

        for index in live:
            fraction = done[index] / total[index]
            gx, gy = state.block_xy(int(index))

            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = now
            marker.ns = 'coverage'
            # Stable id per block, so RViz updates squares in place instead of
            # flickering them away and back.
            marker.id = int((gx + 2000) * 4000 + (gy + 2000))
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            marker.pose.position.x = (gx + 0.5) * self.block_size
            marker.pose.position.y = (gy + 0.5) * self.block_size
            marker.pose.position.z = 0.01
            marker.pose.orientation.w = 1.0

            marker.scale.x = self.block_size * 0.95
            marker.scale.y = self.block_size * 0.95
            marker.scale.z = 0.01

            if fraction >= self.clean_threshold:
                marker.color.r, marker.color.g, marker.color.b = 0.0, 0.8, 0.2
                marker.color.a = 0.35
            elif (gx, gy) in self.dead_blocks:
                # Grey: still not swept, but the robot cannot get at it and
                # has stopped trying. Without this it looks identical to a
                # square the robot is about to do, and the run appears stuck.
                marker.color.r, marker.color.g, marker.color.b = 0.45, 0.45, 0.5
                marker.color.a = 0.25
            elif (gx, gy) == self.target_block:
                # Blue: the square being worked right now.
                marker.color.r, marker.color.g, marker.color.b = 0.1, 0.4, 1.0
                marker.color.a = 0.55
            else:
                # Redder the less of it has been done.
                marker.color.r = 0.9
                marker.color.g = 0.15 + 0.6 * float(fraction)
                marker.color.b = 0.1
                marker.color.a = 0.30

            markers.markers.append(marker)

        self.blocks_pub.publish(markers)

    def escape_heading(self):
        """
        Direction in the robot frame with the most room, or None if boxed in.

        The scan is binned into sectors and each sector scored by its nearest
        return, so the robot backs out towards genuinely open space instead of
        guessing left or right.
        """
        if not self.scan_points:
            return None

        sectors = 16
        span = 2.0 * math.pi / sectors
        nearest = [0.0] * sectors

        for x, y, distance in self.scan_points:
            index = int((math.atan2(y, x) + math.pi) / span) % sectors

            if nearest[index] == 0.0:
                nearest[index] = distance
            else:
                nearest[index] = min(nearest[index], distance)

        # A sector with no returns at all scores 0, not infinity. Indoors that
        # almost always means the obstacle is inside the lidar's 0.30 m blind
        # spot rather than that the way is clear - treating it as wide open is
        # how a wedged robot drives itself further into a wall.
        best = max(range(sectors), key=lambda i: nearest[i])

        if nearest[best] <= self.turn_radius:
            return None

        return -math.pi + (best + 0.5) * span

    def note_progress(self, x, y):
        """Reset the wedge timer whenever the robot actually moves."""
        if self.progress_pose is None:
            self.progress_pose = (x, y)
            self.progress_time = time.monotonic()
            return

        if math.dist((x, y), self.progress_pose) >= self.stuck_distance:
            self.progress_pose = (x, y)
            self.progress_time = time.monotonic()

    def is_stuck(self):
        return time.monotonic() - self.progress_time > self.stuck_timeout

    def begin_recovery(self, x, y):
        """Start an escape manoeuvre, or give up on this goal if we keep failing."""
        self.recovery_attempts += 1
        self.progress_pose = (x, y)
        self.progress_time = time.monotonic()

        if self.recovery_attempts > self.max_recovery_attempts:
            # Repeated escapes have not helped: this goal is not worth more
            # time. Drop it so the mission always moves on.
            self.abandon_goal()
            self.recovery_attempts = 0
            return

        # Stop first; drive_recovery runs the rest of the sequence.
        self.recovery_started = time.monotonic()
        self.recovery_target = None
        self.recovery_turned = False
        self.enter_recovery_phase('stop', self.recovery_settle)
        self.get_logger().warn(
            f'Stuck - recovery attempt {self.recovery_attempts}'
            f'/{self.max_recovery_attempts}'
        )

    def abandon_goal(self):
        """Give up on the current target and move to the next one."""
        if self.mode == 'clean' and self.chasing_blocks:
            # Repeated escapes have not got the robot into this square. Count
            # the attempt, and drop the square once it has had its go - one
            # unreachable corner must not hold up the whole run.
            if self.target_block is not None:
                self.target_tries += 1

                if self.target_tries >= self.block_attempts:
                    self.get_logger().warn(
                        f'Skipping block {self.target_block} - '
                        f'{self.target_tries} attempts and still cannot get '
                        f'into it'
                    )
                    self.dead_blocks.add(self.target_block)
                    self.target_block = None
                    self.target_tries = 0

            self.path = []
            self.recovery_until = 0.0
            self.recovery_phase = None
            return

        if self.mode == 'clean':
            self.get_logger().warn(
                f'Skipping cleaning waypoint {self.goal_index + 1} - cannot reach it'
            )
            self.goal_index += 1
        else:
            # Remember the spot so the frontier picker stops offering it.
            if self.path:
                gx, gy = self.path[-1]
                if self.map_msg is not None:
                    self.blacklist.add(self.cover_key(self.map_msg, gx, gy))
            self.get_logger().warn('Abandoning unreachable frontier')

        self.path = []
        self.recovery_until = 0.0
        self.recovery_phase = None

    def back_out(self, cmd, speed=0.5):
        """
        Reverse, curving away from whatever is closest.

        Committed to for `backout_time`, because the alternative is deciding
        again on the very next tick - by which point reversing has cleared the
        way ahead, so the robot drives forward, blocks, and reverses again.
        That loop is the forward-and-backward shuffle: 179 direction changes
        in 40 s with the heading never moving.

        The curve is what makes the shuffle productive. Each back-out leaves
        the robot pointing a few degrees off where it was, so the next attempt
        at driving forward is a different attempt, not the same one repeated.
        """
        cmd.linear.x = -speed * self.linear_speed
        cmd.angular.z = self.avoid_steer()

        if time.monotonic() >= self.backout_until:
            self.backout_until = time.monotonic() + self.backout_time
            self.backout_started = time.monotonic()

    def backing_out(self):
        """Report whether a committed back-out is still running."""
        return time.monotonic() < self.backout_until

    def drive_recovery_fallback(self, cmd):
        """
        LAST RESORT: back out towards open space; spin only with room to spin.

        Only reached when the stop / aim / go sequence has nothing clear to
        point at and cannot rotate to look for one (see `drive_recovery`).
        Unchanged from when this was the whole recovery.

        The room-to-spin check is the whole point. `publish_safe` refuses any
        rotation while something sits inside the swept circle, which is
        exactly the situation recovery exists to get out of - so a recovery
        that answers with a pure spin gets that spin zeroed, has no linear
        component to fall back on, and publishes nothing at all. The robot
        then sits still, never moves, never resets the progress timer, and
        recovers forever against the same obstacle. Measured before this: four
        of five close-quarters situations ended in a dead stop.

        Reversing is the escape that always works, because `publish_safe`
        deliberately leaves reverse alone. Backing up is also what *creates*
        the room to turn, so it is the right first move even when the way out
        is sideways.
        """
        heading = self.escape_heading()
        behind = self.sector_is_clear(math.pi)
        can_turn = self.can_turn()

        if heading is None:
            # Every direction reads blocked. The lidar cannot see inside its
            # own minimum range, so nudge back rather than sit here forever -
            # motion produces new readings to act on. The turn is only added
            # if it would survive publish_safe; otherwise it is a straight
            # reverse rather than a command that gets half thrown away.
            cmd.linear.x = -0.4 * self.linear_speed
            cmd.angular.z = 0.3 * self.angular_speed if can_turn else 0.0
            return

        if abs(heading) > math.radians(120) and behind:
            # Open space is behind us and the way back is clear: reverse.
            cmd.linear.x = -0.6 * self.linear_speed
            cmd.angular.z = 0.0
            return

        if not can_turn:
            # The way out is sideways but there is no room to turn towards it.
            # Back up to make room, curving away as we go.
            self.back_out(cmd)
            return

        cmd.linear.x = 0.0
        cmd.angular.z = self.angular_speed if heading > 0 else -self.angular_speed

    # The stop / aim / go recovery
    # ------------------------------------------------------------------

    def enter_recovery_phase(self, phase, duration):
        """Start a recovery step that times out after `duration` seconds."""
        now = time.monotonic()
        self.recovery_phase = phase
        self.recovery_phase_until = now + duration
        # `recovery_until` is what the rest of the node reads as "a recovery
        # owns the wheels" (follow_path, the stall watch, the status). The
        # small tail makes sure drive_recovery is ticked once more after the
        # step expires, so the hand-over to the next step is not left to
        # chance; a step that is never ticked just lapses, as it always did.
        self.recovery_until = self.recovery_phase_until + 0.15

    def end_recovery(self):
        """Recovery is over: the normal follower has the wheels again."""
        self.recovery_phase = None
        self.recovery_until = 0.0

    def heading_clearance(self, heading):
        """
        Gap ahead of the front edge if the robot were pointing along `heading`.

        `forward_clearance` for any bearing in the robot frame: the same
        corridor (the robot's actual half width), the same `x > half_length`
        cut-off, so a wall alongside the heading never counts as blocking it.
        """
        cos_h = math.cos(heading)
        sin_h = math.sin(heading)
        best = float('inf')

        for x, y, _ in self.scan_points:
            along = x * cos_h + y * sin_h
            across = -x * sin_h + y * cos_h

            if along > self.half_length and abs(across) <= self.half_width:
                best = min(best, along - self.half_length)

        return best

    def way_is_clear(self, clearance):
        """Whether driving forward would be allowed, and worth it, now."""
        if clearance <= self.stop_distance:
            return False

        # Pinned by something the lidar cannot see (publish_safe refuses
        # forward while the IMU says so): "clear" would be a lie.
        return not (self.use_imu_stall and self.watch.stalled)

    def is_clear_ahead(self, clearance):
        """
        Clear enough to commit to driving: beyond `slow_distance`.

        `slow_distance` is where the follower itself runs at full speed, so a
        gap wider than it is one the robot would drive at without hesitating.
        Between `stop_distance` and `slow_distance` the follower creeps; a
        recovery that has just failed to get through should not start there.
        """
        return self.way_is_clear(clearance) and clearance > self.slow_distance

    def aim_error(self, yaw):
        """Signed shortest rotation from the robot's yaw to the target."""
        delta = self.recovery_target - yaw
        return math.atan2(math.sin(delta), math.cos(delta))

    def start_rotation(self, phase, target):
        """Begin an aim/turn step towards the world heading `target`."""
        remaining = self.recovery_started + self.recovery_budget - time.monotonic()
        self.recovery_target = target
        self.enter_recovery_phase(phase, min(self.aim_timeout, max(0.5, remaining)))

    def start_go(self):
        self.enter_recovery_phase('go', self.recovery_go_time)

    def start_fallback(self):
        self.get_logger().warn(
            'Recovery: nowhere clear to point and no room to turn - '
            'falling back to the bounded back-out',
            throttle_duration_sec=2.0,
        )
        self.enter_recovery_phase('fallback', self.recovery_time)

    def choose_after_aim(self, yaw):
        """
        Aim is done (or was not possible): go if clear, else turn, else back out.

        "In sight" is the way ahead being clear for the robot's body, not a
        line of sight to the goal.
        """
        if self.is_clear_ahead(self.forward_clearance()):
            self.start_go()
            return

        heading = self.escape_heading()

        if (heading is None or not self.can_turn()
                or not self.way_is_clear(self.heading_clearance(heading))):
            # No heading is clear, or there is no room to rotate to one.
            self.start_fallback()
            return

        self.recovery_turned = True
        self.start_rotation('turn', yaw + heading)

    def drive_recovery(self, cmd):
        """
        Primary recovery: STOP, AIM at the goal, GO if clear, else TURN, GO.

        1. STOP    zero Twist for `recovery_settle`.
        2. AIM     spin in place towards the next waypoint at `angular_speed`
                   until within `aim_tolerance` (or `aim_timeout`). Skipped
                   when already aimed, when there is no path, when the goal
                   heading is already known blocked (no point spinning to
                   face a wall; the lidar sees all 360 deg) or when the swept
                   circle is not clear - `can_turn()` is asked every tick.
        3. GO      if the way ahead is clear (beyond `slow_distance` after
                   aiming at the goal), drive forward `recovery_go_time`.
        4. TURN    otherwise rotate to `escape_heading()` and GO from there.
        5. LAST RESORT, only when no heading is clear and rotation is
                   impossible: `drive_recovery_fallback`, the bounded
                   back-out this used to be.

        This chooses WHICH manoeuvre. Every command still goes through
        `publish_safe`, which owns what is allowed.
        """
        phase = self.recovery_phase

        if phase is None:
            # No sequence running (one tick after the goal was abandoned):
            # the old behaviour, as before.
            self.drive_recovery_fallback(cmd)
            return

        pose = self.last_pose

        if pose is None:
            return

        yaw = pose[2]

        # Steps hand over within the same tick (stop -> aim -> turn -> go is
        # four moves at most), so a step ending costs no extra 100 ms.
        for _ in range(4):
            phase = self.recovery_phase
            expired = time.monotonic() >= self.recovery_phase_until

            if phase == 'stop':
                if not expired:
                    return

                target = None

                if self.path:
                    target = math.atan2(self.path[0][1] - pose[1],
                                        self.path[0][0] - pose[0])

                if target is not None and self.can_turn():
                    self.recovery_target = target
                    error = abs(self.aim_error(yaw))
                    blocked = not self.is_clear_ahead(
                        self.heading_clearance(target - yaw))

                    if error > self.aim_tolerance and not blocked:
                        self.start_rotation('aim', target)
                        continue

                self.choose_after_aim(yaw)
                continue

            if phase in ('aim', 'turn'):
                error = self.aim_error(yaw)

                if (abs(error) <= self.aim_tolerance or expired
                        or not self.can_turn()):
                    if phase == 'aim':
                        self.choose_after_aim(yaw)
                    elif self.way_is_clear(self.forward_clearance()):
                        self.start_go()
                    else:
                        self.start_fallback()
                    continue

                cmd.linear.x = 0.0
                cmd.angular.z = self.angular_speed if error > 0 else -self.angular_speed
                return

            if phase == 'go':
                forward = self.forward_clearance()

                if expired or forward <= self.stop_distance:
                    self.end_recovery()
                    return

                span = max(1e-3, self.slow_distance - self.stop_distance)
                scale = max(0.15, min(1.0, (forward - self.stop_distance) / span))
                cmd.linear.x = self.linear_speed * scale
                cmd.angular.z = 0.0
                return

            if phase == 'fallback':
                if expired:
                    self.end_recovery()
                    return

                self.drive_recovery_fallback(cmd)
                return

            return

    def sector_is_clear(self, heading, half_width=math.radians(35)):
        """Report whether the turning circle around `heading` is clear."""
        for x, y, distance in self.scan_points:
            angle = math.atan2(y, x)
            delta = abs(math.atan2(math.sin(angle - heading), math.cos(angle - heading)))

            if delta <= half_width and distance <= self.turn_radius:
                return False

        return True

    def side_clearance(self, sign):
        """Nearest return on one side; sign +1 is left, -1 is right."""
        best = float('inf')

        for _, y, distance in self.scan_points:
            if sign * y > 0.0:
                best = min(best, distance)

        return best

    def pose_is_stale(self):
        """Whether the last fresh pose is older than `pose_timeout`."""
        if self.pose_time is None:
            return True

        return time.monotonic() - self.pose_time > self.pose_timeout

    def localization_status(self):
        """
        Whether the robot is genuinely localized, and why not if it isn't.

        Two conditions, both required:

        * **map -> base_footprint resolves.** Without it there is no pose in
          the map frame at all and every plan is fiction.
        * **and its stamp is fresh.** This is the one a `can_transform` check
          would miss. A lookup at `Time()` asks for the NEWEST transform the
          buffer holds, and the buffer keeps handing that back long after the
          publisher died - slam_toolbox crashing, or odom drying up, reads
          exactly like a healthy robot standing still. So the transform is
          aged by its own header stamp against `pose_timeout`, which is the
          same test `robot_pose` applies once driving.
        * **a /map has been received.** `replan` cannot plan without one, and
          a map -> odom TF can exist a beat before the first grid is out.

        Returns (ready, reason). `reason` is operator-facing: it says what is
        missing, not that something is.
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time())
        except tf2_ros.TransformException:
            return False, (f'waiting for {self.map_frame} -> {self.base_frame} '
                           f'(slam_toolbox has not matched a scan yet)')

        age = (self.get_clock().now()
               - Time.from_msg(tf.header.stamp)).nanoseconds * 1e-9
        if age > self.pose_timeout:
            return False, (f'{self.map_frame} -> {self.base_frame} is {age:.1f} s '
                           f'old (limit {self.pose_timeout:.1f} s) - that is a '
                           f'stale cached transform, not a live pose')

        if self.map_msg is None:
            return False, 'waiting for /map (TF is live but no grid published yet)'

        return True, (f'{self.map_frame} -> {self.base_frame} live '
                      f'({age:.2f} s old), /map '
                      f'{self.map_msg.info.width}x{self.map_msg.info.height}')

    def gate_localization(self):
        """
        Hold station until localized; True once the robot may drive.

        Publishes a zero Twist on every tick it holds - a silent cmd_vel is
        indistinguishable from a crashed node, and the base would simply drop
        its wheels either way. Logs the reason it is waiting (throttled) so
        the operator can see WHICH condition is missing.

        On timeout the mission is failed rather than started blind: the mode
        goes to 'stopped' with the diagnosis, which the web page already
        shows, and another `explore` re-arms the gate for a fresh attempt.
        """
        if self.localized:
            return True

        ready, reason = self.localization_status()
        now = time.monotonic()
        waited = now - self.localize_started

        if ready:
            self.localized = True
            # The wait is not "no progress": every mission watchdog runs on
            # wall clock, so without this `is_stuck` fires on the very first
            # driving tick after a long localization.
            self.unpause_clocks(waited)
            self.get_logger().info(
                f'Localized after {waited:.1f} s - {reason}; starting to drive')
            self.publish_mode()
            return True

        self.cmd_pub.publish(Twist())

        if waited > self.localize_timeout:
            self.mode = 'stopped'
            self.get_logger().error(
                f'NOT LOCALIZED after {waited:.0f} s - {reason}. Not driving '
                f'blind. Check slam_toolbox is running and that bringup is '
                f'publishing /scan and odom -> base_footprint, then send '
                f'"explore" again.')
            self.publish_mode()
            return False

        self.get_logger().info(
            f'Localizing ({waited:.0f}/{self.localize_timeout:.0f} s): {reason}',
            throttle_duration_sec=2.0)
        return False

    def robot_pose(self):
        """Robot (x, y, yaw) in the map frame, or None if TF isn't ready."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time())
        except tf2_ros.TransformException:
            # Lookups drop out for a cycle now and then; reusing the last
            # pose keeps driving smooth instead of stuttering to a stop.
            # But not forever: past `pose_timeout` the last pose is a guess,
            # and a guess is not something to drive on.
            if self.pose_is_stale():
                self.get_logger().warn(
                    f'No {self.map_frame}->{self.base_frame} transform for '
                    f'{self.pose_timeout:.1f} s - holding',
                    throttle_duration_sec=2.0,
                )
                return None

            return self.last_pose

        t = tf.transform.translation
        q = tf.transform.rotation

        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        self.last_pose = (t.x, t.y, yaw)

        # A lookup at Time() hands back the NEWEST transform however old it
        # is, so a micro-ROS agent that stops publishing odom->base never
        # raises above - it just keeps returning the pose from before it
        # died. So the pose is aged by its own stamp, not by when it was
        # fetched: `pose_time` is the wall-clock moment the data was true.
        stamp_age = (self.get_clock().now() - Time.from_msg(tf.header.stamp)).nanoseconds * 1e-9
        self.pose_time = time.monotonic() - max(0.0, stamp_age)

        if self.pose_is_stale():
            self.get_logger().warn(
                f'{self.map_frame}->{self.base_frame} transform is '
                f'{stamp_age:.1f} s old - holding',
                throttle_duration_sec=2.0,
            )
            return None

        return self.last_pose

    # ------------------------------------------------------------------
    # Grid helpers
    # ------------------------------------------------------------------

    def world_to_cell(self, msg, x, y):
        col = int((x - msg.info.origin.position.x) / msg.info.resolution)
        row = int((y - msg.info.origin.position.y) / msg.info.resolution)
        return row, col

    def cell_to_world(self, msg, row, col):
        x = msg.info.origin.position.x + (col + 0.5) * msg.info.resolution
        y = msg.info.origin.position.y + (row + 0.5) * msg.info.resolution
        return x, y

    @staticmethod
    def dilate(mask, radius_cells):
        """
        Grow `mask` by a true circle of `radius_cells`.

        Repeated 4-connected passes would give a diamond, which reaches only
        ~0.71 of the requested radius on the diagonals - that under-inflation
        is exactly what lets a robot clip wall corners.
        """
        out = mask.copy()
        r = int(radius_cells)

        for dr in range(-r, r + 1):
            for dc in range(-r, r + 1):
                if dr * dr + dc * dc > r * r:
                    continue

                if dr == 0 and dc == 0:
                    continue

                # Shift `mask` by (dr, dc) and OR it in.
                src_r = slice(max(0, -dr), mask.shape[0] - max(0, dr))
                dst_r = slice(max(0, dr), mask.shape[0] - max(0, -dr))
                src_c = slice(max(0, -dc), mask.shape[1] - max(0, dc))
                dst_c = slice(max(0, dc), mask.shape[1] - max(0, -dc))

                out[dst_r, dst_c] |= mask[src_r, src_c]

        return out

    # ------------------------------------------------------------------
    # Planning
    # ------------------------------------------------------------------

    def replan(self):
        if self.paused or self.mode in ('done', 'stopped') or self.map_msg is None:
            return

        if self.handoff is not None:
            return          # saving the map; service_handoff owns this phase

        if not self.localized:
            return          # follow_path owns the gate (and the zero Twist)

        pose = self.robot_pose()
        if pose is None:
            self.get_logger().warn('Waiting for map->base TF...', once=True)
            return

        msg = self.map_msg
        h = msg.info.height
        w = msg.info.width
        grid = np.array(msg.data, dtype=np.int16).reshape(h, w)

        free = (grid >= 0) & (grid <= FREE_MAX)
        occupied = grid >= OCCUPIED_MIN
        unknown = grid == UNKNOWN

        # Inflate by the driving half-width, not the turning circle: the wider
        # figure would fence off a whole robot-radius strip along every wall
        # and leave it permanently uncleaned. Rotation clearance is enforced
        # live from the lidar instead, with recovery if a spin is blocked.
        inflate_cells = max(1, int(math.ceil(self.plan_radius / msg.info.resolution)))
        blocked = self.dilate(occupied, inflate_cells)
        traversable = free & ~blocked

        # Frontier = free cell touching unknown space
        frontier = np.zeros_like(free)
        frontier[1:, :] |= unknown[:-1, :]
        frontier[:-1, :] |= unknown[1:, :]
        frontier[:, 1:] |= unknown[:, :-1]
        frontier[:, :-1] |= unknown[:, 1:]
        frontier &= traversable

        start = self.start_cell(pose, msg, traversable, h, w)
        if start is None:
            self.get_logger().warn('Robot is not on traversable space; rotating')
            self.path = []
            return

        dist = self.wavefront(traversable, start)
        self.plan_cache = {'msg': msg, 'traversable': traversable, 'dist': dist}

        if self.mode == 'clean':
            # Chasing the squares directly - either because that is the mode,
            # or because the zig-zag has run out and the squares are what is
            # left to finish.
            if self.chasing_blocks:
                self.advance_blocks(msg, traversable, dist, start, h, w)
                return

            if not self.clean_plan_ready:
                # Started straight in cleaning mode against a map loaded from
                # disk: lay out the sweep the first time a map arrives.
                self.clean_plan_ready = True
                self.start_cleaning(msg, traversable, h, w)
                return

            self.advance_cleaning(msg, traversable, dist, start, h, w)
            return

        min_cells = max(2, int(round(self.min_goal_distance / msg.info.resolution)))
        goal = self.pick_frontier(frontier, dist, h, w, min_cells)

        if goal == 'too_close':
            # Unknown space is right up against the robot but there is nothing
            # far enough away to drive to. Clearing the path hands over to the
            # open-space drive, which moves and so reveals more map. Spinning
            # would not: a 360 deg lidar already sees everything from here.
            self.near_only_count += 1
            self.path = []

            if self.near_only_count >= 15:
                self.get_logger().info('Only unreachable scraps of frontier left')
                self.start_cleaning(msg, traversable, h, w)
            return

        self.near_only_count = 0

        if goal is None:
            self.no_frontier_count += 1

            # A couple of empty passes in a row means the area is mapped;
            # a single one can just be a transient map update.
            if self.no_frontier_count >= 3:
                self.start_cleaning(msg, traversable, h, w)
            else:
                self.get_logger().info('No reachable frontier this pass')
            return

        self.no_frontier_count = 0
        self.handoff_finished = False     # more to explore: save again at the end
        self.path = self.build_path(msg, dist, start, goal, w)
        self.publish_path(msg)

        explored = int(np.count_nonzero(free))
        self.get_logger().info(
            f'Frontier at {dist[goal] * msg.info.resolution:.1f} m '
            f'({len(self.path)} pts), {explored} cells mapped'
        )

    def start_cell(self, pose, msg, traversable, h, w):
        """Robot's cell, or the closest traversable cell if it sits in inflation."""
        row, col = self.world_to_cell(msg, pose[0], pose[1])

        if not (0 <= row < h and 0 <= col < w):
            return None

        if traversable[row, col]:
            return row * w + col

        for radius in range(1, 12):
            for dr in range(-radius, radius + 1):
                for dc in range(-radius, radius + 1):
                    r, c = row + dr, col + dc
                    if 0 <= r < h and 0 <= c < w and traversable[r, c]:
                        return r * w + c

        return None

    def request_replan(self):
        """Replan as soon as the current callback returns, not inside it."""
        self.replan_kick.reset()

    def kicked_replan(self):
        """The one-shot armed by `request_replan`."""
        self.replan_kick.cancel()
        self.replan()

    @staticmethod
    def wavefront(traversable, start, stop=None):
        """
        Breadth-first distance in grid steps from flat cell `start`.

        Returns a flat int32 array, -1 wherever `traversable` cannot be
        reached. The same numbers a FIFO BFS gives, but a whole ring of cells
        per numpy pass instead of a Python step per cell: the per-cell loop
        cost 223 ms a replan on the Pi 5 against the real map, and ran inside
        the 10 Hz drive tick on every arrival.

        With `stop`, it gives up as soon as that cell has its distance. Every
        cell nearer than it is labelled by then, which is all `descend` needs
        to walk back from it.
        """
        h, w = traversable.shape
        width = w + 2

        # One cell of padding all round, so a neighbour is just +-1 / +-width
        # and never needs a bounds check.
        open_ = np.zeros((h + 2, width), dtype=bool)
        open_[1:-1, 1:-1] = traversable
        open_ = open_.reshape(-1)
        dist = np.full(open_.size, -1, dtype=np.int32)

        row, col = divmod(int(start), w)
        front = np.array([(row + 1) * width + col + 1], dtype=np.int64)
        target = (None if stop is None else
                  (int(stop) // w + 1) * width + int(stop) % w + 1)

        dist[front] = 0
        open_[front] = False
        steps = np.array([width, -width, 1, -1], dtype=np.int64)
        owner = np.empty(open_.size, dtype=np.int64)
        layer = 0

        while front.size:
            if target is not None and dist[target] >= 0:
                break

            layer += 1
            ahead = (front[:, None] + steps).reshape(-1)
            ahead = ahead[open_[ahead]]

            # Drop duplicates without a sort: each cell keeps whichever
            # position wrote to it last.
            order = np.arange(ahead.size)
            owner[ahead] = order
            front = ahead[owner[ahead] == order]

            dist[front] = layer
            open_[front] = False

        return dist.reshape(h + 2, width)[1:-1, 1:-1].reshape(-1)

    @staticmethod
    def descend(dist, start, goal, w):
        """
        Flat cells from `start` (exclusive) to `goal` (inclusive), downhill.

        Each step goes to a 4-neighbour exactly one closer, columns tried
        before rows. In open floor that is the same L-shaped route the old
        FIFO BFS parents gave; round obstacles it can pick a different, but
        equally short, route.
        """
        height = dist.size // w
        cells = []
        cur = int(goal)

        while cur != start:
            cells.append(cur)
            d = int(dist[cur]) - 1

            if d < 0:
                break

            row, col = divmod(cur, w)

            if col > 0 and dist[cur - 1] == d:
                cur -= 1
            elif col < w - 1 and dist[cur + 1] == d:
                cur += 1
            elif row > 0 and dist[cur - w] == d:
                cur -= w
            elif row < height - 1 and dist[cur + w] == d:
                cur += w
            else:
                break

        cells.reverse()
        return cells

    def pick_frontier(self, frontier, dist, h, w, min_cells):
        """
        Nearest reachable frontier worth driving to.

        Anything closer than `min_cells` is skipped: the robot is already
        standing on it, so the path would be empty and it would just sit
        there. A 360 deg lidar sees everything from where it is, so spinning
        adds nothing and the map would never change - a permanent deadlock.
        The closest such frontier is kept only as a last resort.
        """
        flat = frontier.reshape(-1)
        reachable = flat & (dist >= 0)

        candidates = np.flatnonzero(reachable)
        if candidates.size == 0:
            return None

        # Group frontier cells so we ignore specks of noise on the map edge.
        seen = set()
        fallback = None

        for idx in candidates[np.argsort(dist[candidates])]:
            if idx in seen:
                continue

            # Frontiers we already failed to reach are not worth retrying.
            if self.blacklist:
                row, col = divmod(int(idx), w)
                point = self.cell_to_world(self.map_msg, row, col)
                if self.cover_key(self.map_msg, point[0], point[1]) in self.blacklist:
                    continue

            cluster = self.cluster(idx, reachable, seen, h, w)

            if len(cluster) < self.min_frontier_size:
                continue

            # Sorted by distance, so the first one far enough away is nearest.
            if dist[idx] >= min_cells:
                return int(idx)

            # Too close to drive to. Remember that one exists, but never
            # return it: a goal inside goal_tolerance counts as reached the
            # instant it is picked, so the robot would clear it, re-pick it
            # and never actually move.
            fallback = int(idx)

        if fallback is not None:
            return 'too_close'

        return None

    # ------------------------------------------------------------------
    # Cleaning (boustrophedon coverage)
    # ------------------------------------------------------------------

    def start_cleaning(self, msg, traversable, h, w):
        """
        Switch from exploring to sweeping the floor.

        SLAM keeps running throughout, so driving the rows also re-observes
        everything from new angles and lets the map tighten up as it goes.
        """
        was_exploring = self.mode == 'explore'

        if was_exploring and not self.handoff_finished:
            # Exploration is done. Stop, capture pose, save the map - and come
            # back here (handoff_finished) once that has answered. replan()
            # is parked until then, so the next call is the real switch.
            self.begin_handoff()
            return

        self.mode = 'clean'
        self.handoff_finished = False

        if self.chasing_blocks:
            self.get_logger().info('=' * 46)

            if was_exploring:
                self.get_logger().info('EXPLORATION COMPLETE - no frontiers left')
            else:
                self.get_logger().info('CLEANING FROM SAVED MAP')

            self.get_logger().info(
                f'Cleaning by coverage blocks: {self.block_size:.2f} m squares, '
                f'each done at {100.0 * self.clean_threshold:.0f}%. The robot '
                f'goes to the nearest red square and stays in it until it is '
                f'green'
            )
            self.get_logger().info('=' * 46)
            return

        self.clean_plan_ready = True
        self.cleaning_pass = 1
        self.goal_index = 0
        pose = self.robot_pose()
        self.cleaning_goals = self.build_cleaning_plan(
            msg, traversable, h, w, pose=pose)

        self.get_logger().info('=' * 46)

        if was_exploring:
            self.get_logger().info('EXPLORATION COMPLETE - no frontiers left')
        else:
            self.get_logger().info('CLEANING FROM SAVED MAP')
        self.get_logger().info(
            f'Starting cleaning sweep: {self.clean_rows} rows, '
            f'{len(self.cleaning_goals)} waypoints, '
            f'{self.row_spacing:.2f} m apart along {self.clean_axis} against '
            f'a {2.0 * self.cleaning_radius:.2f} m swath, '
            f'{self.clean_regions} region(s) split by obstacles'
        )
        self.get_logger().info(
            f'sweep_start={self.sweep_start}: rows run along '
            f'{self.clean_axis}, working from the '
            f'{self.clean_side} towards the other side'
        )

        if self.clean_corner is not None:
            corner_x, corner_y = self.clean_corner

            if pose is None:
                self.get_logger().info(
                    f'Starting at corner ({corner_x:.2f}, {corner_y:.2f}) - '
                    'no pose available, corner not chosen by distance'
                )
            else:
                self.get_logger().info(
                    f'Nearest corner ({corner_x:.2f}, {corner_y:.2f}) is '
                    f'{math.hypot(corner_x - pose[0], corner_y - pose[1]):.2f} m '
                    f'from the robot at ({pose[0]:.2f}, {pose[1]:.2f}, '
                    f'{math.degrees(pose[2]):.0f} deg)'
                )
        self.get_logger().info('=' * 46)

        if not self.cleaning_goals:
            self.complete_cleaning(msg, traversable)

    # ------------------------------------------------------------------
    # Explore -> clean handoff: stop, capture pose, save map, then clean
    # ------------------------------------------------------------------

    def begin_handoff(self):
        """Phase 1: stop the wheels. The pose is taken once they have settled."""
        self.path = []
        self.cmd_pub.publish(Twist())
        self.handoff = {
            'id': uuid.uuid4().hex[:8],
            'phase': 'settle',
            'phase_start': time.monotonic(),
            'pose': None,
        }
        self.get_logger().info('=' * 46)
        self.get_logger().info(
            'EXPLORATION COMPLETE - stopping, then saving pose and map '
            'before cleaning')
        self.get_logger().info('=' * 46)

    def service_handoff(self):
        """10 Hz: drive the handoff state machine (settle -> saving -> done)."""
        h = self.handoff

        if h is None or self.paused:
            return

        now = time.monotonic()
        self.cmd_pub.publish(Twist())

        if h['phase'] == 'settle':
            if now - h['phase_start'] < self.handoff_settle:
                return

            # Phase 2: wheels have been commanded to zero for a while. THIS
            # is the pose the robot rests at, in the map frame of this SLAM.
            pose = self.robot_pose()
            payload = {'name': self.map_name, 'id': h['id']}

            if pose is None:
                self.get_logger().warn(
                    'No pose for the handoff - map_saver will read TF itself')
            else:
                h['pose'] = pose
                payload['pose'] = {'x': pose[0], 'y': pose[1], 'yaw': pose[2]}
                self.get_logger().info(
                    f'Handoff pose: x={pose[0]:.2f} y={pose[1]:.2f} '
                    f'yaw={math.degrees(pose[2]):.1f} deg')

            msg = String()
            msg.data = json.dumps(payload)
            self.save_pub.publish(msg)
            h['phase'] = 'saving'
            h['phase_start'] = now
            self.get_logger().info(
                f'Requested map save "{self.map_name or "(default)"}" on /save_map')
            return

        result = h.get('result')

        if result is None:
            if now - h['phase_start'] < self.handoff_save_timeout:
                return

            result = {'ok': False, 'id': h['id'], 'name': self.map_name,
                      'error': f'no answer from map_saver in '
                               f'{self.handoff_save_timeout:.0f} s - is it running?'}
            # Tell the web UI too; nobody else will.
            out = String()
            out.data = json.dumps(result)
            self.save_result_pub.publish(out)

        if result.get('ok'):
            self.get_logger().info(
                f'Map saved as "{result.get("name")}" '
                f'(pose {result.get("pose")}); starting to clean')
        else:
            self.get_logger().error('!' * 46)
            self.get_logger().error(
                f'MAP SAVE FAILED: {result.get("error")}. Cleaning continues '
                'on the LIVE SLAM map; nothing was saved - press Save map '
                'in the web UI.')
            self.get_logger().error('!' * 46)

        self.handoff = None
        self.handoff_finished = True

    def save_result_callback(self, msg):
        h = self.handoff

        if h is None or h['phase'] != 'saving':
            return

        try:
            result = json.loads(msg.data)
        except ValueError:
            return

        if isinstance(result, dict) and result.get('id') == h['id']:
            h['result'] = result

    def build_cleaning_plan(self, msg, traversable, h, w, pose=None):
        """
        Lay zig-zag rows over the drivable floor, starting from a corner.

        Three decisions are made here, and all three used to be accidents of
        where the map's origin happened to be rather than of where the robot
        and the furniture are. Together they are what "the navigation goes
        randomly" looked like from outside.

        1. **Which way the rows run** - along the room's longer side, so the
           robot makes as few end-of-row turns as possible. See
           `sweep_transposed`.
        2. **Which corner it starts from** - the corner of the drivable floor
           nearest the robot, by straight-line distance with yaw breaking
           ties, so the first waypoint is metres away and not the width of the
           room away. See `start_corner`.
        3. **What to do about blockers** - the floor is split into regions
           that no obstacle divides (`split_regions`), and each region is
           swept from end to end before the robot crosses to the next. A table
           in the middle of a room used to make every single row hop from one
           side of it to the other and back; now one side is finished, then
           the other.
        """
        res = msg.info.resolution

        # Rows must OVERLAP, never merely abut. Two separate things used to
        # be able to push them apart:
        #
        #  * `round` can round the spacing UP to the next whole cell. On a
        #    0.06 m grid 0.22 m rounds to 4 cells, which is 0.24 m - already
        #    wider than the 0.27 m swath allows once heading error is in it.
        #    Flooring can only ever make the rows closer together.
        #  * nothing checked the spacing against the swath at all, so raising
        #    `row_spacing` past 2 * `cleaning_radius` silently left stripes of
        #    dirt between the rows rather than being refused.
        #
        # 0.9 of the swath is the same 10% overlap the 0.22 / 0.27 defaults
        # already have, so this changes nothing unless the defaults do.
        swath = 2.0 * self.cleaning_radius
        step = max(1, int(math.floor(min(self.row_spacing, 0.9 * swath) / res)))
        min_run = max(2, int(round(self.footprint_width / res)))

        # Waypoints dropped along a row, in cells. 0 or less means endpoints
        # only - see `row_point_spacing`.
        along = (0 if self.row_point_spacing <= 0.0
                 else max(1, int(round(self.row_point_spacing / res))))

        # Only the floor the robot can actually get to. A SLAM map carries
        # free cells it can never reach - the inside of a box whose outline
        # was mapped but whose middle reads free, a band of free space outside
        # a wall - and each became a region of its own: 5 of the 29 on the
        # saved sim map. Their waypoints were skipped as unreachable when
        # their turn came, but they still counted as regions, the edge rows
        # were pinned to their extent instead of the real floor's, and
        # "nearest region next" measured from inside them as if the robot
        # had been there.
        if pose is not None:
            traversable = self.reachable_floor(traversable, msg, pose)

        # Everything below works in sweep coordinates: `line` counts across
        # the rows, `index` runs along one. Transposing the mask is all it
        # takes to swing the whole sweep through 90 deg.
        transposed = self.sweep_transposed(traversable)
        grid = traversable.T if transposed else traversable

        def to_world(line, index):
            row, col = (index, line) if transposed else (line, index)
            return self.cell_to_world(msg, row, col)

        # The decomposition is worked out on the floor with every speck-sized
        # hole filled in, so a speck does not split the floor round it into
        # regions of its own; the rows are then cut back to the floor that is
        # really there (`floor_runs`), so they still drive round it. See
        # `fill_specks` for what counts as speck-sized.
        speck = min_run + 2 * max(1, int(math.ceil(self.plan_radius / res)))
        strips = self.sweep_strips(self.fill_specks(grid, speck), step, min_run)

        if not strips:
            self.clean_regions = 0
            self.clean_rows = 0
            self.goal_regions = []
            return []

        line_flip, index_flip, corner = self.start_corner(strips, pose, to_world)
        regions = self.floor_runs(self.split_regions(strips, step), grid, min_run)

        self.clean_regions = len(regions)
        self.clean_rows = sum(len(runs) for region in regions
                              for runs in region.values())
        self.clean_corner = corner
        self.clean_axis = 'y' if transposed else 'x'

        if transposed:
            self.clean_side = 'right' if line_flip else 'left'
        else:
            self.clean_side = 'top' if line_flip else 'bottom'

        here = corner if pose is None else (pose[0], pose[1])
        goals = []
        order = []
        visited = 0
        pending = list(regions)

        while pending:
            # Nearest region next, entered from whichever end of its first row
            # is closer. Sweeping the near half of a flat costs the same as the
            # far half; walking to the far half first does not.
            best = None

            for position, region in enumerate(pending):
                for reverse in (index_flip, not index_flip):
                    entry = self.region_entry(region, line_flip, reverse, to_world)
                    cost = math.hypot(entry[0] - here[0], entry[1] - here[1])

                    if best is None or cost < best[0]:
                        best = (cost, position, reverse)

            _, position, reverse = best
            region = pending.pop(position)
            points = self.sweep_region(region, line_flip, reverse, to_world,
                                       along)

            if points:
                goals.extend(points)
                order.extend([visited] * len(points))
                here = points[-1]
                visited += 1

        self.goal_regions = order

        return goals

    def sweep_transposed(self, traversable):
        """
        Decide which way the zig-zag rows run.

        Naming a starting side settles it: `sweep_start:=top` means the rows
        run across x and the sweep works down y, so there is nothing for
        `sweep_axis` to decide. Only `sweep_start:=auto` consults it, where
        rows along the longer side of the floor means fewer end-of-row turns -
        and turning is where this robot loses both time and heading accuracy.
        """
        if self.sweep_start in ('top', 'bottom'):
            return False

        if self.sweep_start in ('left', 'right'):
            return True

        if self.sweep_axis == 'x':
            return False

        if self.sweep_axis == 'y':
            return True

        rows, cols = np.nonzero(traversable)

        if rows.size == 0:
            return False

        return int(rows.max() - rows.min()) > int(cols.max() - cols.min())

    def sweep_strips(self, grid, step, min_run):
        """
        Sample the floor into sweep lines and the drivable runs on each.

        The lines are laid out across the DRIVABLE extent, not from line 0 of
        the map, and both ends of that extent always get a line.

        That is the edge strip, and it was the single biggest reason a sweep
        left dirt behind. Walking `range(0, height, step)` samples lines at
        multiples of the step measured from the map's own origin, which has
        nothing to do with where the floor is: the first line could fall up to
        `step - 1` cells inside the near wall and the last one up to
        `step - 1` cells short of the far wall. The robot sweeps
        `cleaning_radius` either side of a line, so any part of that offset
        past the swath was floor no row ever went near - a strip down each
        edge of every room, left for the re-sweep passes and then for the
        block chase to pick up one square at a time.

        Pinning the ends and spreading the rest evenly between them also
        keeps every gap at or under `step`, so the overlap the caller worked
        out still holds and `split_regions` still joins consecutive lines.

        Runs the robot could not fit down are dropped here rather than later,
        so a sliver beside a wall can neither become a waypoint nor join two
        regions the robot cannot actually drive between.
        """
        drivable = np.flatnonzero(grid.any(axis=1))

        if drivable.size == 0:
            return {}

        first_line = int(drivable[0])
        last_line = int(drivable[-1])
        span = last_line - first_line

        # Fewest rows that keep every gap at or under `step`, then shared out
        # evenly rather than packed from one end - which is what stops the
        # pinned last line sitting one cell from its neighbour.
        rows = max(1, int(math.ceil(span / step))) if span else 0
        lines = ([first_line] if rows == 0 else
                 [first_line + int(round(i * span / rows))
                  for i in range(rows + 1)])

        strips = {}

        for line in lines:
            cols = np.flatnonzero(grid[line])

            if cols.size == 0:
                continue

            runs = [(first, last) for first, last in self.contiguous_runs(cols)
                    if last - first + 1 >= min_run]

            if runs:
                strips[line] = runs

        return strips

    @staticmethod
    def split_regions(strips, step):
        """
        Split the floor into regions no obstacle divides.

        This is the boustrophedon cellular decomposition. Walking the sweep
        lines in order, a run that carries straight on from exactly one run on
        the line before stays in the same region. Where a run splits in two
        around an obstacle, or two runs merge again past it, the region ends
        and new ones begin - so an obstacle standing in open floor produces a
        region for the floor in front of it, one down each side, and one for
        the floor behind. Each is then swept whole.

        That is the difference between finishing one side of a table and
        crossing to the other side once, and hopping over the table on every
        single row, which is what the plain row-by-row sweep did.
        """
        lines = sorted(strips)
        regions = []
        open_cells = {}
        previous = None

        for line in lines:
            runs = strips[line]

            if previous is None or line - previous > step:
                # A band with nothing drivable in it really does separate what
                # is either side of it; do not join across the gap.
                #
                # `> step`, not `!= step`: the lines are spread evenly over
                # the drivable extent so that both edges get one, which
                # leaves gaps of step or step - 1. Demanding an exact step
                # broke every region apart at the rounding, and in particular
                # orphaned the pinned last row of each one.
                ahead = [[] for _ in runs]
                behind = []
            else:
                ahead = [[index for index, (start, end) in enumerate(strips[previous])
                          if start <= last and first <= end]
                         for first, last in runs]
                behind = [[index for index, links in enumerate(ahead) if back in links]
                          for back in range(len(strips[previous]))]

            cells = {}

            for index, run in enumerate(runs):
                links = ahead[index]
                straight_on = (len(links) == 1
                               and len(behind[links[0]]) == 1
                               and links[0] in open_cells)

                if straight_on:
                    cell = open_cells[links[0]]
                else:
                    cell = len(regions)
                    regions.append({})

                regions[cell].setdefault(line, []).append(run)
                cells[index] = cell

            open_cells = cells
            previous = line

        return regions

    def reachable_floor(self, traversable, msg, pose):
        """
        Keep only the part of `traversable` connected to the robot.

        Seeded from the nearest drivable cell when the robot itself is
        standing in inflation, as it does beside a wall.
        """
        if not traversable.any():
            return traversable

        height, width = traversable.shape
        res = msg.info.resolution
        row = int((pose[1] - msg.info.origin.position.y) / res)
        col = int((pose[0] - msg.info.origin.position.x) / res)

        if not (0 <= row < height and 0 <= col < width and traversable[row, col]):
            free = np.argwhere(traversable)
            row, col = free[np.argmin(((free - (row, col)) ** 2).sum(axis=1))]

        return self.flood(traversable, [int(row) * width + int(col)])

    def floor_runs(self, regions, grid, min_run):
        """
        Cut each region's runs back to the floor actually there.

        A run laid across a filled-in speck becomes the runs either side of
        it, and the region drives them one after the other with the grid
        BFS routing round the speck between - a short detour on the few
        lines the speck spans, against a separate trip back for the floor
        on its far side. A piece too short for the robot is dropped, as
        `sweep_strips` drops it anywhere else.
        """
        out = []

        for region in regions:
            kept = {}

            for line, runs in region.items():
                pieces = []

                for first, last in runs:
                    cols = first + np.flatnonzero(grid[line, first:last + 1])

                    if cols.size:
                        pieces.extend(
                            (start, end)
                            for start, end in self.contiguous_runs(cols)
                            if end - start + 1 >= min_run)

                if pieces:
                    kept[line] = pieces

            if kept:
                out.append(kept)

        return out

    @staticmethod
    def flood(passable, seeds, diagonal=False):
        """
        Every cell of `passable` connected to the flat indices in `seeds`.

        A whole frontier at a time, so it costs one numpy pass per cell of
        distance rather than a Python step per cell of floor.
        """
        height, width = passable.shape
        flat = passable.reshape(-1)
        seen = np.zeros(flat.size, dtype=bool)
        frontier = np.unique(np.asarray(seeds, dtype=np.int64))
        frontier = frontier[flat[frontier]]
        seen[frontier] = True
        moves = [(-1, 0), (1, 0), (0, -1), (0, 1)]

        if diagonal:
            moves += [(-1, -1), (-1, 1), (1, -1), (1, 1)]

        while frontier.size:
            rows, cols = np.divmod(frontier, width)
            ahead = []

            for d_row, d_col in moves:
                inside = ((rows + d_row >= 0) & (rows + d_row < height)
                          & (cols + d_col >= 0) & (cols + d_col < width))
                ahead.append(frontier[inside] + d_row * width + d_col)

            ahead = np.unique(np.concatenate(ahead))
            frontier = ahead[flat[ahead] & ~seen[ahead]]
            seen[frontier] = True

        return seen.reshape(height, width)

    def fill_specks(self, grid, speck):
        """
        Fill every hole in the floor no more than `speck` lines tall.

        A hole is blocked ground with floor all the way round it - not
        joined to the walls. `speck` is how many lines an obstacle no bigger
        than the robot's own footprint covers once `replan` has inflated it
        by `plan_radius` on both sides: 5 + 2 x 4 = 13 cells, 0.65 m, on a
        0.05 m map. A lone SLAM speck in open floor comes out at 9. The
        smallest real obstacle in the sim world, the 0.8 m cylinder, comes
        out at 24 and is left alone.

        Only the decomposition sees the filled grid. The traversable mask,
        and so every path and every waypoint, still has the hole in it -
        a speck and a chair leg look exactly the same in an occupancy grid,
        and planning straight through either would be wrong.
        """
        blocked = ~grid
        border = np.zeros_like(grid)
        border[[0, -1], :] = border[:, [0, -1]] = True

        # Eight-connected: two blocked cells touching only at a corner
        # still wall off the four-connected floor.
        holes = blocked & ~self.flood(
            blocked, np.flatnonzero(blocked & border), diagonal=True)
        filled = grid.copy()

        while holes.any():
            hole = self.flood(holes, [int(np.flatnonzero(holes)[0])],
                              diagonal=True)
            rows = np.flatnonzero(hole.any(axis=1))
            holes &= ~hole

            if rows[-1] - rows[0] + 1 <= speck:
                filled |= hole

        return filled

    def start_corner(self, strips, pose, to_world):
        """
        Pick the corner of the drivable floor to begin the sweep at.

        Returns `(line_flip, index_flip, world_point)`: whether to walk the
        rows from the far end back, whether to walk the first row from its far
        end back, and where that corner actually is. Distance decides it; yaw
        only breaks ties, because a corner behind the robot costs a turn that
        one in front of it does not.
        """
        lines = sorted(strips)

        def corner_of(line_flip, index_flip):
            # The end of the first row the sweep would drive, so the corner is
            # always a cell the robot can actually stand on. Taking the
            # bounding box instead puts the corner of an L-shaped room out in
            # the missing quadrant, where there is no floor at all.
            line = lines[-1] if line_flip else lines[0]
            runs = strips[line]
            index = (max(run[1] for run in runs) if index_flip
                     else min(run[0] for run in runs))
            return line_flip, index_flip, to_world(line, index)

        # `line_flip` True always means "start at the high end of the axis the
        # sweep works along": the top when the rows run across x, the right
        # when they run across y. Naming a side fixes it and leaves only the
        # choice of which end of that first row to enter by.
        fixed = {'top': True, 'right': True, 'bottom': False, 'left': False}
        flips = ((fixed[self.sweep_start],) if self.sweep_start in fixed
                 else (False, True))

        corners = [corner_of(line_flip, index_flip)
                   for line_flip in flips
                   for index_flip in (False, True)]

        if pose is None:
            return corners[0]

        x, y, yaw = pose

        def cost(corner):
            corner_x, corner_y = corner[2]
            reach = math.hypot(corner_x - x, corner_y - y)
            bearing = math.atan2(corner_y - y, corner_x - x)
            turn = abs(math.atan2(math.sin(bearing - yaw), math.cos(bearing - yaw)))
            return reach + self.corner_turn_weight * turn

        return min(corners, key=cost)

    @staticmethod
    def region_lines(region, line_flip):
        """Return the region's sweep lines in the order they get driven."""
        return sorted(region, reverse=line_flip)

    def region_entry(self, region, line_flip, reverse, to_world):
        """Where the robot would first arrive if it swept this region now."""
        line = self.region_lines(region, line_flip)[0]
        runs = sorted(region[line], reverse=reverse)
        first, last = runs[0]
        return to_world(line, last if reverse else first)

    def sweep_region(self, region, line_flip, reverse, to_world, along=0):
        """
        Zig-zag one obstacle-free region end to end.

        Each row is walked in the opposite direction to the one before, so the
        robot finishes a row next to the start of the next instead of driving
        all the way back - the usual boustrophedon (ox-plough) pattern.

        `along` is the intermediate-waypoint spacing in cells; 0 emits the two
        ends of each run and nothing between them. See `row_point_spacing`.
        """
        points = []

        for line in self.region_lines(region, line_flip):
            for first, last in sorted(region[line], reverse=reverse):
                indices = self.run_points(first, last, along)

                if reverse:
                    indices.reverse()

                points.extend(to_world(line, index) for index in indices)

            reverse = not reverse

        return points

    @staticmethod
    def run_points(first, last, along):
        """
        Waypoint indices along one run, ends included, ascending.

        Placed at fixed positions in the run rather than counted out from
        whichever end the robot enters by, so driving a row forwards and
        driving it backwards visit exactly the same cells.
        """
        if along <= 0 or last - first <= along:
            return [first, last]

        parts = int(math.ceil((last - first) / along))

        return [first + int(round(i * (last - first) / parts))
                for i in range(parts + 1)]

    @staticmethod
    def contiguous_runs(cols):
        """Split sorted column indices into unbroken spans."""
        runs = []
        first = previous = int(cols[0])

        for col in cols[1:]:
            col = int(col)

            if col == previous + 1:
                previous = col
            else:
                runs.append((first, previous))
                first = previous = col

        runs.append((first, previous))
        return runs

    def advance_cleaning(self, msg, traversable, dist, start, h, w):
        """Drive to the next cleaning waypoint that is still reachable."""
        self.clean_coverage = self.coverage_percent(msg, traversable)

        while self.goal_index < len(self.cleaning_goals):
            gx, gy = self.cleaning_goals[self.goal_index]
            row, col = self.world_to_cell(msg, gx, gy)

            if 0 <= row < h and 0 <= col < w:
                idx = row * w + col

                if dist[idx] >= 0:
                    self.path = self.build_path(msg, dist, start, idx, w)
                    self.publish_path(msg)
                    self.log_waypoint()
                    return

            # Furniture moved, or the map tightened up and this spot is no
            # longer drivable - skip it rather than stalling the sweep.
            self.goal_index += 1

        self.finish_pass(msg, traversable, h, w)

    def log_waypoint(self):
        """The throttled progress line for the waypoint just routed to."""
        done = self.goal_index + 1
        total = len(self.cleaning_goals)
        self.get_logger().info(
            f'Cleaning pass {self.cleaning_pass}/'
            f'{self.max_cleaning_passes}: '
            f'{self.region_label()}, waypoint {done}/{total}, '
            f'{self.clean_coverage:.0f}% covered',
            throttle_duration_sec=2.0,
        )

    def advance_on_arrival(self, pose):
        """
        Route to the next cleaning waypoint on the last replan's grid.

        What `advance_cleaning` does, minus the replan in front of it: the
        map message and inflated mask are the cached ones, and the wavefront
        is run from where the robot is NOW only as far as the goal - a metre
        or so of floor rather than all of it. The cached wavefront cannot be
        walked directly: it was flooded from where the robot stood at the last
        replan, so its paths start back there.

        Returns False, having changed nothing, whenever the full replan is the
        right tool: no cache yet, the robot off the cached floor, or no
        waypoint left in this pass that the cached grid can reach.
        """
        cache = self.plan_cache

        if cache is None:
            return False

        msg = cache['msg']
        traversable = cache['traversable']
        h, w = traversable.shape
        start = self.start_cell(pose, msg, traversable, h, w)

        if start is None or cache['dist'][start] < 0:
            return False

        # Same component as the replan's start, so its wavefront answers
        # "reachable?" for every waypoint without flooding again.
        reach = cache['dist']
        index = self.goal_index

        while index < len(self.cleaning_goals):
            gx, gy = self.cleaning_goals[index]
            row, col = self.world_to_cell(msg, gx, gy)

            if 0 <= row < h and 0 <= col < w and reach[row * w + col] >= 0:
                goal = row * w + col
                dist = self.wavefront(traversable, start, stop=goal)

                self.goal_index = index
                self.clean_coverage = self.coverage_percent(msg, traversable)
                self.path = self.build_path(msg, dist, start, goal, w)
                self.publish_path(msg)
                self.log_waypoint()
                return True

            index += 1

        return False

    def region_label(self):
        """Which region of how many the current waypoint belongs to."""
        where = self.current_region()

        return (f'region {where or "?"} of '
                f'{max(1, self.clean_regions)}')

    def current_region(self):
        """Which region the current waypoint is in, 1-based, or 0 if unknown."""
        if self.goal_index >= len(self.goal_regions):
            return 0

        return self.goal_regions[self.goal_index] + 1

    def finish_pass(self, msg, traversable, h, w):
        """Re-sweep anything missed, or stop if the floor is done."""
        if self.cleaning_pass < self.max_cleaning_passes:
            plan = self.build_cleaning_plan(
                msg, traversable, h, w, pose=self.robot_pose())

            # Only the spots the swath genuinely missed, and their region
            # labels with them - `build_cleaning_plan` has just refilled
            # `goal_regions` for the FULL plan, so it has to be filtered in
            # step with the waypoints or the status line names the wrong one.
            regions = self.goal_regions
            keep = [index for index, point in enumerate(plan)
                    if not self.is_covered(msg, point)]
            remaining = [plan[index] for index in keep]

            if remaining:
                self.cleaning_pass += 1
                self.goal_index = 0
                self.cleaning_goals = remaining
                self.goal_regions = [regions[index] for index in keep
                                     if index < len(regions)]

                self.get_logger().info(
                    f'Cleaning pass {self.cleaning_pass}/'
                    f'{self.max_cleaning_passes}: {len(remaining)} of '
                    f'{len(plan)} spots still uncovered'
                )
                return

        # The rows are done. Anything still red is floor the fixed pattern
        # could not reach - a row the robot was pushed off, or floor that only
        # appeared after the map grew. Hand over to the block chase rather than
        # declaring the job finished with squares still red on the screen.
        state = self.block_state(msg, traversable)

        if state is not None and self.red_blocks(state).size:
            self.chasing_blocks = True
            self.path = []

            red = int(self.red_blocks(state).size)
            total = int(np.count_nonzero(state.total))

            # If this number is anything but small, the sweep is the thing to
            # fix - the chase is meant to be a gap-fill, not the main event.
            self.get_logger().info(
                f'Rows finished at {self.clean_coverage:.0f}% with {red} of '
                f'{total} square(s) still red - gap-filling them one at a '
                f'time'
            )
            return

        self.complete_cleaning(msg, traversable)

    @staticmethod
    def pack_key(cell_x, cell_y):
        """
        Pack a quantised world cell into one integer.

        Coverage is keyed on world coordinates rather than grid indices so it
        survives slam_toolbox re-anchoring the grid mid-run. The offset keeps
        the value positive for negative coordinates.
        """
        return (cell_x + COVER_ORIGIN) * COVER_STRIDE + (cell_y + COVER_ORIGIN)

    def cover_key(self, msg, x, y):
        res = msg.info.resolution
        return self.pack_key(int(quantise(x, res)), int(quantise(y, res)))

    def is_covered(self, msg, point):
        return self.cover_key(msg, point[0], point[1]) in self.covered

    def mark_covered(self, x, y):
        """Record the floor swept by the robot at this pose."""
        if self.map_msg is None:
            return

        res = self.map_msg.info.resolution
        reach = max(1, int(round(self.cleaning_radius / res)))
        cx, cy = int(quantise(x, res)), int(quantise(y, res))

        for dx in range(-reach, reach + 1):
            for dy in range(-reach, reach + 1):
                if dx * dx + dy * dy <= reach * reach:
                    self.covered.add(self.pack_key(cx + dx, cy + dy))

    def coverage_percent(self, msg, traversable):
        """How much of the drivable floor has been swept."""
        rows, cols = np.nonzero(traversable)

        if rows.size == 0:
            return 0.0

        if not self.covered:
            return 0.0

        # Vectorised: this runs on every cleaning replan over the whole floor,
        # which is far too many cells to walk one at a time in Python.
        res = msg.info.resolution
        xs = msg.info.origin.position.x + (cols + 0.5) * res
        ys = msg.info.origin.position.y + (rows + 0.5) * res

        keys = ((quantise(xs, res) + COVER_ORIGIN) * COVER_STRIDE
                + (quantise(ys, res) + COVER_ORIGIN))

        covered = np.fromiter(self.covered, dtype=np.int64, count=len(self.covered))

        return 100.0 * np.isin(keys, covered).sum() / rows.size

    @staticmethod
    def cluster(seed, reachable, seen, h, w):
        """8-connected group of frontier cells containing `seed`."""
        group = []
        queue = deque([seed])
        seen.add(seed)

        while queue:
            cur = queue.popleft()
            group.append(cur)
            row, col = divmod(cur, w)

            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    r, c = row + dr, col + dc

                    if not (0 <= r < h and 0 <= c < w):
                        continue

                    nxt = r * w + c

                    if reachable[nxt] and nxt not in seen:
                        seen.add(nxt)
                        queue.append(nxt)

        return group

    def build_path(self, msg, dist, start, goal, w):
        cells = self.descend(dist, start, goal, w)

        return [self.cell_to_world(msg, *divmod(c, w)) for c in cells]

    def publish_path(self, msg):
        path_msg = Path()
        path_msg.header.frame_id = self.map_frame
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for x, y in self.path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    # ------------------------------------------------------------------
    # Driving
    # ------------------------------------------------------------------

    def follow_path(self):
        self.publish_mode()

        # Finished, stopped or paused: keep saying "zero" rather than going
        # quiet. The ESP32 drops the wheels 500 ms after the last cmd_vel
        # either way, but a base that stops hearing from this node cannot
        # tell a finished run from a crashed one.
        if self.paused or self.mode in ('done', 'stopped'):
            self.cmd_pub.publish(Twist())
            return

        # Exploration finished and the map is being saved: the wheels stay
        # still until the save has answered (or timed out).
        if self.handoff is not None:
            self.cmd_pub.publish(Twist())
            return

        # Localize BEFORE exploring. `gate_localization` publishes the zero
        # Twist itself on every tick it holds.
        if not self.gate_localization():
            return

        if self.scan_is_stale():
            age = self.scan_age()
            self.get_logger().warn(
                'No lidar yet - holding' if age is None
                else f'No lidar for {age:.1f} s - holding',
                throttle_duration_sec=2.0,
            )
            self.cmd_pub.publish(Twist())
            return

        cmd = Twist()

        pose = self.robot_pose()
        if pose is None:
            self.cmd_pub.publish(cmd)
            return

        x, y, yaw = pose

        # Wherever the robot goes, that floor counts as swept.
        self.mark_covered(x, y)

        # An escape manoeuvre owns the wheels until it times out.
        if time.monotonic() < self.recovery_until:
            self.drive_recovery(cmd)
            self.publish_safe(cmd)
            return

        # A committed back-out owns the wheels until it expires. Re-deciding
        # every tick is what produced the forward-and-backward shuffle.
        if self.backing_out():
            cmd.linear.x = -0.5 * self.linear_speed
            cmd.angular.z = self.avoid_steer()

            # Keep reversing while there is still nowhere to turn. A corridor
            # narrower than the turning circle has to be left end to end, and
            # a fixed 1.5 s just returns the robot to where it started facing
            # the same way - which is the shuffle, one step slower.
            # `backout_max` stops it reversing across the building.
            elapsed = time.monotonic() - self.backout_started
            if not self.can_turn() and elapsed < self.backout_max:
                self.backout_until = time.monotonic() + 0.2

            self.publish_safe(cmd)
            return

        # Progress is tracked even with no path, so spinning on the spot for
        # want of a plan still counts as being stuck and still triggers an
        # escape. Otherwise a wedged robot would turn in place forever.
        self.note_progress(x, y)

        if self.is_stuck():
            self.begin_recovery(x, y)
            self.drive_recovery(cmd)
            self.publish_safe(cmd)
            return

        if not self.path:
            # Nothing planned. Head for open space rather than spinning: the
            # lidar already sees all 360 deg from here, so turning on the spot
            # reveals nothing new and the map would never change.
            heading = self.escape_heading()
            can_turn = self.can_turn()
            ahead = self.forward_clearance() > self.stop_distance

            if heading is None:
                cmd.linear.x = -0.4 * self.linear_speed
                cmd.angular.z = 0.3 * self.angular_speed if can_turn else 0.0
            elif abs(heading) > math.radians(30):
                if can_turn:
                    cmd.angular.z = (self.angular_speed if heading > 0
                                     else -self.angular_speed)
                elif ahead:
                    # Cannot turn towards the opening, but the way ahead is
                    # clear. Driving on changes the geometry and usually frees
                    # the turn; standing here never does.
                    cmd.linear.x = 0.5 * self.linear_speed
                else:
                    cmd.linear.x = -0.5 * self.linear_speed
            elif ahead:
                cmd.linear.x = 0.6 * self.linear_speed
                cmd.angular.z = 0.8 * heading if can_turn else 0.0
            else:
                # Pointing the right way, but blocked ahead and unable to
                # turn. Without this branch nothing was set at all and the
                # robot published a zero command.
                self.back_out(cmd)

            self.publish_safe(cmd)
            return

        # Drop path points we've already passed, but never the goal itself -
        # a short path can otherwise be consumed whole in one pass.
        while len(self.path) > 1 and math.dist((x, y), self.path[0]) < self.lookahead:
            self.path.pop(0)

        target = self.path[0]

        if len(self.path) == 1 and math.dist((x, y), target) < self.arrive_tolerance():
            # Goal reached; pick the next one straight away rather than idling
            # until the next replan tick - but never by replanning in here. A
            # full replan inside this 10 Hz tick (229 ms on the Pi 5 with the
            # old BFS) starved the control loop of two-plus ticks at every
            # single waypoint: the stop-and-go.
            self.path = []
            self.recovery_attempts = 0

            if self.mode == 'clean':
                self.goal_index += 1

                if (self.clean_plan_ready and not self.chasing_blocks
                        and self.advance_on_arrival(pose)):
                    return

            self.request_replan()
            return

        heading = math.atan2(target[1] - y, target[0] - x)
        error = math.atan2(math.sin(heading - yaw), math.cos(heading - yaw))

        forward = self.forward_clearance()

        if forward <= self.stop_distance:
            # Something is inside the drive corridor. Turn towards whichever
            # side has more room rather than towards the goal, which may well
            # be straight through the wall we just stopped for.
            cmd.linear.x = 0.0

            if self.can_turn():
                left = self.side_clearance(1)
                right = self.side_clearance(-1)
                cmd.angular.z = self.angular_speed if left > right else -self.angular_speed
            else:
                # Boxed in: no room to spin, so reverse out of it, curving
                # away from whatever stopped us.
                self.back_out(cmd)

        elif abs(error) > math.radians(35):
            # Too far off heading to drive usefully; rotate in place, but only
            # if the swept circle is actually clear.
            cmd.linear.x = 0.0

            if self.can_turn():
                cmd.angular.z = self.angular_speed if error > 0 else -self.angular_speed
            elif forward > self.stop_distance:
                # Too far off heading to drive at the goal, and no room to
                # turn towards it. Creep forward and let the obstacle bearing
                # bend the path, rather than freezing or shuffling.
                cmd.linear.x = 0.4 * self.linear_speed
                cmd.angular.z = self.avoid_steer()
            else:
                self.back_out(cmd)

        else:
            # Ease off the throttle as the gap ahead closes instead of
            # running at full speed right up to the stopping distance.
            span = max(1e-3, self.slow_distance - self.stop_distance)
            scale = (forward - self.stop_distance) / span
            scale = max(0.15, min(1.0, scale))

            cmd.linear.x = self.linear_speed * scale

            # Steer towards the goal, bent away from whatever is closest. The
            # bend is proportional to how head-on and how near that thing is,
            # so in open space it contributes nothing and near a wall it is
            # what stops the robot driving into it and having to back out.
            steer = 1.2 * error + self.avoid_steer()
            cmd.angular.z = max(-self.angular_speed,
                                min(self.angular_speed, steer))

        self.publish_safe(cmd)

    def arrive_tolerance(self):
        """
        How close to a waypoint counts as having reached it.

        Tighter while sweeping, and that matters. `goal_tolerance` is 0.15 m
        and `cleaning_radius` is 0.135 m, so a robot that stops the full
        tolerance short of a row end marks a disc that does not quite contain
        the cell it was sent to - the same arithmetic that had the block
        chase drive at one leftover cell forever. Over a sweep it leaves a
        nick of dirt at both ends of every single row, which is exactly the
        work the gap-fill is supposed to not have.

        Only the row sweep is tightened. Exploring and the block chase keep
        the looser figure, because neither of them is trying to put the
        swath on top of the goal.
        """
        if self.mode == 'clean' and not self.chasing_blocks:
            return min(self.goal_tolerance, 0.9 * self.cleaning_radius)

        return self.goal_tolerance

    def complete_cleaning(self, msg, traversable):
        self.mode = 'done'
        self.path = []
        self.cmd_pub.publish(Twist())

        covered = self.coverage_percent(msg, traversable)
        self.clean_coverage = covered

        self.get_logger().info('=' * 46)
        self.get_logger().info('CLEANING COMPLETE')
        self.get_logger().info(
            f'{covered:.0f}% of the drivable floor swept '
            f'in {self.cleaning_pass} pass(es).'
        )
        self.get_logger().info('The map has been saved throughout.')
        self.get_logger().info('=' * 46)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        # A launch file's SIGTERM arrives as ExternalShutdownException on
        # Jazzy, with the context already torn down - so the last stop
        # Twist below has nowhere to go. The ESP32's own 500 ms watchdog
        # is what actually stops the wheels then; this is best effort.
        pass
    finally:
        try:
            node.cmd_pub.publish(Twist())
        except Exception:                              # noqa: BLE001
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
