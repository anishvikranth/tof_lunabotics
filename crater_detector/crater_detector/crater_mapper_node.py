"""Crater mapper: turns ToF range readings into a crater layer for Nav2.

Each reading is projected with TF into the map frame. Points clearly below the
local ground level are crater points: a disc around the hit (and the ground the
beam passed over since the last ground point) gets hit counters, and a cell
becomes occupied (100) after confirm_hits. Confirmed cells are never cleared.

Outputs:
  /crater_map    nav_msgs/OccupancyGrid  (follows /map geometry when available)
  /crater_cloud  sensor_msgs/PointCloud2 (confirmed cells, for an obstacle layer)
  /crater_info   std_msgs/String (JSON size estimate of the largest crater)
"""

import json
import math
from collections import deque
from statistics import median

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2, Range
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

UNKNOWN, FREE, CRATER = -1, 0, 100


def quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def convex_hull(pts):
    """Andrew's monotone chain. pts: (N, 2) array -> hull vertices (M, 2)."""
    pts = np.unique(pts, axis=0)
    if len(pts) < 3:
        return pts
    pts = pts[np.lexsort((pts[:, 1], pts[:, 0]))]

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in pts[::-1]:
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return np.array(lower[:-1] + upper[:-1])


def min_area_rect(hull):
    """Minimum-area bounding rectangle of hull points -> (length, width)."""
    if len(hull) < 3:
        span = np.ptp(hull, axis=0) if len(hull) else np.zeros(2)
        return float(max(span)), float(min(span))
    best = None
    for k in range(len(hull)):
        edge = hull[(k + 1) % len(hull)] - hull[k]
        a = math.atan2(edge[1], edge[0])
        rot = np.array([[math.cos(a), math.sin(a)], [-math.sin(a), math.cos(a)]])
        span = np.ptp(hull @ rot.T, axis=0)
        if best is None or span[0] * span[1] < best[0] * best[1]:
            best = span
    return float(max(best)), float(min(best))


class CraterMapper(Node):

    def __init__(self):
        super().__init__('crater_mapper')

        self.declare_parameter('range_topic', '/mavros/rangefinder_pub')
        self.declare_parameter('map_topic', '/map')  # grid geometry follows this map
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('sensor_frame', 'tof')
        # Fallback grid used until a /map message arrives.
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('width_m', 20.0)
        self.declare_parameter('height_m', 20.0)
        self.declare_parameter('origin_x', -10.0)
        self.declare_parameter('origin_y', -10.0)
        self.declare_parameter('mark_radius', 0.12)
        self.declare_parameter('confirm_hits', 3)
        self.declare_parameter('noreturn_weight', 1)
        self.declare_parameter('depth_threshold', 0.08)
        self.declare_parameter('ground_tol', 0.05)
        self.declare_parameter('ground_window', 30)
        self.declare_parameter('median_window', 5)
        # Beam-line marking only if the last ground point is this recent and close.
        self.declare_parameter('line_max_age', 0.3)
        self.declare_parameter('line_max_dist', 1.5)
        # Skip readings when the latest pose is older than this (s), e.g. ZED tracking lost.
        self.declare_parameter('max_tf_age', 0.25)
        self.declare_parameter('publish_rate', 2.0)
        self.declare_parameter('cloud_height', 0.2)  # z above ground for /crater_cloud points
        self.declare_parameter('assumed_depth', 0.50)

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.map_frame = p('map_frame')
        self.sensor_frame = p('sensor_frame')
        self.mark_radius = p('mark_radius')
        self.confirm_hits = p('confirm_hits')
        self.noreturn_weight = p('noreturn_weight')
        self.depth_threshold = p('depth_threshold')
        self.ground_tol = p('ground_tol')
        self.line_max_age = p('line_max_age')
        self.line_max_dist = p('line_max_dist')
        self.max_tf_age = p('max_tf_age')
        self.cloud_height = p('cloud_height')
        self.assumed_depth = p('assumed_depth')

        self.window = deque(maxlen=p('median_window'))
        self.ground_z = deque(maxlen=p('ground_window'))
        self.ground_level = None  # set once the first ground_window readings agree
        self.last_ground = None   # (x, y, time) of the latest ground point

        res = p('resolution')
        self.make_grid(res, int(round(p('width_m') / res)), int(round(p('height_m') / res)),
                       p('origin_x'), p('origin_y'))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=10)
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Range, p('range_topic'), self.on_range, sensor_qos)
        self.create_subscription(OccupancyGrid, p('map_topic'), self.on_map, latched)
        self.map_pub = self.create_publisher(OccupancyGrid, '/crater_map', latched)
        self.cloud_pub = self.create_publisher(PointCloud2, '/crater_cloud', latched)
        self.info_pub = self.create_publisher(String, '/crater_info', latched)
        self.create_service(Trigger, '~/clear', self.on_clear)
        self.create_timer(1.0 / p('publish_rate'), self.publish)
        self.dirty = True

        self.get_logger().info(
            f'Crater mapper: {self.sensor_frame} -> {self.map_frame}, waiting for '
            f'{p("ground_window")} ground readings to set the ground level.')

    # ---------- grid ----------

    def make_grid(self, res, w, h, ox, oy):
        self.res, self.w, self.h, self.ox, self.oy = res, w, h, ox, oy
        self.hits = np.zeros((h, w), np.int16)
        self.state = np.full((h, w), UNKNOWN, np.int8)
        self.depth = np.zeros((h, w), np.float32)
        r = int(math.ceil(self.mark_radius / res))
        dj, di = np.mgrid[-r:r + 1, -r:r + 1]
        inside = (di * res) ** 2 + (dj * res) ** 2 <= self.mark_radius ** 2
        self.disc = (dj[inside], di[inside])

    def on_map(self, msg):
        info = msg.info
        res, w, h = info.resolution, info.width, info.height
        ox, oy = info.origin.position.x, info.origin.position.y
        if (w, h) == (self.w, self.h) and abs(res - self.res) < 1e-6 and \
                abs(ox - self.ox) < 1e-6 and abs(oy - self.oy) < 1e-6:
            return
        old = (self.res, self.w, self.h, self.ox, self.oy, self.hits, self.state, self.depth)
        self.make_grid(res, w, h, ox, oy)
        o_res, o_w, o_h, o_ox, o_oy, o_hits, o_state, o_depth = old
        if abs(o_res - res) < 1e-6:
            # Same resolution: copy the overlap, shifted by the origin change.
            di = int(round((o_ox - ox) / res))
            dj = int(round((o_oy - oy) / res))
            si0, sj0 = max(0, -di), max(0, -dj)
            si1, sj1 = min(o_w, w - di), min(o_h, h - dj)
            if si1 > si0 and sj1 > sj0:
                src = (slice(sj0, sj1), slice(si0, si1))
                dst = (slice(sj0 + dj, sj1 + dj), slice(si0 + di, si1 + di))
                self.hits[dst] = o_hits[src]
                self.state[dst] = o_state[src]
                self.depth[dst] = o_depth[src]
        else:
            # Different resolution: keep confirmed craters only.
            for j, i in np.argwhere(o_state == CRATER):
                c = self.cell(o_ox + (i + 0.5) * o_res, o_oy + (j + 0.5) * o_res)
                if c:
                    self.state[c] = CRATER
                    self.depth[c] = max(self.depth[c], o_depth[j, i])
        self.dirty = True
        self.get_logger().info(
            f'Crater grid now follows {self.get_parameter("map_topic").value}: '
            f'{w}x{h} @ {res:.3f} m, origin ({ox:.2f}, {oy:.2f})')

    def cell(self, x, y):
        i = int(math.floor((x - self.ox) / self.res))
        j = int(math.floor((y - self.oy) / self.res))
        if 0 <= i < self.w and 0 <= j < self.h:
            return j, i
        return None

    def add_hits(self, jj, ii, weight, depth):
        ok = (ii >= 0) & (ii < self.w) & (jj >= 0) & (jj < self.h)
        jj, ii = jj[ok], ii[ok]
        if not len(jj):
            return
        np.add.at(self.hits, (jj, ii), weight)
        if depth is not None:
            self.depth[jj, ii] = np.maximum(self.depth[jj, ii], depth)
        newly = (self.hits[jj, ii] >= self.confirm_hits) & (self.state[jj, ii] != CRATER)
        if newly.any():
            self.state[jj[newly], ii[newly]] = CRATER
            self.dirty = True

    def mark_disc(self, x, y, weight, depth):
        c = self.cell(x, y)
        if c is None:
            return
        self.add_hits(self.disc[0] + c[0], self.disc[1] + c[1], weight, depth)

    def mark_line(self, x0, y0, x1, y1, weight):
        """Ground the beam passed over between the last ground point and the hit."""
        length = math.hypot(x1 - x0, y1 - y0)
        if length < self.res:
            return
        # Start one cell past the ground point so the rim cell itself stays ground.
        t = np.arange(self.res, length, self.res / 2) / length
        ii = np.floor((x0 + t * (x1 - x0) - self.ox) / self.res).astype(int)
        jj = np.floor((y0 + t * (y1 - y0) - self.oy) / self.res).astype(int)
        cells = np.unique(np.stack([jj, ii], axis=1), axis=0)
        self.add_hits(cells[:, 0], cells[:, 1], weight, None)

    def mark_crater(self, x, y, depth, weight):
        self.mark_disc(x, y, weight, depth)
        if self.last_ground is not None:
            gx, gy, gt = self.last_ground
            age = (self.get_clock().now() - gt).nanoseconds / 1e9
            if age < self.line_max_age and math.hypot(x - gx, y - gy) < self.line_max_dist:
                self.mark_line(gx, gy, x, y, weight)

    # ---------- readings ----------

    def lookup(self, stamp):
        # No timeout: waiting would block this node's executor, so the TF listener
        # (same executor) could not receive new transforms and the buffer falls behind.
        try:
            return self.tf_buffer.lookup_transform(
                self.map_frame, self.sensor_frame, Time.from_msg(stamp))
        except TransformException:
            pass
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.sensor_frame, Time())
        except TransformException as e:
            self.get_logger().warn(f'No TF {self.map_frame} -> {self.sensor_frame}: {e}',
                                   throttle_duration_sec=5.0)
            return None
        # A frozen pose (odometry stopped) would put marks at the wrong place.
        # Stamp 0 means an all-static chain (bench test), which is always valid.
        tf_time = Time.from_msg(tf.header.stamp)
        if tf_time.nanoseconds > 0:
            age = (self.get_clock().now() - tf_time).nanoseconds / 1e9
            if age > self.max_tf_age:
                self.get_logger().warn(
                    f'Pose {self.map_frame} -> {self.sensor_frame} is {age:.1f} s old '
                    '(odometry stopped?), skipping readings', throttle_duration_sec=5.0)
                return None
        return tf

    def on_range(self, msg: Range):
        r = msg.range
        no_return = not math.isfinite(r) or (msg.max_range > 0.0 and r >= msg.max_range)
        if not no_return and r < msg.min_range:
            return
        tf = self.lookup(msg.header.stamp)
        if tf is None:
            return
        t = tf.transform.translation
        origin = np.array([t.x, t.y, t.z])
        beam = quat_to_matrix(tf.transform.rotation)[:, 0]  # sensor +x axis in map

        if no_return:
            # Mark where the beam would have hit flat ground: the ground dropped away there.
            if self.ground_level is None or beam[2] > -1e-3:
                return
            s = (self.ground_level - origin[2]) / beam[2]
            if s > 0.0:
                hit = origin + s * beam
                self.mark_crater(hit[0], hit[1], None, self.noreturn_weight)
            return

        self.window.append(r)
        x, y, z = origin + median(self.window) * beam

        if self.ground_level is None:
            self.init_ground(z)
            return

        d = self.ground_level - z  # depth below local ground
        if abs(d) < self.ground_tol:
            self.ground_z.append(z)
            self.ground_level = median(self.ground_z)
            self.last_ground = (x, y, self.get_clock().now())
            c = self.cell(x, y)
            if c and self.state[c] == UNKNOWN:
                self.state[c] = FREE
                self.dirty = True
        elif d > self.depth_threshold:
            self.mark_crater(x, y, d, 1)
        # Between ground_tol and depth_threshold, or above ground (bump): ignore.

    def init_ground(self, z):
        """Ground level = median of the first readings, once they agree within ground_tol."""
        self.ground_z.append(z)
        if len(self.ground_z) < self.ground_z.maxlen:
            return
        g = median(self.ground_z)
        agree = sum(abs(v - g) < self.ground_tol for v in self.ground_z)
        if agree >= 0.8 * len(self.ground_z):
            self.ground_level = g
            self.get_logger().info(f'Ground level set: z = {g:.3f} m in {self.map_frame}')
        else:
            self.ground_z.popleft()  # not flat yet; keep sliding

    def on_clear(self, request, response):
        self.hits[:] = 0
        self.state[:] = UNKNOWN
        self.depth[:] = 0.0
        self.last_ground = None
        self.dirty = True
        response.success = True
        response.message = 'Crater map cleared'
        self.get_logger().info(response.message)
        return response

    # ---------- output ----------

    def publish(self):
        if not self.dirty:
            return
        self.dirty = False
        now = self.get_clock().now().to_msg()
        header = Header(stamp=now, frame_id=self.map_frame)

        grid = OccupancyGrid(header=header)
        grid.info.map_load_time = now
        grid.info.resolution = float(self.res)
        grid.info.width, grid.info.height = self.w, self.h
        grid.info.origin.position.x = float(self.ox)
        grid.info.origin.position.y = float(self.oy)
        grid.info.origin.orientation.w = 1.0
        grid.data = self.state.ravel().tolist()
        self.map_pub.publish(grid)

        cells = np.argwhere(self.state == CRATER)  # (j, i)
        xy = np.column_stack([self.ox + (cells[:, 1] + 0.5) * self.res,
                              self.oy + (cells[:, 0] + 0.5) * self.res])
        z = (self.ground_level or 0.0) + self.cloud_height
        pts = [(float(px), float(py), z) for px, py in xy]
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, pts))

        self.info_pub.publish(String(data=json.dumps(self.crater_info(cells, xy))))

    def crater_info(self, cells, xy):
        if not len(cells):
            return {'craters': 0}
        # 8-connected clusters of confirmed cells.
        index = {tuple(c): k for k, c in enumerate(cells)}
        label = -np.ones(len(cells), int)
        n = 0
        for k in range(len(cells)):
            if label[k] >= 0:
                continue
            label[k] = n
            stack = [k]
            while stack:
                j, i = cells[stack.pop()]
                for dj in (-1, 0, 1):
                    for di in (-1, 0, 1):
                        m = index.get((j + dj, i + di))
                        if m is not None and label[m] < 0:
                            label[m] = n
                            stack.append(m)
            n += 1
        biggest = np.bincount(label).argmax()
        sel = label == biggest
        pts = xy[sel]
        hull = convex_hull(pts)
        length, width = min_area_rect(hull)
        margin = 2 * self.mark_radius
        diff = hull[:, None, :] - hull[None, :, :]
        diameter = float(np.sqrt((diff ** 2).sum(-1)).max()) if len(hull) > 1 else 0.0
        measured = float(self.depth[cells[sel, 0], cells[sel, 1]].max())
        return {
            'craters': int(n),
            'center_x': round(float(pts[:, 0].mean()), 3),
            'center_y': round(float(pts[:, 1].mean()), 3),
            'length': round(max(length - margin, self.res), 3),
            'width': round(max(width - margin, self.res), 3),
            'diameter': round(max(diameter - margin, self.res), 3),
            'depth': round(max(measured, self.assumed_depth), 3),
            'depth_measured': round(measured, 3),
            'depth_source': 'measured' if measured > self.assumed_depth else 'assumed',
            'cells': int(sel.sum()),
        }


def main(args=None):
    rclpy.init(args=args)
    node = CraterMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
