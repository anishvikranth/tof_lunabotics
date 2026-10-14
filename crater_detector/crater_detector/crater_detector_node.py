"""Crater detector: publishes True when the tilted ToF range jumps above the ground baseline.

The sensor is tilted down, so on flat ground it reads a steady distance d.
When the beam passes over a crater it hits a lower surface (or nothing),
so the new reading d_new is larger than d. If d_new - d > jump_threshold
for confirm_count readings in a row, /crater_detected becomes True.
"""

import math
from collections import deque
from statistics import median

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, Float32
from std_srvs.srv import Trigger


class CraterDetector(Node):

    def __init__(self):
        super().__init__('crater_detector')

        self.declare_parameter('range_topic', '/mavros/rangefinder_pub')
        self.declare_parameter('output_topic', '/crater_detected')
        # Ground distance d in metres. 0.0 = measure it at startup.
        self.declare_parameter('baseline', 0.0)
        self.declare_parameter('calibration_samples', 30)
        # How much longer than d a reading must be to count as a crater point.
        self.declare_parameter('jump_threshold', 0.15)
        self.declare_parameter('median_window', 5)
        # Consecutive readings needed to switch True, and to switch back False.
        self.declare_parameter('confirm_count', 3)
        # Treat "no return" (inf / beyond max_range) as a crater point.
        self.declare_parameter('no_return_is_crater', True)
        # Warn if no range message arrives for this long (s).
        self.declare_parameter('sensor_timeout', 1.0)

        p = self.get_parameter
        self.jump_threshold = p('jump_threshold').value
        self.calibration_samples = p('calibration_samples').value
        self.confirm_count = p('confirm_count').value
        self.no_return_is_crater = p('no_return_is_crater').value
        self.window = deque(maxlen=p('median_window').value)
        self.calib = []
        self.baseline = p('baseline').value if p('baseline').value > 0.0 else None

        self.crater = False
        self.streak = 0  # readings in a row that disagree with the current state

        # MAVROS publishes sensor data with best-effort QoS.
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=10)
        self.sub = self.create_subscription(
            Range, p('range_topic').value, self.on_range, sensor_qos)
        self.pub = self.create_publisher(Bool, p('output_topic').value, 10)
        self.baseline_pub = self.create_publisher(Float32, '~/baseline', 10)
        self.srv = self.create_service(Trigger, '~/recalibrate', self.on_recalibrate)

        # Watchdog: warn while the range topic is silent.
        self.range_topic = p('range_topic').value
        self.sensor_timeout = p('sensor_timeout').value
        self.last_msg_time = None
        self.sensor_ok = False
        self.watchdog = self.create_timer(2.0, self.check_sensor)

        if self.baseline is None:
            self.get_logger().info(
                f'Calibrating ground distance from {self.calibration_samples} readings; '
                'keep the rover on flat ground.')
        else:
            self.get_logger().info(f'Using fixed ground distance d = {self.baseline:.3f} m')

    def on_recalibrate(self, request, response):
        self.baseline = None
        self.calib.clear()
        self.window.clear()
        self.crater = False
        self.streak = 0
        response.success = True
        response.message = 'Recalibrating ground distance'
        self.get_logger().info(response.message)
        return response

    def check_sensor(self):
        now = self.get_clock().now()
        if self.last_msg_time is None:
            self.get_logger().warn(
                f'Sensor not connected: no data received on {self.range_topic}')
            return
        silent = (now - self.last_msg_time).nanoseconds / 1e9
        if silent > self.sensor_timeout:
            self.sensor_ok = False
            self.get_logger().warn(
                f'Sensor not connected: no data on {self.range_topic} for {silent:.1f} s')

    def on_range(self, msg: Range):
        self.last_msg_time = self.get_clock().now()
        if not self.sensor_ok:
            self.sensor_ok = True
            self.get_logger().info(f'Sensor connected: receiving data on {self.range_topic}')

        r = msg.range
        no_return = not math.isfinite(r) or (msg.max_range > 0.0 and r >= msg.max_range)
        if not no_return and r < msg.min_range:
            return  # too close / invalid, ignore

        # Calibration: median of the first N valid readings is the ground distance d.
        if self.baseline is None:
            if no_return:
                return
            self.calib.append(r)
            if len(self.calib) >= self.calibration_samples:
                self.baseline = median(self.calib)
                self.calib.clear()
                self.get_logger().info(f'Ground distance d = {self.baseline:.3f} m')
            return

        if no_return:
            is_crater_point = self.no_return_is_crater
        else:
            self.window.append(r)
            d_new = median(self.window)
            is_crater_point = (d_new - self.baseline) > self.jump_threshold

        # Debounce: flip state only after confirm_count disagreeing readings in a row.
        if is_crater_point != self.crater:
            self.streak += 1
            if self.streak >= self.confirm_count:
                self.crater = is_crater_point
                self.streak = 0
                self.get_logger().info('CRATER DETECTED' if self.crater else 'Ground again')
        else:
            self.streak = 0

        self.pub.publish(Bool(data=self.crater))
        self.baseline_pub.publish(Float32(data=float(self.baseline)))


def main(args=None):
    rclpy.init(args=args)
    node = CraterDetector()
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
