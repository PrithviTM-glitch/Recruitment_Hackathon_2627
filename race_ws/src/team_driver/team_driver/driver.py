#!/usr/bin/env python3
"""YOUR DRIVER GOES HERE.

This is the node the judges run. Keep `driver` as the executable name and keep
publishing to the two AutoDRIVE command topics; everything else is yours to
change - rewrite this file completely if you want to.

--------------------------------------------------------------------------
THIS TEMPLATE DOES NOT DRIVE
--------------------------------------------------------------------------
It is wiring, not a driver. It connects to the simulator, subscribes to the
sensors, and then publishes a slow constant throttle with the wheels straight.
It will roll forward off the line and into the first thing in front of it.
That is deliberate and it is the whole point: **there is no algorithm here and
no algorithm is shipped anywhere else in this repository.** Writing one is the
hackathon.

What the template is good for is proving your setup works. If the car moves
when you run it, then the image, the bridge, the QoS settings, the workspace
and your commands are all correct, and every problem left is yours.

    ros2 run team_driver driver
    ros2 launch team_driver driver.launch.py

`docs/04-algorithms.md` lists the approaches worth starting from - reactive
ones that need nothing but the LiDAR, planners that follow a line, model-based
control, and learned policies - with what each needs and where each breaks.
Pick one and replace `plan()` below.

--------------------------------------------------------------------------
THE CONTROL INTERFACE IS NOT A SPEED REQUEST
--------------------------------------------------------------------------
AutoDRIVE takes a normalised throttle, not a target speed:

    /autodrive/roboracer_1/throttle_command   Float32, [-1, 1]
    /autodrive/roboracer_1/steering_command   Float32, [-1, 1]  ->  +-0.5236 rad

-1 is full reverse, 0 is coasting, +1 is full torque. There is no speed
controller between you and the motor, so "go at 3 m/s" is a control problem you
now own. Whatever algorithm you pick will decide a *target* speed; turning that
into a throttle is a separate job, and it is worth several seconds a lap. See
`docs/04-algorithms.md` §4.1.

--------------------------------------------------------------------------
WHAT YOU MAY READ  (see docs/06-rules.md)
--------------------------------------------------------------------------
    .../lidar            1080 beams over 270 deg, 0.06-10 m
    .../imu              orientation, angular velocity, linear acceleration
    .../left_encoder     wheel angle, 16 PPR x 120
    .../right_encoder
    .../front_camera     RGB image (headless runs do not produce one)
    .../throttle         actuator feedback
    .../steering
    .../ips              ground-truth position - ALLOWED here, see below
    .../odom             ground-truth pose and velocity - ALLOWED, RECOMMENDED
    .../lap_count        .../lap_time  .../last_lap_time  .../best_lap_time
    .../collision_count
    /tf, /tf_static

The AutoDRIVE competition rules mark the pose and race-telemetry topics
"restricted": legal for development, not at race time. **The hackathon does not
adopt that restriction for reading.** Every input above is yours to use during
a scored run, ground-truth pose included. What stays restricted is *publishing*
`/autodrive/reset_command`, which is the referee's (rule 31).
"""

import math
import os

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray

# The steering command is normalised; this is what +-1.0 actually means.
MAX_STEERING_RAD = 0.5236


def devkit_qos() -> QoSProfile:
    """Match the QoS the AutoDRIVE bridge publishes and subscribes with.

    RELIABLE / KEEP_LAST(1) / VOLATILE. Get this wrong and the topics simply do
    not connect: a BEST_EFFORT subscriber will never see a RELIABLE publisher's
    messages, and `ros2 topic list` will still cheerfully show the topic.
    """
    return QoSProfile(
        durability=QoSDurabilityPolicy.VOLATILE,
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
    )


# Live-tunable: edit config/driver_params.yaml and save, the driver reloads it within 1 s.
TUNABLES = {
    'fov_deg': 90.0, 'smooth_beams': 5, 'disparity_thresh': 0.5,
    'car_half_width': 0.24, 'bubble_radius': 0.45, 'gap_min_range': 1.5,
    'edge_margin_beams': 30, 'lookahead': 1.5,
    'steer_gain': 1.0, 'max_steer': 1.0, 'steer_smooth': 0.0,
    'front_cone_deg': 8.0, 'brake_dist': 5.0, 'steer_slow': 0.7,
    'v_max': 3.0, 'v_min': 1.0, 'ff_gain': 0.10, 'kp': 0.15, 'ki': 0.05, 'i_max': 1.0,
}


class Driver(Node):

    def __init__(self):
        super().__init__('driver')

        # Declared parameters can be retuned without editing code:
        #   ros2 run team_driver driver --ros-args -p crawl_throttle:=0.3
        # Add your own as you go; config/driver_params.yaml loads them.
        self.declare_parameter('vehicle_ns', '/autodrive/roboracer_1')
        self.declare_parameter('crawl_throttle', 0.12)    # [-1, 1], open loop
        self.declare_parameter('max_range', 10.0)         # [m] clip the scan here
        self.declare_parameter('params_file', '/hackathon/race_ws/src/team_driver/config/driver_params.yaml')
        self.declare_parameter('centerline', '/hackathon/maps/icra26_compete_centerline.csv')
        for k, v in TUNABLES.items():
            self.declare_parameter(k, v)
        self.crawl_throttle = self.get_parameter('crawl_throttle').value
        self.max_range = self.get_parameter('max_range').value
        for k in TUNABLES:
            setattr(self, k, self.get_parameter(k).value)
        self.params_file = self.get_parameter('params_file').value
        self.params_mtime = 0.0
        self.create_timer(1.0, self.reload_params)
        self.prev_steer = 0.0
        self.integral = 0.0
        self.last_t = None
        self.line = np.loadtxt(self.get_parameter('centerline').value, delimiter=',')
        self.line_i = None

        ns = str(self.get_parameter('vehicle_ns').value).rstrip('/')
        qos = devkit_qos()

        self.throttle_pub = self.create_publisher(Float32, f'{ns}/throttle_command', qos)
        self.steering_pub = self.create_publisher(Float32, f'{ns}/steering_command', qos)
        self.marker_pub = self.create_publisher(MarkerArray, '/driver/markers', 1)

        self.create_subscription(LaserScan, f'{ns}/lidar', self.scan_callback, qos)
        self.create_subscription(Odometry, f'{ns}/odom', self.odom_callback, qos)
        self.create_subscription(Point, f'{ns}/ips', self.ips_callback, qos)

        # Latest known pose and speed. Ground truth from the simulator, which
        # the hackathon rules allow you to use - so use it.
        self.position = None      # (x, y) in the `world` frame
        self.yaw = 0.0            # [rad]
        self.speed = 0.0          # [m/s]

        self._marker_divisor = 0
        self.get_logger().warn(
            'team_driver is up, but this template has no driving logic: it will '
            'crawl straight ahead until it hits something. Implement plan().')

    # ------------------------------------------------------------------
    # Where the car is. Free, accurate, and worth building on.
    # ------------------------------------------------------------------
    def reload_params(self):
        """Pick up edits to the yaml file without restarting the node."""
        try:
            mtime = os.path.getmtime(self.params_file)
            if mtime == self.params_mtime:
                return
            self.params_mtime = mtime
            with open(self.params_file) as f:
                cfg = yaml.safe_load(f)['driver']['ros__parameters']
            for k, v in cfg.items():
                if k in TUNABLES:
                    setattr(self, k, type(TUNABLES[k])(v))
            self.get_logger().info('reloaded driver_params.yaml')
        except Exception as e:
            self.get_logger().warn(f'params reload failed: {e}')

    def odom_callback(self, msg):
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)

    def ips_callback(self, msg):
        """Position alone, at the same rate. /odom carries this plus velocity."""
        self.position = (msg.x, msg.y)

    # ------------------------------------------------------------------
    # The control loop, once per LiDAR scan (40 Hz).
    # ------------------------------------------------------------------
    def scan_callback(self, scan):
        ranges, angles = self.preprocess(scan)
        steering, throttle = self.plan(ranges, angles)
        self.publish(steering, throttle)

        self._marker_divisor = (self._marker_divisor + 1) % 10
        if self._marker_divisor == 0:
            self.publish_marker(steering * MAX_STEERING_RAD)

    def preprocess(self, scan):
        """Turn a raw scan into clean ranges plus the angle of each beam.

        Kept because every approach needs some version of it and the details
        are fiddly rather than interesting: the LiDAR reports NaN and inf, and
        arithmetic on those propagates silently through everything downstream.
        """
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        ranges = np.nan_to_num(ranges, nan=0.0, posinf=self.max_range, neginf=0.0)
        ranges = np.clip(ranges, 0.0, self.max_range)

        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        return ranges, angles

    # ==================================================================
    # THIS IS THE PART YOU WRITE.
    # ==================================================================
    def plan(self, ranges, angles):
        """Follow-the-gap with disparity extension, then a PI speed controller."""
        m = np.abs(angles) <= math.radians(self.fov_deg)
        r, a = ranges[m].copy(), angles[m]
        r = np.convolve(r, np.ones(self.smooth_beams) / self.smooth_beams, mode='same')
        da = float(np.mean(np.diff(angles)))

        # Disparity extension: pull the far side of a near/far jump in by the car half width.
        out = r.copy()
        for i in np.where(np.abs(np.diff(r)) > self.disparity_thresh)[0]:
            near, far = (i, i + 1) if r[i] < r[i + 1] else (i + 1, i)
            n = int(math.atan2(self.car_half_width, max(r[near], 0.1)) / da)
            lo, hi = (far, min(far + n, len(r))) if far > near else (max(far - n, 0), far + 1)
            out[lo:hi] = np.minimum(out[lo:hi], r[near])

        # Safety bubble around the closest return.
        c = int(np.argmin(out))
        n = int(math.atan2(self.bubble_radius, max(out[c], 0.1)) / da)
        out[max(c - n, 0):c + n + 1] = 0.0

        # Bearing of the centreline lookahead point, so the car never turns round.
        target = self.track_bearing()

        # Aim inside the free run nearest that bearing (widest run if pose is unknown).
        free = np.append(out > self.gap_min_range, False)
        runs, start = [], None
        for i, f in enumerate(free):
            if f and start is None:
                start = i
            elif not f and start is not None:
                runs.append((start, i - 1))
                start = None
        if not runs:
            runs = [(len(out) // 2, len(out) // 2)]
        if target is None:
            lo, hi = max(runs, key=lambda r: r[1] - r[0])
            angle = a[(lo + hi) // 2]
        else:
            def dist(r):
                return max(a[r[0]] - target, target - a[r[1]], 0.0)
            lo, hi = min(runs, key=dist)
            angle = float(np.clip(target, a[lo], a[hi]))
            m_ = self.edge_margin_beams
            if hi - lo > 2 * m_:
                angle = float(np.clip(angle, a[lo + m_], a[hi - m_]))

        steering = float(np.clip(self.steer_gain * angle / MAX_STEERING_RAD,
                                 -self.max_steer, self.max_steer))
        steering = self.steer_smooth * self.prev_steer + (1 - self.steer_smooth) * steering
        self.prev_steer = steering
        front = float(np.min(ranges[np.abs(angles) < math.radians(self.front_cone_deg)]))
        v_target = (self.v_max * (1 - self.steer_slow * abs(steering))
                    * min(1.0, front / self.brake_dist))
        return steering, self.speed_control(max(v_target, self.v_min))

    def track_bearing(self):
        """Angle in the car frame to the centreline point `lookahead` metres ahead."""
        if self.position is None:
            return None
        d = np.hypot(*(self.line - self.position).T)
        if self.line_i is None:
            self.line_i = int(np.argmin(d))
        else:
            # Search near the previous index only, so the nearest-point match cannot jump laps.
            idx = (self.line_i + np.arange(-5, 15)) % len(self.line)
            self.line_i = int(idx[np.argmin(d[idx])])
        n = len(self.line)
        i, acc = self.line_i, 0.0
        while acc < self.lookahead:
            acc += float(np.hypot(*(self.line[(i + 1) % n] - self.line[i % n])))
            i += 1
        p = self.line[i % n]
        ang = math.atan2(p[1] - self.position[1], p[0] - self.position[0]) - self.yaw
        return math.atan2(math.sin(ang), math.cos(ang))

    def speed_control(self, v_target):
        """Feed-forward + PI with a clamped integral; lift off rather than brake."""
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = 0.025 if self.last_t is None else min(now - self.last_t, 0.1)
        self.last_t = now
        err = v_target - self.speed
        self.integral = float(np.clip(self.integral + err * dt, -self.i_max, self.i_max))
        u = self.ff_gain * v_target + self.kp * err + self.ki * self.integral
        return float(np.clip(u, 0.0, 1.0))

    # ------------------------------------------------------------------
    # Output
    # ------------------------------------------------------------------
    def publish(self, steering, throttle):
        self.steering_pub.publish(Float32(data=float(np.clip(steering, -1.0, 1.0))))
        self.throttle_pub.publish(Float32(data=float(np.clip(throttle, -1.0, 1.0))))

    def publish_marker(self, target_angle):
        """Draw where the car thinks it is going. Add /driver/markers in RViz."""
        marker = Marker()
        marker.header.frame_id = 'roboracer_1'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'team_driver'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x, marker.scale.y, marker.scale.z = 1.5, 0.15, 0.15
        marker.color.g, marker.color.b, marker.color.a = 0.8, 1.0, 0.9
        marker.pose.orientation.z = math.sin(target_angle / 2.0)
        marker.pose.orientation.w = math.cos(target_angle / 2.0)
        array = MarkerArray()
        array.markers.append(marker)
        self.marker_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = Driver()
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
