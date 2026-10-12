"""Exercise the built bridge over isolated ROS topics; no hardware or FSD nodes."""
import math
import os
from pathlib import Path
import subprocess
import time

import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TwistStamped
from msg_interfaces.msg import Hcinspvatzcb
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor


@pytest.fixture(scope='module')
def bridge():
    # Remaps prevent these synthetic measurements from reaching a running vehicle.
    root = '/bridge_velocity_test_{}'.format(os.getpid())
    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = rclpy.create_node('bridge_velocity_test_{}'.format(os.getpid()), context=context)
    executor = SingleThreadedExecutor(context=context)
    received = {'odom': [], 'velocity': []}
    pub = node.create_publisher(Hcinspvatzcb, root + '/input', 10)
    node.create_subscription(Odometry, root + '/odom', received['odom'].append, 10)
    node.create_subscription(TwistStamped, root + '/velocity', received['velocity'].append, 10)
    binary = Path(get_package_prefix('chcnav')) / 'lib/chcnav/ChcnavFsdBridge'
    process = subprocess.Popen([
        str(binary), '--ros-args', '-r', '__node:=bridge_velocity_under_test',
        '-r', '/chcnav/devpvt:=' + root + '/input',
        '-r', '/chcnav/odometry:=' + root + '/odom',
        '-r', '/chcnav/velocity:=' + root + '/velocity',
    ])
    try:
        deadline = time.monotonic() + 12.0
        while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            assert process.poll() is None, 'bridge exited before discovery'
            rclpy.spin_once(node, executor=executor, timeout_sec=0.05)
        assert pub.get_subscription_count() == 1
        yield node, pub, received, executor
    finally:
        process.terminate()
        try:
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)


def measurement(yaw, enu, tag):
    msg = Hcinspvatzcb()
    msg.header.stamp.sec = 1791708854
    msg.header.stamp.nanosec = tag
    msg.latitude, msg.longitude, msg.altitude = 30.516974510, 114.348349426, 12.492
    msg.stat = [2, 4]
    msg.yaw = float(yaw)
    msg.enu_velocity.x, msg.enu_velocity.y, msg.enu_velocity.z = map(float, enu)
    msg.position_stdev = [0.03, 0.04, 0.05]
    msg.euler_stdev = [0.1, 0.1, 0.1]
    msg.enu_velocity_stdev = [0.2, 0.4, 0.6]
    # Regression: the device's vehicle_linear_velocity remains zero.
    return msg


def send(bridge, msg):
    node, pub, received, executor = bridge
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        pub.publish(msg)
        for _ in range(4):
            rclpy.spin_once(node, executor=executor, timeout_sec=0.02)
        odom = next((x for x in received['odom']
                     if x.header.stamp == msg.header.stamp), None)
        velocity = next((x for x in received['velocity']
                         if x.header.stamp == msg.header.stamp), None)
        if odom is not None and velocity is not None:
            return odom, velocity
    pytest.fail('both bridge outputs were not received')


@pytest.mark.parametrize('yaw,enu,expected,tag', [
    (0, (0, 2, 0), (2, 0, 0), 1),       # north, forward
    (-90, (2, 0, 0), (2, 0, 0), 2),     # east, forward
    (180, (0, -2, 0), (2, 0, 0), 3),    # south, forward
    (90, (-2, 0, 0), (2, 0, 0), 4),     # west, forward
    (0, (0, -2, 0), (-2, 0, 0), 5),     # north-facing, reverse
    (0, (-1, 2, 0.3), (2, 1, 0.3), 6), # forward + left + up
])
def test_signed_velocity_and_shared_outputs(bridge, yaw, enu, expected, tag):
    odom, velocity = send(bridge, measurement(yaw, enu, tag))
    v = velocity.twist.linear
    assert (v.x, v.y, v.z) == pytest.approx(expected, abs=1e-6)
    assert velocity.twist == odom.twist.twist
    assert odom.header.stamp == velocity.header.stamp
    assert odom.header.frame_id == 'odom'
    assert odom.child_frame_id == velocity.header.frame_id == 'base_link'
    q = odom.pose.pose.orientation
    # The forward vector from the published pose matches the physical heading.
    forward = (1 - 2 * (q.y * q.y + q.z * q.z), 2 * (q.x * q.y + q.w * q.z))
    physical_forward = {0: (0, 1), -90: (1, 0), 180: (0, -1), 90: (-1, 0)}[yaw]
    assert forward == pytest.approx(physical_forward, abs=1e-6)


def test_covariance_rotates_with_velocity(bridge):
    odom, _ = send(bridge, measurement(-45, (1, 1, 0), 7))
    c = odom.twist.covariance
    assert c[0] == pytest.approx(0.10, abs=1e-6)
    assert c[7] == pytest.approx(0.10, abs=1e-6)
    assert c[1] == pytest.approx(0.06, abs=1e-6)
    assert c[6] == pytest.approx(c[1])
    assert c[14] == pytest.approx(0.36, abs=1e-6)


def test_run_log_sample_no_longer_reports_zero_speed(bridge):
    # devpvt #336, 2026-10-11 16:54:14.561611; speed=1.729, vehicle velocity=0.
    msg = measurement(-1.028, (-0.058, 1.728, 0.012), 8)
    msg.roll, msg.pitch = -1.241, -0.633
    _, velocity = send(bridge, msg)
    v = velocity.twist.linear
    assert 1.72 < v.x < 1.74
    # heading2=1.028°, course=358.080°: a small leftward component is real.
    assert 0.08 < v.y < 0.10
    assert math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z) == pytest.approx(
        math.sqrt(0.058**2 + 1.728**2 + 0.012**2), abs=1e-6)


@pytest.mark.parametrize('field,tag', [('enu_velocity', 9), ('yaw', 10)])
def test_nonfinite_measurements_do_not_publish(bridge, field, tag):
    node, pub, received, executor = bridge
    msg = measurement(0, (0, 2, 0), tag)
    if field == 'enu_velocity':
        msg.enu_velocity.x = float('nan')
    else:
        msg.yaw = float('nan')
    deadline = time.monotonic() + 0.4
    while time.monotonic() < deadline:
        pub.publish(msg)
        rclpy.spin_once(node, executor=executor, timeout_sec=0.02)
    for topic in received.values():
        assert not any(x.header.stamp == msg.header.stamp for x in topic)
