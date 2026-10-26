"""Commands joint1 at 0.3 rad/s for 2 s through the driver and checks that
/joint_states shows the motion and /diagnostics reports both joints."""

import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class Check(Node):
    def __init__(self):
        super().__init__("ros_check")
        self.js = None
        self.diag = None
        self.create_subscription(JointState, "/joint_states", lambda m: setattr(self, "js", m), 10)
        self.create_subscription(DiagnosticArray, "/diagnostics", lambda m: setattr(self, "diag", m), 10)
        self.pub = self.create_publisher(Float64MultiArray, "/actuator_driver/commands", 10)


def spin_for(node, seconds, publish=None):
    end = time.time() + seconds
    while time.time() < end:
        if publish is not None:
            node.pub.publish(Float64MultiArray(data=publish))
        rclpy.spin_once(node, timeout_sec=0.02)


def main():
    rclpy.init()
    node = Check()
    spin_for(node, 2.0)
    if node.js is None or node.diag is None:
        print("FAIL: no /joint_states or /diagnostics received")
        return 1
    start = node.js.position[0]
    spin_for(node, 2.0, publish=[0.3, 0.0])
    moved = node.js.position[0] - start
    vel = node.js.velocity[0]
    names = [s.name for s in node.diag.status]
    print(f"joint1 moved {moved:+.3f} rad, velocity {vel:+.3f} rad/s; diagnostics: {names}")
    rclpy.shutdown()
    ok = 0.45 < moved < 0.75 and abs(vel - 0.3) < 0.05 and len(names) == 2
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
