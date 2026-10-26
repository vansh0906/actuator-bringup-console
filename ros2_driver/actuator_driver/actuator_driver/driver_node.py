"""ROS 2 driver for the CAN smart actuators.

Publishes
  /joint_states        sensor_msgs/JointState   position, velocity, effort (Nm)
  /diagnostics         diagnostic_msgs/DiagnosticArray, 1 Hz, one status per joint
Subscribes
  ~/commands           std_msgs/Float64MultiArray, one setpoint per joint
                       (same shape as a ros2_control forward_command_controller)
Services (std_srvs/Trigger)
  ~/enable  ~/disable  ~/clear_faults  ~/estop
Parameters
  url, node_ids, joint_names, command_mode (velocity|position|current),
  publish_rate, command_timeout, torque_constant
"""

from __future__ import annotations

import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

from bringup import LinkError, connect

from .driver_core import DriverCore


class ActuatorDriver(Node):
    def __init__(self):
        super().__init__("actuator_driver")
        self.declare_parameter("url", "tcp://localhost:29536")
        self.declare_parameter("node_ids", [1, 2])
        self.declare_parameter("joint_names", ["joint1", "joint2"])
        self.declare_parameter("command_mode", "velocity")
        self.declare_parameter("publish_rate", 100.0)
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("torque_constant", 4.0)

        url = self.get_parameter("url").value
        node_ids = list(self.get_parameter("node_ids").value)
        joint_names = list(self.get_parameter("joint_names").value)
        mode = self.get_parameter("command_mode").value
        rate = float(self.get_parameter("publish_rate").value)

        self.link = self.dm = None
        for attempt in range(30):
            try:
                self.link, self.dm, pc = connect(url)
                break
            except LinkError as exc:
                self.get_logger().warn(f"{exc} (retry {attempt + 1}/30)")
                time.sleep(1.0)
        if self.dm is None:
            raise RuntimeError(f"could not connect to {url}")
        self.core = DriverCore(self.dm, pc, node_ids, joint_names, mode,
                               float(self.get_parameter("torque_constant").value),
                               float(self.get_parameter("command_timeout").value))
        self.get_logger().info(f"connected to {url}: joints {joint_names} on CAN nodes {node_ids}, {mode} mode")

        self.js_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Float64MultiArray, "~/commands", self.on_commands, qos)
        self.create_service(Trigger, "~/enable", self._service(self.core.enable))
        self.create_service(Trigger, "~/disable", self._service(self.core.disable))
        self.create_service(Trigger, "~/clear_faults", self._service(self.core.clear_faults))
        self.create_service(Trigger, "~/estop", self._service(self.core.estop))
        self.create_timer(1.0 / rate, self.publish_state)
        self.create_timer(1.0, self.publish_diagnostics)

    def _service(self, fn):
        def handler(request, response):
            response.success, response.message = fn()
            log = self.get_logger().info if response.success else self.get_logger().warn
            log(response.message)
            return response
        return handler

    def on_commands(self, msg: Float64MultiArray) -> None:
        error = self.core.apply_commands(list(msg.data))
        if error:
            self.get_logger().warn(f"command ignored: {error}", throttle_duration_sec=2.0)

    def publish_state(self) -> None:
        if self.core.check_command_timeout():
            self.get_logger().warn("no command for %.2f s: stopping joints" % self.core.command_timeout)
        names, pos, vel, eff = self.core.joint_state()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name, msg.position, msg.velocity, msg.effort = names, pos, vel, eff
        self.js_pub.publish(msg)

    def publish_diagnostics(self) -> None:
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        for d in self.core.diagnostics():
            st = DiagnosticStatus()
            st.level = bytes([d.level])
            st.name, st.message, st.hardware_id = d.name, d.message, d.hardware_id
            st.values = [KeyValue(key=k, value=v) for k, v in d.values.items()]
            arr.status.append(st)
        self.diag_pub.publish(arr)

    def shutdown(self) -> None:
        if self.dm:
            for n in self.core.node_ids:
                self.dm.disable(n)
            time.sleep(0.05)
            self.dm.stop()
        if self.link:
            self.link.close()


def main(args=None):
    rclpy.init(args=args)
    node = ActuatorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
