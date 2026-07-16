"""Bridge simulator ground truth into the Level A WUTA-FSD interfaces."""

from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster
from wuta_msgs.msg import MissionState


class SimulationBridge(Node):
    """Adapt simulator truth to the interfaces needed by WUTA-FSD."""

    def __init__(self) -> None:
        super().__init__("simulation_bridge")

        self.declare_parameter("ground_truth_topic", "/sim/ground_truth")
        self.declare_parameter("publish_mission_state", True)
        self.declare_parameter("publish_truth_localization", False)
        self.declare_parameter("mission_mode", "trackdrive")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")

        ground_truth_topic = str(
            self.get_parameter("ground_truth_topic").value
        )
        self.publish_mission_state = bool(
            self.get_parameter("publish_mission_state").value
        )
        self.publish_truth_localization = bool(
            self.get_parameter("publish_truth_localization").value
        )
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.mission_mode = self._mission_mode_value(
            str(self.get_parameter("mission_mode").value)
        )

        self.pose_pub = self.create_publisher(
            PoseStamped, "/localization/pose", 10
        )
        self.vel_pub = self.create_publisher(
            TwistStamped, "/localization/velocity", 10
        )
        self.localization_ready_pub = self.create_publisher(
            Bool, "/system/localization_ready", 10
        )
        self.lidar_ready_pub = self.create_publisher(
            Bool, "/system/lidar_ready", 10
        )
        self.mission_state_pub = self.create_publisher(
            MissionState, "/system/mission_state", 10
        )
        self.tf_broadcaster = TransformBroadcaster(self)

        self.ground_truth_sub = self.create_subscription(
            Odometry, ground_truth_topic, self._on_ground_truth, 10
        )
        self.mission_complete_sub = self.create_subscription(
            Bool, "/system/mission_complete", self._on_mission_complete, 10
        )
        self.status_timer = self.create_timer(0.1, self._publish_status)
        self.received_ground_truth = False
        self.mission_complete = False

        self.get_logger().info(
            "Simulation bridge waiting for ground truth on %s; truth localization=%s"
            % (ground_truth_topic, self.publish_truth_localization)
        )

    @staticmethod
    def _mission_mode_value(mode: str) -> int:
        values = {
            "trackdrive": MissionState.MISSION_TRACKDRIVE,
            "skidpad": MissionState.MISSION_SKIDPAD,
            "acceleration": MissionState.MISSION_ACCELERATION,
        }
        if mode not in values:
            raise ValueError(
                "mission_mode must be trackdrive, skidpad or acceleration"
            )
        return values[mode]

    def _on_ground_truth(self, msg: Odometry) -> None:
        self.received_ground_truth = True

        if not self.publish_truth_localization:
            return

        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = self.map_frame
        pose.pose = msg.pose.pose
        self.pose_pub.publish(pose)

        vel = TwistStamped()
        vel.header = pose.header
        vel.twist = msg.twist.twist
        self.vel_pub.publish(vel)

        transform = TransformStamped()
        transform.header = pose.header
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = pose.pose.position.x
        transform.transform.translation.y = pose.pose.position.y
        transform.transform.translation.z = pose.pose.position.z
        transform.transform.rotation = pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    def _on_mission_complete(self, msg: Bool) -> None:
        if msg.data and not self.mission_complete:
            self.mission_complete = True
            self.get_logger().info(
                "Received /system/mission_complete; publishing FINISH state"
            )

    def _publish_status(self) -> None:
        lidar_ready = Bool()
        lidar_ready.data = self.received_ground_truth
        self.lidar_ready_pub.publish(lidar_ready)

        if self.publish_truth_localization:
            localization_ready = Bool()
            localization_ready.data = self.received_ground_truth
            self.localization_ready_pub.publish(localization_ready)

        if not self.publish_mission_state:
            return

        state = MissionState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.state = (
            MissionState.FINISH
            if self.mission_complete
            else MissionState.EXPLORE
        )
        state.mission_mode = self.mission_mode
        state.localization_mode = MissionState.LOC_KISS_ICP
        state.description = (
            "simulation skidpad complete"
            if self.mission_complete
            else "simulation auto-start"
        )
        self.mission_state_pub.publish(state)


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node: Optional[SimulationBridge] = None
    try:
        node = SimulationBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
