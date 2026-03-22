import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from px4_msgs.msg import TrajectorySetpoint, OffboardControlMode, VehicleCommand
import numpy as np
import math

NUM_SECTORS = 72
SECTOR_ANGLE = 2 * math.pi / NUM_SECTORS
THRESHOLD_HIGH = 3
MIN_VALLEY_WIDTH = 4
CRUISE_SPEED = 0.3

class VFHPlanner(Node):
    def __init__(self):
        super().__init__("vfh_planner")
        self.sub_costmap = self.create_subscription(
            OccupancyGrid, "/costmap/occupancy", self.costmap_callback, 10)
        self.pub_trajectory = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10)
        self.pub_offboard = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", 10)
        self.pub_command = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", 10)
        self.timer = self.create_timer(0.1, self.heartbeat)
        self.counter = 0
        self.state = "init"  # init -> arming -> climbing -> navigating
        self.climb_ticks = 0
        self.get_logger().info("VFH+ planner ready.")

    def heartbeat(self):
        # Always publish offboard heartbeat
        ocm = OffboardControlMode()
        ocm.position = False
        ocm.velocity = True
        ocm.acceleration = False
        ocm.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_offboard.publish(ocm)

        self.counter += 1

        if self.state == "init":
            # Send some heartbeats first before switching mode
            if self.counter == 10:
                self.set_offboard_mode()
            if self.counter == 12:
                self.arm()
                self.state = "climbing"
                self.get_logger().info("Armed — climbing.")

        elif self.state == "climbing":
            traj = TrajectorySetpoint()
            traj.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            traj.velocity[0] = float("nan")
            traj.velocity[1] = float("nan")
            traj.velocity[2] = -0.5
            traj.yaw = 0.0
            self.pub_trajectory.publish(traj)
            self.climb_ticks += 1
            if self.climb_ticks >= 50:  # 5 seconds at 10Hz
                self.state = "navigating"
                self.get_logger().info("Takeoff complete — navigating.")

    def set_offboard_mode(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.param1 = 1.0
        msg.param2 = 6.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_command.publish(msg)
        self.get_logger().info("Offboard mode set.")

    def arm(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 1.0
        msg.param2 = 21196.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_command.publish(msg)

    def costmap_callback(self, msg):
        if self.state != "navigating":
            return

        grid = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        resolution = msg.info.resolution
        center = msg.info.width // 2

        histogram = np.zeros(NUM_SECTORS)
        for row in range(msg.info.height):
            for col in range(msg.info.width):
                if grid[row, col] > 50:
                    dx = (col - center) * resolution
                    dy = row * resolution
                    angle = math.atan2(dy, dx)
                    sector = int((angle + math.pi) / SECTOR_ANGLE) % NUM_SECTORS
                    histogram[sector] += 1

        binary = (histogram < THRESHOLD_HIGH).astype(int)
        best_sector = self.find_best_valley(binary)

        traj = TrajectorySetpoint()
        traj.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        if best_sector is None:
            traj.velocity[0] = 0.0
            traj.velocity[1] = 0.0
            traj.velocity[2] = 0.0
            traj.yaw = 0.0
            self.get_logger().warn("No free valley — hovering.")
        else:
            target_angle = (best_sector - NUM_SECTORS // 2) * SECTOR_ANGLE
            traj.velocity[0] = CRUISE_SPEED * math.cos(target_angle)
            traj.velocity[1] = CRUISE_SPEED * math.sin(target_angle)
            traj.velocity[2] = 0.0
            traj.yaw = float("nan")
            self.get_logger().info(
                f"Steering to sector {best_sector}, angle {math.degrees(target_angle):.1f}deg")

        self.pub_trajectory.publish(traj)

    def find_best_valley(self, binary):
        goal_sector = NUM_SECTORS // 2
        best_sector = None
        best_score = float("inf")
        i = 0
        while i < NUM_SECTORS:
            if binary[i] == 1:
                j = i
                while j < NUM_SECTORS and binary[j] == 1:
                    j += 1
                width = j - i
                if width >= MIN_VALLEY_WIDTH:
                    mid = (i + j) // 2
                    score = abs(mid - goal_sector)
                    if score < best_score:
                        best_score = score
                        best_sector = mid
                i = j
            else:
                i += 1
        return best_sector

def main(args=None):
    rclpy.init(args=args)
    node = VFHPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
