#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from aimdk_msgs.msg import JointCommandArray, JointStateArray, JointCommand

import ruckig
from enum import Enum
from dataclasses import dataclass
from typing import List, Dict


# ============================================================
# QoS
# ============================================================

subscriber_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE
)

publisher_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE
)


# ============================================================
# JOINT AREA
# ============================================================

class JointArea(Enum):
    ARM = 'ARM'


# ============================================================
# JOINT INFORMATION
# ============================================================

@dataclass
class JointInfo:
    name: str
    lower_limit: float
    upper_limit: float
    kp: float
    kd: float


# ============================================================
# ROBOT MODEL
# ============================================================

robot_model: Dict[JointArea, List[JointInfo]] = {

    JointArea.ARM: [

        JointInfo(
            "left_shoulder_pitch_joint",
            -3.089233, 2.024582,
            20.0, 2.0
        ),

        JointInfo(
            "left_shoulder_roll_joint",
            -0.069813, 3.001966,
            20.0, 2.0
        ),

        JointInfo(
            "left_shoulder_yaw_joint",
            -2.565635, 2.565635,
            20.0, 2.0
        ),

        JointInfo(
            "left_elbow_joint",
            -2.356194, 0.000000,
            40.0, 2.0
        ),

        JointInfo(
            "left_wrist_yaw_joint",
            -2.565635, 2.565635,
            20.0, 2.0
        ),

        JointInfo(
            "left_wrist_pitch_joint",
            -0.558505, 0.558505,
            20.0, 2.0
        ),

        JointInfo(
            "left_wrist_roll_joint",
            -1.570796, 0.733038,
            20.0, 2.0
        ),

        JointInfo(
            "right_shoulder_pitch_joint",
            -3.089233, 2.024582,
            20.0, 2.0
        ),

        JointInfo(
            "right_shoulder_roll_joint",
            -0.069813, 3.001966,
            20.0, 2.0
        ),

        JointInfo(
            "right_shoulder_yaw_joint",
            -2.565635, 2.565635,
            20.0, 2.0
        ),

        JointInfo(
            "right_elbow_joint",
            -2.356194, 0.000000,
            20.0, 2.0
        ),

        JointInfo(
            "right_wrist_yaw_joint",
            -2.565635, 2.565635,
            20.0, 2.0
        ),

        JointInfo(
            "right_wrist_pitch_joint",
            -0.558505, 0.558505,
            20.0, 2.0
        ),

        JointInfo(
            "right_wrist_roll_joint",
            -1.570796, 0.733038,
            20.0, 2.0
        ),
    ],
}


# ============================================================
# CONTROLLER
# ============================================================

class JointControllerNode(Node):

    def __init__(
        self,
        node_name: str,
        sub_topic: str,
        pub_topic: str,
        area: JointArea,
        dofs: int
    ):

        super().__init__(node_name)

        self.joint_info = robot_model[area]
        self.dofs = dofs

        # ----------------------------------------------------
        # Ruckig
        # ----------------------------------------------------

        self.ruckig = ruckig.Ruckig(
            dofs,
            0.002
        )

        self.input = ruckig.InputParameter(dofs)
        self.output = ruckig.OutputParameter(dofs)

        # ----------------------------------------------------
        # Initial state
        # ----------------------------------------------------

        self.input.current_position = [0.0] * dofs
        self.input.current_velocity = [0.0] * dofs
        self.input.current_acceleration = [0.0] * dofs

        # ----------------------------------------------------
        # Target
        # ----------------------------------------------------

        self.input.target_position = [0.0] * dofs
        self.input.target_velocity = [0.0] * dofs
        self.input.target_acceleration = [0.0] * dofs

        # ----------------------------------------------------
        # Ruckig limits
        # ----------------------------------------------------

        self.input.max_velocity = [0.3] * dofs
        self.input.max_acceleration = [0.3] * dofs
        self.input.max_jerk = [0.5] * dofs

        # ----------------------------------------------------
        # ROS2
        # ----------------------------------------------------

        self.sub = self.create_subscription(
            JointStateArray,
            sub_topic,
            self.joint_state_callback,
            subscriber_qos
        )

        self.pub = self.create_publisher(
            JointCommandArray,
            pub_topic,
            publisher_qos
        )

        # ----------------------------------------------------
        # Ruckig status
        # ----------------------------------------------------

        self.trajectory_running = False

        # ----------------------------------------------------
        # Control timer
        #
        # 0.002 sec = 500 Hz
        # ----------------------------------------------------

        self.control_timer = self.create_timer(
            0.002,
            self.control_loop
        )

        self.get_logger().info(
            "X2 multi-joint Ruckig controller started"
        )

    # ========================================================
    # JOINT STATE
    # ========================================================

    def joint_state_callback(self, msg: JointStateArray):

        # For now we only check that state messages arrive.
        # We are NOT using the state to initialize Ruckig yet.

        if len(msg.joints) > 0:
            pass

    # ========================================================
    # SET TARGET
    # ========================================================

    def set_target_positions(self, targets):

        # Start with all joints at zero
        target = [0.0] * self.dofs

        # Change only the requested joints
        for i, joint in enumerate(self.joint_info):

            if joint.name in targets:

                position = targets[joint.name]

                # Check joint limits
                if position < joint.lower_limit or position > joint.upper_limit:

                    self.get_logger().error(
                        f"{joint.name} target {position} is outside limits"
                    )

                    return

                target[i] = position

        # Set Ruckig target
        self.input.target_position = target

        self.input.target_velocity = [0.0] * self.dofs
        self.input.target_acceleration = [0.0] * self.dofs

        self.trajectory_running = True

        self.get_logger().info(
            f"New target: {targets}"
        )

    # ========================================================
    # RUCKIG CONTROL LOOP
    # ========================================================

    def control_loop(self):

        if not self.trajectory_running:
            return

        result = self.ruckig.update(
            self.input,
            self.output
        )

        if result not in [
            ruckig.Result.Working,
            ruckig.Result.Finished
        ]:

            self.get_logger().error(
                f"Ruckig error: {result}"
            )

            self.trajectory_running = False

            return

        # ----------------------------------------------------
        # Publish trajectory point
        # ----------------------------------------------------

        cmd = JointCommandArray()

        for i, joint in enumerate(self.joint_info):

            j = JointCommand()

            j.name = joint.name

            j.position = self.output.new_position[i]
            j.velocity = self.output.new_velocity[i]

            j.effort = 0.0

            j.stiffness = joint.kp
            j.damping = joint.kd

            cmd.joints.append(j)

        self.pub.publish(cmd)

        # ----------------------------------------------------
        # Update Ruckig current state
        # ----------------------------------------------------

        self.input.current_position = list(
            self.output.new_position
        )

        self.input.current_velocity = list(
            self.output.new_velocity
        )

        self.input.current_acceleration = list(
            self.output.new_acceleration
        )

        # ----------------------------------------------------
        # Trajectory finished
        # ----------------------------------------------------

        if result == ruckig.Result.Finished:

            self.trajectory_running = False

            self.get_logger().info(
                "Trajectory finished"
            )


# ============================================================
# MAIN
# ============================================================

def main(args=None):

    rclpy.init(args=args)

    node = JointControllerNode(
        "x2_arm_controller",
        "/aima/hal/joint/arm/state",
        "/aima/hal/joint/arm/command",
        JointArea.ARM,
        14
    )


    # --------------------------------------------------------
    # TEST TARGETS
    # --------------------------------------------------------

    node.set_target_positions({

        # Horizontal rotation
        #"left_shoulder_yaw_joint": 0.9,
        #"left_wrist_yaw_joint": 0.998132,     # +40 degrees
        #"left_wrist_roll_joint": -0.733038, 

        # Vertical elbow
        "left_shoulder_roll_joint": 0.533038            # -15 degrees
    })

    # --------------------------------------------------------
    # Spin
    # --------------------------------------------------------

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

