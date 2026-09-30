# AGIBOT X2 ROS 2 Control and Testing

## 1. Project Overview

This project focuses on developing and testing ROS 2-based joint control for the **AGIBOT X2 humanoid robot**.

The initial goal is to understand the robot's ROS 2 communication interface and safely test joint-level position commands before moving toward more advanced motion control.

The project uses Python and ROS 2 to communicate with the robot's joint controllers. **Ruckig** is used to generate smooth joint trajectories.

---

## 2. Main Objectives

The project is being developed in stages:

1. Understand the AGIBOT X2 ROS 2 interface.
2. Identify the available joint command and state topics.
3. Monitor the robot's current joint state.
4. Test communication between the ROS 2 controller and the robot.
5. Control individual arm joints.
6. Extend the controller to multiple joints.
7. Generate smooth trajectories using Ruckig.
8. Understand the controller's behavior during and after a trajectory.
9. Develop a reliable foundation for more advanced robot control.

---

## 3. Robot Control Area

The current controller focuses on the **14 arm joints** of the X2.

The arm contains seven joints on each side:

### Left Arm

* `left_shoulder_pitch_joint`
* `left_shoulder_roll_joint`
* `left_shoulder_yaw_joint`
* `left_elbow_joint`
* `left_wrist_yaw_joint`
* `left_wrist_pitch_joint`
* `left_wrist_roll_joint`

### Right Arm

* `right_shoulder_pitch_joint`
* `right_shoulder_roll_joint`
* `right_shoulder_yaw_joint`
* `right_elbow_joint`
* `right_wrist_yaw_joint`
* `right_wrist_pitch_joint`
* `right_wrist_roll_joint`

## The controller defines joint limits and proportional/derivative gains for each joint.

## 4. ROS 2 Communication

The current controller communicates with the X2 arm through two main ROS 2 topics:

### Joint State

```text
/aima/hal/joint/arm/state
```

This topic is subscribed to by the controller to receive arm joint-state information.

### Joint Command

```text
/aima/hal/joint/arm/command
```

The controller publishes joint commands to this topic.

The current implementation creates a `JointStateArray` subscriber and a `JointCommandArray` publisher for these topics.

---

## 5. Trajectory Generation

The controller uses **Ruckig** for online trajectory generation.

The current configuration uses:

```text
Control period:       0.002 seconds
Control frequency:    500 Hz
Maximum velocity:     0.3
Maximum acceleration: 0.3
Maximum jerk:         0.5
```

These limits are applied to all 14 controlled joints.

Ruckig generates a new trajectory point during each control-cycle update. The generated position and velocity are then placed into a `JointCommandArray` and published to the robot.

---

## 6. Current Development Stage

The project is currently in the **testing and understanding stage**.

The controller has been structured to:

```text
Target Joint Position
        ↓
Joint Limit Check
        ↓
Ruckig Trajectory Generation
        ↓
500 Hz Control Loop
        ↓
JointCommandArray
        ↓
X2 Arm Command Topic
        ↓
Robot
```

At this stage, the controller's internal Ruckig state is initialized to zero rather than being initialized from the robot's actual joint positions. The joint-state callback currently only checks whether state messages are received.
This behavior is important and will be discussed in detail in the controller documentation.

---

## 7. Example Test

The current test configuration sends a target to:

```text
left_shoulder_roll_joint
```

with a target position of:

```text
0.533038 rad
```

Other test targets are currently commented out in the source code.

---


