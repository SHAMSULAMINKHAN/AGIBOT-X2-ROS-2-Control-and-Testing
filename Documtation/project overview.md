# AGIBOT X2 ROS 2 Control

This project is about controlling and testing the **AGIBOT X2 humanoid robot** using **ROS 2** and Python.

The current work focuses on controlling the robot's **arm joints**.

## What We Are Doing

The main steps are:

1. Check the robot's ROS 2 topics.
2. Read the robot's joint state.
3. Send joint position commands.
4. Test one joint at a time.
5. Test multiple joints.
6. Use **Ruckig** for smooth movement.

## Robot

The current controller works with the X2's **14 arm joints**:

* 7 left-arm joints
* 7 right-arm joints

## ROS 2 Topics

We mainly use:

```text
/aima/hal/joint/arm/state
```

to read the arm state.

```text
/aima/hal/joint/arm/command
```

to send commands to the arm.

## Controller

The controller uses:

* **ROS 2**
* **Python**
* **rclpy**
* **AIMDK messages**
* **Ruckig**

Ruckig generates smooth joint positions and velocities before they are sent to the robot.

## Current Status

The project is currently in the **testing stage**.

We are first understanding how the X2 receives commands and how the joints respond before building more advanced control.
