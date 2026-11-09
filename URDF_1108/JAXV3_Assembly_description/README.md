# JAXV3_Assembly — Robot Description

![JAXV3_Assembly](images/robot.png)

## Overview

| Property | Value |
|----------|-------|
| Total mass | 143.370 kg |
| Links | 17 |
| Joints | 16 (14 movable) |
| Assemblies | 1 |
| Root link | `base_link` |

## Table of Contents

- [Kinematic Tree](#kinematic-tree)
- [Link Properties](#link-properties)
- [Joint Properties](#joint-properties)
- [Assembly Breakdown](#assembly-breakdown)
- [Quick Start (ROS 2)](#quick-start-ros-2)
- [Files](#files)

## Kinematic Tree

```
base_link
  └─ base_link_to_shoulder_pitch [fixed]
    Shoulder_Pitch_1
      └─ shoulder_pitch_to_shoulder_roll [revolute]
        Shoulder_Roll_1 [BAKE]
          └─ shoulder_roll_to_shoulder_yaw [revolute]
            Shoulder_Yaw_1 [BAKE]
              └─ shoulder_yaw_to_upper_arm [revolute]
                Upper_Arm_1 [BAKE]
                  └─ upper_arm_to_under_arm [revolute]
                    Under_Arm_1 [BAKE]
                      └─ under_arm_to_wrist_pitch [revolute]
                        Wrist_Pitch_1 [BAKE]
                          └─ wrist_pitch_to_wrist_yaw [revolute]
                            Wrist_yaw_1 [BAKE]
                              └─ wrist_yaw_to_hand_base [revolute]
                                MG996R_1 [BAKE]
                                  └─ hand_base_to_rotation_base [revolute]
                                    RotationBody_1 [BAKE]
                                      └─ rotation_base_to_finger_3_1 [revolute]
                                        Finger_3_1_1 [BAKE]
                                          └─ finger_3_1_to_finger_3_2 [revolute]
                                            Finger_3_2_1 [BAKE]
                                  └─ hand_base_to_finger_1_1 [revolute]
                                    Finger_1_1_1 [BAKE]
                                      └─ finger_1_1_to_finger_1_2 [revolute]
                                        Finger_1_2_1 [BAKE]
                                  └─ hand_base_to_finger_2_1 [revolute]
                                    Finger_2_1_1 [BAKE]
                                      └─ finger_1_2_to_finger_2_2 [revolute]
                                        Finger_2_2_1 [BAKE]
  └─ base_link_to_holder [fixed]
    Holder_1
```

## Link Properties

| Link | Mass (kg) | Material | Collision | Bodies |
|------|-----------|----------|-----------|--------|
| `Finger_1_1_1` | 0.0203 | Acetal_Resin_White | cylinder | 1 |
| `Finger_1_2_1` | 0.0113 | Acetal_Resin_White | box | 1 |
| `Finger_2_1_1` | 0.0203 | Acetal_Resin_White | cylinder | 1 |
| `Finger_2_2_1` | 0.0113 | Acetal_Resin_White | box | 1 |
| `Finger_3_1_1` | 0.0203 | Acetal_Resin_White | cylinder | 1 |
| `Finger_3_2_1` | 0.0113 | Acetal_Resin_White | box | 1 |
| `Holder_1` | 3.3311 | Steel | box | 2 |
| `MG996R_1` | 0.0995 | Generic | box | 1 |
| `RotationBody_1` | 0.0233 | Acetal_Resin_White | cylinder | 1 |
| `Shoulder_Pitch_1` | 0.9453 | Steel | box | 1 |
| `Shoulder_Roll_1` | 0.2628 | Acetal_Resin_White | sphere | 1 |
| `Shoulder_Yaw_1` | 0.3205 | Acetal_Resin_White | box | 1 |
| `Under_Arm_1` | 0.3319 | Acetal_Resin_White | box | 1 |
| `Upper_Arm_1` | 0.3594 | Acetal_Resin_White | box | 1 |
| `Wrist_Pitch_1` | 0.1892 | Acetal_Resin_White | box | 3 |
| `Wrist_yaw_1` | 0.1457 | Acetal_Resin_White | box | 1 |
| `base_link` | 137.2661 | Steel | box | 1 |

## Joint Properties

| Joint | Type | Parent → Child | Axis | Limits |
|-------|------|---------------|------|--------|
| `base_link_to_holder` | fixed | `base_link` → `Holder_1` | (0,0,1) | — |
| `base_link_to_shoulder_pitch` | fixed | `base_link` → `Shoulder_Pitch_1` | (0,0,1) | — |
| `finger_1_1_to_finger_1_2` | revolute | `Finger_1_1_1` → `Finger_1_2_1` | (0,0,1) | [0.0°, 120.0°] |
| `finger_1_2_to_finger_2_2` | revolute | `Finger_2_1_1` → `Finger_2_2_1` | (0,0,1) | [0.0°, 120.0°] |
| `finger_3_1_to_finger_3_2` | revolute | `Finger_3_1_1` → `Finger_3_2_1` | (0,0,1) | [0.0°, 120.0°] |
| `hand_base_to_finger_1_1` | revolute | `MG996R_1` → `Finger_1_1_1` | (0,0,1) | [-90.0°, 90.0°] |
| `hand_base_to_finger_2_1` | revolute | `MG996R_1` → `Finger_2_1_1` | (0,0,1) | [-90.0°, 90.0°] |
| `hand_base_to_rotation_base` | revolute | `MG996R_1` → `RotationBody_1` | (0,0,1) | [-180.0°, 0.0°] |
| `rotation_base_to_finger_3_1` | revolute | `RotationBody_1` → `Finger_3_1_1` | (0,0,1) | [-90.0°, 90.0°] |
| `shoulder_pitch_to_shoulder_roll` | revolute | `Shoulder_Pitch_1` → `Shoulder_Roll_1` | (0,0,1) | [-45.0°, 45.0°] |
| `shoulder_roll_to_shoulder_yaw` | revolute | `Shoulder_Roll_1` → `Shoulder_Yaw_1` | (0,0,1) | [-50.0°, 0.0°] |
| `shoulder_yaw_to_upper_arm` | revolute | `Shoulder_Yaw_1` → `Upper_Arm_1` | (0,0,1) | [-90.0°, 240.0°] |
| `under_arm_to_wrist_pitch` | revolute | `Under_Arm_1` → `Wrist_Pitch_1` | (0,0,1) | [-90.0°, 90.0°] |
| `upper_arm_to_under_arm` | revolute | `Upper_Arm_1` → `Under_Arm_1` | (0,0,1) | [-100.0°, 100.0°] |
| `wrist_pitch_to_wrist_yaw` | revolute | `Wrist_Pitch_1` → `Wrist_yaw_1` | (0,0,1) | [-90.0°, 90.0°] |
| `wrist_yaw_to_hand_base` | revolute | `Wrist_yaw_1` → `MG996R_1` | (0,0,1) | [-120.0°, 120.0°] |

## Assembly Breakdown

### JAXV3_Assembly

- **Links**: base_link, Shoulder_Pitch_1, Shoulder_Roll_1, Shoulder_Yaw_1, Upper_Arm_1, Under_Arm_1, Wrist_yaw_1, Wrist_Pitch_1, MG996R_1, RotationBody_1, Finger_1_1_1, Finger_1_2_1, Finger_2_1_1, Finger_2_2_1, Finger_3_1_1, Finger_3_2_1, Holder_1
- **Total mass**: 143.370 kg

## Quick Start (ROS 2)

```bash
# 1. Copy package to your ROS 2 workspace
cp -r JAXV3_Assembly_description ~/ros2_ws/src/

# 2. Build
cd ~/ros2_ws
colcon build --packages-select JAXV3_Assembly_description
source install/setup.bash

# 3. Visualize in RViz2
ros2 launch JAXV3_Assembly_description display.launch.py

# 4. Validate URDF structure
check_urdf install/JAXV3_Assembly_description/share/JAXV3_Assembly_description/urdf/JAXV3_Assembly.urdf

# 5. Print kinematic tree
urdf_to_graphviz install/JAXV3_Assembly_description/share/JAXV3_Assembly_description/urdf/JAXV3_Assembly.urdf
```

**Joint control**: The launch file includes `joint_state_publisher_gui` —
use the sliders to move revolute/prismatic joints in RViz2.

**Topic inspection**:
```bash
# See published joint states
ros2 topic echo /joint_states

# See robot description parameter
ros2 param get /robot_state_publisher robot_description
```

## Files

| Path | Description |
|------|-------------|
| `urdf/JAXV3_Assembly.urdf.xacro` | Top-level xacro (entry point) |
| `urdf/JAXV3_Assembly.urdf` | Flat URDF (for validation) |
| `urdf/assemblies/` | Per-assembly xacro macros |
| `meshes/` | Visual (OBJ) and collision (STL) meshes |
| `launch/display.launch.py` | Launch robot_state_publisher, RViz, and generated controllers |
| `config/joint_state.yaml` | Joint state publisher config |
| `config/ros2_controllers.yaml` | Generated ros2_control controller manager config |
| `robot_data.yaml` | Supplementary data (beyond URDF) |
| `docs/transforms.md` | Transformation matrices (KaTeX) |

## Customizing

Assemblies tagged `!dummy_` are designed to be swapped out. To replace one:

1. Create your replacement as a xacro macro with the same interface
2. Place it in `urdf/assemblies/`
3. Update the `<xacro:include>` in `urdf/JAXV3_Assembly.urdf.xacro`
4. Update meshes in `meshes/<your_assembly>/`

The xacro prefix system (`${prefix}`) ensures link names stay unique
when multiple instances of the same assembly are used.

---
*Generated by Fusion URDF/XACRO Exporter v3.1.1*