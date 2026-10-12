# Validation Report: JAXV2_Assembly

## Status: PASS

## Summary

| Metric | Value |
|--------|-------|
| Links | 16 |
| Joints | 15 |
| Assemblies | 1 |
| Root | `base_link` |
| Errors | 0 |
| Warnings | 0 |

## Kinematic Tree

```
base_link [primitive]
  └─ base_link_to_shoulder_pitch [fixed]
    Shoulder_Pitch [primitive]
      └─ shoulder_pitch_to_shoulder_roll [revolute]
        Shoulder_Roll [BAKE] [primitive]
          └─ shoulder_roll_to_shoulder_yaw [revolute]
            Shoulder_Yaw [BAKE] [primitive]
              └─ shoulder_yaw_to_upper_arm [revolute]
                Upper_Arm [BAKE] [primitive]
                  └─ upper_arm_to_under_arm [revolute]
                    Under_Arm [BAKE] [primitive]
                      └─ under_arm_to_wrist_pitch [revolute]
                        Wrist_Pitch [BAKE] [primitive]
                          └─ wrist_pitch_to_wrist_yaw [revolute]
                            Wrist_yaw [BAKE] [primitive]
                              └─ wrist_yaw_to_hand_base [revolute]
                                MG996R [BAKE] [primitive]
                                  └─ hand_base_to_rotation_base [revolute]
                                    RotationBody [BAKE] [primitive]
                                      └─ rotation_base_to_finger_3_1 [revolute]
                                        Finger_3_1 [BAKE] [primitive]
                                          └─ finger_3_1_to_finger_3_2 [revolute]
                                            Finger_3_2 [BAKE] [primitive]
                                  └─ hand_base_to_finger_1_1 [revolute]
                                    Finger_1_1 [BAKE] [primitive]
                                      └─ finger_1_1_to_finger_1_2 [revolute]
                                        Finger_1_2 [BAKE] [primitive]
                                  └─ hand_base_to_finger_2_1 [revolute]
                                    Finger_2_1 [BAKE] [primitive]
                                      └─ finger_1_2_to_finger_2_2 [revolute]
                                        Finger_2_2 [BAKE] [primitive]
```

## Collision Geometry

| Link | Source | Shape/File |
|------|--------|------------|
| `Finger_1_1` | primitive STL | box |
| `Finger_1_2` | primitive STL | box |
| `Finger_2_1` | primitive STL | box |
| `Finger_2_2` | primitive STL | box |
| `Finger_3_1` | primitive STL | box |
| `Finger_3_2` | primitive STL | box |
| `MG996R` | primitive STL | box |
| `RotationBody` | primitive STL | box |
| `Shoulder_Pitch` | primitive STL | sphere |
| `Shoulder_Roll` | primitive STL | sphere |
| `Shoulder_Yaw` | primitive STL | box |
| `Under_Arm` | primitive STL | box |
| `Upper_Arm` | primitive STL | box |
| `Wrist_Pitch` | primitive STL | box |
| `Wrist_yaw` | primitive STL | box |
| `base_link` | primitive STL | box |

## Mesh Bake Offsets

Links where joint frame ≠ component origin. Visual/inertial/collision origins shifted.

| Link | Offset (mm) |
|------|-------------|
| `Shoulder_Roll` | (0.0, 0.0, 0.0) |
| `Shoulder_Yaw` | (0.0, 0.0, -4.5) |
| `Upper_Arm` | (0.0, 0.0, 0.0) |
| `Under_Arm` | (0.0, -0.0, 0.0) |
| `Wrist_yaw` | (0.0, 0.0, 0.0) |
| `Wrist_Pitch` | (0.0, 0.0, -1.0) |
| `MG996R` | (-13.5, 9.2, -57.1) |
| `RotationBody` | (15.5, 34.6, 10.9) |
| `Finger_1_1` | (6.5, 15.2, -42.8) |
| `Finger_1_2` | (6.7, 79.7, -43.1) |
| `Finger_2_1` | (6.5, 15.3, -5.8) |
| `Finger_2_2` | (6.5, 80.2, -5.8) |
| `Finger_3_1` | (36.5, 15.2, -10.8) |
| `Finger_3_2` | (36.5, 80.2, -10.8) |
