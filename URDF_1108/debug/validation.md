# Validation Report: JAXV3_Assembly

## Status: PASS

## Summary

| Metric | Value |
|--------|-------|
| Links | 17 |
| Joints | 16 |
| Assemblies | 1 |
| Root | `base_link` |
| Errors | 0 |
| Warnings | 0 |

## Kinematic Tree

```
base_link [primitive]
  └─ base_link_to_shoulder_pitch [fixed]
    Shoulder_Pitch_1 [primitive]
      └─ shoulder_pitch_to_shoulder_roll [revolute]
        Shoulder_Roll_1 [BAKE] [primitive]
          └─ shoulder_roll_to_shoulder_yaw [revolute]
            Shoulder_Yaw_1 [BAKE] [primitive]
              └─ shoulder_yaw_to_upper_arm [revolute]
                Upper_Arm_1 [BAKE] [primitive]
                  └─ upper_arm_to_under_arm [revolute]
                    Under_Arm_1 [BAKE] [primitive]
                      └─ under_arm_to_wrist_pitch [revolute]
                        Wrist_Pitch_1 [BAKE] [primitive]
                          └─ wrist_pitch_to_wrist_yaw [revolute]
                            Wrist_yaw_1 [BAKE] [primitive]
                              └─ wrist_yaw_to_hand_base [revolute]
                                MG996R_1 [BAKE] [primitive]
                                  └─ hand_base_to_rotation_base [revolute]
                                    RotationBody_1 [BAKE] [primitive]
                                      └─ rotation_base_to_finger_3_1 [revolute]
                                        Finger_3_1_1 [BAKE] [primitive]
                                          └─ finger_3_1_to_finger_3_2 [revolute]
                                            Finger_3_2_1 [BAKE] [primitive]
                                  └─ hand_base_to_finger_1_1 [revolute]
                                    Finger_1_1_1 [BAKE] [primitive]
                                      └─ finger_1_1_to_finger_1_2 [revolute]
                                        Finger_1_2_1 [BAKE] [primitive]
                                  └─ hand_base_to_finger_2_1 [revolute]
                                    Finger_2_1_1 [BAKE] [primitive]
                                      └─ finger_1_2_to_finger_2_2 [revolute]
                                        Finger_2_2_1 [BAKE] [primitive]
  └─ base_link_to_holder [fixed]
    Holder_1 [primitive]
```

## Collision Geometry

| Link | Source | Shape/File |
|------|--------|------------|
| `Finger_1_1_1` | primitive STL | cylinder |
| `Finger_1_2_1` | primitive STL | box |
| `Finger_2_1_1` | primitive STL | cylinder |
| `Finger_2_2_1` | primitive STL | box |
| `Finger_3_1_1` | primitive STL | cylinder |
| `Finger_3_2_1` | primitive STL | box |
| `Holder_1` | primitive STL | box |
| `MG996R_1` | primitive STL | box |
| `RotationBody_1` | primitive STL | cylinder |
| `Shoulder_Pitch_1` | primitive STL | box |
| `Shoulder_Roll_1` | primitive STL | sphere |
| `Shoulder_Yaw_1` | primitive STL | box |
| `Under_Arm_1` | primitive STL | box |
| `Upper_Arm_1` | primitive STL | box |
| `Wrist_Pitch_1` | primitive STL | box |
| `Wrist_yaw_1` | primitive STL | box |
| `base_link` | primitive STL | box |

## Mesh Bake Offsets

Links where joint frame ≠ component origin. Visual/inertial/collision origins shifted.

| Link | Offset (mm) |
|------|-------------|
| `Shoulder_Roll_1` | (0.0, 0.0, 0.0) |
| `Shoulder_Yaw_1` | (0.0, 0.0, 4.5) |
| `Upper_Arm_1` | (0.0, 0.0, 0.0) |
| `Under_Arm_1` | (0.0, -0.0, 0.0) |
| `Wrist_yaw_1` | (0.0, 0.0, 0.0) |
| `Wrist_Pitch_1` | (0.0, 0.0, 0.0) |
| `MG996R_1` | (-13.5, 9.2, -53.6) |
| `RotationBody_1` | (15.5, 34.6, 10.9) |
| `Finger_1_1_1` | (6.5, 15.2, -42.8) |
| `Finger_1_2_1` | (6.7, 79.7, -43.1) |
| `Finger_2_1_1` | (6.5, 15.2, -5.8) |
| `Finger_2_2_1` | (6.5, 80.2, -5.8) |
| `Finger_3_1_1` | (36.5, 15.2, -10.8) |
| `Finger_3_2_1` | (36.5, 80.2, -10.8) |
