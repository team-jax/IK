# Transformation Matrices - JAXV2_Assembly

Homogeneous transformation matrices between consecutive frames.
Convention: URDF RPY (XYZ extrinsic / ZYX intrinsic).

## Notation

### Frames

| Index | Link |
|-------|------|
| $L_{0}$ | base_link |
| $L_{1}$ | Shoulder_Pitch |
| $L_{2}$ | Shoulder_Roll |
| $L_{3}$ | Shoulder_Yaw |
| $L_{4}$ | Upper_Arm |
| $L_{5}$ | Under_Arm |
| $L_{6}$ | Wrist_Pitch |
| $L_{7}$ | Wrist_yaw |
| $L_{8}$ | MG996R |
| $L_{9}$ | RotationBody |
| $L_{10}$ | Finger_1_1 |
| $L_{11}$ | Finger_2_1 |
| $L_{12}$ | Finger_3_1 |
| $L_{13}$ | Finger_1_2 |
| $L_{14}$ | Finger_2_2 |
| $L_{15}$ | Finger_3_2 |

### Joint Variables

| Variable | Joint | Type | From | To |
|----------|-------|------|------|----|
| $q_{1}$ | shoulder_pitch_to_shoulder_roll | revolute (rad) | $L_{1}$ | $L_{2}$ |
| $q_{2}$ | shoulder_roll_to_shoulder_yaw | revolute (rad) | $L_{2}$ | $L_{3}$ |
| $q_{3}$ | shoulder_yaw_to_upper_arm | revolute (rad) | $L_{3}$ | $L_{4}$ |
| $q_{4}$ | upper_arm_to_under_arm | revolute (rad) | $L_{4}$ | $L_{5}$ |
| $q_{5}$ | under_arm_to_wrist_pitch | revolute (rad) | $L_{5}$ | $L_{6}$ |
| $q_{6}$ | wrist_pitch_to_wrist_yaw | revolute (rad) | $L_{6}$ | $L_{7}$ |
| $q_{7}$ | wrist_yaw_to_hand_base | revolute (rad) | $L_{7}$ | $L_{8}$ |
| $q_{8}$ | hand_base_to_rotation_base | revolute (rad) | $L_{8}$ | $L_{9}$ |
| $q_{9}$ | hand_base_to_finger_1_1 | revolute (rad) | $L_{8}$ | $L_{10}$ |
| $q_{10}$ | hand_base_to_finger_2_1 | revolute (rad) | $L_{8}$ | $L_{11}$ |
| $q_{11}$ | rotation_base_to_finger_3_1 | revolute (rad) | $L_{9}$ | $L_{12}$ |
| $q_{12}$ | finger_1_1_to_finger_1_2 | revolute (rad) | $L_{10}$ | $L_{13}$ |
| $q_{13}$ | finger_1_2_to_finger_2_2 | revolute (rad) | $L_{11}$ | $L_{14}$ |
| $q_{14}$ | finger_3_1_to_finger_3_2 | revolute (rad) | $L_{12}$ | $L_{15}$ |

Shorthand: $c_i = \cos(q_i)$, $s_i = \sin(q_i)$

### Kinematic Tree

```
L0: base_link
  +-- [fixed] base_link_to_shoulder_pitch
      L1: Shoulder_Pitch
        +-- [revolute] shoulder_pitch_to_shoulder_roll (q1)
            L2: Shoulder_Roll
              +-- [revolute] shoulder_roll_to_shoulder_yaw (q2)
                  L3: Shoulder_Yaw
                    +-- [revolute] shoulder_yaw_to_upper_arm (q3)
                        L4: Upper_Arm
                          +-- [revolute] upper_arm_to_under_arm (q4)
                              L5: Under_Arm
                                +-- [revolute] under_arm_to_wrist_pitch (q5)
                                    L6: Wrist_Pitch
                                      +-- [revolute] wrist_pitch_to_wrist_yaw (q6)
                                          L7: Wrist_yaw
                                            +-- [revolute] wrist_yaw_to_hand_base (q7)
                                                L8: MG996R
                                                  |-- [revolute] hand_base_to_rotation_base (q8)
                                                  |   L9: RotationBody
                                                  |     +-- [revolute] rotation_base_to_finger_3_1 (q11)
                                                  |         L12: Finger_3_1
                                                  |           +-- [revolute] finger_3_1_to_finger_3_2 (q14)
                                                  |               L15: Finger_3_2
                                                  |-- [revolute] hand_base_to_finger_1_1 (q9)
                                                  |   L10: Finger_1_1
                                                  |     +-- [revolute] finger_1_1_to_finger_1_2 (q12)
                                                  |         L13: Finger_1_2
                                                  +-- [revolute] hand_base_to_finger_2_1 (q10)
                                                      L11: Finger_2_1
                                                        +-- [revolute] finger_1_2_to_finger_2_2 (q13)
                                                            L14: Finger_2_2
```

## Transforms

## base_link_to_shoulder_pitch

$L_{0}$ **base_link** -> $L_{1}$ **Shoulder_Pitch** (fixed)

- **origin xyz**: (-0.16, 0, 0.85) m
- **origin rpy**: (0, 0, 0) rad

### Local Transform

$$
T^{0}_{1} = \begin{bmatrix}
1 & 0 & 0 & -0.16 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & 0.85 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## shoulder_pitch_to_shoulder_roll

$L_{1}$ **Shoulder_Pitch** -> $L_{2}$ **Shoulder_Roll** (revolute)
  Variable: $q_{1}$

- **origin xyz**: (-0.052, 0, 0) m
- **origin rpy**: (-1.570796, 0, 1.570796) rad
- **axis**: (0, 0, 1)
- **limits**: [-0.785398, 0.785398] rad ([-45deg, 45deg])

### Local Transform

$T^{1}_{2}(q_{1}) = T_{fixed} \cdot R_{axis}(q_{1})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & 0 & -1 & -0.052 \\
1 & 0 & 0 & 0 \\
0 & -1 & 0 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{1}) = \begin{bmatrix}
c_{1} & -s_{1} & 0 & 0 \\
s_{1} & c_{1} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## shoulder_roll_to_shoulder_yaw

$L_{2}$ **Shoulder_Roll** -> $L_{3}$ **Shoulder_Yaw** (revolute)
  Variable: $q_{2}$

- **origin xyz**: (-0.0245, 0, 0.05) m
- **origin rpy**: (0, 1.570796, 0) rad
- **axis**: (0, 0, 1)
- **limits**: [0, 0.785398] rad ([0deg, 45deg])

### Local Transform

$T^{2}_{3}(q_{2}) = T_{fixed} \cdot R_{axis}(q_{2})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & 0 & 1 & -0.0245 \\
0 & 1 & 0 & 0 \\
-1 & 0 & 0 & 0.05 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{2}) = \begin{bmatrix}
c_{2} & -s_{2} & 0 & 0 \\
s_{2} & c_{2} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## shoulder_yaw_to_upper_arm

$L_{3}$ **Shoulder_Yaw** -> $L_{4}$ **Upper_Arm** (revolute)
  Variable: $q_{3}$

- **origin xyz**: (0, 0.127, 0.0255) m
- **origin rpy**: (-1.570796, 0, 0) rad
- **axis**: (0, 0, 1)
- **limits**: [-1.570796, 4.18879] rad ([-90deg, 240deg])

### Local Transform

$T^{3}_{4}(q_{3}) = T_{fixed} \cdot R_{axis}(q_{3})$ where:

$$
T_{fixed} = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 0 & 1 & 0.127 \\
0 & -1 & 0 & 0.0255 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{3}) = \begin{bmatrix}
c_{3} & -s_{3} & 0 & 0 \\
s_{3} & c_{3} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## upper_arm_to_under_arm

$L_{4}$ **Upper_Arm** -> $L_{5}$ **Under_Arm** (revolute)
  Variable: $q_{4}$

- **origin xyz**: (-0.0245, -0.000208, 0.129792) m
- **origin rpy**: (1.570796, 0, -1.570796) rad
- **axis**: (0, 0, 1)
- **limits**: [-1.832596, 1.832596] rad ([-105deg, 105deg])

### Local Transform

$T^{4}_{5}(q_{4}) = T_{fixed} \cdot R_{axis}(q_{4})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & 0 & -1 & -0.0245 \\
-1 & 0 & 0 & -0.000208 \\
0 & 1 & 0 & 0.129792 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{4}) = \begin{bmatrix}
c_{4} & -s_{4} & 0 & 0 \\
s_{4} & c_{4} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## under_arm_to_wrist_pitch

$L_{5}$ **Under_Arm** -> $L_{6}$ **Wrist_Pitch** (revolute)
  Variable: $q_{5}$

- **origin xyz**: (0, 0.120997, -0.0245) m
- **origin rpy**: (1.570796, 1.570796, 0) rad
- **axis**: (0, 0, 1)
- **limits**: [-1.570796, 1.570796] rad ([-90deg, 90deg])

### Local Transform

$T^{5}_{6}(q_{5}) = T_{fixed} \cdot R_{axis}(q_{5})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & 1 & 0 & 0 \\
0 & 0 & -1 & 0.120997 \\
-1 & 0 & 0 & -0.0245 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{5}) = \begin{bmatrix}
c_{5} & -s_{5} & 0 & 0 \\
s_{5} & c_{5} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## wrist_pitch_to_wrist_yaw

$L_{6}$ **Wrist_Pitch** -> $L_{7}$ **Wrist_yaw** (revolute)
  Variable: $q_{6}$

- **origin xyz**: (-0.02, 0, -0.0712) m
- **origin rpy**: (-1.570796, 0, 1.570796) rad
- **axis**: (0, 0, 1)
- **limits**: [-1.570796, 1.570796] rad ([-90deg, 90deg])

### Local Transform

$T^{6}_{7}(q_{6}) = T_{fixed} \cdot R_{axis}(q_{6})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & 0 & -1 & -0.02 \\
1 & 0 & 0 & 0 \\
0 & -1 & 0 & -0.0712 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{6}) = \begin{bmatrix}
c_{6} & -s_{6} & 0 & 0 \\
s_{6} & c_{6} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## wrist_yaw_to_hand_base

$L_{7}$ **Wrist_yaw** -> $L_{8}$ **MG996R** (revolute)
  Variable: $q_{7}$

- **origin xyz**: (0, 0.077, -0.02) m
- **origin rpy**: (1.570796, 1.570796, 0) rad
- **axis**: (0, 0, 1)
- **limits**: [-2.094395, 2.094395] rad ([-120deg, 120deg])

### Local Transform

$T^{7}_{8}(q_{7}) = T_{fixed} \cdot R_{axis}(q_{7})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & 1 & 0 & 0 \\
0 & 0 & -1 & 0.077 \\
-1 & 0 & 0 & -0.02 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{7}) = \begin{bmatrix}
c_{7} & -s_{7} & 0 & 0 \\
s_{7} & c_{7} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## hand_base_to_rotation_base

$L_{8}$ **MG996R** -> $L_{9}$ **RotationBody** (revolute)
  Variable: $q_{8}$

- **origin xyz**: (0.002087, -0.025561, -0.045758) m
- **origin rpy**: (3.141593, 0, 0) rad
- **axis**: (0, 0, 1)
- **limits**: [-3.141593, 0] rad ([-180deg, 0deg])

### Local Transform

$T^{8}_{9}(q_{8}) = T_{fixed} \cdot R_{axis}(q_{8})$ where:

$$
T_{fixed} = \begin{bmatrix}
1 & 0 & 0 & 0.002087 \\
0 & -1 & 0 & -0.025561 \\
0 & 0 & -1 & -0.045758 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{8}) = \begin{bmatrix}
c_{8} & -s_{8} & 0 & 0 \\
s_{8} & c_{8} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## hand_base_to_finger_1_1

$L_{8}$ **MG996R** -> $L_{10}$ **Finger_1_1** (revolute)
  Variable: $q_{9}$

- **origin xyz**: (-0.02, -0.033335, -0.072321) m
- **origin rpy**: (1.570796, 0, 0) rad
- **axis**: (0, 0, 1)
- **limits**: [-1.570796, 1.570796] rad ([-90deg, 90deg])

### Local Transform

$T^{8}_{10}(q_{9}) = T_{fixed} \cdot R_{axis}(q_{9})$ where:

$$
T_{fixed} = \begin{bmatrix}
1 & 0 & 0 & -0.02 \\
0 & 0 & -1 & -0.033335 \\
0 & 1 & 0 & -0.072321 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{9}) = \begin{bmatrix}
c_{9} & -s_{9} & 0 & 0 \\
s_{9} & c_{9} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## hand_base_to_finger_2_1

$L_{8}$ **MG996R** -> $L_{11}$ **Finger_2_1** (revolute)
  Variable: $q_{10}$

- **origin xyz**: (-0.02, 0.003665, -0.072321) m
- **origin rpy**: (1.570796, 0, 0) rad
- **axis**: (0, 0, 1)
- **limits**: [-1.570796, 1.570796] rad ([-90deg, 90deg])

### Local Transform

$T^{8}_{11}(q_{10}) = T_{fixed} \cdot R_{axis}(q_{10})$ where:

$$
T_{fixed} = \begin{bmatrix}
1 & 0 & 0 & -0.02 \\
0 & 0 & -1 & 0.003665 \\
0 & 1 & 0 & -0.072321 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{10}) = \begin{bmatrix}
c_{10} & -s_{10} & 0 & 0 \\
s_{10} & c_{10} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## rotation_base_to_finger_3_1

$L_{9}$ **RotationBody** -> $L_{12}$ **Finger_3_1** (revolute)
  Variable: $q_{11}$

- **origin xyz**: (-0.021, 0.045117, 0.026147) m
- **origin rpy**: (-1.570796, 0, 0) rad
- **axis**: (0, 0, 1)
- **limits**: [-1.570796, 1.570796] rad ([-90deg, 90deg])

### Local Transform

$T^{9}_{12}(q_{11}) = T_{fixed} \cdot R_{axis}(q_{11})$ where:

$$
T_{fixed} = \begin{bmatrix}
1 & 0 & 0 & -0.021 \\
0 & 0 & 1 & 0.045117 \\
0 & -1 & 0 & 0.026147 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{11}) = \begin{bmatrix}
c_{11} & -s_{11} & 0 & 0 \\
s_{11} & c_{11} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## finger_1_1_to_finger_1_2

$L_{10}$ **Finger_1_1** -> $L_{13}$ **Finger_1_2** (revolute)
  Variable: $q_{12}$

- **origin xyz**: (0, -0.065, -0.0003) m
- **origin rpy**: (0, 0, 0) rad
- **axis**: (0, 0, 1)
- **limits**: [0, 2.094395] rad ([0deg, 120deg])

### Local Transform

$$
T^{10}_{13}(q_{12}) = \begin{bmatrix}
c_{12} & -s_{12} & 0 & 0 \\
s_{12} & c_{12} & 0 & -0.065 \\
0 & 0 & 1 & -0.0003 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## finger_1_2_to_finger_2_2

$L_{11}$ **Finger_2_1** -> $L_{14}$ **Finger_2_2** (revolute)
  Variable: $q_{13}$

- **origin xyz**: (0, -0.065, -0.0003) m
- **origin rpy**: (0, 0, 0) rad
- **axis**: (0, 0, 1)
- **limits**: [0, 2.094395] rad ([0deg, 120deg])

### Local Transform

$$
T^{11}_{14}(q_{13}) = \begin{bmatrix}
c_{13} & -s_{13} & 0 & 0 \\
s_{13} & c_{13} & 0 & -0.065 \\
0 & 0 & 1 & -0.0003 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## finger_3_1_to_finger_3_2

$L_{12}$ **Finger_3_1** -> $L_{15}$ **Finger_3_2** (revolute)
  Variable: $q_{14}$

- **origin xyz**: (0, -0.065, -0.0003) m
- **origin rpy**: (0, 0, 0) rad
- **axis**: (0, 0, 1)
- **limits**: [0, 2.094395] rad ([0deg, 120deg])

### Local Transform

$$
T^{12}_{15}(q_{14}) = \begin{bmatrix}
c_{14} & -s_{14} & 0 & 0 \\
s_{14} & c_{14} & 0 & -0.065 \\
0 & 0 & 1 & -0.0003 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Global Transform Chains

Transform from root $L_0$ to any link, as product of local transforms along the kinematic chain.

$$T^{0}_{2} = T^{0}_{1} \cdot T^{1}_{2}(q_{1})\quad (L_0 \to L_{2}: \text{Shoulder_Roll})$$

$$T^{0}_{3} = T^{0}_{1} \cdot T^{1}_{2}(q_{1}) \cdot T^{2}_{3}(q_{2})\quad (L_0 \to L_{3}: \text{Shoulder_Yaw})$$

$$T^{0}_{4} = T^{0}_{1} \cdot T^{1}_{2}(q_{1}) \cdot T^{2}_{3}(q_{2}) \cdot T^{3}_{4}(q_{3})\quad (L_0 \to L_{4}: \text{Upper_Arm})$$

$$T^{0}_{5} = T^{0}_{1} \cdot T^{1}_{2}(q_{1}) \cdot T^{2}_{3}(q_{2}) \cdot T^{3}_{4}(q_{3}) \cdot T^{4}_{5}(q_{4})\quad (L_0 \to L_{5}: \text{Under_Arm})$$

$$T^{0}_{6} = T^{0}_{1} \cdot T^{1}_{2}(q_{1}) \cdot T^{2}_{3}(q_{2}) \cdot T^{3}_{4}(q_{3}) \cdot T^{4}_{5}(q_{4}) \cdot T^{5}_{6}(q_{5})\quad (L_0 \to L_{6}: \text{Wrist_Pitch})$$

$$T^{0}_{7} = T^{0}_{1} \cdot T^{1}_{2}(q_{1}) \cdot T^{2}_{3}(q_{2}) \cdot T^{3}_{4}(q_{3}) \cdot T^{4}_{5}(q_{4}) \cdot T^{5}_{6}(q_{5}) \cdot T^{6}_{7}(q_{6})\quad (L_0 \to L_{7}: \text{Wrist_yaw})$$

$$T^{0}_{8} = T^{0}_{1} \cdot T^{1}_{2}(q_{1}) \cdot T^{2}_{3}(q_{2}) \cdot T^{3}_{4}(q_{3}) \cdot T^{4}_{5}(q_{4}) \cdot T^{5}_{6}(q_{5}) \cdot T^{6}_{7}(q_{6}) \cdot T^{7}_{8}(q_{7})\quad (L_0 \to L_{8}: \text{MG996R})$$

$$T^{0}_{9} = T^{0}_{1} \cdot T^{1}_{2}(q_{1}) \cdot T^{2}_{3}(q_{2}) \cdot T^{3}_{4}(q_{3}) \cdot T^{4}_{5}(q_{4}) \cdot T^{5}_{6}(q_{5}) \cdot T^{6}_{7}(q_{6}) \cdot T^{7}_{8}(q_{7}) \cdot T^{8}_{9}(q_{8})\quad (L_0 \to L_{9}: \text{RotationBody})$$

$$T^{0}_{10} = T^{0}_{1} \cdot T^{1}_{2}(q_{1}) \cdot T^{2}_{3}(q_{2}) \cdot T^{3}_{4}(q_{3}) \cdot T^{4}_{5}(q_{4}) \cdot T^{5}_{6}(q_{5}) \cdot T^{6}_{7}(q_{6}) \cdot T^{7}_{8}(q_{7}) \cdot T^{8}_{10}(q_{9})\quad (L_0 \to L_{10}: \text{Finger_1_1})$$

$$T^{0}_{11} = T^{0}_{1} \cdot T^{1}_{2}(q_{1}) \cdot T^{2}_{3}(q_{2}) \cdot T^{3}_{4}(q_{3}) \cdot T^{4}_{5}(q_{4}) \cdot T^{5}_{6}(q_{5}) \cdot T^{6}_{7}(q_{6}) \cdot T^{7}_{8}(q_{7}) \cdot T^{8}_{11}(q_{10})\quad (L_0 \to L_{11}: \text{Finger_2_1})$$

$$T^{0}_{12} = T^{0}_{1} \cdot T^{1}_{2}(q_{1}) \cdot T^{2}_{3}(q_{2}) \cdot T^{3}_{4}(q_{3}) \cdot T^{4}_{5}(q_{4}) \cdot T^{5}_{6}(q_{5}) \cdot T^{6}_{7}(q_{6}) \cdot T^{7}_{8}(q_{7}) \cdot T^{8}_{9}(q_{8}) \cdot T^{9}_{12}(q_{11})\quad (L_0 \to L_{12}: \text{Finger_3_1})$$

$$T^{0}_{13} = T^{0}_{1} \cdot T^{1}_{2}(q_{1}) \cdot T^{2}_{3}(q_{2}) \cdot T^{3}_{4}(q_{3}) \cdot T^{4}_{5}(q_{4}) \cdot T^{5}_{6}(q_{5}) \cdot T^{6}_{7}(q_{6}) \cdot T^{7}_{8}(q_{7}) \cdot T^{8}_{10}(q_{9}) \cdot T^{10}_{13}(q_{12})\quad (L_0 \to L_{13}: \text{Finger_1_2})$$

$$T^{0}_{14} = T^{0}_{1} \cdot T^{1}_{2}(q_{1}) \cdot T^{2}_{3}(q_{2}) \cdot T^{3}_{4}(q_{3}) \cdot T^{4}_{5}(q_{4}) \cdot T^{5}_{6}(q_{5}) \cdot T^{6}_{7}(q_{6}) \cdot T^{7}_{8}(q_{7}) \cdot T^{8}_{11}(q_{10}) \cdot T^{11}_{14}(q_{13})\quad (L_0 \to L_{14}: \text{Finger_2_2})$$

$$T^{0}_{15} = T^{0}_{1} \cdot T^{1}_{2}(q_{1}) \cdot T^{2}_{3}(q_{2}) \cdot T^{3}_{4}(q_{3}) \cdot T^{4}_{5}(q_{4}) \cdot T^{5}_{6}(q_{5}) \cdot T^{6}_{7}(q_{6}) \cdot T^{7}_{8}(q_{7}) \cdot T^{8}_{9}(q_{8}) \cdot T^{9}_{12}(q_{11}) \cdot T^{12}_{15}(q_{14})\quad (L_0 \to L_{15}: \text{Finger_3_2})$$

