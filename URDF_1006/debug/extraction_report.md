# Extraction Report: JAXV2-Assembly v4

**Exported:** 2026-10-06T03:00:23.638438
**Exporter:** v3.1.1

## Summary

| Metric | Value |
|--------|-------|
| Occurrences | 16 |
| Subassemblies | 0 |
| Leaf components | 16 |
| Joints (total) | 15 |
| As-built joints | 0 |
| Regular joints | 15 |
| Max nesting depth | 0 |

## Assembly Hierarchy

```
[JAXV2_Assembly]  (design root)
  ├── Finger_1_1  (71.4g)
  ├── Finger_1_2  (11.3g)
  ├── Finger_2_1  (71.4g)
  ├── Finger_2_2  (11.3g)
  ├── Finger_3_1  (71.4g)
  ├── Finger_3_2  (11.3g)
  ├── MG996R  (419.9g)
  ├── RotationBody  (74.5g)
  ├── Shoulder_Pitch  (997.3g)
  ├── Shoulder_Roll  (1087.7g)
  ├── Shoulder_Yaw  (1145.4g)
  ├── Under_Arm  (346.1g)
  ├── Upper_Arm  (410.6g)
  ├── Wrist_Pitch  (189.2g)
  ├── Wrist_yaw  (146.3g)
  ├── base_link  (18851.8g)
```

## Occurrences

### Depth 0

#### 🔧 COMPONENT: `Finger_1_1`

| Property | Value |
|----------|-------|
| Full path | `Finger_1-1:1` |
| Component name | Finger_1-1 |
| Depth | 0 |
| Path segments | Finger_1_1 |
| **Transforms** | |
| Global position (m) | (-0.275500, 0.010674, 0.266940) |
| Global position (mm) | (-275.50, 10.67, 266.94) |
| Local transform (m) | (-0.275500, 0.010674, 0.266940) |
| Assembly context depth | 0 |
| transform2 (m) | (-0.275500, 0.010674, 0.266940) |
| **Physical** | |
| Mass | 0.071419 kg (71.419 g) |
| Volume | 2.027132e-05 m³ |
| Density | 3523.1 kg/m³ |
| Surface area | 7.422380e-03 m² |
| Body count | 3 |
| CoM (component-local, m) | (0.070028, 0.024033, 0.006505) |
| CoM (global, m) | (-0.205473, 0.034707, 0.273445) |
| CoM (global, mm) | (-205.47, 34.71, 273.45) |
| **Inertia at origin (kg·m²)** | |
| Ixx, Iyy, Izz | 5.009158e-05, 3.680337e-04, 4.103326e-04 |
| Ixy, Ixz, Iyz | -1.185995e-04, -3.252283e-05, -1.116320e-05 |
| **Inertia at CoM (kg·m²)** | |
| Ixx, Iyy, Izz | 5.819045e-06, 1.478499e-05, 1.885508e-05 |
| Ixy, Ixz, Iyz | 1.596518e-06, 9.291950e-09, 1.658307e-09 |
| Bounding box (m) | 0.0914 × 0.0347 × 0.0159 |
| Bounding box (mm) | 91.43 × 34.71 × 15.93 |
| **Material & Appearance** | |
| Material | Acetal_Resin_White |
| Appearance | Paint_Enamel_Glossy_Black |
| Color (RGB 0-1) | (0.098, 0.098, 0.098) |
| Color (RGB 0-255) | (25, 25, 25) |
| **Per-body breakdown** | |
| Body 0: Finger_1-1 | mass=12.498g, material=Acetal_Resin_White, inertia_src=api |
| Body 1: Horn_1-1 | mass=0.306g, material=Acetal_Resin_White, inertia_src=api |
| Body 2: MG90s_1-2 | mass=58.614g, material=Steel, inertia_src=api |

#### 🔧 COMPONENT: `Finger_1_2`

| Property | Value |
|----------|-------|
| Full path | `Finger_1-2:1` |
| Component name | Finger_1-2 |
| Depth | 0 |
| Path segments | Finger_1_2 |
| **Transforms** | |
| Global position (m) | (-0.275263, 0.011276, 0.266373) |
| Global position (mm) | (-275.26, 11.28, 266.37) |
| Local transform (m) | (-0.275263, 0.011276, 0.266373) |
| Assembly context depth | 0 |
| transform2 (m) | (-0.275263, 0.011276, 0.266373) |
| **Physical** | |
| Mass | 0.011312 kg (11.312 g) |
| Volume | 1.131188e-05 m³ |
| Density | 1000.0 kg/m³ |
| Surface area | 4.147482e-03 m² |
| Body count | 2 |
| CoM (component-local, m) | (0.107556, 0.028997, 0.006738) |
| CoM (global, m) | (-0.167707, 0.040273, 0.273110) |
| CoM (global, mm) | (-167.71, 40.27, 273.11) |
| **Inertia at origin (kg·m²)** | |
| Ixx, Iyy, Izz | 1.110142e-05, 1.327696e-04, 1.425824e-04 |
| Ixy, Ixz, Iyz | -3.496860e-05, -8.197338e-06, -2.210036e-06 |
| **Inertia at CoM (kg·m²)** | |
| Ixx, Iyy, Izz | 1.076266e-06, 1.397398e-06, 2.212028e-06 |
| Ixy, Ixz, Iyz | 3.114506e-07, -1.544988e-18, 7.674119e-19 |
| Bounding box (m) | 0.0657 × 0.0344 × 0.0130 |
| Bounding box (mm) | 65.74 × 34.43 × 13.00 |
| **Material & Appearance** | |
| Material | Acetal_Resin_White |
| Appearance | Paint_Enamel_Glossy_Black |
| Color (RGB 0-1) | (0.098, 0.098, 0.098) |
| Color (RGB 0-255) | (25, 25, 25) |
| **Per-body breakdown** | |
| Body 0: Finger_1-2 | mass=11.005g, material=Acetal_Resin_White, inertia_src=api |
| Body 1: Horn_1-2 | mass=0.306g, material=Acetal_Resin_White, inertia_src=api |

#### 🔧 COMPONENT: `Finger_2_1`

| Property | Value |
|----------|-------|
| Full path | `Finger_2-1:1` |
| Component name | Finger_2-1 |
| Depth | 0 |
| Path segments | Finger_2_1 |
| **Transforms** | |
| Global position (m) | (-0.275500, 0.010674, 0.266940) |
| Global position (mm) | (-275.50, 10.67, 266.94) |
| Local transform (m) | (-0.275500, 0.010674, 0.266940) |
| Assembly context depth | 0 |
| transform2 (m) | (-0.275500, 0.010674, 0.266940) |
| **Physical** | |
| Mass | 0.071419 kg (71.419 g) |
| Volume | 2.027132e-05 m³ |
| Density | 3523.1 kg/m³ |
| Surface area | 7.422380e-03 m² |
| Body count | 3 |
| CoM (component-local, m) | (0.070028, -0.012967, 0.006505) |
| CoM (global, m) | (-0.205473, -0.002293, 0.273445) |
| CoM (global, mm) | (-205.47, -2.29, 273.45) |
| **Inertia at origin (kg·m²)** | |
| Ixx, Iyy, Izz | 2.084933e-05, 3.680337e-04, 3.810903e-04 |
| Ixy, Ixz, Iyz | 6.644763e-05, -3.252283e-05, 6.025595e-06 |
| **Inertia at CoM (kg·m²)** | |
| Ixx, Iyy, Izz | 5.819047e-06, 1.478499e-05, 1.885509e-05 |
| Ixy, Ixz, Iyz | 1.596510e-06, 9.291950e-09, 1.657525e-09 |
| Bounding box (m) | 0.0914 × 0.0347 × 0.0159 |
| Bounding box (mm) | 91.43 × 34.72 × 15.93 |
| **Material & Appearance** | |
| Material | Acetal_Resin_White |
| Appearance | Paint_Enamel_Glossy_Black |
| Color (RGB 0-1) | (0.098, 0.098, 0.098) |
| Color (RGB 0-255) | (25, 25, 25) |
| **Per-body breakdown** | |
| Body 0: Finger_2-1 | mass=12.498g, material=Acetal_Resin_White, inertia_src=api |
| Body 1: Horn_2-1 | mass=0.306g, material=Acetal_Resin_White, inertia_src=api |
| Body 2: MG90s_2-2 | mass=58.614g, material=Steel, inertia_src=api |

#### 🔧 COMPONENT: `Finger_2_2`

| Property | Value |
|----------|-------|
| Full path | `Finger_2-2:1` |
| Component name | Finger_2-2 |
| Depth | 0 |
| Path segments | Finger_2_2 |
| **Transforms** | |
| Global position (m) | (-0.275500, 0.010974, 0.266940) |
| Global position (mm) | (-275.50, 10.97, 266.94) |
| Local transform (m) | (-0.275500, 0.010974, 0.266940) |
| Assembly context depth | 0 |
| transform2 (m) | (-0.275500, 0.010974, 0.266940) |
| **Physical** | |
| Mass | 0.011312 kg (11.312 g) |
| Volume | 1.131188e-05 m³ |
| Density | 1000.0 kg/m³ |
| Surface area | 4.147482e-03 m² |
| Body count | 2 |
| CoM (component-local, m) | (0.108123, -0.008305, 0.006500) |
| CoM (global, m) | (-0.167377, 0.002669, 0.273440) |
| CoM (global, mm) | (-167.38, 2.67, 273.44) |
| **Inertia at origin (kg·m²)** | |
| Ixx, Iyy, Izz | 2.334342e-06, 1.341190e-04, 1.352358e-04 |
| Ixy, Ixz, Iyz | 1.046870e-05, -7.950019e-06, 6.106180e-07 |
| **Inertia at CoM (kg·m²)** | |
| Ixx, Iyy, Izz | 1.076266e-06, 1.397398e-06, 2.212028e-06 |
| Ixy, Ixz, Iyz | 3.114506e-07, -1.548376e-18, 7.725999e-19 |
| Bounding box (m) | 0.0657 × 0.0344 × 0.0130 |
| Bounding box (mm) | 65.74 × 34.43 × 13.00 |
| **Material & Appearance** | |
| Material | Acetal_Resin_White |
| Appearance | Paint_Enamel_Glossy_Black |
| Color (RGB 0-1) | (0.098, 0.098, 0.098) |
| Color (RGB 0-255) | (25, 25, 25) |
| **Per-body breakdown** | |
| Body 0: Finger_2-2 | mass=11.005g, material=Acetal_Resin_White, inertia_src=api |
| Body 1: Horn_2-2 | mass=0.306g, material=Acetal_Resin_White, inertia_src=api |

#### 🔧 COMPONENT: `Finger_3_1`

| Property | Value |
|----------|-------|
| Full path | `Finger_3-1:1` |
| Component name | Finger_3-1 |
| Depth | 0 |
| Path segments | Finger_3_1 |
| **Transforms** | |
| Global position (m) | (-0.244413, -0.058655, 0.267356) |
| Global position (mm) | (-244.41, -58.65, 267.36) |
| Local transform (m) | (-0.244413, -0.058655, 0.267356) |
| Assembly context depth | 0 |
| transform2 (m) | (-0.244413, -0.058655, 0.267356) |
| **Physical** | |
| Mass | 0.071419 kg (71.419 g) |
| Volume | 2.027132e-05 m³ |
| Density | 3523.1 kg/m³ |
| Surface area | 7.422380e-03 m² |
| Body count | 3 |
| CoM (component-local, m) | (0.070028, 0.007952, -0.036505) |
| CoM (global, m) | (-0.174385, -0.050703, 0.230851) |
| CoM (global, mm) | (-174.39, -50.70, 230.85) |
| **Inertia at origin (kg·m²)** | |
| Ixx, Iyy, Izz | 1.055078e-04, 4.601843e-04, 3.735982e-04 |
| Ixy, Ixz, Iyz | -4.136758e-05, 1.825611e-04, 2.073399e-05 |
| **Inertia at CoM (kg·m²)** | |
| Ixx, Iyy, Izz | 5.819043e-06, 1.478499e-05, 1.885509e-05 |
| Ixy, Ixz, Iyz | -1.596511e-06, -9.298790e-09, 1.656854e-09 |
| Bounding box (m) | 0.0914 × 0.0347 × 0.0159 |
| Bounding box (mm) | 91.43 × 34.72 × 15.93 |
| **Material & Appearance** | |
| Material | Acetal_Resin_White |
| Appearance | Paint_Enamel_Glossy_Black |
| Color (RGB 0-1) | (0.098, 0.098, 0.098) |
| Color (RGB 0-255) | (25, 25, 25) |
| **Per-body breakdown** | |
| Body 0: Finger_3-1 | mass=12.498g, material=Acetal_Resin_White, inertia_src=api |
| Body 1: Horn_3-1 | mass=0.306g, material=Acetal_Resin_White, inertia_src=api |
| Body 2: MG90s_3-2 | mass=58.614g, material=Steel, inertia_src=api |

#### 🔧 COMPONENT: `Finger_3_2`

| Property | Value |
|----------|-------|
| Full path | `Finger_3-2:1` |
| Component name | Finger_3-2 |
| Depth | 0 |
| Path segments | Finger_3_2 |
| **Transforms** | |
| Global position (m) | (-0.244413, -0.058355, 0.267356) |
| Global position (mm) | (-244.41, -58.35, 267.36) |
| Local transform (m) | (-0.244413, -0.058355, 0.267356) |
| Assembly context depth | 0 |
| transform2 (m) | (-0.244413, -0.058355, 0.267356) |
| **Physical** | |
| Mass | 0.011312 kg (11.312 g) |
| Volume | 1.131188e-05 m³ |
| Density | 1000.0 kg/m³ |
| Surface area | 4.147482e-03 m² |
| Body count | 2 |
| CoM (component-local, m) | (0.108123, 0.003290, -0.036500) |
| CoM (global, m) | (-0.136289, -0.055065, 0.230856) |
| CoM (global, mm) | (-136.29, -55.07, 230.86) |
| **Inertia at origin (kg·m²)** | |
| Ixx, Iyy, Izz | 1.626895e-05, 1.487113e-04, 1.345781e-04 |
| Ixy, Ixz, Iyz | -4.335286e-06, 4.464242e-05, 1.358354e-06 |
| **Inertia at CoM (kg·m²)** | |
| Ixx, Iyy, Izz | 1.076266e-06, 1.397398e-06, 2.212028e-06 |
| Ixy, Ixz, Iyz | -3.114506e-07, 1.538212e-18, 7.608473e-19 |
| Bounding box (m) | 0.0657 × 0.0344 × 0.0130 |
| Bounding box (mm) | 65.74 × 34.43 × 13.00 |
| **Material & Appearance** | |
| Material | Acetal_Resin_White |
| Appearance | Paint_Enamel_Glossy_Black |
| Color (RGB 0-1) | (0.098, 0.098, 0.098) |
| Color (RGB 0-255) | (25, 25, 25) |
| **Per-body breakdown** | |
| Body 0: Finger_3-2 | mass=11.005g, material=Acetal_Resin_White, inertia_src=api |
| Body 1: Horn_3-2 | mass=0.306g, material=Acetal_Resin_White, inertia_src=api |

#### 🔧 COMPONENT: `MG996R`

| Property | Value |
|----------|-------|
| Full path | `MG996R:1` |
| Component name | MG996R |
| Depth | 0 |
| Path segments | MG996R |
| **Transforms** | |
| Global position (m) | (-0.275500, 0.010374, 0.266940) |
| Global position (mm) | (-275.50, 10.37, 266.94) |
| Local transform (m) | (-0.275500, 0.010374, 0.266940) |
| Assembly context depth | 0 |
| transform2 (m) | (-0.275500, 0.010374, 0.266940) |
| **Physical** | |
| Mass | 0.419885 kg (419.885 g) |
| Volume | 9.961465e-05 m³ |
| Density | 4215.1 kg/m³ |
| Surface area | 2.995476e-02 m² |
| Body count | 6 |
| CoM (component-local, m) | (-0.014834, -0.002290, 0.000519) |
| CoM (global, m) | (-0.290335, 0.008084, 0.267460) |
| CoM (global, mm) | (-290.33, 8.08, 267.46) |
| **Inertia at origin (kg·m²)** | |
| Ixx, Iyy, Izz | 2.792041e-04, 2.897543e-04, 4.845482e-04 |
| Ixy, Ixz, Iyz | -2.124609e-05, -3.211140e-05, 2.287924e-05 |
| **Inertia at CoM (kg·m²)** | |
| Ixx, Iyy, Izz | 2.768890e-04, 1.972417e-04, 3.899470e-04 |
| Ixy, Ixz, Iyz | -6.982530e-06, -3.534616e-05, 2.237989e-05 |
| Bounding box (m) | 0.0832 × 0.0765 × 0.0431 |
| Bounding box (mm) | 83.17 × 76.48 × 43.09 |
| **Material & Appearance** | |
| Material | Generic |
| Appearance | Paint_Enamel_Glossy_Black |
| Color (RGB 0-1) | (0.098, 0.098, 0.098) |
| Color (RGB 0-255) | (25, 25, 25) |
| **Per-body breakdown** | |
| Body 0: MG996R | mass=27.545g, material=Generic, inertia_src=api |
| Body 1: Hand1 | mass=25.314g, material=Acetal_Resin_White, inertia_src=api |
| Body 2: Hand2 | mass=191.183g, material=Steel, inertia_src=api |
| Body 3: MG90s_rotation | mass=58.614g, material=Steel, inertia_src=api |
| Body 4: MG90s_1-1 | mass=58.614g, material=Steel, inertia_src=api |
| Body 5: MG90s_2-1 | mass=58.614g, material=Steel, inertia_src=api |

#### 🔧 COMPONENT: `RotationBody`

| Property | Value |
|----------|-------|
| Full path | `RotationBody:1` |
| Component name | RotationBody |
| Depth | 0 |
| Path segments | RotationBody |
| **Transforms** | |
| Global position (m) | (-0.244413, -0.058955, 0.267356) |
| Global position (mm) | (-244.41, -58.95, 267.36) |
| Local transform (m) | (-0.244413, -0.058955, 0.267356) |
| Assembly context depth | 0 |
| transform2 (m) | (-0.244413, -0.058955, 0.267356) |
| **Physical** | |
| Mass | 0.074463 kg (74.463 g) |
| Volume | 2.331590e-05 m³ |
| Density | 3193.7 kg/m³ |
| Surface area | 8.167482e-03 m² |
| Body count | 3 |
| CoM (component-local, m) | (0.004021, 0.010174, -0.036145) |
| CoM (global, m) | (-0.240392, -0.048781, 0.231211) |
| CoM (global, mm) | (-240.39, -48.78, 231.21) |
| **Inertia at origin (kg·m²)** | |
| Ixx, Iyy, Izz | 1.111152e-04, 1.163294e-04, 3.044369e-05 |
| Ixy, Ixz, Iyz | -3.888355e-07, 1.125606e-05, 2.708717e-05 |
| **Inertia at CoM (kg·m²)** | |
| Ixx, Iyy, Izz | 6.125770e-06, 1.784337e-05, 2.153303e-05 |
| Ixy, Ixz, Iyz | 2.656916e-06, 4.350216e-07, -2.944843e-07 |
| Bounding box (m) | 0.0680 × 0.0485 × 0.0318 |
| Bounding box (mm) | 68.00 × 48.47 × 31.84 |
| **Material & Appearance** | |
| Material | Acetal_Resin_White |
| Appearance | Paint_Enamel_Glossy_Black |
| Color (RGB 0-1) | (0.098, 0.098, 0.098) |
| Color (RGB 0-255) | (25, 25, 25) |
| **Per-body breakdown** | |
| Body 0: RotationBody | mass=15.601g, material=Acetal_Resin_White, inertia_src=api |
| Body 1: Horn_rotation | mass=0.248g, material=Acetal_Resin_White, inertia_src=api |
| Body 2: MG90s_3-1 | mass=58.614g, material=Steel, inertia_src=api |

#### 🔧 COMPONENT: `Shoulder_Pitch`

| Property | Value |
|----------|-------|
| Full path | `Shoulder_Pitch:1` |
| Component name | Shoulder_Pitch |
| Depth | 0 |
| Path segments | Shoulder_Pitch |
| **Transforms** | |
| Global position (m) | (-0.160000, -0.000000, 0.850000) |
| Global position (mm) | (-160.00, -0.00, 850.00) |
| Local transform (m) | (-0.160000, -0.000000, 0.850000) |
| Assembly context depth | 0 |
| transform2 (m) | (-0.160000, -0.000000, 0.850000) |
| **Physical** | |
| Mass | 0.997346 kg (997.346 g) |
| Volume | 1.724458e-04 m³ |
| Density | 5783.5 kg/m³ |
| Surface area | 2.987765e-02 m² |
| Body count | 2 |
| CoM (component-local, m) | (-0.024397, 0.000000, 0.000306) |
| CoM (global, m) | (-0.184397, 0.000000, 0.850306) |
| CoM (global, mm) | (-184.40, 0.00, 850.31) |
| **Inertia at origin (kg·m²)** | |
| Ixx, Iyy, Izz | 3.941295e-04, 1.035516e-03, 1.042723e-03 |
| Ixy, Ixz, Iyz | -9.632745e-15, 1.600417e-06, -2.449187e-15 |
| **Inertia at CoM (kg·m²)** | |
| Ixx, Iyy, Izz | 3.940360e-04, 4.417818e-04, 4.490818e-04 |
| Ixy, Ixz, Iyz | -1.200910e-14, -5.849335e-06, -2.419365e-15 |
| Bounding box (m) | 0.0640 × 0.0750 × 0.0750 |
| Bounding box (mm) | 64.00 × 75.00 × 75.00 |
| **Material & Appearance** | |
| Material | Acetal_Resin_White |
| Appearance | Paint_Enamel_Glossy_Black |
| Color (RGB 0-1) | (0.098, 0.098, 0.098) |
| Color (RGB 0-255) | (25, 25, 25) |
| **Per-body breakdown** | |
| Body 0: Shoulder_Pitch | mass=52.022g, material=Acetal_Resin_White, inertia_src=api |
| Body 1: AK-Shdr_PITCH | mass=945.323g, material=Steel, inertia_src=api |

#### 🔧 COMPONENT: `Shoulder_Roll`

| Property | Value |
|----------|-------|
| Full path | `Shoulder_Roll:1` |
| Component name | Shoulder_Roll |
| Depth | 0 |
| Path segments | Shoulder_Roll |
| **Transforms** | |
| Global position (m) | (-0.212000, -0.000000, 0.850000) |
| Global position (mm) | (-212.00, -0.00, 850.00) |
| Local transform (m) | (-0.212000, -0.000000, 0.850000) |
| Assembly context depth | 0 |
| transform2 (m) | (-0.212000, -0.000000, 0.850000) |
| **Physical** | |
| Mass | 1.087699 kg (1087.699 g) |
| Volume | 2.627993e-04 m³ |
| Density | 4138.9 kg/m³ |
| Surface area | 5.442752e-02 m² |
| Body count | 2 |
| CoM (component-local, m) | (-0.047977, 0.002826, 0.000221) |
| CoM (global, m) | (-0.259977, 0.002826, 0.850221) |
| CoM (global, mm) | (-259.98, 2.83, 850.22) |
| **Inertia at origin (kg·m²)** | |
| Ixx, Iyy, Izz | 5.262827e-04, 3.069249e-03, 3.105552e-03 |
| Ixy, Ixz, Iyz | 1.486096e-04, 1.200790e-05, -5.325362e-06 |
| **Inertia at CoM (kg·m²)** | |
| Ixx, Iyy, Izz | 5.175399e-04, 5.655319e-04, 5.931985e-04 |
| Ixy, Ixz, Iyz | 1.110163e-06, 4.857296e-07, -4.646552e-06 |
| Bounding box (m) | 0.0875 × 0.0750 × 0.0750 |
| Bounding box (mm) | 87.50 × 75.00 × 75.00 |
| **Material & Appearance** | |
| Material | Acetal_Resin_White |
| Appearance | Paint_Enamel_Glossy_Black |
| Color (RGB 0-1) | (0.098, 0.098, 0.098) |
| Color (RGB 0-255) | (25, 25, 25) |
| **Per-body breakdown** | |
| Body 0: Shoulder_Roll | mass=142.376g, material=Acetal_Resin_White, inertia_src=api |
| Body 1: AK_Shdr_ROLL | mass=945.323g, material=Steel, inertia_src=api |

#### 🔧 COMPONENT: `Shoulder_Yaw`

| Property | Value |
|----------|-------|
| Full path | `Shoulder_Yaw:1` |
| Component name | Shoulder_Yaw |
| Depth | 0 |
| Path segments | Shoulder_Yaw |
| **Transforms** | |
| Global position (m) | (-0.262000, -0.029000, 0.850000) |
| Global position (mm) | (-262.00, -29.00, 850.00) |
| Local transform (m) | (-0.262000, -0.029000, 0.850000) |
| Assembly context depth | 0 |
| transform2 (m) | (-0.262000, -0.029000, 0.850000) |
| **Physical** | |
| Mass | 1.145356 kg (1145.356 g) |
| Volume | 3.204557e-04 m³ |
| Density | 3574.1 kg/m³ |
| Surface area | 6.115127e-02 m² |
| Body count | 2 |
| CoM (component-local, m) | (-0.000000, 0.026639, -0.091392) |
| CoM (global, m) | (-0.262000, -0.002361, 0.758608) |
| CoM (global, mm) | (-262.00, -2.36, 758.61) |
| **Inertia at origin (kg·m²)** | |
| Ixx, Iyy, Izz | 1.165704e-02, 1.080887e-02, 1.418543e-03 |
| Ixy, Ixz, Iyz | 8.312838e-12, -4.610779e-11, 3.046666e-03 |
| **Inertia at CoM (kg·m²)** | |
| Ixx, Iyy, Izz | 1.277749e-03, 1.242378e-03, 6.057515e-04 |
| Ixy, Ixz, Iyz | -1.590802e-11, 3.698748e-11, 2.581956e-04 |
| Bounding box (m) | 0.0750 × 0.0775 × 0.1665 |
| Bounding box (mm) | 75.00 × 77.50 × 166.50 |
| **Material & Appearance** | |
| Material | Acetal_Resin_White |
| Appearance | Paint_Enamel_Glossy_Black |
| Color (RGB 0-1) | (0.098, 0.098, 0.098) |
| Color (RGB 0-255) | (25, 25, 25) |
| **Per-body breakdown** | |
| Body 0: Shoulder_Yaw | mass=200.032g, material=Acetal_Resin_White, inertia_src=api |
| Body 1: AK-Shdr_YAW | mass=945.323g, material=Steel, inertia_src=api |

#### 🔧 COMPONENT: `Under_Arm`

| Property | Value |
|----------|-------|
| Full path | `Under_Arm:1` |
| Component name | Under_Arm |
| Depth | 0 |
| Path segments | Under_Arm |
| **Transforms** | |
| Global position (m) | (-0.286500, 0.001208, 0.593211) |
| Global position (mm) | (-286.50, 1.21, 593.21) |
| Local transform (m) | (-0.286500, 0.001208, 0.593211) |
| Assembly context depth | 0 |
| transform2 (m) | (-0.286500, 0.001208, 0.593211) |
| **Physical** | |
| Mass | 0.346145 kg (346.145 g) |
| Volume | 3.319186e-04 m³ |
| Density | 1042.9 kg/m³ |
| Surface area | 5.097712e-02 m² |
| Body count | 2 |
| CoM (component-local, m) | (0.011447, 0.004802, -0.058743) |
| CoM (global, m) | (-0.275053, 0.006010, 0.534468) |
| CoM (global, mm) | (-275.05, 6.01, 534.47) |
| **Inertia at origin (kg·m²)** | |
| Ixx, Iyy, Izz | 1.749409e-03, 1.766429e-03, 3.416252e-04 |
| Ixy, Ixz, Iyz | -2.883514e-05, 3.261759e-04, 1.350745e-04 |
| **Inertia at CoM (kg·m²)** | |
| Ixx, Iyy, Izz | 5.469735e-04, 5.266168e-04, 2.882880e-04 |
| Ixy, Ixz, Iyz | -9.809798e-06, 9.341698e-05, 3.744171e-05 |
| Bounding box (m) | 0.0766 × 0.0810 × 0.1585 |
| Bounding box (mm) | 76.56 × 81.00 × 158.50 |
| **Material & Appearance** | |
| Material | Acetal_Resin_White |
| Appearance | Paint_Enamel_Glossy_Black |
| Color (RGB 0-1) | (0.098, 0.098, 0.098) |
| Color (RGB 0-255) | (25, 25, 25) |
| **Per-body breakdown** | |
| Body 0: Under_Arm | mass=298.445g, material=Acetal_Resin_White, inertia_src=api |
| Body 1: DM-UD | mass=47.700g, material=Acetal_Resin_Black, inertia_src=api |

#### 🔧 COMPONENT: `Upper_Arm`

| Property | Value |
|----------|-------|
| Full path | `Upper_Arm:1` |
| Component name | Upper_Arm |
| Depth | 0 |
| Path segments | Upper_Arm |
| **Transforms** | |
| Global position (m) | (-0.262000, 0.001000, 0.723000) |
| Global position (mm) | (-262.00, 1.00, 723.00) |
| Local transform (m) | (-0.262000, 0.001000, 0.723000) |
| Assembly context depth | 0 |
| transform2 (m) | (-0.262000, 0.001000, 0.723000) |
| **Physical** | |
| Mass | 0.410582 kg (410.582 g) |
| Volume | 3.594020e-04 m³ |
| Density | 1142.4 kg/m³ |
| Surface area | 8.554249e-02 m² |
| Body count | 2 |
| CoM (component-local, m) | (0.008735, -0.000211, -0.090878) |
| CoM (global, m) | (-0.253265, 0.000789, 0.632122) |
| CoM (global, mm) | (-253.27, 0.79, 632.12) |
| **Inertia at origin (kg·m²)** | |
| Ixx, Iyy, Izz | 4.603688e-03, 4.622029e-03, 3.778075e-04 |
| Ixy, Ixz, Iyz | 2.554027e-06, 3.554927e-04, -1.125337e-05 |
| **Inertia at CoM (kg·m²)** | |
| Ixx, Iyy, Izz | 1.212777e-03, 1.199811e-03, 3.464644e-04 |
| Ixy, Ixz, Iyz | 1.795709e-06, 2.958031e-05, -3.363590e-06 |
| Bounding box (m) | 0.0750 × 0.0750 × 0.1725 |
| Bounding box (mm) | 75.00 × 75.00 × 172.50 |
| **Material & Appearance** | |
| Material | Acetal_Resin_White |
| Appearance | Paint_Enamel_Glossy_Black |
| Color (RGB 0-1) | (0.098, 0.098, 0.098) |
| Color (RGB 0-255) | (25, 25, 25) |
| **Per-body breakdown** | |
| Body 0: Upper_Arm | mass=238.979g, material=Acetal_Resin_White, inertia_src=api |
| Body 1: AK_UP | mass=171.603g, material=Acetal_Resin_Black, inertia_src=api |

#### 🔧 COMPONENT: `Wrist_Pitch`

| Property | Value |
|----------|-------|
| Full path | `Wrist_Pitch:1` |
| Component name | Wrist_Pitch |
| Depth | 0 |
| Path segments | Wrist_Pitch |
| **Transforms** | |
| Global position (m) | (-0.262000, 0.001208, 0.471211) |
| Global position (mm) | (-262.00, 1.21, 471.21) |
| Local transform (m) | (-0.262000, 0.001208, 0.471211) |
| Assembly context depth | 0 |
| transform2 (m) | (-0.262000, 0.001208, 0.471211) |
| **Physical** | |
| Mass | 0.189235 kg (189.235 g) |
| Volume | 1.747327e-04 m³ |
| Density | 1083.0 kg/m³ |
| Surface area | 5.179143e-02 m² |
| Body count | 3 |
| CoM (component-local, m) | (0.009082, 0.002213, -0.042135) |
| CoM (global, m) | (-0.252919, 0.003421, 0.429076) |
| CoM (global, mm) | (-252.92, 3.42, 429.08) |
| **Inertia at origin (kg·m²)** | |
| Ixx, Iyy, Izz | 6.405054e-04, 6.237601e-04, 1.826445e-04 |
| Ixy, Ixz, Iyz | -5.904770e-06, 1.067687e-04, 2.894618e-05 |
| **Inertia at CoM (kg·m²)** | |
| Ixx, Iyy, Izz | 3.036172e-04, 2.721912e-04, 1.661101e-04 |
| Ixy, Ixz, Iyz | -2.101562e-06, 3.435637e-05, 1.130097e-05 |
| Bounding box (m) | 0.0756 × 0.0817 × 0.1177 |
| Bounding box (mm) | 75.60 × 81.65 × 117.70 |
| **Material & Appearance** | |
| Material | Acetal_Resin_White |
| Appearance | Paint_Enamel_Glossy_Black |
| Color (RGB 0-1) | (0.098, 0.098, 0.098) |
| Color (RGB 0-255) | (25, 25, 25) |
| **Per-body breakdown** | |
| Body 0: Wrist_Pitch | mass=140.610g, material=Acetal_Resin_White, inertia_src=api |
| Body 1: Horn-Wrist_Pitch | mass=0.926g, material=Acetal_Resin_Black, inertia_src=api |
| Body 2: DM-Wrist_PITCH | mass=47.700g, material=Acetal_Resin_Black, inertia_src=api |

#### 🔧 COMPONENT: `Wrist_yaw`

| Property | Value |
|----------|-------|
| Full path | `Wrist_yaw:1` |
| Component name | Wrist_yaw |
| Depth | 0 |
| Path segments | Wrist_yaw |
| **Transforms** | |
| Global position (m) | (-0.282000, 0.001208, 0.401011) |
| Global position (mm) | (-282.00, 1.21, 401.01) |
| Local transform (m) | (-0.282000, 0.001208, 0.401011) |
| Assembly context depth | 0 |
| transform2 (m) | (-0.282000, 0.001208, 0.401011) |
| **Physical** | |
| Mass | 0.146251 kg (146.251 g) |
| Volume | 1.456986e-04 m³ |
| Density | 1003.8 kg/m³ |
| Surface area | 3.818031e-02 m² |
| Body count | 3 |
| CoM (component-local, m) | (0.004141, -0.000000, -0.032472) |
| CoM (global, m) | (-0.277859, 0.001208, 0.368539) |
| CoM (global, mm) | (-277.86, 1.21, 368.54) |
| **Inertia at origin (kg·m²)** | |
| Ixx, Iyy, Izz | 4.174133e-04, 4.121934e-04, 1.129886e-04 |
| Ixy, Ixz, Iyz | -1.706300e-10, 8.151919e-05, -1.202984e-09 |
| **Inertia at CoM (kg·m²)** | |
| Ixx, Iyy, Izz | 2.632011e-04, 2.554734e-04, 1.104806e-04 |
| Ixy, Ixz, Iyz | -2.619650e-10, 6.185315e-05, -4.867768e-10 |
| Bounding box (m) | 0.0750 × 0.0750 × 0.1180 |
| Bounding box (mm) | 75.00 × 75.00 × 118.00 |
| **Material & Appearance** | |
| Material | Acetal_Resin_White |
| Appearance | Paint_Enamel_Glossy_Black |
| Color (RGB 0-1) | (0.098, 0.098, 0.098) |
| Color (RGB 0-255) | (25, 25, 25) |
| **Per-body breakdown** | |
| Body 0: Wrist_yaw | mass=144.399g, material=Acetal_Resin_White, inertia_src=api |
| Body 1: Horn-1-Wrist_YAW | mass=0.926g, material=Acetal_Resin_Black, inertia_src=api |
| Body 2: Horn-2-Wrist_YAW | mass=0.926g, material=Acetal_Resin_Black, inertia_src=api |

#### 🔧 COMPONENT: `base_link`

| Property | Value |
|----------|-------|
| Full path | `base_link v3:1` |
| Component name | base_link v3 |
| Depth | 0 |
| Path segments | base_link |
| **Transforms** | |
| Global position (m) | (0.000000, 0.000000, 0.000000) |
| Global position (mm) | (0.00, 0.00, 0.00) |
| Local transform (m) | (0.000000, 0.000000, 0.000000) |
| Assembly context depth | 0 |
| transform2 (m) | (0.000000, 0.000000, 0.000000) |
| **Physical** | |
| Mass | 18.851839 kg (18851.839 g) |
| Volume | 1.885184e-02 m³ |
| Density | 1000.0 kg/m³ |
| Surface area | 5.606795e-01 m² |
| Body count | 1 |
| CoM (component-local, m) | (0.000001, -0.000024, 0.310385) |
| CoM (global, m) | (0.000001, -0.000024, 0.310385) |
| CoM (global, mm) | (0.00, -0.02, 310.39) |
| **Inertia at origin (kg·m²)** | |
| Ixx, Iyy, Izz | 3.227259e+00, 3.240793e+00, 1.017141e-01 |
| Ixy, Ixz, Iyz | -2.540125e-05, -6.090551e-06, 4.018005e-04 |
| **Inertia at CoM (kg·m²)** | |
| Ixx, Iyy, Izz | 1.411093e+00, 1.424626e+00, 1.017141e-01 |
| Ixy, Ixz, Iyz | -2.540148e-05, -3.122552e-06, 2.611578e-04 |
| Bounding box (m) | 0.3000 × 0.2500 × 0.8950 |
| Bounding box (mm) | 300.00 × 250.00 × 895.00 |
| **Material & Appearance** | |
| Material | Acetal_Resin_White |
| Appearance | Opaque_179_179_179 |
| Color (RGB 0-1) | (0.702, 0.702, 0.702) |
| Color (RGB 0-255) | (179, 179, 179) |

## Joints

#### 🔧 Joint: `base_link_to_shoulder_pitch` (regular)

| Property | Value |
|----------|-------|
| Defining component | JAXV2_Assembly (JAXV2-Assembly v4) |
| Suppressed | False |
| Motion type | rigid (enum=0) |
| Axis | (0.0000, 0.0000, 1.0000) |
| **Connections** | |
| Parent (occ2) | `base_link` |
| Parent path | `base_link v3:1` |
| Child (occ1) | `Shoulder_Pitch` |
| Child path | `Shoulder_Pitch:1` |
| **Geometry (all sources, raw cm)** | |
| geometryOrOriginOne | (-15.0000, -0.0000, 85.0000) |
| geometryOrOriginTwo | (-15.0000, -0.0000, 85.0000) |
| occ1.transform | (-16.0000, -0.0000, 85.0000) ctx_depth=0 |
| occ1.global (assembled) | (-16.0000, -0.0000, 85.0000) |
| occ2.transform | (0.0000, 0.0000, 0.0000) ctx_depth=0 |
| occ2.global (assembled) | (0.0000, 0.0000, 0.0000) |
| **Picked origin (m)** | (-0.150000, -0.000000, 0.850000) via `geometryOrOriginOne` |

#### 🔧 Joint: `finger_1_1_to_finger_1_2` (regular)

| Property | Value |
|----------|-------|
| Defining component | JAXV2_Assembly (JAXV2-Assembly v4) |
| Suppressed | False |
| Motion type | revolute (enum=1) |
| Axis | (-0.0000, -1.0000, 0.0000) |
| **Connections** | |
| Parent (occ2) | `Finger_1_1` |
| Parent path | `Finger_1-1:1` |
| Child (occ1) | `Finger_1_2` |
| Child path | `Finger_1-2:1` |
| **Geometry (all sources, raw cm)** | |
| geometryOrOriginOne | (-28.2000, -3.1826, 18.6690) |
| geometryOrOriginTwo | (-28.2000, -3.1826, 18.6690) |
| occ1.transform | (-27.5263, 1.1276, 26.6373) ctx_depth=0 |
| occ1.global (assembled) | (-27.5263, 1.1276, 26.6373) |
| occ2.transform | (-27.5500, 1.0674, 26.6940) ctx_depth=0 |
| occ2.global (assembled) | (-27.5500, 1.0674, 26.6940) |
| **Picked origin (m)** | (-0.282000, -0.031826, 0.186690) via `geometryOrOriginOne` |
| Rotation limits (rad) | [0.0000, 2.0944] |

#### 🔧 Joint: `finger_1_2_to_finger_2_2` (regular)

| Property | Value |
|----------|-------|
| Defining component | JAXV2_Assembly (JAXV2-Assembly v4) |
| Suppressed | False |
| Motion type | revolute (enum=1) |
| Axis | (-0.0000, -1.0000, 0.0000) |
| **Connections** | |
| Parent (occ2) | `Finger_2_1` |
| Parent path | `Finger_2-1:1` |
| Child (occ1) | `Finger_2_2` |
| Child path | `Finger_2-2:1` |
| **Geometry (all sources, raw cm)** | |
| geometryOrOriginOne | (-28.2000, 0.5174, 18.6690) |
| geometryOrOriginTwo | (-28.2000, 0.5174, 18.6690) |
| occ1.transform | (-27.5500, 1.0974, 26.6940) ctx_depth=0 |
| occ1.global (assembled) | (-27.5500, 1.0974, 26.6940) |
| occ2.transform | (-27.5500, 1.0674, 26.6940) ctx_depth=0 |
| occ2.global (assembled) | (-27.5500, 1.0674, 26.6940) |
| **Picked origin (m)** | (-0.282000, 0.005174, 0.186690) via `geometryOrOriginOne` |
| Rotation limits (rad) | [0.0000, 2.0944] |

#### 🔧 Joint: `finger_3_1_to_finger_3_2` (regular)

| Property | Value |
|----------|-------|
| Defining component | JAXV2_Assembly (JAXV2-Assembly v4) |
| Suppressed | False |
| Motion type | revolute (enum=1) |
| Axis | (-0.0000, -1.0000, -0.0000) |
| **Connections** | |
| Parent (occ2) | `Finger_3_1` |
| Parent path | `Finger_3-1:1` |
| Child (occ1) | `Finger_3_2` |
| Child path | `Finger_3-2:1` |
| **Geometry (all sources, raw cm)** | |
| geometryOrOriginOne | (-28.0913, -6.9170, 18.7106) |
| geometryOrOriginTwo | (-28.0913, -6.9170, 18.7106) |
| occ1.transform | (-24.4413, -5.8355, 26.7356) ctx_depth=0 |
| occ1.global (assembled) | (-24.4413, -5.8355, 26.7356) |
| occ2.transform | (-24.4413, -5.8655, 26.7356) ctx_depth=0 |
| occ2.global (assembled) | (-24.4413, -5.8655, 26.7356) |
| **Picked origin (m)** | (-0.280913, -0.069170, 0.187106) via `geometryOrOriginOne` |
| Rotation limits (rad) | [0.0000, 2.0944] |

#### 🔧 Joint: `hand_base_to_finger_1_1` (regular)

| Property | Value |
|----------|-------|
| Defining component | JAXV2_Assembly (JAXV2-Assembly v4) |
| Suppressed | False |
| Motion type | revolute (enum=1) |
| Axis | (-0.0000, -1.0000, 0.0000) |
| **Connections** | |
| Parent (occ2) | `MG996R` |
| Parent path | `MG996R:1` |
| Child (occ1) | `Finger_1_1` |
| Child path | `Finger_1-1:1` |
| **Geometry (all sources, raw cm)** | |
| geometryOrOriginOne | (-28.2000, -3.2126, 25.1690) |
| geometryOrOriginTwo | (-28.2000, -3.2126, 25.1690) |
| occ1.transform | (-27.5500, 1.0674, 26.6940) ctx_depth=0 |
| occ1.global (assembled) | (-27.5500, 1.0674, 26.6940) |
| occ2.transform | (-27.5500, 1.0374, 26.6940) ctx_depth=0 |
| occ2.global (assembled) | (-27.5500, 1.0374, 26.6940) |
| **Picked origin (m)** | (-0.282000, -0.032126, 0.251690) via `geometryOrOriginOne` |
| Rotation limits (rad) | [-1.5708, 1.5708] |

#### 🔧 Joint: `hand_base_to_finger_2_1` (regular)

| Property | Value |
|----------|-------|
| Defining component | JAXV2_Assembly (JAXV2-Assembly v4) |
| Suppressed | False |
| Motion type | revolute (enum=1) |
| Axis | (-0.0000, -1.0000, 0.0000) |
| **Connections** | |
| Parent (occ2) | `MG996R` |
| Parent path | `MG996R:1` |
| Child (occ1) | `Finger_2_1` |
| Child path | `Finger_2-1:1` |
| **Geometry (all sources, raw cm)** | |
| geometryOrOriginOne | (-28.2000, 0.4874, 25.1690) |
| geometryOrOriginTwo | (-28.2000, 0.4874, 25.1690) |
| occ1.transform | (-27.5500, 1.0674, 26.6940) ctx_depth=0 |
| occ1.global (assembled) | (-27.5500, 1.0674, 26.6940) |
| occ2.transform | (-27.5500, 1.0374, 26.6940) ctx_depth=0 |
| occ2.global (assembled) | (-27.5500, 1.0374, 26.6940) |
| **Picked origin (m)** | (-0.282000, 0.004874, 0.251690) via `geometryOrOriginOne` |
| Rotation limits (rad) | [-1.5708, 1.5708] |

#### 🔧 Joint: `hand_base_to_rotation_base` (regular)

| Property | Value |
|----------|-------|
| Defining component | JAXV2_Assembly (JAXV2-Assembly v4) |
| Suppressed | False |
| Motion type | revolute (enum=1) |
| Axis | (-0.0000, -0.0000, -1.0000) |
| **Connections** | |
| Parent (occ2) | `MG996R` |
| Parent path | `MG996R:1` |
| Child (occ1) | `RotationBody` |
| Child path | `RotationBody:1` |
| **Geometry (all sources, raw cm)** | |
| geometryOrOriginOne | (-25.9913, -2.4352, 27.8254) |
| geometryOrOriginTwo | (-25.9913, -2.4352, 27.8254) |
| occ1.transform | (-24.4413, -5.8955, 26.7356) ctx_depth=0 |
| occ1.global (assembled) | (-24.4413, -5.8955, 26.7356) |
| occ2.transform | (-27.5500, 1.0374, 26.6940) ctx_depth=0 |
| occ2.global (assembled) | (-27.5500, 1.0374, 26.6940) |
| **Picked origin (m)** | (-0.259913, -0.024352, 0.278254) via `geometryOrOriginOne` |
| Rotation limits (rad) | [-3.1416, 0.0000] |

#### 🔧 Joint: `rotation_base_to_finger_3_1` (regular)

| Property | Value |
|----------|-------|
| Defining component | JAXV2_Assembly (JAXV2-Assembly v4) |
| Suppressed | False |
| Motion type | revolute (enum=1) |
| Axis | (-0.0000, -1.0000, 0.0000) |
| **Connections** | |
| Parent (occ2) | `RotationBody` |
| Parent path | `RotationBody:1` |
| Child (occ1) | `Finger_3_1` |
| Child path | `Finger_3-1:1` |
| **Geometry (all sources, raw cm)** | |
| geometryOrOriginOne | (-28.0913, -6.9470, 25.2106) |
| geometryOrOriginTwo | (-28.0913, -6.9470, 25.2106) |
| occ1.transform | (-24.4413, -5.8655, 26.7356) ctx_depth=0 |
| occ1.global (assembled) | (-24.4413, -5.8655, 26.7356) |
| occ2.transform | (-24.4413, -5.8955, 26.7356) ctx_depth=0 |
| occ2.global (assembled) | (-24.4413, -5.8955, 26.7356) |
| **Picked origin (m)** | (-0.280913, -0.069470, 0.252106) via `geometryOrOriginOne` |
| Rotation limits (rad) | [-1.5708, 1.5708] |

#### 🔧 Joint: `shoulder_pitch_to_shoulder_roll` (regular)

| Property | Value |
|----------|-------|
| Defining component | JAXV2_Assembly (JAXV2-Assembly v4) |
| Suppressed | False |
| Motion type | revolute (enum=1) |
| Axis | (-1.0000, 0.0000, 0.0000) |
| **Connections** | |
| Parent (occ2) | `Shoulder_Pitch` |
| Parent path | `Shoulder_Pitch:1` |
| Child (occ1) | `Shoulder_Roll` |
| Child path | `Shoulder_Roll:1` |
| **Geometry (all sources, raw cm)** | |
| geometryOrOriginOne | (-21.2000, -0.0000, 85.0000) |
| geometryOrOriginTwo | (-21.2000, -0.0000, 85.0000) |
| occ1.transform | (-21.2000, -0.0000, 85.0000) ctx_depth=0 |
| occ1.global (assembled) | (-21.2000, -0.0000, 85.0000) |
| occ2.transform | (-16.0000, -0.0000, 85.0000) ctx_depth=0 |
| occ2.global (assembled) | (-16.0000, -0.0000, 85.0000) |
| **Picked origin (m)** | (-0.212000, -0.000000, 0.850000) via `geometryOrOriginOne` |
| Rotation limits (rad) | [-0.7854, 0.7854] |

#### 🔧 Joint: `shoulder_roll_to_shoulder_yaw` (regular)

| Property | Value |
|----------|-------|
| Defining component | JAXV2_Assembly (JAXV2-Assembly v4) |
| Suppressed | False |
| Motion type | revolute (enum=1) |
| Axis | (0.0000, 1.0000, -0.0000) |
| **Connections** | |
| Parent (occ2) | `Shoulder_Roll` |
| Parent path | `Shoulder_Roll:1` |
| Child (occ1) | `Shoulder_Yaw` |
| Child path | `Shoulder_Yaw:1` |
| **Geometry (all sources, raw cm)** | |
| geometryOrOriginOne | (-26.2000, -2.4500, 85.0000) |
| geometryOrOriginTwo | (-26.2000, -2.4500, 85.0000) |
| occ1.transform | (-26.2000, -2.9000, 85.0000) ctx_depth=0 |
| occ1.global (assembled) | (-26.2000, -2.9000, 85.0000) |
| occ2.transform | (-21.2000, -0.0000, 85.0000) ctx_depth=0 |
| occ2.global (assembled) | (-21.2000, -0.0000, 85.0000) |
| **Picked origin (m)** | (-0.262000, -0.024500, 0.850000) via `geometryOrOriginOne` |
| Rotation limits (rad) | [0.0000, 0.7854] |

#### 🔧 Joint: `shoulder_yaw_to_upper_arm` (regular)

| Property | Value |
|----------|-------|
| Defining component | JAXV2_Assembly (JAXV2-Assembly v4) |
| Suppressed | False |
| Motion type | revolute (enum=1) |
| Axis | (0.0000, -0.0000, -1.0000) |
| **Connections** | |
| Parent (occ2) | `Shoulder_Yaw` |
| Parent path | `Shoulder_Yaw:1` |
| Child (occ1) | `Upper_Arm` |
| Child path | `Upper_Arm:1` |
| **Geometry (all sources, raw cm)** | |
| geometryOrOriginOne | (-26.2000, 0.1000, 72.3000) |
| geometryOrOriginTwo | (-26.2000, 0.1000, 72.3000) |
| occ1.transform | (-26.2000, 0.1000, 72.3000) ctx_depth=0 |
| occ1.global (assembled) | (-26.2000, 0.1000, 72.3000) |
| occ2.transform | (-26.2000, -2.9000, 85.0000) ctx_depth=0 |
| occ2.global (assembled) | (-26.2000, -2.9000, 85.0000) |
| **Picked origin (m)** | (-0.262000, 0.001000, 0.723000) via `geometryOrOriginOne` |
| Rotation limits (rad) | [-1.5708, 4.1888] |

#### 🔧 Joint: `under_arm_to_wrist_pitch` (regular)

| Property | Value |
|----------|-------|
| Defining component | JAXV2_Assembly (JAXV2-Assembly v4) |
| Suppressed | False |
| Motion type | revolute (enum=1) |
| Axis | (-0.0000, 0.0000, 1.0000) |
| **Connections** | |
| Parent (occ2) | `Under_Arm` |
| Parent path | `Under_Arm:1` |
| Child (occ1) | `Wrist_Pitch` |
| Child path | `Wrist_Pitch:1` |
| **Geometry (all sources, raw cm)** | |
| geometryOrOriginOne | (-26.2000, 0.1208, 47.2211) |
| geometryOrOriginTwo | (-26.2000, 0.1208, 47.2211) |
| occ1.transform | (-26.2000, 0.1208, 47.1211) ctx_depth=0 |
| occ1.global (assembled) | (-26.2000, 0.1208, 47.1211) |
| occ2.transform | (-28.6500, 0.1208, 59.3211) ctx_depth=0 |
| occ2.global (assembled) | (-28.6500, 0.1208, 59.3211) |
| **Picked origin (m)** | (-0.262000, 0.001208, 0.472211) via `geometryOrOriginOne` |
| Rotation limits (rad) | [-1.5708, 1.5708] |

#### 🔧 Joint: `upper_arm_to_under_arm` (regular)

| Property | Value |
|----------|-------|
| Defining component | JAXV2_Assembly (JAXV2-Assembly v4) |
| Suppressed | False |
| Motion type | revolute (enum=1) |
| Axis | (-1.0000, 0.0000, 0.0000) |
| **Connections** | |
| Parent (occ2) | `Upper_Arm` |
| Parent path | `Upper_Arm:1` |
| Child (occ1) | `Under_Arm` |
| Child path | `Under_Arm:1` |
| **Geometry (all sources, raw cm)** | |
| geometryOrOriginOne | (-28.6500, 0.1208, 59.3208) |
| geometryOrOriginTwo | (-28.6500, 0.1208, 59.3208) |
| occ1.transform | (-28.6500, 0.1208, 59.3211) ctx_depth=0 |
| occ1.global (assembled) | (-28.6500, 0.1208, 59.3211) |
| occ2.transform | (-26.2000, 0.1000, 72.3000) ctx_depth=0 |
| occ2.global (assembled) | (-26.2000, 0.1000, 72.3000) |
| **Picked origin (m)** | (-0.286500, 0.001208, 0.593208) via `geometryOrOriginOne` |
| Rotation limits (rad) | [-1.8326, 1.8326] |

#### 🔧 Joint: `wrist_pitch_to_wrist_yaw` (regular)

| Property | Value |
|----------|-------|
| Defining component | JAXV2_Assembly (JAXV2-Assembly v4) |
| Suppressed | False |
| Motion type | revolute (enum=1) |
| Axis | (-1.0000, 0.0000, 0.0000) |
| **Connections** | |
| Parent (occ2) | `Wrist_Pitch` |
| Parent path | `Wrist_Pitch:1` |
| Child (occ1) | `Wrist_yaw` |
| Child path | `Wrist_yaw:1` |
| **Geometry (all sources, raw cm)** | |
| geometryOrOriginOne | (-28.2000, 0.1208, 40.1011) |
| geometryOrOriginTwo | (-28.2000, 0.1208, 40.1011) |
| occ1.transform | (-28.2000, 0.1208, 40.1011) ctx_depth=0 |
| occ1.global (assembled) | (-28.2000, 0.1208, 40.1011) |
| occ2.transform | (-26.2000, 0.1208, 47.1211) ctx_depth=0 |
| occ2.global (assembled) | (-26.2000, 0.1208, 47.1211) |
| **Picked origin (m)** | (-0.282000, 0.001208, 0.401011) via `geometryOrOriginOne` |
| Rotation limits (rad) | [-1.5708, 1.5708] |

#### 🔧 Joint: `wrist_yaw_to_hand_base` (regular)

| Property | Value |
|----------|-------|
| Defining component | JAXV2_Assembly (JAXV2-Assembly v4) |
| Suppressed | False |
| Motion type | revolute (enum=1) |
| Axis | (-0.0000, 0.0000, 1.0000) |
| **Connections** | |
| Parent (occ2) | `Wrist_yaw` |
| Parent path | `Wrist_yaw:1` |
| Child (occ1) | `MG996R` |
| Child path | `MG996R:1` |
| **Geometry (all sources, raw cm)** | |
| geometryOrOriginOne | (-26.2000, 0.1208, 32.4011) |
| geometryOrOriginTwo | (-26.2000, 0.1208, 32.4011) |
| occ1.transform | (-27.5500, 1.0374, 26.6940) ctx_depth=0 |
| occ1.global (assembled) | (-27.5500, 1.0374, 26.6940) |
| occ2.transform | (-28.2000, 0.1208, 40.1011) ctx_depth=0 |
| occ2.global (assembled) | (-28.2000, 0.1208, 40.1011) |
| **Picked origin (m)** | (-0.262000, 0.001208, 0.324011) via `geometryOrOriginOne` |
| Rotation limits (rad) | [-2.0944, 2.0944] |

## Quick Comparison Table

Compare these values with Fusion 360 Properties panel (right-click → Properties).

| Component | Mass (g) | World X,Y,Z (mm) | CoM X,Y,Z (mm) | Material |
|-----------|----------|-------------------|-----------------|----------|
| Finger_1_1 | 71.419 | (-275.50, 10.67, 266.94) | (-205.47, 34.71, 273.45) | Acetal_Resin_White |
| Finger_1_2 | 11.312 | (-275.26, 11.28, 266.37) | (-167.71, 40.27, 273.11) | Acetal_Resin_White |
| Finger_2_1 | 71.419 | (-275.50, 10.67, 266.94) | (-205.47, -2.29, 273.45) | Acetal_Resin_White |
| Finger_2_2 | 11.312 | (-275.50, 10.97, 266.94) | (-167.38, 2.67, 273.44) | Acetal_Resin_White |
| Finger_3_1 | 71.419 | (-244.41, -58.65, 267.36) | (-174.39, -50.70, 230.85) | Acetal_Resin_White |
| Finger_3_2 | 11.312 | (-244.41, -58.35, 267.36) | (-136.29, -55.07, 230.86) | Acetal_Resin_White |
| MG996R | 419.885 | (-275.50, 10.37, 266.94) | (-290.33, 8.08, 267.46) | Generic |
| RotationBody | 74.463 | (-244.41, -58.95, 267.36) | (-240.39, -48.78, 231.21) | Acetal_Resin_White |
| Shoulder_Pitch | 997.346 | (-160.00, -0.00, 850.00) | (-184.40, 0.00, 850.31) | Acetal_Resin_White |
| Shoulder_Roll | 1087.699 | (-212.00, -0.00, 850.00) | (-259.98, 2.83, 850.22) | Acetal_Resin_White |
| Shoulder_Yaw | 1145.356 | (-262.00, -29.00, 850.00) | (-262.00, -2.36, 758.61) | Acetal_Resin_White |
| Under_Arm | 346.145 | (-286.50, 1.21, 593.21) | (-275.05, 6.01, 534.47) | Acetal_Resin_White |
| Upper_Arm | 410.582 | (-262.00, 1.00, 723.00) | (-253.27, 0.79, 632.12) | Acetal_Resin_White |
| Wrist_Pitch | 189.235 | (-262.00, 1.21, 471.21) | (-252.92, 3.42, 429.08) | Acetal_Resin_White |
| Wrist_yaw | 146.251 | (-282.00, 1.21, 401.01) | (-277.86, 1.21, 368.54) | Acetal_Resin_White |
| base_link | 18851.839 | (0.00, 0.00, 0.00) | (0.00, -0.02, 310.39) | Acetal_Resin_White |

## Joint Origins Comparison

All origins shown in multiple coordinate systems for debugging.

| Joint | Source | Origin (cm, raw) | Origin (m, picked) | Motion | Axis |
|-------|--------|------------------|-------------------|--------|------|
| base_link_to_shoulder_pitch | geometryOrOriginOne | goo1(-15.00, -0.00, 85.00) | (-0.1500, -0.0000, 0.8500) | rigid | (0.0, 0.0, 1.0) |
| finger_1_1_to_finger_1_2 | geometryOrOriginOne | goo1(-28.20, -3.18, 18.67) | (-0.2820, -0.0318, 0.1867) | revolute | (-0.0, -1.0, 0.0) |
| finger_1_2_to_finger_2_2 | geometryOrOriginOne | goo1(-28.20, 0.52, 18.67) | (-0.2820, 0.0052, 0.1867) | revolute | (-0.0, -1.0, 0.0) |
| finger_3_1_to_finger_3_2 | geometryOrOriginOne | goo1(-28.09, -6.92, 18.71) | (-0.2809, -0.0692, 0.1871) | revolute | (-0.0, -1.0, -0.0) |
| hand_base_to_finger_1_1 | geometryOrOriginOne | goo1(-28.20, -3.21, 25.17) | (-0.2820, -0.0321, 0.2517) | revolute | (-0.0, -1.0, 0.0) |
| hand_base_to_finger_2_1 | geometryOrOriginOne | goo1(-28.20, 0.49, 25.17) | (-0.2820, 0.0049, 0.2517) | revolute | (-0.0, -1.0, 0.0) |
| hand_base_to_rotation_base | geometryOrOriginOne | goo1(-25.99, -2.44, 27.83) | (-0.2599, -0.0244, 0.2783) | revolute | (-0.0, -0.0, -1.0) |
| rotation_base_to_finger_3_1 | geometryOrOriginOne | goo1(-28.09, -6.95, 25.21) | (-0.2809, -0.0695, 0.2521) | revolute | (-0.0, -1.0, 0.0) |
| shoulder_pitch_to_shoulder_roll | geometryOrOriginOne | goo1(-21.20, -0.00, 85.00) | (-0.2120, -0.0000, 0.8500) | revolute | (-1.0, 0.0, 0.0) |
| shoulder_roll_to_shoulder_yaw | geometryOrOriginOne | goo1(-26.20, -2.45, 85.00) | (-0.2620, -0.0245, 0.8500) | revolute | (0.0, 1.0, -0.0) |
| shoulder_yaw_to_upper_arm | geometryOrOriginOne | goo1(-26.20, 0.10, 72.30) | (-0.2620, 0.0010, 0.7230) | revolute | (0.0, -0.0, -1.0) |
| under_arm_to_wrist_pitch | geometryOrOriginOne | goo1(-26.20, 0.12, 47.22) | (-0.2620, 0.0012, 0.4722) | revolute | (-0.0, 0.0, 1.0) |
| upper_arm_to_under_arm | geometryOrOriginOne | goo1(-28.65, 0.12, 59.32) | (-0.2865, 0.0012, 0.5932) | revolute | (-1.0, 0.0, 0.0) |
| wrist_pitch_to_wrist_yaw | geometryOrOriginOne | goo1(-28.20, 0.12, 40.10) | (-0.2820, 0.0012, 0.4010) | revolute | (-1.0, 0.0, 0.0) |
| wrist_yaw_to_hand_base | geometryOrOriginOne | goo1(-26.20, 0.12, 32.40) | (-0.2620, 0.0012, 0.3240) | revolute | (-0.0, 0.0, 1.0) |
