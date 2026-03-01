# 3-DOF RRR Planar Robot Arm — Kinematic Analysis

## Project Description

This project presents a comprehensive kinematic analysis of a 3-DOF (three degrees of freedom) RRR planar robot arm. The analysis includes:

- **Denavit–Hartenberg (DH) modeling** of the robot's kinematic chain
- **Forward Kinematics (FK)** derivation and implementation
- **Inverse Kinematics (IK)** analytical solution with singularity analysis
- **Workspace analysis** under full and constrained joint limits

## Robot Configuration

| Parameter | Value |
|-----------|-------|
| Type | RRR Planar (3 revolute joints) |
| Link 1 (L₁) | 0.5 m |
| Link 2 (L₂) | 0.4 m |
| Link 3 (L₃) | 0.3 m |
| Joint limits | θᵢ ∈ [−π, π] |
| Max reach | 1.2 m |

## Repository Contents

```
├── main.tex              # LaTeX source for the report
├── main.pdf              # Compiled report (CI journal format)
├── src/
│   └── kinematics.py     # Python implementation (FK, IK, workspace)
├── figures/
│   ├── robot_arm.png
│   ├── fk_test.png
│   ├── ik_elbow_up.png
│   ├── ik_elbow_down.png
│   ├── workspace_full.png
│   └── workspace_restricted.png
└── README.md
```

## How to Run

### Requirements
```bash
pip install numpy matplotlib
```

### Execution
```bash
python src/kinematics.py
```

This will:
1. Compute forward kinematics for q = [0, π/4, π/2]
2. Solve inverse kinematics for target position (0.8, 0.2) m
3. Generate workspace visualizations

## Key Results

- **FK verification:** For q = [0, π/4, π/2], end-effector position = (0.5707, 0.4950) m, orientation = 135°
- **IK solutions:** Two valid configurations (elbow-up, elbow-down) found for target (0.8, 0.2) m
- **Singularity:** Occurs when θ₂ = 0 or θ₂ = π (det(J) = 0)

## References

1. Craig, J. J. (2005). *Introduction to Robotics: Mechanics and Control*. 3rd Ed., Pearson.
2. Siciliano, B. & Khatib, O. (2009). *Springer Handbook of Robotics*. Springer.
3. Spong, M. W., Hutchinson, S. & Vidyasagar, M. (2006). *Robot Modeling and Control*. Wiley.
4. Denavit, J. & Hartenberg, R. S. (1955). A Kinematic Notation for Lower-Pair Mechanisms. *ASME J. Applied Mechanics*, 22(2), 215–221.
5. Angeles, J. (2007). *Fundamentals of Robotic Mechanical Systems*. 3rd Ed., Springer.
6. Tsai, L.-W. (1999). *Robot Analysis*. Wiley.

## Author

Maedeh Abdollahi Baktashi
