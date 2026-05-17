# Intro to Robotics — Colour Sorting with the Niryo Ned2

A colour-based **pick-and-place sorting** project built for the **B38RO Introduction to
Robotics** lab (Year 2, Semester 2). Blocks arrive on a conveyor belt, a camera detects
each block's colour and position, and a 6-DOF **Niryo Ned2** arm picks them up and stacks
them onto separate red, green and blue piles.

The same task is implemented twice:

- **Simulation** — in [RoboDK](https://robodk.com/) (`main_sim.py` + `helpers_sim.py`)
- **Hardware** — on a real Niryo Ned2 over Wi-Fi using `pyniryo` (`main_hardware.py`)

A recorded run of the simulation is included in [`robot sim.mp4`](robot%20sim.mp4).

---

## Demo

| | |
|---|---|
| Video | [`robot sim.mp4`](robot%20sim.mp4) (tracked with Git LFS) |
| Station | [`Niryo.rdk`](Niryo.rdk) — open this in RoboDK |
| Lab brief | [`B38RO_Lab Manual.pdf`](B38RO_Lab%20Manual.pdf) |

---

## Repository structure

| File | Description |
|------|-------------|
| `main_sim.py` | Simulation entry point. Runs the full sort routine in RoboDK. |
| `helpers_sim.py` | `NiryoRobot` class: RoboDK connection, kinematics, camera, gripper and conveyor helpers. |
| `main_hardware.py` | Hardware entry point. Runs the same routine on a physical Niryo Ned2 via `pyniryo`. |
| `Niryo.rdk` | RoboDK station (robot, camera, conveyor, targets and parts). |
| `notes.md` | Course/lab notes. |
| `robot sim.mp4` | Demo recording of the simulation (Git LFS). |
| `B38RO_Lab Manual.pdf` | The lab manual the project is based on. |

---

## Requirements

**Python 3.10+**

### Simulation
- [RoboDK](https://robodk.com/) (desktop application)
- Python packages:
  ```bash
  pip install robodk numpy opencv-python
  ```

### Hardware
- A Niryo Ned2 robot with a conveyor belt, IR sensor and camera
- Python package:
  ```bash
  pip install pyniryo
  ```

---

## Running the simulation

1. Open the **RoboDK** desktop app.
2. Load the station file [`Niryo.rdk`](Niryo.rdk) (`File → Open`).
3. From this folder, run:
   ```bash
   python main_sim.py
   ```

The robot will repeatedly observe the conveyor, detect a block's colour with the camera,
pick it up and place it on the matching coloured pile until all 6 blocks are sorted.

## Running on the hardware

1. Connect to the robot's Wi-Fi hotspot.
   - Hotspot password: `niryorobot`
   - Robot IP used in the script: `10.10.10.10`
2. Make sure the camera **workspace name** in `main_hardware.py` matches the one saved on
   the robot (currently `"paufraser"`).
3. Run:
   ```bash
   python main_hardware.py
   ```

The script auto-calibrates the arm, starts the conveyor, and sorts blocks by colour,
lighting the LED ring and playing sounds for feedback.

> ⚠️ Edit `robot_ip_address` and `workspace_name` at the top of `main_hardware.py` if your
> robot uses different values.

---

## How it works

Both versions follow the same high-level loop:

1. **Move to the observation pose** so the camera can see the conveyor.
2. **Wait for a block** — the IR sensor detects when a part reaches the pick point and the
   conveyor stops.
3. **Detect colour and position:**
   - *Simulation:* a camera snapshot is converted to HSV, masked per colour, and circles
     are found with the Hough transform. The image point is back-projected through the
     camera intrinsics and transformed into robot-base coordinates.
   - *Hardware:* `get_target_pose_from_cam()` returns the block's pose and colour using the
     robot's saved vision workspace.
4. **Pick** the block (move to it, close the gripper).
5. **Place** it on the red, green or blue pile via an intermediate "approach" pose to
   avoid collisions. In the simulation the target height is raised after each block so
   they stack instead of colliding.
6. **Repeat** until all blocks are sorted, then return to the home pose.

### Kinematics (`helpers_sim.py`)

The `NiryoRobot` helper class wraps RoboDK and also implements the robotics theory from
the course:

- `forward_kinematics()` / `inverse_kinematics()` — analytic FK/IK via RoboDK.
- `jacobian_matrix()` — numerical Jacobian by finite differences.
- `numerical_inverse_kinematics()` — iterative IK using the Jacobian, with a singularity
  check.
- `move_with_jacobian()` — Cartesian velocity control through the inverse Jacobian.
- Camera helpers — intrinsic matrix, image→camera and camera→robot coordinate transforms,
  and HSV colour/circle detection.

---

## Notes

- `.mp4` files are stored with **Git LFS**. Install it (`git lfs install`) before cloning
  to get the demo video.
- This is **coursework**; values such as poses, joint angles and the camera workspace name
  are tuned for the specific lab setup and will need adjusting for other environments.
