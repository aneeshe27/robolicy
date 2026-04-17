# External Gazebo Pick-and-Place Pivot

This folder is now the primary Gazebo path for the demo.

## Fresh Clone Setup

If you cloned this repo from GitHub, fetch the upstream repos first:

```bash
./scripts/fetch_external_demo_repos.sh
```

That script populates:

- `external/panda_simulator`
- `external/pick-and-place`
- `external/openpi`

## Why we pivoted

The hand-rolled Panda cell in this repo was not reliable enough for a serious demo. The robot pose looked physically wrong, the joint conventions were inconsistent, and the overall setup was becoming harder to trust than to replace.

For the `pi0 fails -> LLM repairs policy-as-code` story, it is better to start from an existing manipulation stack that already has:

- a sane Franka Panda Gazebo model
- ROS controllers and MoveIt integration
- an existing pick-and-place task

## Upstream repos

- [external/panda_simulator](/home/aneeshe/projects/robolicy/external/panda_simulator): Gazebo simulator for the Franka Panda with ROS interface, controllers, and MoveIt support.
- [external/pick-and-place](/home/aneeshe/projects/robolicy/external/pick-and-place): Panda pick-and-place application in ROS/Gazebo using a state machine and object detection.

These fit together directly because `pick-and-place` explicitly depends on `panda_simulator`.

## Repo-local workspace

This repo now includes a local catkin workspace scaffold at [external/noetic_ws](/home/aneeshe/projects/robolicy/external/noetic_ws).

To link the two upstream repos into that workspace:

```bash
./scripts/setup_external_pickplace_ws.sh
```

If ROS Noetic is installed on the host and you want dependency setup as well:

```bash
./scripts/setup_external_pickplace_ws.sh --deps
```

To build the workspace on a host with ROS Noetic + catkin tools:

```bash
./scripts/build_external_pickplace_ws.sh
```

## Launch flow

After the workspace is built and sourced:

```bash
source external/noetic_ws/devel/setup.bash
roslaunch pick_and_place panda_world.launch
roslaunch panda_sim_moveit sim_move_group.launch
rosrun pick_and_place object_detector.py
rosrun pick_and_place pick_and_place_state_machine.py
```

## Docker path for SSH

This repo also includes a Dockerized ROS Noetic path so you can run the Gazebo task on `presto` over SSH and capture artifacts without changing machines.

Build the image:

```bash
./scripts/docker_build_external_pickplace_image.sh
```

Capture a clean still image of the task:

```bash
BOOT_WAIT=50 ./scripts/docker_capture_external_pickplace_image.sh outputs/pickplace_scene_clean.png
```

Record a short mp4:

```bash
BOOT_WAIT=50 DURATION=10 ./scripts/docker_record_external_pickplace.sh outputs/pickplace_demo_clean.mp4
```

Artifacts produced during setup:

- screenshot: [outputs/pickplace_scene_clean.png](/home/aneeshe/projects/robolicy/outputs/pickplace_scene_clean.png)
- video: [outputs/pickplace_demo_clean.mp4](/home/aneeshe/projects/robolicy/outputs/pickplace_demo_clean.mp4)

To copy one down locally:

```bash
scp aneeshe@presto:/home/aneeshe/projects/robolicy/outputs/pickplace_demo_clean.mp4 .
```

## OpenPI Zero-Shot Bridge

There is now a repo-local zero-shot bridge for trying a DROID-style `openpi` policy against this Panda Gazebo task.

Build the ROS image and the separate `openpi` server image:

```bash
./scripts/docker_build_external_pickplace_image.sh
./scripts/docker_build_openpi_server_image.sh
```

Start the GPU-backed `openpi` DROID server:

```bash
OPENPI_SERVER_PORT=8765 \
SERVER_ARGS='--port 8765 policy:checkpoint --policy.config=pi0_droid --policy.dir=gs://openpi-assets/checkpoints/pi0_droid' \
./scripts/docker_run_openpi_droid_server.sh
```

Then, from a second shell, start the Gazebo zero-shot rollout:

```bash
OPENPI_SERVER_PORT=8765 \
OPENPI_PROMPT="pick up the red cube and place it in the red bin" \
./scripts/docker_run_openpi_pickplace_zero_shot.sh
```

To record the rollout instead:

```bash
OPENPI_SERVER_PORT=8765 \
OPENPI_PROMPT="pick up the red cube and place it in the red bin" \
DURATION=12 \
./scripts/docker_record_openpi_pickplace_zero_shot.sh outputs/pickplace_openpi_zero_shot.mp4
```

This currently captures the full Gazebo window rather than a tightly cropped scene-only view.

Implementation notes:

- The bridge script is [scripts/openpi_pickplace_zero_shot.py](/home/aneeshe/projects/robolicy/scripts/openpi_pickplace_zero_shot.py).
- It sends Panda joint-position commands to `/panda_simulator/motion_controller/arm/joint_commands`.
- It uses the real Panda gripper action servers for open/close.
- It now includes a real robot-mounted wrist RGB camera plus a dedicated exterior camera in the custom `pick_and_place_openpi.world` scene.
- The current challenge is camera calibration quality, not fake grasp teleportation.

## Why this is a better fit for pi0

`openpi` says the public checkpoints are closest to ALOHA and the DROID Franka setup. That makes a Franka-based Gazebo stack a much better bridge to a later `pi0` demo than a custom abstract arm.

## Notes

- `panda_simulator` last cloned commit: `6cec91d` from 2021-05-27
- `pick-and-place` last cloned commit: `a4c3b22` from 2023-11-29
- both upstream repos target Ubuntu 20.04 + ROS Noetic + Gazebo 11
- I also checked `nicholaspalomo/panda_ros2_gazebo`, but its README says the `picknplace` demo is currently broken, so I do not recommend it as the starting point here
