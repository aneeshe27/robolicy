#!/usr/bin/env python3
"""Scripted ground-truth pick-and-place for the openpi Gazebo world.

Bypasses the VLA entirely. Reads the red cube and red bin poses directly
from Gazebo's ``/gazebo/get_model_state`` service, then drives the Panda
arm through pre-pick / pick / grasp / lift / pre-place / place / retract
waypoints. IK is computed with KDL via ``PandaArm.inverse_kinematics``
and joint targets are sent through the panda_simulator
``JointTrajectoryActionClient`` (``move_to_joint_position(...,
use_moveit=False)``), so no MoveIt / move_group is required.

Intended to be launched after:
  1. ``roslaunch pick_and_place panda_world.launch``

Arguments mirror a typical pick-and-place control loop. Defaults match the
``pick_and_place_openpi.world`` layout (5 cm red cube at (0.53, 0, 0.125);
red bin at (-0.5, 0, 0.11)).
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Optional, Tuple

import numpy as np
import rospy
import tf2_ros
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from tf.transformations import quaternion_from_euler


# Panda joint names, kept here to avoid importing ``panda_robot`` just for a constant
JOINT_NAMES = [
    "panda_joint1",
    "panda_joint2",
    "panda_joint3",
    "panda_joint4",
    "panda_joint5",
    "panda_joint6",
    "panda_joint7",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cube-model", default="block_red_target")
    parser.add_argument("--bin-model", default="bin_red")
    parser.add_argument(
        "--cube-height",
        type=float,
        default=0.04,
        help="Side length of the cube (m). Used to offset the grasp height.",
    )
    parser.add_argument(
        "--fingertip-z-extra",
        type=float,
        default=0.045,
        help=(
            "Extra distance (m) from the ``panda_*finger`` link origins "
            "(knuckles) down to the actual fingertip midpoint along the "
            "hand's tool axis. We add this to the measured knuckle midpoint "
            "during calibration so the grasp targets the very tip of the "
            "fingers, not the knuckle."
        ),
    )
    parser.add_argument(
        "--grasp-depth",
        type=float,
        default=0.02,
        help=(
            "How far (m) below the cube center the fingertip should drive "
            "during the grasp. Positive = deeper into/below the cube center. "
            "For a 4 cm cube sitting on a workbench, the cube center is 2 cm "
            "above the workbench, so a 2 cm grasp depth lands the fingertip "
            "almost at the workbench level -- fingers straddle the whole "
            "cube, not just the top edge."
        ),
    )
    parser.add_argument("--pre-pick-height", type=float, default=0.25, help="Pre-pick clearance above the cube (m).")
    parser.add_argument("--place-hover-height", type=float, default=0.30, help="EEF z above bin origin for hover (m).")
    parser.add_argument("--release-height", type=float, default=0.24, help="EEF z above bin origin for release (m).")
    parser.add_argument(
        "--travel-height",
        type=float,
        default=0.55,
        help=(
            "EEF z for the safe intermediate hover above the robot base when "
            "swinging between cube side (+x) and bin side (-x). Keeps the "
            "arm in a compact, high posture so joint trajectory errors don't "
            "preempt the motion and fling the cube out."
        ),
    )
    parser.add_argument(
        "--joint-speed",
        type=float,
        default=0.15,
        help=(
            "Joint position speed factor in [0,1] passed to "
            "PandaArm.set_joint_position_speed. Lower = slower & smoother "
            "trajectories, which dramatically reduces the chance the gripper "
            "shakes off the cube during large swings."
        ),
    )
    parser.add_argument("--grasp-width", type=float, default=0.0)
    parser.add_argument("--grasp-force", type=float, default=80.0)
    parser.add_argument(
        "--grasp-speed",
        type=float,
        default=0.02,
        help=(
            "Finger close speed (m/s). Slower = both fingers contact the cube "
            "at nearly the same instant, which keeps the cube centered "
            "instead of having the leading finger kick it into a corner grip."
        ),
    )
    parser.add_argument("--grasp-epsilon-inner", type=float, default=0.005)
    parser.add_argument("--grasp-epsilon-outer", type=float, default=0.06)
    parser.add_argument(
        "--grasp-settle-time",
        type=float,
        default=1.0,
        help=(
            "Seconds to sleep after the grasp action before the arm starts "
            "moving. Lets Gazebo's ODE physics converge on a stable finger/"
            "cube contact so the first upward acceleration doesn't fling "
            "the cube out."
        ),
    )
    parser.add_argument(
        "--attempts",
        type=int,
        default=1,
        help="How many times to retry the whole pick-and-place sequence if a grasp fails.",
    )
    parser.add_argument(
        "--skip-home-after",
        action="store_true",
        help="Don't move the arm back to the neutral pose at the end (keeps the last frame in the bin hover).",
    )
    parser.add_argument(
        "--ros-master-retries",
        type=int,
        default=60,
        help="How many seconds to wait for MoveIt + Gazebo services to come up before giving up.",
    )
    return parser.parse_args()


def wait_for_service(name: str, timeout_s: float) -> None:
    """Wait for ``name`` or exit with a clear error."""
    rospy.loginfo("Waiting up to %ds for service %s", int(timeout_s), name)
    try:
        rospy.wait_for_service(name, timeout=timeout_s)
    except rospy.ROSException as exc:
        rospy.logerr("Service %s never appeared: %s", name, exc)
        sys.exit(2)


def get_model_pose(service: rospy.ServiceProxy, model_name: str) -> np.ndarray:
    req = GetModelStateRequest()
    req.model_name = model_name
    req.relative_entity_name = "world"
    resp = service(req)
    if not resp.success:
        raise RuntimeError(f"/gazebo/get_model_state failed for '{model_name}': {resp.status_message}")
    return np.array([resp.pose.position.x, resp.pose.position.y, resp.pose.position.z], dtype=np.float64)


def lookup_xyz(tf_buffer: tf2_ros.Buffer, target_frame: str, source_frame: str = "world") -> np.ndarray:
    """Block briefly for the TF from ``source_frame`` to ``target_frame`` and return its translation."""
    t = tf_buffer.lookup_transform(source_frame, target_frame, rospy.Time(0), rospy.Duration(2.0))
    return np.array(
        [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z],
        dtype=np.float64,
    )


def calibrate_tip_offset(
    panda,
    tf_buffer: tf2_ros.Buffer,
    grasp_ori,
    ref_xyz: np.ndarray,
    fingertip_z_extra: float,
) -> np.ndarray:
    """Move the arm to ``ref_xyz`` with the grasp orientation, then measure the
    world-frame offset from the IK target frame (``panda_hand``) to the actual
    fingertip midpoint. Returns ``offset_world = tip_midpoint - hand_origin``.

    Because we always grasp with the same top-down ``grasp_ori``, the world
    offset is effectively constant across pick operations and can just be
    subtracted from the desired fingertip world position to get the EE target.
    """
    rospy.loginfo("[calibrate] moving to reference pose %s", np.round(ref_xyz, 3).tolist())
    seed = panda.joint_ordered_angles()
    ok, soln = panda.inverse_kinematics(pos=ref_xyz, ori=grasp_ori, seed=seed)
    if not ok or soln is None:
        raise RuntimeError("IK failed at calibration reference pose")
    panda.move_to_joint_position(list(soln), use_moveit=False)
    rospy.sleep(0.5)

    hand_xyz = lookup_xyz(tf_buffer, "panda_hand")
    left_xyz = lookup_xyz(tf_buffer, "panda_leftfinger")
    right_xyz = lookup_xyz(tf_buffer, "panda_rightfinger")
    knuckle_mid = 0.5 * (left_xyz + right_xyz)
    # Fingers extend in world -z (hand is rotated pitch=pi), so the tip midpoint
    # is fingertip_z_extra below the knuckle midpoint.
    tip_mid = knuckle_mid + np.array([0.0, 0.0, -abs(fingertip_z_extra)])
    offset = tip_mid - hand_xyz
    rospy.loginfo(
        "[calibrate] panda_hand=%s knuckle_mid=%s tip_mid=%s -> offset=%s",
        np.round(hand_xyz, 3).tolist(),
        np.round(knuckle_mid, 3).tolist(),
        np.round(tip_mid, 3).tolist(),
        np.round(offset, 3).tolist(),
    )
    return offset


def move_safe(panda, pos: np.ndarray, ori, tag: str, seed=None) -> bool:
    """Compute KDL IK for (pos, ori) and drive the arm to the resulting joint
    configuration via the panda_simulator trajectory action server. Returns
    True iff IK found a solution and the motion was executed.
    """
    rospy.loginfo("[%s] IK target pos=%s", tag, np.round(pos, 3).tolist())
    if seed is None:
        seed = panda.joint_ordered_angles()
    ok, soln = panda.inverse_kinematics(pos=pos, ori=ori, seed=seed)
    if not ok or soln is None:
        rospy.logwarn("[%s] IK failed for pos=%s", tag, np.round(pos, 3).tolist())
        return False
    panda.move_to_joint_position(list(soln), use_moveit=False)
    rospy.sleep(0.1)
    try:
        ee_pos, _ee_ori = panda.ee_pose()
        rospy.loginfo(
            "[%s] EE pose after move: pos=%s (target was %s, delta=%s)",
            tag,
            np.round(ee_pos, 3).tolist(),
            np.round(pos, 3).tolist(),
            np.round(np.array(ee_pos) - pos, 3).tolist(),
        )
    except Exception as exc:  # pragma: no cover - diagnostic only
        rospy.logdebug("[%s] ee_pose() failed: %s", tag, exc)
    return True


def run_sequence(
    panda,
    gripper,
    args: argparse.Namespace,
    cube_xyz: np.ndarray,
    bin_xyz: np.ndarray,
    tip_offset_world: np.ndarray,
) -> bool:
    """One attempt. Returns True iff the grasp action server reported success.

    ``tip_offset_world`` is the measured vector from the IK target frame
    (``panda_hand``) to the actual fingertip midpoint in world coordinates,
    under the same ``grasp_ori``. We invert it to convert "desired fingertip
    world position" into "EE target for IK".
    """
    grasp_ori = quaternion_from_euler(0.0, np.pi, 0.0)

    # Desired fingertip midpoint in world: directly above the cube in x/y,
    # and at cube center height minus the grasp depth, so the fingers clamp
    # around the middle of the cube rather than pinching just the top edge.
    desired_tip_pick = np.array(
        [cube_xyz[0], cube_xyz[1], cube_xyz[2] - args.grasp_depth]
    )
    pick_ee = desired_tip_pick - tip_offset_world
    pre_pick_ee = pick_ee + np.array([0.0, 0.0, args.pre_pick_height])

    # Place waypoints: we want the fingertip midpoint to be at
    # (bin.x, bin.y, bin.z + release_height); convert to EE target the same way.
    desired_tip_hover = np.array([bin_xyz[0], bin_xyz[1], bin_xyz[2] + args.place_hover_height])
    desired_tip_release = np.array([bin_xyz[0], bin_xyz[1], bin_xyz[2] + args.release_height])
    place_hover_ee = desired_tip_hover - tip_offset_world
    release_ee = desired_tip_release - tip_offset_world

    # Intermediate hover midway between pick and place, at travel_height, so
    # the IK stays in a single continuous branch throughout the swing and we
    # don't trigger the JointTrajectoryActionServer error threshold.
    mid_xy = 0.5 * (np.array([cube_xyz[0], cube_xyz[1]]) + np.array([bin_xyz[0], bin_xyz[1]]))
    mid_pick_to_bin = np.array([mid_xy[0], mid_xy[1], args.travel_height])
    mid_bin_to_pick = mid_pick_to_bin.copy()

    rospy.loginfo("Cube pose = %s", np.round(cube_xyz, 3).tolist())
    rospy.loginfo("Bin  pose = %s", np.round(bin_xyz, 3).tolist())
    rospy.loginfo(
        "Desired fingertip mid at pick = %s (cube center = %s, grasp_depth=%.3f)",
        np.round(desired_tip_pick, 3).tolist(),
        np.round(cube_xyz, 3).tolist(),
        args.grasp_depth,
    )
    rospy.loginfo(
        "Pick EE target (panda_hand) = %s  tip_offset_world = %s",
        np.round(pick_ee, 3).tolist(),
        np.round(tip_offset_world, 3).tolist(),
    )

    gripper.open()
    rospy.sleep(0.3)

    move_safe(panda, pre_pick_ee, grasp_ori, "pre-pick")
    move_safe(panda, pick_ee, grasp_ori, "pick")

    rospy.loginfo("Commanding grasp (width=%.3f m, force=%.1f N)", args.grasp_width, args.grasp_force)
    grasp_ok = gripper.grasp(
        width=args.grasp_width,
        force=args.grasp_force,
        speed=args.grasp_speed,
        epsilon_inner=args.grasp_epsilon_inner,
        epsilon_outer=args.grasp_epsilon_outer,
    )
    rospy.loginfo("Grasp action result: %s", grasp_ok)

    # Give Gazebo ODE time to establish stable contact between fingers and
    # cube. Without this the first waypoint after the grasp accelerates
    # before the contact constraint has settled and the cube slips out.
    if args.grasp_settle_time > 0:
        rospy.loginfo("Settling grasp for %.2fs", args.grasp_settle_time)
        rospy.sleep(args.grasp_settle_time)

    move_safe(panda, pre_pick_ee, grasp_ori, "post-pick-lift")
    move_safe(panda, mid_pick_to_bin, grasp_ori, "transit-to-bin")
    move_safe(panda, place_hover_ee, grasp_ori, "pre-place")
    move_safe(panda, release_ee, grasp_ori, "place")

    gripper.open()
    rospy.sleep(0.3)

    move_safe(panda, place_hover_ee, grasp_ori, "post-place-lift")
    move_safe(panda, mid_bin_to_pick, grasp_ori, "transit-back")
    return bool(grasp_ok)


def main() -> None:
    args = parse_args()
    rospy.init_node("pickplace_scripted_groundtruth", anonymous=False)

    wait_for_service("/gazebo/get_model_state", timeout_s=args.ros_master_retries)

    # Importing ``panda_robot`` / ``franka_interface`` pulls in the whole
    # franka stack and blocks until action servers are up, so we do it here
    # after we know ROS is alive.
    from franka_interface import GripperInterface
    from panda_robot import PandaArm

    rospy.loginfo("Constructing PandaArm + GripperInterface (this can take several seconds)")
    panda = PandaArm()
    gripper = GripperInterface()

    rospy.loginfo("Setting joint-position speed factor = %.2f", args.joint_speed)
    panda.set_joint_position_speed(float(args.joint_speed))

    rospy.loginfo("Moving to neutral pose")
    panda.move_to_neutral()
    rospy.sleep(0.3)

    model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    cube_xyz = get_model_pose(model_state, args.cube_model)
    bin_xyz = get_model_pose(model_state, args.bin_model)

    # Measure the true fingertip-midpoint offset relative to the IK frame
    # (panda_hand) under the grasp orientation. We calibrate above the cube so
    # IK converges easily, and then reuse the offset for every pick & place.
    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(1.0)
    grasp_ori_q = quaternion_from_euler(0.0, np.pi, 0.0)
    cal_ref = np.array([cube_xyz[0], cube_xyz[1], cube_xyz[2] + args.pre_pick_height + 0.1])
    tip_offset_world = calibrate_tip_offset(
        panda,
        tf_buffer,
        grasp_ori_q,
        cal_ref,
        args.fingertip_z_extra,
    )

    grasp_ok = False
    for attempt in range(1, max(1, args.attempts) + 1):
        rospy.loginfo("===== Pick-and-place attempt %d/%d =====", attempt, args.attempts)
        grasp_ok = run_sequence(panda, gripper, args, cube_xyz, bin_xyz, tip_offset_world)
        if grasp_ok:
            break
        rospy.logwarn("Attempt %d ended without a successful grasp; re-reading cube pose and retrying", attempt)
        time.sleep(0.5)
        cube_xyz = get_model_pose(model_state, args.cube_model)

    if not args.skip_home_after:
        rospy.loginfo("Returning to neutral pose")
        panda.move_to_neutral()

    rospy.loginfo("Pick-and-place finished (grasp_reported_ok=%s)", grasp_ok)


if __name__ == "__main__":
    main()
