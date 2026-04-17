#!/usr/bin/env python3

import argparse
import os
import sys
import time

import actionlib
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from franka_core_msgs.msg import JointCommand
from franka_gripper.msg import GraspAction
from franka_gripper.msg import GraspGoal
from franka_gripper.msg import MoveAction
from franka_gripper.msg import MoveGoal
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState


REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
OPENPI_CLIENT_SRC = os.path.join(REPO_ROOT, "external", "openpi", "packages", "openpi-client", "src")
if OPENPI_CLIENT_SRC not in sys.path:
    sys.path.insert(0, OPENPI_CLIENT_SRC)

from openpi_client import image_tools
from openpi_client import websocket_client_policy


JOINT_NAMES = [f"panda_joint{i}" for i in range(1, 8)]


class ObservationBuffer:
    def __init__(self, args):
        self._args = args
        self._bridge = CvBridge()
        self.exterior_rgb = None
        self.wrist_rgb = None
        self.joint_position = None
        self.gripper_position = None
        self._saved_debug_observation = False

        rospy.Subscriber(args.exterior_image_topic, Image, self._exterior_cb, queue_size=1)
        if args.use_exterior_as_wrist:
            rospy.logwarn("Using exterior camera frame as a fake wrist camera. This is a major embodiment mismatch for DROID checkpoints.")
        elif args.wrist_image_topic:
            rospy.Subscriber(args.wrist_image_topic, Image, self._wrist_cb, queue_size=1)
        rospy.Subscriber(args.joint_state_topic, JointState, self._joint_state_cb, queue_size=1)
        rospy.Subscriber(args.gripper_joint_state_topic, JointState, self._gripper_state_cb, queue_size=1)

    def _exterior_cb(self, msg):
        rgb = self._bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        self.exterior_rgb = np.asarray(rgb, dtype=np.uint8)
        if self._args.use_exterior_as_wrist:
            self.wrist_rgb = self.exterior_rgb

    def _wrist_cb(self, msg):
        rgb = self._bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        self.wrist_rgb = np.asarray(rgb, dtype=np.uint8)

    def _joint_state_cb(self, msg):
        if not all(name in msg.name for name in JOINT_NAMES):
            return
        self.joint_position = np.asarray(
            [msg.position[msg.name.index(name)] for name in JOINT_NAMES],
            dtype=np.float32,
        )

    def _gripper_state_cb(self, msg):
        finger_joints = [name for name in msg.name if name.startswith("panda_finger_joint")]
        if len(finger_joints) < 2:
            return
        width = float(sum(msg.position[msg.name.index(name)] for name in finger_joints[:2]))
        normalized = np.clip(width / self._args.gripper_open_width, 0.0, 1.0)
        self.gripper_position = np.asarray([normalized], dtype=np.float32)

    def wait_until_ready(self, timeout_sec):
        deadline = time.time() + timeout_sec
        last_log_time = 0.0
        while not rospy.is_shutdown() and time.time() < deadline:
            if (
                self.exterior_rgb is not None
                and self.wrist_rgb is not None
                and self.joint_position is not None
                and self.gripper_position is not None
            ):
                return True
            if time.time() - last_log_time > 5.0:
                missing = []
                if self.exterior_rgb is None:
                    missing.append("exterior_rgb")
                if self.wrist_rgb is None:
                    missing.append("wrist_rgb")
                if self.joint_position is None:
                    missing.append("joint_position")
                if self.gripper_position is None:
                    missing.append("gripper_position")
                rospy.loginfo("Still waiting for observations: %s", ", ".join(missing))
                last_log_time = time.time()
            rospy.sleep(0.1)
        return False

    def make_policy_observation(self, prompt):
        exterior_image = image_tools.resize_with_pad(self.exterior_rgb, 224, 224)
        wrist_image = image_tools.resize_with_pad(self.wrist_rgb, 224, 224)
        if self._args.save_debug_dir and not self._saved_debug_observation:
            os.makedirs(self._args.save_debug_dir, exist_ok=True)
            cv2.imwrite(
                os.path.join(self._args.save_debug_dir, "exterior_input.png"),
                cv2.cvtColor(exterior_image, cv2.COLOR_RGB2BGR),
            )
            cv2.imwrite(
                os.path.join(self._args.save_debug_dir, "wrist_input.png"),
                cv2.cvtColor(wrist_image, cv2.COLOR_RGB2BGR),
            )
            np.save(os.path.join(self._args.save_debug_dir, "joint_position.npy"), self.joint_position)
            np.save(os.path.join(self._args.save_debug_dir, "gripper_position.npy"), self.gripper_position)
            self._saved_debug_observation = True
            rospy.loginfo("Saved debug observation to %s", self._args.save_debug_dir)
        return {
            "observation/exterior_image_1_left": exterior_image,
            "observation/wrist_image_left": wrist_image,
            "observation/joint_position": self.joint_position,
            "observation/gripper_position": self.gripper_position,
            "prompt": prompt,
        }


class PandaVelocityController:
    def __init__(self, args):
        self._args = args
        self._publisher = rospy.Publisher(args.joint_command_topic, JointCommand, queue_size=1, tcp_nodelay=True)
        self._move_client = actionlib.SimpleActionClient(args.gripper_move_action, MoveAction)
        self._grasp_client = actionlib.SimpleActionClient(args.gripper_grasp_action, GraspAction)
        # Continuous gripper command tracking: last *width in meters* we asked
        # the action server for, and whether that command was in grasp mode.
        self._last_gripper_width = None
        self._last_gripper_grasping = None
        self._joint_lower = np.asarray(args.joint_soft_lower, dtype=np.float32)
        self._joint_upper = np.asarray(args.joint_soft_upper, dtype=np.float32)
        self._joint_margin = float(args.joint_soft_margin)

    def wait_until_ready(self, timeout_sec):
        if not self._move_client.wait_for_server(rospy.Duration(timeout_sec)):
            return False
        if not self._grasp_client.wait_for_server(rospy.Duration(timeout_sec)):
            return False
        return True

    def command_joint_velocities(self, joint_velocities):
        msg = JointCommand()
        msg.header.stamp = rospy.Time.now()
        msg.names = JOINT_NAMES
        msg.velocity = list(joint_velocities)
        msg.mode = JointCommand.VELOCITY_MODE
        self._publisher.publish(msg)

    def command_joint_positions(self, joint_positions):
        msg = JointCommand()
        msg.header.stamp = rospy.Time.now()
        msg.names = JOINT_NAMES
        msg.position = list(joint_positions)
        msg.mode = JointCommand.POSITION_MODE
        self._publisher.publish(msg)

    def guard_joint_velocities(self, velocities, current_positions):
        """Attenuate velocities that would push joints past their soft limits.

        We linearly fade velocity components that point toward a soft limit
        as the joint approaches it, reaching zero at the limit itself. This
        is a cheap workspace safety (no FK required) that's enough to stop
        pi0's broad "dive forward" chunks from driving the arm into the
        table while keeping normal motion unaffected.
        """
        if current_positions is None:
            return velocities
        out = np.asarray(velocities, dtype=np.float32).copy()
        positions = np.asarray(current_positions, dtype=np.float32)
        for i in range(len(out)):
            low = self._joint_lower[i]
            high = self._joint_upper[i]
            margin = max(self._joint_margin, 1e-6)
            if positions[i] <= low and out[i] < 0:
                out[i] = 0.0
            elif positions[i] < low + margin and out[i] < 0:
                out[i] *= (positions[i] - low) / margin
            if positions[i] >= high and out[i] > 0:
                out[i] = 0.0
            elif positions[i] > high - margin and out[i] > 0:
                out[i] *= (high - positions[i]) / margin
        return out

    def stop(self):
        if self._args.action_mode == "position":
            return
        self.command_joint_velocities(np.zeros(7, dtype=np.float32))

    def _send_move(self, width_m):
        goal = MoveGoal(width=float(width_m), speed=self._args.gripper_speed)
        self._move_client.send_goal(goal)

    def _send_grasp(self):
        goal = GraspGoal()
        goal.width = self._args.grasp_width
        goal.speed = self._args.gripper_speed
        goal.force = self._args.grasp_force
        goal.epsilon.inner = self._args.grasp_epsilon_inner
        goal.epsilon.outer = self._args.grasp_epsilon_outer
        self._grasp_client.send_goal(goal)

    def command_gripper_normalized(self, normalized):
        """Continuous gripper command: 0.0 = closed, 1.0 = fully open.

        Mirrors the DROID gripper action convention. When the target width
        is below ``grasp_width + margin`` we switch to ``GraspAction`` so
        the fingers actually squeeze onto objects; otherwise we use
        ``MoveAction`` to the absolute width. We rate-limit by only
        re-sending when the target width changed more than
        ``gripper_width_deadband`` or the grasp/release mode flipped.
        """
        normalized = float(np.clip(normalized, 0.0, 1.0))
        target_width = normalized * self._args.gripper_open_width
        should_grasp = target_width <= (self._args.grasp_width + self._args.gripper_grasp_margin)

        if self._last_gripper_grasping == should_grasp and self._last_gripper_width is not None:
            if abs(self._last_gripper_width - target_width) < self._args.gripper_width_deadband:
                return

        if should_grasp:
            self._send_grasp()
        else:
            self._send_move(target_width)

        self._last_gripper_grasping = should_grasp
        self._last_gripper_width = target_width

    def set_gripper_open(self, should_open):
        # Legacy binary API used during startup to fully open the gripper.
        if should_open:
            self.command_gripper_normalized(1.0)
        else:
            self.command_gripper_normalized(0.0)


def parse_args():
    parser = argparse.ArgumentParser(description="Run an openpi DROID policy zero-shot on the Panda Gazebo task.")
    parser.add_argument("--server-host", default="127.0.0.1")
    parser.add_argument("--server-port", type=int, default=8000)
    parser.add_argument("--prompt", default="pick up the red cube and place it in the red bin")
    parser.add_argument("--steps", type=int, default=150)
    parser.add_argument("--rate-hz", type=float, default=15.0)
    parser.add_argument("--open-loop-horizon", type=int, default=8)
    parser.add_argument("--action-mode", choices=("position", "velocity"), default="velocity")
    parser.add_argument(
        "--max-abs-joint-velocity",
        type=float,
        default=1.2,
        help=(
            "Per-joint safety clamp (rad/s) applied to DROID joint velocities "
            "before publishing. The raw policy output is passed through "
            "as-is up to this magnitude; it is NOT rescaled by this value."
        ),
    )
    parser.add_argument(
        "--velocity-scale",
        type=float,
        default=1.0,
        help=(
            "Multiplicative scale (<=1) applied to policy joint velocities "
            "before the safety clamp. 1.0 = DROID-faithful; smaller values "
            "slow the rollout down without changing the plan shape."
        ),
    )
    parser.add_argument("--open-threshold", type=float, default=0.5)
    parser.add_argument(
        "--gripper-mode",
        choices=("continuous", "binary"),
        default="continuous",
        help="continuous: send normalized width target (DROID spec). binary: open/close around --open-threshold.",
    )
    parser.add_argument("--gripper-grasp-margin", type=float, default=0.005)
    parser.add_argument("--gripper-width-deadband", type=float, default=0.004)
    parser.add_argument(
        "--disable-joint-guard",
        action="store_true",
        help="Disable the soft joint-limit guard (used to stop the arm diving into the table).",
    )
    parser.add_argument(
        "--joint-soft-lower",
        type=float,
        nargs=7,
        default=(-2.6, -1.3, -2.6, -2.9, -2.6, 0.05, -2.6),
        help="Per-joint soft lower bound (rad). Tighter than Panda's hard limits to keep the end-effector above the table.",
    )
    parser.add_argument(
        "--joint-soft-upper",
        type=float,
        nargs=7,
        default=(2.6, 1.3, 2.6, -0.2, 2.6, 3.5, 2.6),
        help="Per-joint soft upper bound (rad).",
    )
    parser.add_argument(
        "--joint-soft-margin",
        type=float,
        default=0.15,
        help="Approach-zone width (rad) for the soft limit fade-out.",
    )
    parser.add_argument("--startup-timeout", type=float, default=40.0)
    parser.add_argument("--settle-seconds", type=float, default=2.0)
    parser.add_argument("--save-debug-dir", default=os.path.join(REPO_ROOT, "outputs", "openpi_debug"))
    parser.add_argument("--exterior-image-topic", default="/camera/color/image_raw")
    parser.add_argument("--wrist-image-topic", default="/wrist_camera/color/image_raw")
    parser.add_argument("--use-exterior-as-wrist", action="store_true")
    parser.add_argument("--joint-state-topic", default="/panda_simulator/custom_franka_state_controller/joint_states")
    parser.add_argument("--gripper-joint-state-topic", default="/franka_gripper/joint_states")
    parser.add_argument("--joint-command-topic", default="/panda_simulator/motion_controller/arm/joint_commands")
    parser.add_argument("--gripper-move-action", default="/franka_gripper/move")
    parser.add_argument("--gripper-grasp-action", default="/franka_gripper/grasp")
    parser.add_argument("--gripper-open-width", type=float, default=0.08)
    parser.add_argument("--gripper-speed", type=float, default=0.08)
    parser.add_argument("--grasp-width", type=float, default=0.04)
    parser.add_argument("--grasp-force", type=float, default=15.0)
    parser.add_argument("--grasp-epsilon-inner", type=float, default=0.01)
    parser.add_argument("--grasp-epsilon-outer", type=float, default=0.01)
    return parser.parse_args()


def main():
    args = parse_args()
    rospy.init_node("openpi_pickplace_zero_shot")

    obs = ObservationBuffer(args)
    controller = PandaVelocityController(args)

    rospy.loginfo("Waiting for Gazebo observations and gripper action servers...")
    if not obs.wait_until_ready(args.startup_timeout):
        raise RuntimeError("Timed out waiting for camera/joint/gripper observations.")
    if not controller.wait_until_ready(args.startup_timeout):
        raise RuntimeError("Timed out waiting for Panda gripper action servers.")

    rospy.loginfo(
        "Connecting to openpi policy server at %s:%s with prompt: %s",
        args.server_host,
        args.server_port,
        args.prompt,
    )
    policy_client = websocket_client_policy.WebsocketClientPolicy(args.server_host, args.server_port)
    controller.set_gripper_open(True)
    rospy.sleep(args.settle_seconds)

    rate = rospy.Rate(args.rate_hz)
    action_chunk = None
    chunk_index = 0

    for step in range(args.steps):
        if action_chunk is None or chunk_index >= min(args.open_loop_horizon, len(action_chunk)):
            request = obs.make_policy_observation(args.prompt)
            response = policy_client.infer(request)
            action_chunk = np.asarray(response["actions"], dtype=np.float32)
            if action_chunk.ndim != 2 or action_chunk.shape[1] < 8:
                raise RuntimeError(f"Unexpected action shape from policy server: {action_chunk.shape}")
            chunk_index = 0
            rospy.loginfo(
                "Received action chunk %s at step %s | first_action[min=%.3f max=%.3f gripper=%.3f]",
                action_chunk.shape,
                step,
                float(np.min(action_chunk[0, :7])),
                float(np.max(action_chunk[0, :7])),
                float(action_chunk[0, 7]),
            )

        raw_action = np.asarray(action_chunk[chunk_index], dtype=np.float32)
        chunk_index += 1

        if args.action_mode == "position":
            controller.command_joint_positions(raw_action[:7])
        else:
            # DROID policy outputs 7 joint velocities in rad/s directly.
            # We (optionally) scale, then clamp to a safety bound. The clamp
            # does NOT rescale; it only caps.
            joint_velocity = raw_action[:7] * float(args.velocity_scale)
            joint_velocity = np.clip(
                joint_velocity, -args.max_abs_joint_velocity, args.max_abs_joint_velocity
            )
            if not args.disable_joint_guard:
                joint_velocity = controller.guard_joint_velocities(
                    joint_velocity, obs.joint_position
                )
            controller.command_joint_velocities(joint_velocity)
        if args.gripper_mode == "continuous":
            controller.command_gripper_normalized(float(raw_action[7]))
        else:
            controller.set_gripper_open(bool(raw_action[7] > args.open_threshold))
        rate.sleep()

    controller.stop()
    rospy.loginfo("Zero-shot rollout complete.")


if __name__ == "__main__":
    main()
