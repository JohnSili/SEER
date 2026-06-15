from pathlib import Path

from scene_config import (
    DEFAULT_COLLISION_IGNORE_PAIRS,
    LEFT_ARM_JOINT_NAMES,
    RIGHT_ARM_JOINT_NAMES,
    ROBOT_SRDF,
    ROBOT_URDF,
)

# Importing scene_config (above) imports paths, which puts REPO_ROOT on
# sys.path -- so the evaluation package resolves here.
from evaluation.collision_guard import SelfCollisionGuard, parse_ignore_pairs


def create_self_collision_guard(collision_distance=0.0):
    return SelfCollisionGuard(
        urdf_path=Path(ROBOT_URDF),
        srdf_path=Path(ROBOT_SRDF),
        right_joint_names=RIGHT_ARM_JOINT_NAMES,
        left_joint_names=LEFT_ARM_JOINT_NAMES,
        collision_distance=collision_distance,
        extra_ignored_pairs=parse_ignore_pairs(DEFAULT_COLLISION_IGNORE_PAIRS),
        mesh_share_root=Path(ROBOT_URDF).parent,
    )


def check_default_pose(collision_guard, left_pose, right_pose):
    collision, report = collision_guard.check(
        q_right=list(right_pose),
        q_left=list(left_pose),
    )
    if collision:
        raise RuntimeError(f"Default pose has self-collision: {report}")
    return report
