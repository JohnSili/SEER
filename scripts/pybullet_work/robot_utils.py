from dataclasses import dataclass

import pybullet as p

from scene_config import LEFT_ARM_JOINT_NAMES, RIGHT_ARM_JOINT_NAMES


@dataclass(frozen=True)
class RobotModel:
    left_arm: list[int]
    right_arm: list[int]
    movable_joints: list[int]
    joint_limits: dict[int, tuple[float, float]]


def build_robot_model(robot_id, client_id=0):
    joint_name_to_idx = {}
    movable_joints = []
    joint_limits = {}

    for joint_idx in range(p.getNumJoints(robot_id, physicsClientId=client_id)):
        info = p.getJointInfo(robot_id, joint_idx, physicsClientId=client_id)
        joint_name = info[1].decode()
        joint_name_to_idx[joint_name] = joint_idx

        if info[2] != p.JOINT_FIXED:
            movable_joints.append(joint_idx)
            joint_limits[joint_idx] = (info[8], info[9])

    return RobotModel(
        left_arm=[joint_name_to_idx[name] for name in LEFT_ARM_JOINT_NAMES],
        right_arm=[joint_name_to_idx[name] for name in RIGHT_ARM_JOINT_NAMES],
        movable_joints=movable_joints,
        joint_limits=joint_limits,
    )


def set_arm_pose(robot_id, joint_indices, pose, client_id=0):
    for joint_idx, value in zip(joint_indices, pose):
        p.resetJointState(robot_id, joint_idx, float(value), physicsClientId=client_id)
        p.setJointMotorControl2(
            robot_id,
            joint_idx,
            p.POSITION_CONTROL,
            targetPosition=float(value),
            physicsClientId=client_id,
        )


def set_default_arm_pose(robot_id, model, left_pose, right_pose, client_id=0):
    set_arm_pose(robot_id, model.left_arm, left_pose, client_id=client_id)
    set_arm_pose(robot_id, model.right_arm, right_pose, client_id=client_id)
    p.performCollisionDetection(physicsClientId=client_id)


def hold_all_joints(robot_id, movable_joints, client_id=0, force=None):
    """Keep every movable joint at its current position so nothing droops under gravity.

    force=None uses PyBullet's default position-control force, which is what holds
    the arms at the configured pose. Pass an explicit force only to cap it.
    """
    states = p.getJointStates(robot_id, movable_joints, physicsClientId=client_id)
    for joint_idx, state in zip(movable_joints, states):
        kwargs = {"physicsClientId": client_id}
        if force is not None:
            kwargs["force"] = force
        p.setJointMotorControl2(
            robot_id,
            joint_idx,
            p.POSITION_CONTROL,
            targetPosition=state[0],
            **kwargs,
        )