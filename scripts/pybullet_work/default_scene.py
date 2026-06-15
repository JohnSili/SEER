import argparse
import math
import time
from dataclasses import dataclass, field
from pathlib import Path

import pybullet as p
import pybullet_data

from collision_utils import check_default_pose, create_self_collision_guard
from robot_utils import (
    RobotModel,
    build_robot_model,
    hold_all_joints,
    set_default_arm_pose,
)
from scene_config import (
    CUBE_Y_OFFSET_FRACTION,
    DEFAULT_LEFT_ARM_POSE,
    DEFAULT_RIGHT_ARM_POSE,
    GRAVITY_Z,
    OBSTACLE_RGBA,
    OBSTACLE_WALL_HALF_HEIGHT,
    OBSTACLE_WALL_HALF_THICKNESS,
    OBSTACLE_WALL_LENGTH_FRACTION,
    ROBOT_POS,
    ROBOT_URDF,
    SIM_DT,
    TABLE_POS,
    TABLE_URDF,
)
from urdf_utils import prepare_urdf_path


@dataclass
class Scene:
    client_id: int
    robot_id: int
    table_id: int
    model: RobotModel
    cube_id: int
    obstacle_ids: list[int] = field(default_factory=list)


def color_whole_body(uid, rgba, client_id=0):
    for link_idx in range(-1, p.getNumJoints(uid, physicsClientId=client_id)):
        p.changeVisualShape(uid, link_idx, rgbaColor=rgba, physicsClientId=client_id)


def right_arm_ee_world_pos(robot_id, model, client_id=0):
    """World position of the right-arm end link (child of arm_right_7)."""
    state = p.getLinkState(robot_id, model.right_arm[-1], physicsClientId=client_id)
    return state[4]  # worldLinkFramePosition


def spawn_box(position, half_extents, mass=0.0, rgba=(0.0, 0.4, 1.0, 1.0), client_id=0):
    """Create a box. mass=0 -> static obstacle; give it mass to make it pushable."""
    col = p.createCollisionShape(
        p.GEOM_BOX, halfExtents=list(half_extents), physicsClientId=client_id
    )
    vis = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=list(half_extents),
        rgbaColor=list(rgba),
        physicsClientId=client_id,
    )
    return p.createMultiBody(
        baseMass=mass,
        baseCollisionShapeIndex=col,
        baseVisualShapeIndex=vis,
        basePosition=list(position),
        physicsClientId=client_id,
    )


def spawn_cube(position, half_extent=0.03, mass=0.0, rgba=(0.0, 0.4, 1.0, 1.0), client_id=0):
    return spawn_box(
        position, [half_extent] * 3, mass=mass, rgba=rgba, client_id=client_id
    )


def body_aabb(body_id, client_id=0):
    """Union AABB over the base and all links of a body -> (lo, hi) world corners."""
    lo, hi = p.getAABB(body_id, -1, physicsClientId=client_id)
    lo, hi = list(lo), list(hi)
    for link in range(p.getNumJoints(body_id, physicsClientId=client_id)):
        link_lo, link_hi = p.getAABB(body_id, link, physicsClientId=client_id)
        lo = [min(a, b) for a, b in zip(lo, link_lo)]
        hi = [max(a, b) for a, b in zip(hi, link_hi)]
    return lo, hi


def spawn_obstacle_wall(table_lo, table_hi, client_id=0):
    """Стенка-барьер поперёк середины стола (вдоль x, тонкая по y)."""
    table_top = table_hi[2]
    cx = (table_lo[0] + table_hi[0]) / 2
    cy = (table_lo[1] + table_hi[1]) / 2
    half_len_x = (table_hi[0] - table_lo[0]) / 2 * OBSTACLE_WALL_LENGTH_FRACTION
    half_extents = [
        half_len_x,
        OBSTACLE_WALL_HALF_THICKNESS,
        OBSTACLE_WALL_HALF_HEIGHT,
    ]
    pos = [cx, cy, table_top + OBSTACLE_WALL_HALF_HEIGHT]
    wall_id = spawn_box(
        pos, half_extents, mass=0.0, rgba=OBSTACLE_RGBA, client_id=client_id
    )
    print(
        f"[scene] obstacle wall: id={wall_id}, pos={['%.3f' % c for c in pos]}, "
        f"half_extents={['%.3f' % c for c in half_extents]}"
    )
    return wall_id


def load_scene(gui=True, with_obstacles=False):
    client_id = p.connect(p.GUI if gui else p.DIRECT)
    if client_id < 0:
        raise RuntimeError("Failed to connect PyBullet simulation")

    # NOTE: setAdditionalSearchPath stores only ONE path (last call wins), so set
    # it once. Every URDF below is loaded via an absolute path and robot mesh
    # paths are pre-resolved by prepare_urdf_path, so a single path is enough.
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client_id)
    p.setTimeStep(SIM_DT, physicsClientId=client_id)
    p.setGravity(0, 0, GRAVITY_Z, physicsClientId=client_id)
    p.configureDebugVisualizer(
        p.COV_ENABLE_MOUSE_PICKING, 0, physicsClientId=client_id
    )

    plane_path = str(Path(pybullet_data.getDataPath()) / "plane.urdf")
    p.loadURDF(plane_path, physicsClientId=client_id)

    table_id = p.loadURDF(
        str(TABLE_URDF),
        TABLE_POS,
        p.getQuaternionFromEuler([0, 0, math.pi / 2]),
        physicsClientId=client_id,
    )
    color_whole_body(table_id, [1, 0, 0, 1], client_id=client_id)

    # Spawn the cube on the table BEFORE loading the robot. Position comes from
    # the table AABB (robot not needed); static (mass=0) -> no startup motion.
    # Pass mass>0 to spawn_cube to make it pushable.
    table_lo, table_hi = body_aabb(table_id, client_id=client_id)
    table_top = table_hi[2]
    cube_y = (table_lo[1] + table_hi[1]) / 2
    if with_obstacles:
        # Стенка займёт центр -- куб уезжает на одну половину стола.
        cube_y += (table_hi[1] - table_lo[1]) * CUBE_Y_OFFSET_FRACTION
    cube_pos = [
        (table_lo[0] + table_hi[0]) / 2,
        cube_y,
        table_top + 0.03,
    ]
    print(f"[scene] table top z={table_top:.3f}, cube at {cube_pos}")
    cube_id = spawn_cube(cube_pos, client_id=client_id)

    obstacle_ids = []
    if with_obstacles:
        obstacle_ids.append(
            spawn_obstacle_wall(table_lo, table_hi, client_id=client_id)
        )

    robot_urdf = prepare_urdf_path(ROBOT_URDF)
    robot_id = p.loadURDF(
        str(robot_urdf),
        ROBOT_POS,
        p.getQuaternionFromEuler([0, 0, math.pi]),
        useFixedBase=True,
        # Self-collision is handled by the external SelfCollisionGuard, which
        # reads the SRDF ignore pairs. Letting PyBullet resolve self-collision
        # makes overlapping base/arm hulls explode apart on the first frames.
        flags=0,
        physicsClientId=client_id,
    )
    if robot_id < 0:
        raise RuntimeError(f"Failed to load robot URDF: {robot_urdf}")

    visual_shapes = p.getVisualShapeData(robot_id, physicsClientId=client_id)
    if not visual_shapes:
        raise RuntimeError(f"Robot URDF loaded without visual meshes: {robot_urdf}")
    print(f"[scene] robot loaded: id={robot_id}, visual_shapes={len(visual_shapes)}")

    model = build_robot_model(robot_id, client_id=client_id)
    set_default_arm_pose(
        robot_id,
        model,
        DEFAULT_LEFT_ARM_POSE,
        DEFAULT_RIGHT_ARM_POSE,
        client_id=client_id,
    )
    hold_all_joints(robot_id, model.movable_joints, client_id=client_id)

    # Right-arm EE world position, printed as a calibration reference. NOTE:
    # anchoring the cube to the EE needs the robot loaded + posed, so to do that
    # the cube spawn above would have to move back down below this point.
    ee_pos = right_arm_ee_world_pos(robot_id, model, client_id=client_id)
    print(f"[scene] right-arm EE world pos: {ee_pos}")

    p.resetDebugVisualizerCamera(
        cameraDistance=3.6,
        cameraYaw=45,
        cameraPitch=-22,
        cameraTargetPosition=[0.55, -0.15, 0.75],
        physicsClientId=client_id,
    )

    return Scene(
        client_id=client_id,
        robot_id=robot_id,
        table_id=table_id,
        model=model,
        cube_id=cube_id,
        obstacle_ids=obstacle_ids,
    )


def validate_default_pose():
    """Check the default arm pose for self-collision before opening the sim."""
    guard = create_self_collision_guard()
    try:
        check_default_pose(guard, DEFAULT_LEFT_ARM_POSE, DEFAULT_RIGHT_ARM_POSE)
        print("[self-collision] default arm pose OK")
    finally:
        guard.close()


def run(scene, hold=True):
    """Step the sim until interrupted; return immediately if not holding."""
    if not hold:
        return
    while True:
        p.stepSimulation(physicsClientId=scene.client_id)
        time.sleep(SIM_DT)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--direct", action="store_true", help="Run without GUI.")
    parser.add_argument(
        "--no-hold",
        action="store_true",
        help="Exit after loading and validating the scene.",
    )
    parser.add_argument(
        "--obstacles",
        action="store_true",
        help="Spawn obstacle wall in the middle of the table.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    scene = None
    try:
        validate_default_pose()
        scene = load_scene(gui=not args.direct, with_obstacles=args.obstacles)
        print("[scene] loaded default arm pose scene")
        run(scene, hold=not args.no_hold)
    except KeyboardInterrupt:
        pass
    finally:
        if scene is not None:
            p.disconnect(scene.client_id)


if __name__ == "__main__":
    main()