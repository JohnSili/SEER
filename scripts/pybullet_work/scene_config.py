from pathlib import Path

from paths import REPO_ROOT

# --- Asset paths --------------------------------------------------------------
ROBOT_URDF = REPO_ROOT / "self-collision-mesh" / "robot_moveit_abs.urdf"
ROBOT_SRDF = REPO_ROOT / "self-collision-mesh" / "wheeled_humanoid_v3_2.srdf"
TABLE_URDF = Path(__file__).resolve().parent / "assets" / "table" / "table.urdf"

# --- World placement ----------------------------------------------------------
ROBOT_POS = [0.9, 0.0, 0.0]
# Стол опущен: базу уводим под пол на TABLE_Z_OFFSET (ножки утоплены, столешница ниже).
TABLE_Z_OFFSET = -0.08
TABLE_POS = [0.0, 0.0, TABLE_Z_OFFSET]

# --- Obstacles (опционально, --obstacles) --------------------------------------
# Стенка-барьер поперёк середины стола: тонкая по y, вытянута по x, чтобы
# траектория с одной половины стола на другую должна была идти ПОВЕРХ неё.
OBSTACLE_WALL_HALF_THICKNESS = 0.03   # половина толщины (y), м
OBSTACLE_WALL_HALF_HEIGHT = 0.15      # половина высоты (z), м
OBSTACLE_WALL_LENGTH_FRACTION = 0.8   # доля длины стола (x), которую занимает стенка
OBSTACLE_RGBA = (0.95, 0.6, 0.1, 1.0)
# Куб при включённых препятствиях сдвигается с центра на эту долю ширины стола,
# чтобы не пересекаться со стенкой.
CUBE_Y_OFFSET_FRACTION = 0.25

# --- Simulation ---------------------------------------------------------------
SIM_DT = 1.0 / 240.0
GRAVITY_Z = -10.0

# --- Arm joints (single source of truth) --------------------------------------
N_ARM_JOINTS = 7
RIGHT_ARM_JOINT_NAMES = [f"arm_right_{i}" for i in range(1, N_ARM_JOINTS + 1)]
LEFT_ARM_JOINT_NAMES = [f"arm_left_{i}" for i in range(1, N_ARM_JOINTS + 1)]

DEFAULT_LEFT_ARM_POSE = [0.0] * N_ARM_JOINTS
DEFAULT_RIGHT_ARM_POSE = [0.0] * N_ARM_JOINTS

# --- Self-collision config ----------------------------------------------------
# NOTE: these pairs use *link* names (arm1_/arm2_), while the joints above are
# arm_right_/arm_left_. Confirm which physical side arm1_/arm2_ map to in your
# URDF -- nothing in code enforces that mapping.
DEFAULT_COLLISION_IGNORE_PAIRS = (
    "base_link:arm1_1_link,base_link:arm1_2_link,"
    "base_link:arm1_3_link,base_link:arm1_4_link,"
    "base_link:arm1_5_link,base_link:arm1_6_link,"
    "base_link:arm1_7_link,"
    "base_link:arm2_1_link,base_link:arm2_2_link,"
    "base_link:arm2_3_link,base_link:arm2_4_link,"
    "base_link:arm2_5_link,base_link:arm2_6_link,"
    "base_link:arm2_7_link"
)