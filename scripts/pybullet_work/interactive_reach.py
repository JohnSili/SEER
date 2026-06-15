"""Click-to-reach: клик по сцене -> выбор руки -> IK-траектория -> движение.

Управление в GUI (фокус на окне PyBullet):
  T          -- вкл/выкл режим прицеливания (когда выключен, клики игнорируются,
                можно спокойно крутить камеру)
  L / R      -- выбрать левую / правую руку
  наведение  -- белая сфера-курсор показывает, куда попадёт клик
  ЛКМ        -- зафиксировать цель (зелёный маркер), IK, проверка пути, движение
  Ctrl+C в терминале -- выход.

Запуск:  python interactive_reach.py
"""
import argparse
import math
import random
import time

import pybullet as p

from collision_utils import create_self_collision_guard
from default_scene import load_scene, validate_default_pose
from scene_config import SIM_DT

IK_MAX_ITERATIONS = 200
IK_RESIDUAL = 1e-4
IK_REFINE_ROUNDS = 5        # итеративное дорешивание IK от предыдущего решения
IK_RANDOM_SEEDS = 6         # случайных стартовых конфигураций (мульти-старт)
REACH_TOLERANCE = 0.05      # м: если IK не дотянулся ближе -- не едем
MAX_JOINT_SPEED = 0.5       # рад/с: ограничение скорости самого быстрого сустава
MIN_MOVE_DURATION_S = 1.5   # с: короткие движения не быстрее этого
N_PATH_CHECKS = 20          # промежуточных конфигураций для self-collision проверки
TARGET_LIFT = 0.08          # подъём цели над поверхностью: меш кисти длиннее EE-фрейма
APPROACH_HEIGHT = 0.18      # м: запас высоты безопасного пролёта над целью/стартом
CART_STEP = 0.12            # м: шаг декартовых вейпоинтов горизонтального пролёта
SEGMENT_MIN_DURATION_S = 0.3  # с: мин. длительность одного сегмента составного пути
ENV_CONTACT_DISTANCE = 0.005  # м: ближе этого к столу/препятствиям = столкновение

HOVER_MARKER_RADIUS = 0.7            # крупный курсор: видно, куда уйдёт клик
HOVER_MARKER_RGBA = (0.2, 0.8, 1.0, 0.45)   # голубой, полупрозрачный
TARGET_MARKER_RADIUS = 0.03           # зафиксированная цель
TARGET_MARKER_RGBA = (0.1, 1.0, 0.1, 0.9)   # зелёный

HIDDEN_POS = [0.0, 0.0, -10.0]


def ray_from_mouse(mouse_x, mouse_y, client_id):
    (width, height, _, _, _,
     cam_forward, horizon, vertical,
     _, _, dist, cam_target) = p.getDebugVisualizerCamera(physicsClientId=client_id)

    cam_pos = [cam_target[i] - dist * cam_forward[i] for i in range(3)]
    # ВАЖНО: horizon/vertical из getDebugVisualizerCamera отмасштабированы под
    # дальнюю плоскость 10000 (как в каноничном сниппете pybullet). Меньший
    # far ломает направление луча тем сильнее, чем дальше клик от центра экрана.
    far = 10000.0
    ray_forward = [far * cam_forward[i] for i in range(3)]

    d_hor = [h / width for h in horizon]
    d_ver = [v / height for v in vertical]
    ray_to = [
        cam_pos[i] + ray_forward[i]
        - 0.5 * horizon[i] + 0.5 * vertical[i]
        + mouse_x * d_hor[i] - mouse_y * d_ver[i]
        for i in range(3)
    ]
    return cam_pos, ray_to


def pick_point(mouse_x, mouse_y, client_id):
    """rayTest по пикселю; возвращает (точка, нормаль, body_id) или None."""
    ray_from, ray_to = ray_from_mouse(mouse_x, mouse_y, client_id)
    hit = p.rayTest(ray_from, ray_to, physicsClientId=client_id)[0]
    body_id, hit_pos, hit_normal = hit[0], hit[3], hit[4]
    if body_id < 0:
        return None
    point = [hit_pos[i] + TARGET_LIFT * hit_normal[i] for i in range(3)]
    return point, hit_normal, body_id


# --- Маркеры -------------------------------------------------------------------

def create_marker(client_id, radius, rgba):
    vis = p.createVisualShape(
        p.GEOM_SPHERE, radius=radius, rgbaColor=list(rgba), physicsClientId=client_id
    )
    return p.createMultiBody(
        baseMass=0.0,
        baseCollisionShapeIndex=-1,  # без коллизии: не мешает rayTest и рукам
        baseVisualShapeIndex=vis,
        basePosition=HIDDEN_POS,
        physicsClientId=client_id,
    )


def move_marker(marker_id, pos, client_id):
    p.resetBasePositionAndOrientation(
        marker_id, pos, [0, 0, 0, 1], physicsClientId=client_id
    )


# --- IK --------------------------------------------------------------------------

def _null_space_params(robot_id, model, client_id):
    lowers, uppers, ranges, rests = [], [], [], []
    states = p.getJointStates(robot_id, model.movable_joints, physicsClientId=client_id)
    for joint_idx, state in zip(model.movable_joints, states):
        lo, hi = model.joint_limits[joint_idx]
        if lo > hi:  # continuous joint
            lo, hi = -2 * math.pi, 2 * math.pi
        lowers.append(lo)
        uppers.append(hi)
        ranges.append(hi - lo)
        rests.append(state[0])
    return lowers, uppers, ranges, rests


def _arm_sample_limits(model, arm_joints):
    limits = []
    for joint_idx in arm_joints:
        lo, hi = model.joint_limits[joint_idx]
        if lo > hi:  # continuous joint
            lo, hi = -math.pi, math.pi
        limits.append((lo, hi))
    return limits


def _refine_ik(robot_id, model, arm_joints, target_pos, client_id):
    """Дорешивание IK от ТЕКУЩЕГО состояния руки (несколько раундов reset->IK).

    Не сохраняет состояние и не трогает рендер -- это забота вызывающего.
    Возвращает (q, err).
    """
    ee_link = arm_joints[-1]
    idx_of = {j: i for i, j in enumerate(model.movable_joints)}
    best_q, best_err = None, math.inf
    for _ in range(IK_REFINE_ROUNDS):
        lowers, uppers, ranges, rests = _null_space_params(robot_id, model, client_id)
        solution = p.calculateInverseKinematics(
            robot_id,
            ee_link,
            target_pos,
            lowerLimits=lowers,
            upperLimits=uppers,
            jointRanges=ranges,
            restPoses=rests,
            maxNumIterations=IK_MAX_ITERATIONS,
            residualThreshold=IK_RESIDUAL,
            physicsClientId=client_id,
        )
        q = [solution[idx_of[j]] for j in arm_joints]
        for j, v in zip(arm_joints, q):
            p.resetJointState(robot_id, j, v, physicsClientId=client_id)
        ee = p.getLinkState(robot_id, ee_link, physicsClientId=client_id)[4]
        err = math.dist(ee, target_pos)
        if err < best_err:
            best_q, best_err = q, err
        if best_err < 1e-3:
            break
    return best_q, best_err


def solve_arm_ik_candidates(robot_id, model, arm_joints, target_pos, client_id):
    """Мульти-старт IK; возвращает СПИСОК кандидатов [(q, err)], отсортированный
    по ошибке EE. Состояние симуляции откатывается.

    Возвращаем лучшее решение КАЖДОГО сида, а не одно глобально лучшее:
    разные сиды дают разные конфигурации (локоть-вверх / локоть-вниз), и
    выбор между ними должен делаться по проходимости пути, а не только по
    точности EE.
    """
    saved = [
        s[0]
        for s in p.getJointStates(robot_id, arm_joints, physicsClientId=client_id)
    ]
    sample_limits = _arm_sample_limits(model, arm_joints)
    seeds = [list(saved), [0.0] * len(arm_joints)]
    seeds += [
        [random.uniform(lo, hi) for lo, hi in sample_limits]
        for _ in range(IK_RANDOM_SEEDS)
    ]

    candidates = []
    # В GUI-режиме рендер живёт в отдельном потоке и успевает показать
    # промежуточные resetJointState -- на время решения выключаем рендер.
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0, physicsClientId=client_id)
    try:
        for seed in seeds:
            for j, v in zip(arm_joints, seed):
                p.resetJointState(robot_id, j, v, physicsClientId=client_id)
            q, err = _refine_ik(robot_id, model, arm_joints, target_pos, client_id)
            if q is not None:
                candidates.append((q, err))
    finally:
        for j, v in zip(arm_joints, saved):
            p.resetJointState(robot_id, j, v, physicsClientId=client_id)
        p.configureDebugVisualizer(
            p.COV_ENABLE_RENDERING, 1, physicsClientId=client_id
        )

    candidates.sort(key=lambda c: c[1])
    # Дедупликация почти одинаковых конфигураций (разные сиды часто сходятся
    # в одно и то же решение) -- чтобы не гонять проверку пути по дублям.
    unique = []
    for q, err in candidates:
        if any(max(abs(a - b) for a, b in zip(q, uq)) < 0.05 for uq, _ in unique):
            continue
        unique.append((q, err))
    return unique


def solve_arm_ik_seeded(robot_id, model, arm_joints, target_pos, seed, client_id):
    """IK от одного конкретного сида; решение получается БЛИЗКИМ к сиду в
    joint space (важно для короткого сегмента спуска). Состояние откатывается.
    """
    saved = [
        s[0]
        for s in p.getJointStates(robot_id, arm_joints, physicsClientId=client_id)
    ]
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0, physicsClientId=client_id)
    try:
        for j, v in zip(arm_joints, seed):
            p.resetJointState(robot_id, j, v, physicsClientId=client_id)
        return _refine_ik(robot_id, model, arm_joints, target_pos, client_id)
    finally:
        for j, v in zip(arm_joints, saved):
            p.resetJointState(robot_id, j, v, physicsClientId=client_id)
        p.configureDebugVisualizer(
            p.COV_ENABLE_RENDERING, 1, physicsClientId=client_id
        )


# --- Проверка пути и исполнение ---------------------------------------------------

def arm_q(robot_id, arm_joints, client_id):
    return [
        s[0]
        for s in p.getJointStates(robot_id, arm_joints, physicsClientId=client_id)
    ]


def _n_checks(start_q, goal_q):
    """Число проверок пропорционально длине сегмента в joint space."""
    max_delta = max(abs(g - s) for s, g in zip(start_q, goal_q))
    return min(N_PATH_CHECKS, max(3, int(max_delta / 0.05)))


def validate_path(guard, arm, start_q, goal_q, other_q):
    """Линейный joint-space путь: промежуточные конфигурации через guard."""
    n = _n_checks(start_q, goal_q)
    for k in range(1, n + 1):
        t = k / n
        q = [s + (g - s) * t for s, g in zip(start_q, goal_q)]
        q_left = q if arm == "left" else other_q
        q_right = q if arm == "right" else other_q
        collision, report = guard.check(q_right=list(q_right), q_left=list(q_left))
        if collision:
            return False, (t, report)
    return True, None


def validate_path_env(scene, arm_joints, start_q, goal_q):
    """Кинематическая проверка пути на столкновения с окружением.

    Рука последовательно ставится (reset) в промежуточные конфигурации, и для
    каждой проверяется близость ЛИНКОВ ДВИЖУЩЕЙСЯ РУКИ (а не всего робота --
    статичный корпус/вторая рука не должны блокировать движение) к столу,
    кубу и препятствиям через getClosestPoints. Состояние откатывается,
    рендер на время проверки выключен.
    """
    cid = scene.client_id
    env_bodies = [scene.table_id, scene.cube_id, *scene.obstacle_ids]
    saved = arm_q(scene.robot_id, arm_joints, cid)
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0, physicsClientId=cid)
    try:
        n = _n_checks(start_q, goal_q)
        for k in range(1, n + 1):
            t = k / n
            q = [s + (g - s) * t for s, g in zip(start_q, goal_q)]
            for j, v in zip(arm_joints, q):
                p.resetJointState(scene.robot_id, j, v, physicsClientId=cid)
            for body in env_bodies:
                for link in arm_joints:  # link index == joint index дочернего линка
                    points = p.getClosestPoints(
                        scene.robot_id,
                        body,
                        ENV_CONTACT_DISTANCE,
                        linkIndexA=link,
                        physicsClientId=cid,
                    )
                    if points:
                        return False, (t, body)
        return True, None
    finally:
        for j, v in zip(arm_joints, saved):
            p.resetJointState(scene.robot_id, j, v, physicsClientId=cid)
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1, physicsClientId=cid)


def execute_motion(scene, arm_joints, goal_q, min_duration=MIN_MOVE_DURATION_S, verbose=True):
    cid = scene.client_id
    start_q = arm_q(scene.robot_id, arm_joints, cid)
    # Длительность из ограничения скорости: самый "длинный" сустав едет
    # не быстрее MAX_JOINT_SPEED, короткие движения -- не короче минимума.
    max_delta = max(abs(g - s) for s, g in zip(start_q, goal_q))
    duration = max(min_duration, max_delta / MAX_JOINT_SPEED)
    if verbose:
        print(f"[reach] движение: {duration:.1f} с (макс. дельта {max_delta:.2f} рад)")
    n_steps = max(1, int(duration / SIM_DT))
    for k in range(1, n_steps + 1):
        t = k / n_steps
        s = t * t * (3 - 2 * t)  # smoothstep: мягкий разгон/торможение
        for j, q0, q1 in zip(arm_joints, start_q, goal_q):
            p.setJointMotorControl2(
                scene.robot_id,
                j,
                p.POSITION_CONTROL,
                targetPosition=q0 + (q1 - q0) * s,
                physicsClientId=cid,
            )
        p.stepSimulation(physicsClientId=cid)
        time.sleep(SIM_DT)


def handle_click(scene, guard, arm, mouse_x, mouse_y, target_marker):
    cid = scene.client_id
    picked = pick_point(mouse_x, mouse_y, cid)
    if picked is None:
        print("[reach] клик мимо сцены -- цели нет")
        return
    target, _, body_id = picked

    if body_id == scene.robot_id:
        print("[reach] клик по самому роботу -- игнорирую (цельтесь в стол/куб/пол)")
        return

    move_marker(target_marker, target, cid)
    print(f"[reach] цель {['%.3f' % c for c in target]} (body {body_id}), рука: {arm}")

    arm_joints = scene.model.left_arm if arm == "left" else scene.model.right_arm
    other_joints = scene.model.right_arm if arm == "left" else scene.model.left_arm

    candidates = solve_arm_ik_candidates(
        scene.robot_id, scene.model, arm_joints, target, cid
    )
    reachable = [(q, e) for q, e in candidates if e <= REACH_TOLERANCE]
    if not reachable:
        best_err = candidates[0][1] if candidates else math.inf
        print(
            f"[reach] точка вне досягаемости {arm}-руки: "
            f"лучшая ошибка IK {best_err * 100:.1f} см -- отмена "
            f"(попробуйте другую руку: клавиша {'R' if arm == 'left' else 'L'})"
        )
        return

    start_q = arm_q(scene.robot_id, arm_joints, cid)
    other_q = arm_q(scene.robot_id, other_joints, cid)
    env_names = {scene.table_id: "стол", scene.cube_id: "куб"}
    env_names.update({oid: "препятствие" for oid in scene.obstacle_ids})
    rejections = []

    def segment_ok(seg_start, seg_goal):
        ok, info = validate_path(guard, arm, seg_start, seg_goal, other_q)
        if not ok:
            t, report = info
            pair = report.get("pair", report) if isinstance(report, dict) else report
            rejections.append(f"self-collision t={t:.2f} {pair}")
            return False
        ok, info = validate_path_env(scene, arm_joints, seg_start, seg_goal)
        if not ok:
            t, body = info
            rejections.append(
                f"окружение t={t:.2f} {env_names.get(body, f'body {body}')}"
            )
            return False
        return True

    # --- Фаза 1: прямой joint-space путь -------------------------------------
    for goal_q, err in reachable:
        if segment_ok(start_q, goal_q):
            print(f"[reach] прямой путь чист (ошибка IK {err * 100:.1f} см)")
            execute_motion(scene, arm_joints, goal_q)
            print("[reach] готово")
            return

    # --- Фаза 2: декартов путь "подъём -> пролёт -> спуск" ----------------------
    # Прямой свинг в joint space ведёт EE по дуге, которая проваливается ниже
    # столешницы. Поэтому строим путь декартовыми вейпоинтами: подъём EE
    # вертикально на безопасную высоту, горизонтальный пролёт к точке над
    # целью с шагом CART_STEP, спуск к цели. Каждый вейпоинт решается IK ОТ
    # конфигурации предыдущего -- соседние конфигурации близки в joint space,
    # и дуге между ними негде нырнуть (анalog computeCartesianPath в MoveIt).
    ee_now = p.getLinkState(scene.robot_id, arm_joints[-1], physicsClientId=cid)[4]
    z_safe = max(ee_now[2], target[2]) + APPROACH_HEIGHT
    cart_points = [[ee_now[0], ee_now[1], z_safe]]
    horiz_dist = math.dist(ee_now[:2], target[:2])
    n_mid = max(1, math.ceil(horiz_dist / CART_STEP))
    for i in range(1, n_mid + 1):
        a = i / n_mid
        cart_points.append([
            ee_now[0] + (target[0] - ee_now[0]) * a,
            ee_now[1] + (target[1] - ee_now[1]) * a,
            z_safe,
        ])
    cart_points.append(list(target))
    print(
        f"[reach] прямой путь блокирован, строю декартов путь: "
        f"{len(cart_points)} вейпоинтов, высота пролёта {z_safe:.2f} м"
    )

    configs = []
    seed = start_q
    for pt in cart_points:
        q, err = solve_arm_ik_seeded(
            scene.robot_id, scene.model, arm_joints, pt, seed, cid
        )
        if q is None or err > REACH_TOLERANCE:
            rejections.append(
                f"IK вейпоинта ({pt[0]:.2f},{pt[1]:.2f},{pt[2]:.2f}): "
                f"{(err if err is not None else math.inf) * 100:.0f} см"
            )
            configs = None
            break
        configs.append(q)
        seed = q

    if configs is not None:
        prev = start_q
        path_clear = True
        for q in configs:
            if not segment_ok(prev, q):
                path_clear = False
                break
            prev = q
        if path_clear:
            print(f"[reach] декартов путь чист: {len(configs)} сегментов")
            for q in configs:
                execute_motion(
                    scene,
                    arm_joints,
                    q,
                    min_duration=SEGMENT_MIN_DURATION_S,
                    verbose=False,
                )
            print("[reach] готово")
            return

    print(
        f"[reach] путь не найден -- отмена. Отклонено сегментов: "
        f"{len(rejections)}. Причины: {'; '.join(rejections[:6])}"
        f"{' ...' if len(rejections) > 6 else ''}. "
        f"Попробуйте промежуточную точку (клик на верх стенки), затем цель."
    )


# --- HUD и главный цикл -------------------------------------------------------------

def update_hud(client_id, arm, targeting, hud_id=None):
    mode = "ON " if targeting else "OFF"
    text = f"Targeting: {mode} [T]   Arm: {arm.upper()} [L]/[R]   click = move"
    color = [0.1, 1.0, 0.1] if targeting else [1.0, 0.3, 0.3]
    kwargs = dict(textColorRGB=color, textSize=1.4, physicsClientId=client_id)
    if hud_id is not None:
        kwargs["replaceItemUniqueId"] = hud_id
    return p.addUserDebugText(text, [0.0, 0.0, 1.9], **kwargs)


def key_triggered(keys, char):
    return ord(char) in keys and (keys[ord(char)] & p.KEY_WAS_TRIGGERED)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--obstacles",
        action="store_true",
        help="Spawn obstacle wall in the middle of the table.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    validate_default_pose()
    scene = load_scene(gui=True, with_obstacles=args.obstacles)
    guard = create_self_collision_guard()

    hover_marker = create_marker(
        scene.client_id, radius=HOVER_MARKER_RADIUS, rgba=HOVER_MARKER_RGBA
    )
    target_marker = create_marker(
        scene.client_id, radius=TARGET_MARKER_RADIUS, rgba=TARGET_MARKER_RGBA
    )

    arm = "right"
    targeting = False  # старт с выключенным: сначала настройте камеру
    hud_id = update_hud(scene.client_id, arm, targeting)
    print("[reach] T -- режим прицеливания, L/R -- рука, клик ЛКМ -- движение")

    try:
        while True:
            keys = p.getKeyboardEvents(physicsClientId=scene.client_id)
            if key_triggered(keys, "t"):
                targeting = not targeting
                if not targeting:
                    move_marker(hover_marker, HIDDEN_POS, scene.client_id)
                hud_id = update_hud(scene.client_id, arm, targeting, hud_id)
                print(f"[reach] прицеливание: {'ВКЛ' if targeting else 'ВЫКЛ'}")
            if key_triggered(keys, "l"):
                arm = "left"
                hud_id = update_hud(scene.client_id, arm, targeting, hud_id)
                print("[reach] выбрана левая рука")
            if key_triggered(keys, "r"):
                arm = "right"
                hud_id = update_hud(scene.client_id, arm, targeting, hud_id)
                print("[reach] выбрана правая рука")

            last_move = None
            click = None
            for ev in p.getMouseEvents(physicsClientId=scene.client_id):
                event_type, mx, my, button_idx, button_state = ev
                if event_type == 1:  # движение мыши
                    last_move = (mx, my)
                elif (
                    event_type == 2          # кнопка
                    and button_idx == 0      # ЛКМ
                    and (button_state & p.KEY_WAS_TRIGGERED)
                ):
                    click = (mx, my)

            if targeting and last_move is not None:
                picked = pick_point(*last_move, scene.client_id)
                if picked is not None and picked[2] != scene.robot_id:
                    move_marker(hover_marker, picked[0], scene.client_id)
                else:
                    move_marker(hover_marker, HIDDEN_POS, scene.client_id)

            if targeting and click is not None:
                handle_click(scene, guard, arm, *click, target_marker)

            p.stepSimulation(physicsClientId=scene.client_id)
            time.sleep(SIM_DT)
    except KeyboardInterrupt:
        pass
    finally:
        guard.close()
        p.disconnect(scene.client_id)


if __name__ == "__main__":
    main()
