#!/usr/bin/env python3
"""
Телеоп руки через ArmPlanning (RBK4, порт 19205, тип 2999).

По умолчанию читаются обе руки; активная — правая или левая (--arm), переключение клавишей t.
Режим управления переключается клавишей m:
- JOINT: MoveJ по суставам
- CART: линейная команда TCP в базе через move_right_line / move_left_line

Управление (TTY + curses):
  m       — переключить режим JOINT ↔ CART
  JOINT:
    1–7     — выбрать сустав (индекс 1…7 → joint 0…6)
    ↑ / ↓   — увеличить / уменьшить угол выбранного сустава активной руки (рад, шаг --step)
    w / s   — то же
    [ ]     — уменьшить / увеличить шаг по суставам
  CART:
    1–3     — выбрать ось X / Y / Z (м, шаг --cart-step-pos)
    4–6     — выбрать Roll / Pitch / Yaw (рад, шаг --cart-step-rot)
    ↑ / ↓   — изменить выбранную компоненту позы активной руки
    w / s   — то же
    [ ]     — уменьшить / увеличить шаг по выбранной компоненте
  w / s   — то же
  t       — переключить активную руку (левая ↔ права), если доступны обе
  r       — синхронизировать активное состояние с роботом
  q       — выход

Перед запуском: свободное пространство, аварийная остановка под рукой.
В CART режим передаётся pose в базе в формате [x, y, z, qx, qy, qz, qw].

Примеры:
  python3 teleop_arm.py --ip 192.168.192.7 --arm right
  python3 teleop_arm.py --ip 192.168.192.7 --arm left

  Локальная проверка self-collision (PyBullet + URDF коллизии + SRDF disable_collisions):
  pip install pybullet
  python3 teleop_arm.py --ip 192.168.192.7 --self-collision-check

  Все ресурсы для self-collision собраны в self-collision-mesh/:
  python3 teleop_arm.py --ip 192.168.192.7 --self-collision-check \
    --collision-urdf self-collision-mesh/robot_moveit_abs.urdf \
    --collision-srdf self-collision-mesh/wheeled_humanoid_v3_2.srdf \
    --collision-mesh-share self-collision-mesh
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

from evaluation.collision_guard import (
    SelfCollisionGuard,
    parse_csv_names,
    parse_ignore_pairs,
)

try:
    import curses
except ImportError:
    print("Нужен curses (Linux / TTY).", file=sys.stderr)
    sys.exit(1)


from arm_planning_tcp import ArmPlanningClient, status_from_response


def _status_dict(resp: dict) -> dict | None:
    st = status_from_response(resp)
    return st if isinstance(st, dict) else None


def joints_from_get_pos(resp: dict, arm: str) -> list[float]:
    key = "right_joints_pos" if arm == "right" else "left_joints_pos"
    st = _status_dict(resp)
    if not st or key not in st:
        return []
    raw = st[key]
    if not isinstance(raw, list) or len(raw) != 7:
        return []
    return [float(x) for x in raw]


def pose7_from_get_pos(resp: dict, arm: str) -> tuple[list[float], list[float]] | None:
    key = "right_ee_pose" if arm == "right" else "left_ee_pose"
    st = _status_dict(resp)
    if st is None:
        return None

    def from_list7(v) -> tuple[list[float], list[float]] | None:
        if isinstance(v, list) and len(v) == 7:
            try:
                pos = [float(v[0]), float(v[1]), float(v[2])]
                quat = [float(v[3]), float(v[4]), float(v[5]), float(v[6])]
            except (TypeError, ValueError):
                return None
            return pos, quat
        return None

    def walk(x) -> tuple[list[float], list[float]] | None:
        if isinstance(x, dict):
            for nested_key in (key, "ee_pose", "pose", "data"):
                if nested_key in x:
                    got = walk(x[nested_key])
                    if got is not None:
                        return got
            if all(k in x for k in ("x", "y", "z")):
                quat = None
                for qk in (
                    ("qx", "qy", "qz", "qw"),
                    ("orientation_x", "orientation_y", "orientation_z", "orientation_w"),
                ):
                    if all(k in x for k in qk):
                        quat = qk
                        break
                if quat is not None:
                    try:
                        pos = [float(x["x"]), float(x["y"]), float(x["z"])]
                        q = [float(x[quat[0]]), float(x[quat[1]]), float(x[quat[2]]), float(x[quat[3]])]
                    except (TypeError, ValueError):
                        return None
                    return pos, q
            for v in x.values():
                got = walk(v)
                if got is not None:
                    return got
        elif isinstance(x, list):
            got = from_list7(x)
            if got is not None:
                return got
            for item in x:
                got = walk(item)
                if got is not None:
                    return got
        return None

    return walk(st)


def _extract_scalar_from_status(resp: dict, keys: tuple[str, ...]) -> float | None:
    """Достаёт одиночное числовое состояние из status (float/int или [x])."""
    st = _status_dict(resp)
    if not st:
        return None
    for key in keys:
        v = st.get(key)
        if isinstance(v, (int, float)):
            return float(v)
        if isinstance(v, list) and len(v) == 1 and isinstance(v[0], (int, float)):
            return float(v[0])
    return None


def move_feedback(resp: dict) -> str:
    """Краткая строка об ответе robotControl."""
    t = resp.get("_binary_tail_json")
    if isinstance(t, dict):
        s = t.get("status")
        if s is not None:
            return str(s)[:120]
    st = _status_dict(resp)
    if isinstance(st, str):
        return st[:120]
    return "ok"


def _normalize_quat_xyzw(q: list[float]) -> list[float]:
    n = math.sqrt(sum(v * v for v in q))
    if n <= 1e-12:
        return [0.0, 0.0, 0.0, 1.0]
    return [v / n for v in q]


def _quat_xyzw_to_rpy(q: list[float]) -> list[float]:
    x, y, z, w = _normalize_quat_xyzw(q)

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return [roll, pitch, yaw]


def _rpy_to_quat_xyzw(rpy: list[float]) -> list[float]:
    roll, pitch, yaw = rpy
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return _normalize_quat_xyzw([qx, qy, qz, qw])


def run(
    stdscr,
    ip: str,
    port: int,
    initial_arm: str,
    velocity: float,
    acceleration: float,
    step: float,
    cart_step_pos: float,
    cart_step_rot: float,
    min_interval: float,
    collision_guard: SelfCollisionGuard | None,
    collision_waist_joint_name: str,
    collision_waist_state_command: str,
) -> None:
    curses.cbreak()
    stdscr.nodelay(True)
    stdscr.keypad(True)
    curses.curs_set(0)

    joint_idx = 0
    stp = float(step)
    cart_idx = 0
    cart_pos_step = float(cart_step_pos)
    cart_rot_step = float(cart_step_rot)
    last_send = 0.0
    last_fb = ""
    last_err = ""

    with ArmPlanningClient(ip, port=port) as client:
        rr = client.get_current_state("right_joints_pos")
        rl = client.get_current_state("left_joints_pos")
        rp = client.get_current_state("right_ee_pose")
        lp = client.get_current_state("left_ee_pose")
        q_right = joints_from_get_pos(rr, "right")
        q_left = joints_from_get_pos(rl, "left")
        p_right_raw = pose7_from_get_pos(rp, "right")
        p_left_raw = pose7_from_get_pos(lp, "left")
        ok_r = len(q_right) == 7
        ok_l = len(q_left) == 7
        pose_ok_r = p_right_raw is not None
        pose_ok_l = p_left_raw is not None
        p_right = p_right_raw[0] if p_right_raw is not None else [0.0, 0.0, 0.0]
        p_left = p_left_raw[0] if p_left_raw is not None else [0.0, 0.0, 0.0]
        rpy_right = _quat_xyzw_to_rpy(p_right_raw[1]) if p_right_raw is not None else [0.0, 0.0, 0.0]
        rpy_left = _quat_xyzw_to_rpy(p_left_raw[1]) if p_left_raw is not None else [0.0, 0.0, 0.0]

        if not ok_r and not ok_l:
            msg = (
                f"Не удалось прочитать углы (нужно по 7 на руку). "
                f"right_ok={ok_r} left_ok={ok_l}  rr={str(rr)[:120]}  rl={str(rl)[:120]}"
            )
            stdscr.addstr(0, 0, msg[: curses.COLS - 1])
            stdscr.refresh()
            stdscr.nodelay(False)
            stdscr.getch()
            return

        dual_ok = ok_r and ok_l
        active_arm: str
        if dual_ok:
            active_arm = initial_arm if initial_arm in ("right", "left") else "right"
        else:
            active_arm = "right" if ok_r else "left"
        active_mode = "joint"

        last_waist: float | None = None

        def sync_joint_state(arm: str) -> bool:
            nonlocal last_fb, last_err
            cmd_pos = "right_joints_pos" if arm == "right" else "left_joints_pos"
            rs = client.get_current_state(cmd_pos)
            nq = joints_from_get_pos(rs, arm)
            if len(nq) != 7:
                last_err = f"не удалось прочитать {cmd_pos}"
                return False
            if arm == "right":
                q_right[:] = nq
            else:
                q_left[:] = nq
            last_err = ""
            last_fb = "synced joints"
            return True

        def sync_pose_state(arm: str) -> bool:
            nonlocal pose_ok_r, pose_ok_l, last_fb, last_err
            cmd_pose = "right_ee_pose" if arm == "right" else "left_ee_pose"
            rs = client.get_current_state(cmd_pose)
            pq = pose7_from_get_pos(rs, arm)
            if pq is None:
                last_err = f"не удалось прочитать {cmd_pose}"
                return False
            pos, quat = pq
            if arm == "right":
                p_right[:] = pos
                rpy_right[:] = _quat_xyzw_to_rpy(quat)
                pose_ok_r = True
            else:
                p_left[:] = pos
                rpy_left[:] = _quat_xyzw_to_rpy(quat)
                pose_ok_l = True
            last_err = ""
            last_fb = "synced pose"
            return True

        def send_move() -> None:
            nonlocal last_send, last_fb, last_err, last_waist
            now = time.monotonic()
            if now - last_send < min_interval:
                return
            if active_mode == "joint" and collision_guard is not None:
                # Синхронизируем туловище с роботом: иначе локальная модель остаётся в URDF-ноле.
                try:
                    w_resp = client.get_current_state(collision_waist_state_command)
                    w = _extract_scalar_from_status(
                        w_resp,
                        (
                            collision_waist_state_command,
                            "waist_joint_pos",
                            "waist_pos",
                        ),
                    )
                    if w is not None:
                        collision_guard.set_joint_state(collision_waist_joint_name, w)
                        last_waist = w
                except Exception:
                    # Не ломаем телеоп, если чтение/маппинг waist временно недоступны.
                    pass
                col, msg = collision_guard.check(q_right, q_left)
                if col:
                    last_fb = "blocked"
                    last_err = f"SELF-COLLISION: {msg}"
                    return
            arm = active_arm
            kw: dict = {
                "move_group_name": f"{arm}_arm",
                "velocity": velocity,
                "acceleration": acceleration,
            }
            if active_mode == "joint":
                q = q_right if arm == "right" else q_left
                if arm == "right":
                    kw["move_right_joints"] = list(q)
                else:
                    kw["move_left_joints"] = list(q)
            else:
                if arm == "right" and not pose_ok_r:
                    last_err = "нет right_ee_pose для CART"
                    last_fb = "not sent"
                    return
                if arm == "left" and not pose_ok_l:
                    last_err = "нет left_ee_pose для CART"
                    last_fb = "not sent"
                    return
                pos = p_right if arm == "right" else p_left
                rpy = rpy_right if arm == "right" else rpy_left
                pose7 = list(pos) + _rpy_to_quat_xyzw(rpy)
                if arm == "right":
                    kw["move_right_line"] = pose7
                else:
                    kw["move_left_line"] = pose7
            try:
                resp = client.robot_control(**kw)
                last_send = now
                hint = move_feedback(resp)
                last_fb = hint
                low = hint.lower()
                if "success" in low:
                    last_err = ""
                elif "error" in low or "fail" in low:
                    last_err = hint
                else:
                    last_err = ""
            except OSError as e:
                last_err = str(e)
                last_fb = ""

        mode = "RIGHT+LEFT (t)" if dual_ok else ("RIGHT only" if ok_r else "LEFT only")
        title = f"ArmPlanning teleop  {ip}:{port}  {mode}  vel={velocity} acc={acceleration}"

        try:
            while True:
                ch = stdscr.getch()
                changed = False
                while ch != -1:
                    if ch in (ord("m"), ord("M")):
                        if active_mode == "joint":
                            if sync_pose_state(active_arm):
                                active_mode = "cart"
                                cart_idx = min(cart_idx, 5)
                                last_fb = "cart mode"
                        else:
                            active_mode = "joint"
                            last_fb = "joint mode"
                    if dual_ok and ch in (ord("t"), ord("T")):
                        active_arm = "left" if active_arm == "right" else "right"
                    if ch in (ord("q"), ord("Q")):
                        return
                    elif ch in (ord("r"), ord("R")):
                        if active_mode == "joint":
                            sync_joint_state(active_arm)
                        else:
                            sync_pose_state(active_arm)
                    elif active_mode == "joint" and ord("1") <= ch <= ord("7"):
                        joint_idx = ch - ord("1")
                    elif active_mode == "cart" and ord("1") <= ch <= ord("6"):
                        cart_idx = ch - ord("1")
                    elif ch in (curses.KEY_UP, ord("w"), ord("W")):
                        if active_mode == "joint":
                            q = q_right if active_arm == "right" else q_left
                            q[joint_idx] += stp
                        else:
                            arr = p_right if active_arm == "right" else p_left
                            ang = rpy_right if active_arm == "right" else rpy_left
                            if cart_idx < 3:
                                arr[cart_idx] += cart_pos_step
                            else:
                                ang[cart_idx - 3] += cart_rot_step
                        changed = True
                    elif ch in (curses.KEY_DOWN, ord("s"), ord("S")):
                        if active_mode == "joint":
                            q = q_right if active_arm == "right" else q_left
                            q[joint_idx] -= stp
                        else:
                            arr = p_right if active_arm == "right" else p_left
                            ang = rpy_right if active_arm == "right" else rpy_left
                            if cart_idx < 3:
                                arr[cart_idx] -= cart_pos_step
                            else:
                                ang[cart_idx - 3] -= cart_rot_step
                        changed = True
                    elif ch == ord("["):
                        if active_mode == "joint":
                            stp = max(0.01, stp - 0.01)
                        elif cart_idx < 3:
                            cart_pos_step = max(0.001, cart_pos_step - 0.001)
                        else:
                            cart_rot_step = max(0.01, cart_rot_step - 0.01)
                    elif ch == ord("]"):
                        if active_mode == "joint":
                            stp = min(0.5, stp + 0.01)
                        elif cart_idx < 3:
                            cart_pos_step = min(0.1, cart_pos_step + 0.001)
                        else:
                            cart_rot_step = min(0.5, cart_rot_step + 0.01)
                    ch = stdscr.getch()

                if changed:
                    send_move()

                stdscr.erase()
                help1 = (
                    "JOINT: 1-7 joint  ↑↓/ws  [ ] step(rad)  m mode  t arm  r sync  q quit"
                    if active_mode == "joint"
                    else "CART: 1-3 xyz 4-6 rpy  ↑↓/ws  [ ] step(m/rad)  m mode  t arm  r sync  q quit"
                )
                stdscr.addstr(0, 0, title[: curses.COLS - 1])
                stdscr.addstr(1, 0, help1[: curses.COLS - 1])
                act = f"{active_arm.upper()}*" if dual_ok else active_arm.upper()
                if active_mode == "joint":
                    stdscr.addstr(
                        2,
                        0,
                        f"ACTIVE {act}  MODE=JOINT  step={stp:.3f} rad  j={joint_idx + 1}/7  interval={min_interval:.2f}s",
                    )
                else:
                    labels = ("X", "Y", "Z", "R", "P", "Y")
                    sel = labels[cart_idx]
                    cur_step = cart_pos_step if cart_idx < 3 else cart_rot_step
                    unit = "m" if cart_idx < 3 else "rad"
                    stdscr.addstr(
                        2,
                        0,
                        f"ACTIVE {act}  MODE=CART  sel={sel}  step={cur_step:.3f} {unit}  interval={min_interval:.2f}s",
                    )
                mark_r = "*" if active_arm == "right" else " "
                mark_l = "*" if active_arm == "left" else " "
                if active_mode == "joint" and ok_r:
                    qr = "  ".join(f"{x:+.4f}" for x in q_right)
                    stdscr.addstr(4, 0, f"R{mark_r} {qr}"[: curses.COLS - 1])
                elif active_mode == "joint":
                    stdscr.addstr(4, 0, "R   (нет данных)"[: curses.COLS - 1])
                elif pose_ok_r:
                    xyzrpy = "  ".join(f"{x:+.4f}" for x in (p_right + rpy_right))
                    stdscr.addstr(4, 0, f"R{mark_r} {xyzrpy}"[: curses.COLS - 1])
                else:
                    stdscr.addstr(4, 0, "R   (нет ee_pose)"[: curses.COLS - 1])
                if active_mode == "joint" and ok_l:
                    ql = "  ".join(f"{x:+.4f}" for x in q_left)
                    stdscr.addstr(5, 0, f"L{mark_l} {ql}"[: curses.COLS - 1])
                elif active_mode == "joint":
                    stdscr.addstr(5, 0, "L   (нет данных)"[: curses.COLS - 1])
                elif pose_ok_l:
                    xyzrpy = "  ".join(f"{x:+.4f}" for x in (p_left + rpy_left))
                    stdscr.addstr(5, 0, f"L{mark_l} {xyzrpy}"[: curses.COLS - 1])
                else:
                    stdscr.addstr(5, 0, "L   (нет ee_pose)"[: curses.COLS - 1])
                stdscr.addstr(6, 0, f"last: {last_fb}"[: curses.COLS - 1])
                if active_mode == "cart" and collision_guard is not None:
                    stdscr.addstr(7, 0, "note: self-collision-check only for JOINT mode"[: curses.COLS - 1])
                elif collision_guard is not None and last_waist is not None:
                    stdscr.addstr(7, 0, f"waist={last_waist:+.4f} rad"[: curses.COLS - 1])
                if last_err:
                    stdscr.addstr(
                        8,
                        0,
                        f"ERR: {last_err}"[: curses.COLS - 1],
                        curses.A_REVERSE,
                    )
                stdscr.refresh()
                time.sleep(0.02)
        finally:
            if collision_guard is not None:
                collision_guard.close()


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Teleop arms (JOINT MoveJ + CART line, ArmPlanning); обе руки, t — переключение"
    )
    ap.add_argument("--ip", default="192.168.192.7")
    ap.add_argument("--port", type=int, default=19205)
    ap.add_argument(
        "--arm",
        choices=("right", "left"),
        default="right",
        help="какая рука активна при старте (если с обеих сторон пришли углы); дальше — клавиша t",
    )
    ap.add_argument("--velocity", type=float, default=0.3)
    ap.add_argument("--acceleration", type=float, default=0.2)
    ap.add_argument(
        "--step",
        type=float,
        default=0.05,
        help="рад за одно ↑/↓ (по умолчанию ~3°)",
    )
    ap.add_argument(
        "--cart-step-pos",
        type=float,
        default=0.01,
        help="метры за одно ↑/↓ в CART для X/Y/Z",
    )
    ap.add_argument(
        "--cart-step-rot",
        type=float,
        default=0.10,
        help="рад за одно ↑/↓ в CART для roll/pitch/yaw",
    )
    ap.add_argument(
        "--min-interval",
        type=float,
        default=0.25,
        metavar="SEC",
        help="не чаще одного MoveJ за этот интервал (защита от флуда)",
    )
    ap.add_argument(
        "--self-collision-check",
        action="store_true",
        help="проверять self-collision по URDF+SRDF перед отправкой шага",
    )
    ap.add_argument(
        "--collision-urdf",
        default="self-collision-mesh/robot_moveit_abs.urdf",
        help="URDF для локальной проверки коллизий",
    )
    ap.add_argument(
        "--collision-srdf",
        default="self-collision-mesh/wheeled_humanoid_v3_2.srdf",
        help="SRDF (disable_collisions) для фильтрации пар",
    )
    ap.add_argument(
        "--collision-mesh-share",
        default="self-collision-mesh",
        metavar="DIR",
        help="корень модели/пакета с meshes/, meshes_bin/ или meshes_obj/; нужен для package:// и относительных STL в URDF",
    )
    ap.add_argument(
        "--collision-distance",
        type=float,
        default=0.0,
        help="порог getClosestPoints: 0.0=только проникновение, >0 ловить близкие пары",
    )
    ap.add_argument(
        "--collision-srdf-include-default-arm",
        action="store_true",
        help="отключать коллизии по всем Default из SRDF как в MoveIt (в т.ч. arm*↔base_link); по умолчанию base_link↔arm* кроме крепления проверяются строго",
    )
    ap.add_argument(
        "--right-joint-names",
        default="arm_right_1,arm_right_2,arm_right_3,arm_right_4,arm_right_5,arm_right_6,arm_right_7",
        help="CSV joint names для правой руки в URDF",
    )
    ap.add_argument(
        "--left-joint-names",
        default="arm_left_1,arm_left_2,arm_left_3,arm_left_4,arm_left_5,arm_left_6,arm_left_7",
        help="CSV joint names для левой руки в URDF",
    )
    ap.add_argument(
        "--collision-waist-joint-name",
        default="waist1_joint",
        help="имя сустава талии в URDF для синхронизации позы корпуса в collision-check",
    )
    ap.add_argument(
        "--collision-waist-state-command",
        default="waist_joint_pos",
        help="command для getCurrentState, из которого читается угол талии",
    )
    ap.add_argument(
        "--collision-ignore-pairs",
        default=(
            "base_link:arm1_1_link,base_link:arm1_2_link,base_link:arm1_3_link,base_link:arm1_4_link, base_link:arm1_5_link, base_link:arm1_6_link, base_link:arm1_7_link, "
            "base_link:arm2_1_link,base_link:arm2_2_link,base_link:arm2_3_link,base_link:arm2_4_link, base_link:arm2_5_link, base_link:arm2_6_link, base_link:arm2_7_link"
        ),
        help="доп. игнор пар коллизий (CSV linkA:linkB), поверх SRDF",
    )
    args = ap.parse_args()

    guard: SelfCollisionGuard | None = None
    if args.self_collision_check:
        try:
            mesh_override = Path(args.collision_mesh_share) if args.collision_mesh_share.strip() else None
            guard = SelfCollisionGuard(
                urdf_path=Path(args.collision_urdf),
                srdf_path=Path(args.collision_srdf),
                right_joint_names=parse_csv_names(args.right_joint_names),
                left_joint_names=parse_csv_names(args.left_joint_names),
                collision_distance=args.collision_distance,
                include_default_arm_pairs=args.collision_srdf_include_default_arm,
                extra_ignored_pairs=parse_ignore_pairs(args.collision_ignore_pairs),
                mesh_share_root=mesh_override,
            )
            print(
                f"[self-collision] enabled: urdf={args.collision_urdf} srdf={args.collision_srdf} "
                f"distance={args.collision_distance} "
                f"srdf_default_arm={args.collision_srdf_include_default_arm} "
                f"extra_ignored={args.collision_ignore_pairs}",
                file=sys.stderr,
            )
        except Exception as e:
            raise SystemExit(f"Не удалось включить self-collision-check: {e}")

    def _run(stdscr) -> None:
        run(
            stdscr,
            args.ip,
            args.port,
            args.arm,
            args.velocity,
            args.acceleration,
            args.step,
            args.cart_step_pos,
            args.cart_step_rot,
            max(0.05, args.min_interval),
            guard,
            args.collision_waist_joint_name,
            args.collision_waist_state_command,
        )

    curses.wrapper(_run)


if __name__ == "__main__":
    main()
