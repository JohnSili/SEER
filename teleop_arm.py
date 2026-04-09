#!/usr/bin/env python3
"""
Телеоп суставов (MoveJ) через ArmPlanning (RBK4, порт 19205, тип 2999).

По умолчанию читаются обе руки; активная — правая или левая (--arm), переключение клавишей t.

Управление (TTY + curses):
  1–7     — выбрать сустав (индекс 1…7 → joint 0…6)
  ↑ / ↓   — увеличить / уменьшить угол выбранного сустава активной руки (рад, шаг --step)
  w / s   — то же
  [ ]     — уменьшить / увеличить шаг (рад)
  t       — переключить активную руку (левая ↔ права), если доступны обе
  r       — синхронизировать углы активной руки с роботом
  q       — выход

Перед запуском: свободное пространство, аварийная остановка под рукой.
MoveJ не линейный в декартовых координатах — только суставы.

Примеры:
  python3 teleop_arm.py --ip 192.168.192.7 --arm right
  python3 teleop_arm.py --ip 192.168.192.7 --arm left
"""

from __future__ import annotations

import argparse
import sys
import time

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


def run(
    stdscr,
    ip: str,
    port: int,
    initial_arm: str,
    velocity: float,
    acceleration: float,
    step: float,
    min_interval: float,
) -> None:
    curses.cbreak()
    stdscr.nodelay(True)
    stdscr.keypad(True)
    curses.curs_set(0)

    joint_idx = 0
    stp = float(step)
    last_send = 0.0
    last_fb = ""
    last_err = ""

    with ArmPlanningClient(ip, port=port) as client:
        rr = client.get_current_state("right_joints_pos")
        rl = client.get_current_state("left_joints_pos")
        q_right = joints_from_get_pos(rr, "right")
        q_left = joints_from_get_pos(rl, "left")
        ok_r = len(q_right) == 7
        ok_l = len(q_left) == 7

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

        def send_move() -> None:
            nonlocal last_send, last_fb, last_err
            now = time.monotonic()
            if now - last_send < min_interval:
                return
            arm = active_arm
            q = q_right if arm == "right" else q_left
            kw: dict = {
                "move_group_name": f"{arm}_arm",
                "velocity": velocity,
                "acceleration": acceleration,
            }
            if arm == "right":
                kw["move_right_joints"] = list(q)
            else:
                kw["move_left_joints"] = list(q)
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
        title = f"ArmPlanning MoveJ  {ip}:{port}  {mode}  vel={velocity} acc={acceleration}"
        help1 = "1-7 joint  ↑↓/ws  [ ] step  t arm  r sync  q quit"

        try:
            while True:
                ch = stdscr.getch()
                changed = False
                while ch != -1:
                    if dual_ok and ch in (ord("t"), ord("T")):
                        active_arm = "left" if active_arm == "right" else "right"
                    q = q_right if active_arm == "right" else q_left
                    if ch in (ord("q"), ord("Q")):
                        return
                    elif ch in (ord("r"), ord("R")):
                        cmd_pos = (
                            "right_joints_pos" if active_arm == "right" else "left_joints_pos"
                        )
                        rs = client.get_current_state(cmd_pos)
                        nq = joints_from_get_pos(rs, active_arm)
                        if len(nq) == 7:
                            if active_arm == "right":
                                q_right[:] = nq
                            else:
                                q_left[:] = nq
                            last_err = ""
                            last_fb = "synced"
                    elif ord("1") <= ch <= ord("7"):
                        joint_idx = ch - ord("1")
                    elif ch in (curses.KEY_UP, ord("w"), ord("W")):
                        q[joint_idx] += stp
                        changed = True
                    elif ch in (curses.KEY_DOWN, ord("s"), ord("S")):
                        q[joint_idx] -= stp
                        changed = True
                    elif ch == ord("["):
                        stp = max(0.01, stp - 0.01)
                    elif ch == ord("]"):
                        stp = min(0.5, stp + 0.01)
                    ch = stdscr.getch()

                if changed:
                    send_move()

                stdscr.erase()
                stdscr.addstr(0, 0, title[: curses.COLS - 1])
                stdscr.addstr(1, 0, help1[: curses.COLS - 1])
                act = f"{active_arm.upper()}*" if dual_ok else active_arm.upper()
                stdscr.addstr(
                    2,
                    0,
                    f"ACTIVE {act}  step={stp:.3f} rad  j={joint_idx + 1}/7  interval={min_interval:.2f}s",
                )
                mark_r = "*" if active_arm == "right" else " "
                mark_l = "*" if active_arm == "left" else " "
                if ok_r:
                    qr = "  ".join(f"{x:+.4f}" for x in q_right)
                    stdscr.addstr(4, 0, f"R{mark_r} {qr}"[: curses.COLS - 1])
                else:
                    stdscr.addstr(4, 0, "R   (нет данных)"[: curses.COLS - 1])
                if ok_l:
                    ql = "  ".join(f"{x:+.4f}" for x in q_left)
                    stdscr.addstr(5, 0, f"L{mark_l} {ql}"[: curses.COLS - 1])
                else:
                    stdscr.addstr(5, 0, "L   (нет данных)"[: curses.COLS - 1])
                stdscr.addstr(6, 0, f"last: {last_fb}"[: curses.COLS - 1])
                if last_err:
                    stdscr.addstr(
                        7,
                        0,
                        f"ERR: {last_err}"[: curses.COLS - 1],
                        curses.A_REVERSE,
                    )
                stdscr.refresh()
                time.sleep(0.02)
        finally:
            pass


def main() -> None:
    ap = argparse.ArgumentParser(description="Teleop arms (MoveJ, ArmPlanning); обе руки, t — переключение")
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
        "--min-interval",
        type=float,
        default=0.25,
        metavar="SEC",
        help="не чаще одного MoveJ за этот интервал (защита от флуда)",
    )
    args = ap.parse_args()

    def _run(stdscr) -> None:
        run(
            stdscr,
            args.ip,
            args.port,
            args.arm,
            args.velocity,
            args.acceleration,
            args.step,
            max(0.05, args.min_interval),
        )

    curses.wrapper(_run)


if __name__ == "__main__":
    main()
