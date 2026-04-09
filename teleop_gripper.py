#!/usr/bin/env python3
"""
Телеоп простого гриппера / одного канала «руки» через GripManager.sendControl (док 灵巧手).

Стрелки меняют проценты bend_angle (захват) и опционально rotation_angle у thumb.
Тот же RBK4, порт 19205, тип 2999, что и test_hands.py.

Управление:
  ↑ / ↓  — увеличить / уменьшить bend_angle (шаг --step-bend)
  ← / →  — rotation_angle для thumb (шаг --step-rot; если не нужен — запустите с --no-rotation)
  [ ]    — уменьшить / увеличить шаг bend
  { }    — уменьшить / увеличить шаг rotation
  0      — сбросить bend в --rest-bend
  пробел — отправить текущие значения без изменения (удержание команды)
  q      — выход

Запуск (TTY терминал):
  python3 teleop_gripper.py --ip 192.168.192.7 --band-name Gripper-000

Опционально показать состояние манипуляторов (ArmPlanning get_arm_state, как arm_planning_tcp.py):
  python3 teleop_gripper.py --ip 192.168.192.7 --band-name Gripper-000 --show-arm-state

Важно: этот скрипт — только GripManager (гриппер/кисть). Суставы манипуляторов
управляются через ArmPlanning (см. arm_planning_tcp.py, arm_diagnose.py), не через sendControl.
"""

from __future__ import annotations

import argparse
import json
import socket
import struct
import sys
import time

try:
    import curses
except ImportError:
    print("Нужен curses (Linux / TTY).", file=sys.stderr)
    sys.exit(1)

SYNC = 0x5A
VERSION = 0x01
HEADER_SIZE = 16
HEADER_FMT_RBK4 = "!BBHLH2sH2s"
RESERVED_2 = b"\x00\x00"
ZIP_TYPE_TAG = b"\x00\x00"

MSG_TYPE = 2999
NODE = "GripManager"
SERVICE = "serviceDispatcher"

DEFAULT_PORT = 19205
CONNECT_TIMEOUT = 3.0
IO_TIMEOUT = 2.0


def pack_frame_rbk4(msg_number: int, msg_type: int, body_obj: dict) -> bytes:
    json_bytes = json.dumps(body_obj, ensure_ascii=False, separators=(",", ":")).encode(
        "utf-8"
    )
    json_len = len(json_bytes)
    stream_len = json_len
    return struct.pack(
        HEADER_FMT_RBK4,
        SYNC,
        VERSION,
        msg_number & 0xFFFF,
        stream_len,
        msg_type & 0xFFFF,
        RESERVED_2,
        json_len,
        ZIP_TYPE_TAG,
    ) + json_bytes


def recv_exact(sock: socket.socket, n: int) -> bytes:
    out = bytearray()
    while len(out) < n:
        chunk = sock.recv(n - len(out))
        if not chunk:
            raise ConnectionError("connection closed")
        out += chunk
    return bytes(out)


def recv_response_rbk4(sock: socket.socket) -> dict:
    hdr = recv_exact(sock, HEADER_SIZE)
    _s, _v, _n, stream_len, _t, _r2, jlen, _z = struct.unpack(HEADER_FMT_RBK4, hdr)
    stream = recv_exact(sock, stream_len) if stream_len else b""
    json_part = stream[:jlen]
    tail = stream[jlen:]
    out: dict = {}
    if json_part:
        try:
            out = json.loads(json_part.decode("utf-8"))
        except json.JSONDecodeError:
            out = {"_raw_json_hex": json_part.hex()}
    if tail:
        out["_binary_tail_len"] = len(tail)
        try:
            txt = tail.decode("utf-8")
            try:
                out["_binary_tail_json"] = json.loads(txt)
            except json.JSONDecodeError:
                out["_binary_tail_preview"] = txt[:800]
        except UnicodeDecodeError:
            out["_binary_tail_hex_preview"] = tail[:64].hex()
    return out


def response_error_hint(resp: dict) -> str | None:
    """Краткое описание ошибки из ответа RBK4; None если явной ошибки нет."""
    for key in ("ret_code", "errCode"):
        v = resp.get(key)
        if v not in (None, 0, "0", ""):
            em = resp.get("err_msg") or resp.get("message") or ""
            return f"{key}={v} {em}".strip()
    em = resp.get("err_msg") or resp.get("message") or ""
    if em:
        return str(em)
    tail = resp.get("_binary_tail_json")
    if isinstance(tail, dict):
        for key in ("ret_code", "errCode"):
            v = tail.get(key)
            if v not in (None, 0, "0", ""):
                em = tail.get("err_msg") or tail.get("message") or ""
                return f"{key}={v} {em}".strip()
        if tail.get("err_msg"):
            return str(tail["err_msg"])
        st = tail.get("status")
        if isinstance(st, str):
            low = st.lower()
            if "success" in low or "send success" in low:
                return None
            if any(x in low for x in ("error", "fail", "invalid", "refuse", "deny")):
                return st
    return None


def send_control(
    sock: socket.socket,
    seq: list,
    band_name: str,
    cmd_type: str,
    finger: str,
    bend: float,
    rotation: float,
    velocity: float,
    use_rotation: bool,
    hand_id: int | None,
) -> str | None:
    seq[0] = (seq[0] + 1) & 0xFFFF
    param: dict = {
        "finger_name": finger,
        "bend_angle": bend,
        "velocity": velocity,
    }
    if use_rotation:
        param["rotation_angle"] = rotation
    req: dict = {
        "func_name": "sendControl",
        "band_name": [band_name],
        "cmd_type": cmd_type,
        "param": [param],
    }
    if hand_id is not None:
        req["hand_id"] = hand_id
    body = {
        "node_name": NODE,
        "service_name": SERVICE,
        "request": req,
    }
    sock.sendall(pack_frame_rbk4(seq[0], MSG_TYPE, body))
    try:
        resp = recv_response_rbk4(sock)
    except (OSError, ConnectionError) as e:
        return f"recv: {e}"
    return response_error_hint(resp)


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def poll_arm_planning_state(ip: str, port: int) -> str:
    """get_arm_state через arm_planning_tcp.call_once (отдельное TCP на тот же порт)."""
    try:
        from arm_planning_tcp import call_once, status_from_response

        r = call_once(
            ip,
            {"func_name": "getCurrentState", "command": "get_arm_state"},
            port=port,
        )
        st = status_from_response(r)
        if isinstance(st, dict) and "get_arm_state" in st:
            return str(st["get_arm_state"])
    except OSError:
        pass
    except Exception:
        pass
    return "?"


def run(
    stdscr,
    ip: str,
    port: int,
    band_name: str,
    cmd_type: str,
    finger: str,
    hz: float,
    step_bend: float,
    step_rot: float,
    rest_bend: float,
    velocity: float,
    use_rotation: bool,
    hand_id: int | None,
    show_arm_state: bool,
    arm_status_interval: float,
) -> None:
    curses.cbreak()
    stdscr.nodelay(True)
    stdscr.keypad(True)
    curses.curs_set(0)

    period = 1.0 / max(hz, 1.0)
    bend = float(rest_bend)
    rotation = 50.0
    sb = step_bend
    sr = step_rot

    sock = socket.create_connection((ip, port), timeout=CONNECT_TIMEOUT)
    sock.settimeout(IO_TIMEOUT)
    seq = [0]

    arm_line = (
        f"ArmPlanning get_arm_state: {poll_arm_planning_state(ip, port)}"
        if show_arm_state
        else ""
    )
    last_arm_poll = time.monotonic()
    last_grip_err = ""

    lines = [
        f"GripManager teleop  {ip}:{port}  band={band_name}  finger={finger}",
        f"bend {sb:.1f}% / step   rot {sr:.1f}% / step   vel={velocity}   cmd={cmd_type}",
        "↑↓ bend   ←→ rotation (thumb)   0 reset bend   [ ] step bend   {{ }} step rot   q quit",
        arm_line if show_arm_state else "",
    ]

    try:
        while True:
            ch = stdscr.getch()
            while ch != -1:
                if ch in (curses.KEY_UP, ord("w"), ord("W")):
                    bend = clamp(bend + sb, 0.0, 100.0)
                elif ch in (curses.KEY_DOWN, ord("s"), ord("S")):
                    bend = clamp(bend - sb, 0.0, 100.0)
                elif use_rotation and ch in (curses.KEY_LEFT, ord("a"), ord("A")):
                    rotation = clamp(rotation + sr, 0.0, 100.0)
                elif use_rotation and ch in (curses.KEY_RIGHT, ord("d"), ord("D")):
                    rotation = clamp(rotation - sr, 0.0, 100.0)
                elif ch == ord("0"):
                    bend = float(rest_bend)
                elif ch == ord("["):
                    sb = max(0.5, sb - 0.5)
                elif ch == ord("]"):
                    sb = min(20.0, sb + 0.5)
                elif ch == ord("{"):
                    sr = max(0.5, sr - 0.5)
                elif ch == ord("}"):
                    sr = min(20.0, sr + 0.5)
                elif ch in (ord("q"), ord("Q")):
                    return
                ch = stdscr.getch()

            err = send_control(
                sock,
                seq,
                band_name,
                cmd_type,
                finger,
                bend,
                rotation,
                velocity,
                use_rotation,
                hand_id,
            )
            if err:
                last_grip_err = err
            else:
                last_grip_err = ""

            if show_arm_state and time.monotonic() - last_arm_poll >= arm_status_interval:
                lines[3] = f"ArmPlanning get_arm_state: {poll_arm_planning_state(ip, port)}"
                last_arm_poll = time.monotonic()

            stdscr.erase()
            for i, line in enumerate(lines):
                stdscr.addstr(i, 0, line[: stdscr.getmaxyx()[1] - 1])
            rot_s = f" rot={rotation:5.1f}" if use_rotation else ""
            stdscr.addstr(
                len(lines),
                0,
                f"bend={bend:5.1f}%{rot_s}   step_bend={sb:.1f} step_rot={sr:.1f}",
            )
            _my, mx = stdscr.getmaxyx()
            if last_grip_err:
                stdscr.addstr(
                    min(len(lines) + 1, _my - 1),
                    0,
                    f"GripManager ERR: {last_grip_err}"[: mx - 1],
                    curses.A_REVERSE,
                )
            stdscr.refresh()
            time.sleep(period)
    finally:
        sock.close()


def main() -> None:
    ap = argparse.ArgumentParser(description="Arrow-key teleop for simple gripper (sendControl)")
    ap.add_argument("--ip", default="192.168.192.7")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--band-name", default="Gripper-000", help="Имя модели в Roboshop")
    ap.add_argument("--cmd-type", default="EtherCAT", choices=("EtherCAT", "RS485"))
    ap.add_argument("--finger", default="thumb", help="Для простого гриппера обычно thumb")
    ap.add_argument("--velocity", type=float, default=40.0, help="скорость в процентах")
    ap.add_argument("--hz", type=float, default=8.0, help="частота sendControl")
    ap.add_argument("--step-bend", type=float, default=2.0, help="процентов bend за ↑/↓")
    ap.add_argument("--step-rot", type=float, default=3.0, help="процентов rotation за ←/→")
    ap.add_argument("--rest-bend", type=float, default=50.0, help="сброс клавишей 0")
    ap.add_argument("--hand-id", type=int, default=None, help="если нужно по доке (2/3 и т.д.)")
    ap.add_argument(
        "--no-rotation",
        action="store_true",
        help="не слать rotation_angle (только bend)",
    )
    ap.add_argument(
        "--show-arm-state",
        action="store_true",
        help="периодически опрашивать ArmPlanning get_arm_state (как arm_planning_tcp.py)",
    )
    ap.add_argument(
        "--arm-status-interval",
        type=float,
        default=2.0,
        metavar="SEC",
        help="интервал опроса при --show-arm-state (по умолчанию 2 с)",
    )
    args = ap.parse_args()
    use_rotation = not args.no_rotation and args.finger == "thumb"

    def _run(stdscr) -> None:
        run(
            stdscr,
            args.ip,
            args.port,
            args.band_name,
            args.cmd_type,
            args.finger,
            args.hz,
            args.step_bend,
            args.step_rot,
            args.rest_bend,
            args.velocity,
            use_rotation,
            args.hand_id,
            args.show_arm_state,
            max(0.5, args.arm_status_interval),
        )

    curses.wrapper(_run)


if __name__ == "__main__":
    main()
