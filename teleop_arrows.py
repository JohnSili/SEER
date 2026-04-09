#!/usr/bin/env python3
"""
Телеоп с клавиатуры: стрелки → manual (vx, vy, vw) на порт 19205, тип 2999, RBK4.

Требования: запускать в обычном терминале (не в среде без TTY). Стрелки — через curses.

Управление:
  ↑↓   — вперёд / назад (vx, м/с)
  ←→   — поворот (vw, рад/с; по доке: + против часовой)
  , .  — влево / вправо стрейф (vy), если нужно
  пробел — всё в 0
  [ ] — шаг линейной скорости   { } — шаг угловой
  q     — выход (перед выходом шлёт vx=vy=vw=0)

По доке: если 500 ms нет ручного ввода — считается 0; скрипт шлёт команды чаще (~10 Hz),
чтобы удерживать задание, пока вы держите стрелку или сразу после нажатия.
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
    print("Нужен модуль curses (есть в стандартной библиотеке Linux).", file=sys.stderr)
    sys.exit(1)

# Совпадает с test_seer.py (RBK4)
SYNC = 0x5A
VERSION = 0x01
HEADER_SIZE = 16
HEADER_FMT_RBK4 = "!BBHLH2sH2s"
RESERVED_2 = b"\x00\x00"
ZIP_TYPE_TAG = b"\x00\x00"

MSG_TYPE_TRACKING = 2999
NODE = "Tracking"
SERVICE = "serviceDispatcher"


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


def drain_response(sock: socket.socket) -> None:
    """Прочитать ответ целиком, чтобы сокет не забился."""
    try:
        hdr = recv_exact(sock, HEADER_SIZE)
        _sync, _ver, _num, stream_len, _typ, _r2, _jlen, _z = struct.unpack(
            HEADER_FMT_RBK4, hdr
        )
        if stream_len:
            recv_exact(sock, stream_len)
    except OSError:
        pass


def send_manual(
    sock: socket.socket,
    seq: list,
    vx: float,
    vy: float,
    vw: float,
) -> None:
    seq[0] = (seq[0] + 1) & 0xFFFF
    body = {
        "node_name": NODE,
        "service_name": SERVICE,
        "request": {
            "func_name": "manual",
            "vx": vx,
            "vy": vy,
            "vw": vw,
        },
    }
    sock.sendall(pack_frame_rbk4(seq[0], MSG_TYPE_TRACKING, body))
    drain_response(sock)


def run_teleop(
    stdscr,
    ip: str,
    port: int,
    linear: float,
    angular: float,
    period: float,
) -> None:
    curses.cbreak()
    stdscr.nodelay(True)
    stdscr.keypad(True)
    curses.curs_set(0)

    sock = socket.create_connection((ip, port), timeout=3.0)
    sock.settimeout(2.0)
    seq = [0]

    vx = vy = vw = 0.0

    help_lines = [
        f"SEER teleop  {ip}:{port}  type={MSG_TYPE_TRACKING}",
        f"linear={linear:.3f} m/s   angular={angular:.3f} rad/s   period={period*1000:.0f} ms",
        "↑↓ vx   ←→ vw   , . vy   WASD — то же   SPACE стоп   q выход",
        "Стоп только пробелом (иначе последняя команда повторяется с --hz).",
        "",
    ]

    try:
        while True:
            ch = stdscr.getch()
            while ch != -1:
                if ch in (curses.KEY_UP, ord("w"), ord("W")):
                    vx = linear
                elif ch in (curses.KEY_DOWN, ord("s"), ord("S")):
                    vx = -linear
                elif ch in (curses.KEY_LEFT, ord("a"), ord("A")):
                    vw = angular
                elif ch in (curses.KEY_RIGHT, ord("d"), ord("D")):
                    vw = -angular
                elif ch == ord(","):
                    vy = linear
                elif ch == ord("."):
                    vy = -linear
                elif ch == ord("["):
                    linear = max(0.02, linear - 0.02)
                elif ch == ord("]"):
                    linear = min(0.8, linear + 0.02)
                elif ch == ord("{"):
                    angular = max(0.05, angular - 0.05)
                elif ch == ord("}"):
                    angular = min(2.0, angular + 0.05)
                elif ch == ord(" "):
                    vx = vy = vw = 0.0
                elif ch in (ord("q"), ord("Q")):
                    send_manual(sock, seq, 0.0, 0.0, 0.0)
                    return
                ch = stdscr.getch()

            send_manual(sock, seq, vx, vy, vw)

            stdscr.erase()
            for i, line in enumerate(help_lines):
                stdscr.addstr(i, 0, line[: stdscr.getmaxyx()[1] - 1])
            stdscr.addstr(
                len(help_lines),
                0,
                f"cmd: vx={vx:+.3f}  vy={vy:+.3f}  vw={vw:+.3f}   "
                f"linear={linear:.3f} ang={angular:.3f}",
            )
            stdscr.refresh()
            time.sleep(period)
    finally:
        try:
            send_manual(sock, seq, 0.0, 0.0, 0.0)
        except OSError:
            pass
        sock.close()


def main() -> None:
    ap = argparse.ArgumentParser(description="Arrow-key teleop for SEER Tracking manual mode")
    ap.add_argument("--ip", default="192.168.192.7")
    ap.add_argument("--port", type=int, default=19205)
    ap.add_argument("--linear", type=float, default=0.1, help="м/с для стрелок вперёд/назад")
    ap.add_argument("--angular", type=float, default=0.25, help="рад/с для поворота")
    ap.add_argument(
        "--hz",
        type=float,
        default=10.0,
        help="частота отправки команд (рекомендуется > 2, чтобы не упереться в 500 ms)",
    )
    args = ap.parse_args()
    period = 1.0 / max(args.hz, 1.0)

    def _run(stdscr) -> None:
        run_teleop(stdscr, args.ip, args.port, args.linear, args.angular, period)

    curses.wrapper(_run)


if __name__ == "__main__":
    main()
