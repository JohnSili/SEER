#!/usr/bin/env python3
"""
TCP-клиент для 机械臂 API (node ArmPlanning), RBK4, порт 19205, тип 2999.

Док: node_name=ArmPlanning, service_name=serviceDispatcher,
     getCurrentState / robotControl и т.д.

Быстрые проверки:
  python3 arm_planning_tcp.py --ip 192.168.192.7 --preset get_arm_state
  python3 arm_planning_tcp.py --ip 192.168.192.7 --preset all_joints_state

Произвольный запрос (поле request целиком в JSON):
  python3 arm_planning_tcp.py --ip 192.168.192.7 --request-json \\
    '{"func_name":"getCurrentState","command":"right_joints_pos"}'

Реальное время (100 Hz) и импеданс — циклом в своём коде, вызывая ArmPlanningClient.call().
"""

from __future__ import annotations

import argparse
import json
import socket
import struct
import sys
from typing import Any

SYNC = 0x5A
VERSION = 0x01
HEADER_SIZE = 16
HEADER_FMT_RBK4 = "!BBHLH2sH2s"
RESERVED_2 = b"\x00\x00"
ZIP_TYPE_TAG = b"\x00\x00"

MSG_TYPE = 2999
DEFAULT_PORT = 19205
CONNECT_TIMEOUT = 3.0
IO_TIMEOUT = 5.0

NODE = "ArmPlanning"
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
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("connection closed")
        buf += chunk
    return bytes(buf)


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
            out["_binary_tail_preview"] = txt[:800]
            try:
                out["_binary_tail_json"] = json.loads(txt)
            except json.JSONDecodeError:
                pass
        except UnicodeDecodeError:
            out["_binary_tail_hex_preview"] = tail[:64].hex()
    return out


def status_from_response(resp: dict) -> Any | None:
    """Из ответа RBK4: реальные данные обычно в хвосте потока как JSON с ключом status."""
    tail = resp.get("_binary_tail_json")
    if isinstance(tail, dict) and "status" in tail:
        return tail["status"]
    return None


def display_payload(resp: dict, raw: bool) -> dict:
    """CLI: по умолчанию только {\"status\": ...} из хвоста; при --raw весь объект."""
    if raw:
        return resp
    st = status_from_response(resp)
    if st is not None:
        return {"status": st}
    return resp


def build_body(request_inner: dict) -> dict:
    return {"node_name": NODE, "service_name": SERVICE, "request": request_inner}


def call_once(
    ip: str,
    request_inner: dict,
    port: int = DEFAULT_PORT,
    msg_type: int = MSG_TYPE,
    seq: int = 1,
) -> dict:
    body = build_body(request_inner)
    sock = socket.create_connection((ip, port), timeout=CONNECT_TIMEOUT)
    sock.settimeout(IO_TIMEOUT)
    try:
        sock.sendall(pack_frame_rbk4(seq, msg_type, body))
        return recv_response_rbk4(sock)
    finally:
        sock.close()


class ArmPlanningClient:
    """Одно TCP-соединение, последовательные call (для циклов 10 ms)."""

    def __init__(self, ip: str, port: int = DEFAULT_PORT, msg_type: int = MSG_TYPE):
        self.ip = ip
        self.port = port
        self.msg_type = msg_type
        self._sock: socket.socket | None = None
        self._seq = 0

    def connect(self) -> None:
        self.close()
        self._sock = socket.create_connection((self.ip, self.port), timeout=CONNECT_TIMEOUT)
        self._sock.settimeout(IO_TIMEOUT)

    def close(self) -> None:
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    def __enter__(self) -> ArmPlanningClient:
        self.connect()
        return self

    def __exit__(self, *args: Any) -> None:
        self.close()

    def call(self, request_inner: dict) -> dict:
        if not self._sock:
            self.connect()
        assert self._sock is not None
        self._seq = (self._seq + 1) & 0xFFFF
        body = build_body(request_inner)
        self._sock.sendall(pack_frame_rbk4(self._seq, self.msg_type, body))
        return recv_response_rbk4(self._sock)

    def get_current_state(self, command: str, **extra: Any) -> dict:
        req: dict = {"func_name": "getCurrentState", "command": command}
        req.update(extra)
        return self.call(req)

    def robot_control(self, **kwargs: Any) -> dict:
        return self.call({"func_name": "robotControl", **kwargs})


# Пресеты command для getCurrentState (см. доку)
STATE_COMMANDS = (
    "get_arm_state",
    "all_state",
    "all_joints_state",
    "right_joints_pos",
    "right_joints_vel",
    "right_joints_effort",
    "right_joints_state",
    "left_joints_pos",
    "left_joints_vel",
    "left_joints_effort",
    "left_joints_state",
    "right_ee_pose",
    "left_ee_pose",
    "head_joints_pos",
    "jack_pos",
    "waist_joint_pos",
)


def main() -> None:
    ap = argparse.ArgumentParser(description="ArmPlanning TCP (RBK4) helper")
    ap.add_argument("--ip", required=True)
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--seq", type=int, default=1)
    ap.add_argument(
        "--preset",
        choices=list(STATE_COMMANDS),
        help="getCurrentState + command",
    )
    ap.add_argument(
        "--request-json",
        default="",
        help='Целиком объект request, JSON, например {"func_name":"getCurrentState","command":"..."}',
    )
    ap.add_argument(
        "--raw",
        action="store_true",
        help="Печатать полный ответ (route echo + _binary_tail_*); иначе только status из хвоста",
    )
    args = ap.parse_args()

    if args.request_json.strip():
        req = json.loads(args.request_json)
        if not isinstance(req, dict):
            sys.exit("request-json must be an object")
    elif args.preset:
        req = {"func_name": "getCurrentState", "command": args.preset}
    else:
        sys.exit("Укажите --preset или --request-json")

    try:
        resp = call_once(args.ip, req, port=args.port, seq=args.seq)
        out = display_payload(resp, raw=args.raw)
        print(json.dumps(out, ensure_ascii=False, indent=2))
    except OSError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
