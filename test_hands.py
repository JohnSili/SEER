#!/usr/bin/env python3
"""
Тест API «灵巧手» (GripManager): RBK4, порт 19205, тип 2999 — как в доке.

Пресеты повторяют структуру из документации; подставьте свои band_name / hand_id.

Обычный гриппер (два кулачка, не «пять пальцев»): в Roboshop устройство всё равно может
называться Gripper-xxx; часто в API остаётся один канал — пробуйте только ``thumb`` и пресеты
``get_status_simple`` / ``send_simple_gripper``. Имя модели смотрите в Roboshop (band_name).

Примеры:
  python3 test_hands.py --ip 192.168.192.7 --preset get_status
  python3 test_hands.py --ip 192.168.192.7 --preset send_thumb --merge \\
    --json '{"request":{"band_name":["Gripper-000"]}}'
"""

from __future__ import annotations

import argparse
import json
import socket
import struct
import sys

# Согласовано с test_seer.py (RBK4)
SYNC = 0x5A
VERSION = 0x01
HEADER_SIZE = 16
HEADER_FMT_RBK4 = "!BBHLH2sH2s"
RESERVED_2 = b"\x00\x00"
ZIP_TYPE_TAG = b"\x00\x00"

CONNECT_TIMEOUT = 3.0
IO_TIMEOUT = 3.0

# Часто управление идёт через 19205 и тип 2999 (как Tracking); если в доке другое — задайте флаги.
DEFAULT_PORT = 19205
DEFAULT_TYPE = 2999


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
            out["_binary_tail_preview"] = txt[:500]
            try:
                out["_binary_tail_json"] = json.loads(txt)
            except json.JSONDecodeError:
                pass
        except UnicodeDecodeError:
            out["_binary_tail_hex_preview"] = tail[:64].hex()
    return out


# Док: node_name = GripManager; в тексте местами опечатка getSataus — в примерах JSON используется getStatus.
PRESETS: dict[str, dict] = {
    "template": {
        "node_name": "GripManager",
        "service_name": "serviceDispatcher",
        "request": {
            "func_name": "sendControl",
            "band_name": ["Gripper-000"],
        },
    },
    # Запрос состояния (все пальцы, если finger_names задан полностью)
    "get_status": {
        "node_name": "GripManager",
        "service_name": "serviceDispatcher",
        "request": {
            "func_name": "getStatus",
            "band_name": ["Gripper-000"],
            "finger_names": ["thumb", "index", "middle", "ring", "little"],
        },
    },
    # Вариант с опечаткой из заголовка дока (если прошивка ожидает именно её)
    "get_status_typo": {
        "node_name": "GripManager",
        "service_name": "serviceDispatcher",
        "request": {
            "func_name": "getSataus",
            "band_name": ["Gripper-000"],
            "finger_names": ["thumb", "index", "middle", "ring", "little"],
        },
    },
    # Простой гриппер: только thumb (часто так мапится один привод закрытия)
    "get_status_simple": {
        "node_name": "GripManager",
        "service_name": "serviceDispatcher",
        "request": {
            "func_name": "getStatus",
            "band_name": ["Gripper-000"],
            "finger_names": ["thumb"],
        },
    },
    # Закрыть/открыть на малый процент — подберите bend_angle под механику
    "send_simple_gripper": {
        "node_name": "GripManager",
        "service_name": "serviceDispatcher",
        "request": {
            "func_name": "sendControl",
            "band_name": ["Gripper-000"],
            "cmd_type": "EtherCAT",
            "param": [
                {
                    "finger_name": "thumb",
                    "bend_angle": 25.0,
                    "velocity": 30.0,
                }
            ],
        },
    },
    # 智元 / общий пример: thumb, bend_angle в snake_case
    "send_thumb": {
        "node_name": "GripManager",
        "service_name": "serviceDispatcher",
        "request": {
            "func_name": "sendControl",
            "band_name": ["Gripper-000"],
            "hand_id": 1,
            "cmd_type": "EtherCAT",
            "param": [
                {
                    "finger_name": "thumb",
                    "bend_angle": 12.0,
                    "velocity": 0.0,
                    "acceleration": 0.0,
                    "deceleration": 0.0,
                    "force": 0.0,
                }
            ],
        },
    },
    # 智元 OmniPicker в доке указано bend_Angle (camelCase) — отдельный пресет
    "send_thumb_omni": {
        "node_name": "GripManager",
        "service_name": "serviceDispatcher",
        "request": {
            "func_name": "sendControl",
            "band_name": ["Gripper-000"],
            "hand_id": 2,
            "cmd_type": "RS485",
            "param": [
                {
                    "finger_name": "thumb",
                    "bend_Angle": 50.0,
                    "velocity": 100.0,
                    "acceleration": 100.0,
                    "deceleration": 100.0,
                    "force": 100.0,
                }
            ],
        },
    },
    # 傲意: четыре пальца (проверьте hand_id: 2 левая, 3 правая)
    "send_four_fingers": {
        "node_name": "GripManager",
        "service_name": "serviceDispatcher",
        "request": {
            "func_name": "sendControl",
            "band_name": ["Gripper-000", "Gripper-001"],
            "hand_id": 2,
            "cmd_type": "EtherCAT",
            "param": [
                {"finger_name": "index", "bend_angle": 40.0, "velocity": 50.0},
                {"finger_name": "middle", "bend_angle": 50.0, "velocity": 50.0},
                {"finger_name": "ring", "bend_angle": 50.0, "velocity": 50.0},
                {"finger_name": "little", "bend_angle": 50.0, "velocity": 50.0},
            ],
        },
    },
    # 强脑: пять пальцев, у всех velocity ненулевая (как в доке)
    "send_hand_brainco": {
        "node_name": "GripManager",
        "service_name": "serviceDispatcher",
        "request": {
            "func_name": "sendControl",
            "band_name": ["Gripper-000", "Gripper-001"],
            "param": [
                {
                    "finger_name": "thumb",
                    "bend_angle": 50.0,
                    "rotation_angle": 0.0,
                    "velocity": 10.0,
                },
                {"finger_name": "index", "bend_angle": 50.0, "velocity": 50.0},
                {"finger_name": "middle", "bend_angle": 50.0, "velocity": 50.0},
                {"finger_name": "ring", "bend_angle": 50.0, "velocity": 50.0},
                {"finger_name": "little", "bend_angle": 50.0, "velocity": 50.0},
            ],
        },
    },
}


def _merge_body(base: dict, extra: dict) -> dict:
    """Поверхностное слияние; для ключа request — слияние вложенных dict."""
    out = dict(base)
    for k, v in extra.items():
        if k == "request" and isinstance(v, dict) and isinstance(out.get("request"), dict):
            merged_req = dict(out["request"])
            merged_req.update(v)
            out["request"] = merged_req
        else:
            out[k] = v
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description="Test dexterous hand API (one-shot RBK4 request)")
    ap.add_argument("--ip", default="192.168.192.7")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--type", type=int, default=DEFAULT_TYPE, help="报文类型, напр. 2999")
    ap.add_argument("--number", type=int, default=1)
    ap.add_argument(
        "--json",
        default="",
        help='Тело запроса JSON. Если задан --preset, --json может дополнять/переопределять поля.',
    )
    ap.add_argument(
        "--preset",
        choices=list(PRESETS.keys()),
        help="Готовая заготовка тела запроса (всё равно проверьте имена в доке)",
    )
    ap.add_argument(
        "--merge",
        action="store_true",
        help="Если заданы и --preset и --json: слить JSON поверх preset (поверх верхнего уровня)",
    )
    args = ap.parse_args()

    body: dict
    if args.preset:
        body = json.loads(json.dumps(PRESETS[args.preset]))
        if args.json.strip():
            extra = json.loads(args.json)
            if not isinstance(extra, dict):
                sys.exit("--json must be an object")
            if args.merge:
                body = _merge_body(body, extra)
            else:
                body = extra
    else:
        if not args.json.strip():
            sys.exit("Укажите --json или --preset (см. python3 test_hands.py --help)")
        body = json.loads(args.json)
        if not isinstance(body, dict):
            sys.exit("--json must be an object")

    sock = socket.create_connection((args.ip, args.port), timeout=CONNECT_TIMEOUT)
    sock.settimeout(IO_TIMEOUT)
    try:
        frame = pack_frame_rbk4(args.number, args.type, body)
        sock.sendall(frame)
        resp = recv_response_rbk4(sock)
        print(json.dumps(resp, ensure_ascii=False, indent=2))
        tail = resp.get("_binary_tail_json")
        err_tail = isinstance(tail, dict) and (
            tail.get("ret_code") or tail.get("err_msg")
        )
        rc = resp.get("ret_code") or 0
        em = resp.get("err_msg") or ""
        if rc or em or err_tail:
            sys.exit(1)
    finally:
        sock.close()


if __name__ == "__main__":
    main()
