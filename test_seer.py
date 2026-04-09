#!/usr/bin/env python3
import argparse
import json
import socket
import struct
from datetime import datetime

ROBOT_IP = "192.168.192.7"
DEFAULT_PORT = 19205
CONNECT_TIMEOUT = 3.0
IO_TIMEOUT = 3.0
SYNC = 0x5A
VERSION = 0x01
HEADER_SIZE = 16
# RBK4 (официальный пример из доки): sync, ver, number, stream_len, type, res[2], json_len, zip[2]
HEADER_FMT_RBK4 = "!BBHLH2sH2s"
RESERVED_2 = b"\x00\x00"
ZIP_TYPE_TAG = b"\x00\x00"
# Упрощённый вариант из таблицы «报文结构»: только JSON, 6 байт reserved
HEADER_FMT_SIMPLE = "!BBHIH6s"


def now():
    return datetime.now().strftime("%H:%M:%S")


def hexdump(data: bytes, width: int = 16) -> str:
    lines = []
    for i in range(0, len(data), width):
        chunk = data[i:i + width]
        hex_part = " ".join(f"{b:02X}" for b in chunk)
        ascii_part = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
        lines.append(f"{i:04X}  {hex_part:<{width*3}}  {ascii_part}")
    return "\n".join(lines) if lines else "<empty>"


class RobotTcpClient:
    def __init__(self, ip: str, port: int):
        self.ip = ip
        self.port = port
        self.sock = None

    def connect(self):
        self.close()
        self.sock = socket.create_connection((self.ip, self.port), timeout=CONNECT_TIMEOUT)
        self.sock.settimeout(IO_TIMEOUT)
        print(f"[{now()}] Connected to {self.ip}:{self.port}")

    def close(self):
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None

    def send_raw(self, payload: bytes):
        if not self.sock:
            raise RuntimeError("Socket is not connected")
        self.sock.sendall(payload)
        print(f"[{now()}] TX {len(payload)} bytes")
        print(hexdump(payload))

    def recv_exact(self, size: int) -> bytes:
        if not self.sock:
            raise RuntimeError("Socket is not connected")
        chunks = []
        remaining = size
        while remaining > 0:
            chunk = self.sock.recv(remaining)
            if not chunk:
                raise ConnectionError("Socket closed while receiving")
            chunks.append(chunk)
            remaining -= len(chunk)
        data = b"".join(chunks)
        print(f"[{now()}] RX {len(data)} bytes")
        print(hexdump(data))
        return data


def pack_frame_simple(
    msg_number: int, msg_type: int, body_obj: dict | None = None, endian_big: bool = True
) -> bytes:
    if body_obj is None:
        body_obj = {}
    fmt = HEADER_FMT_SIMPLE if endian_big else HEADER_FMT_SIMPLE.replace("!", "<")
    body = json.dumps(body_obj, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    header = struct.pack(
        fmt,
        SYNC,
        VERSION,
        msg_number & 0xFFFF,
        len(body),
        msg_type & 0xFFFF,
        b"\x00" * 6,
    )
    return header + body


def unpack_header_simple(header: bytes, endian_big: bool = True) -> dict:
    fmt = HEADER_FMT_SIMPLE if endian_big else HEADER_FMT_SIMPLE.replace("!", "<")
    sync, version, number, length, msg_type, reserved = struct.unpack(fmt, header)
    return {
        "sync": sync,
        "version": version,
        "number": number,
        "stream_len": length,
        "json_len": length,
        "type": msg_type,
        "reserved": reserved.hex(),
    }


def pack_frame_rbk4(
    msg_number: int,
    msg_type: int,
    body_obj: dict | None = None,
    extra: bytes = b"",
) -> bytes:
    """Как в примере HeaderMgr: stream_len = len(json) + len(extra), json_len отдельно."""
    if body_obj is None:
        body_obj = {}
    json_bytes = json.dumps(body_obj, ensure_ascii=False, separators=(",", ":")).encode(
        "utf-8"
    )
    json_len = len(json_bytes)
    stream_len = json_len + len(extra)
    header = struct.pack(
        HEADER_FMT_RBK4,
        SYNC,
        VERSION,
        msg_number & 0xFFFF,
        stream_len,
        msg_type & 0xFFFF,
        RESERVED_2,
        json_len,
        ZIP_TYPE_TAG,
    )
    return header + json_bytes + extra


def unpack_header_rbk4(header: bytes) -> dict:
    sync, ver, number, stream_len, msg_type, res2, json_len, zip_tag = struct.unpack(
        HEADER_FMT_RBK4, header
    )
    return {
        "sync": sync,
        "version": ver,
        "number": number,
        "stream_len": stream_len,
        "type": msg_type,
        "json_len": json_len,
        "reserved2": res2.hex(),
        "zip_tag": zip_tag.hex(),
    }


def send_request(
    ip: str,
    port: int,
    msg_type: int,
    msg_number: int,
    body_obj: dict,
    proto: str,
    endian_big: bool,
):
    client = RobotTcpClient(ip, port)
    try:
        client.connect()
        if proto == "rbk4":
            frame = pack_frame_rbk4(
                msg_number=msg_number, msg_type=msg_type, body_obj=body_obj
            )
        else:
            frame = pack_frame_simple(
                msg_number=msg_number,
                msg_type=msg_type,
                body_obj=body_obj,
                endian_big=endian_big,
            )
        client.send_raw(frame)

        header = client.recv_exact(HEADER_SIZE)
        if proto == "rbk4":
            parsed = unpack_header_rbk4(header)
            print(f"[{now()}] Header parsed: {parsed}")
            slen = parsed["stream_len"]
            jlen = parsed["json_len"]
            stream = client.recv_exact(slen) if slen else b""
            json_part = stream[:jlen]
            bin_part = stream[jlen:]
            if json_part:
                try:
                    body_json = json.loads(json_part.decode("utf-8"))
                except Exception:
                    body_json = {"_raw_json": json_part.hex()}
            else:
                body_json = {}
            if bin_part:
                body_json["_binary_tail_len"] = len(bin_part)
                try:
                    tail_text = bin_part.decode("utf-8")
                    body_json["_binary_tail_text_preview"] = tail_text[:300]
                    try:
                        body_json["_binary_tail_json"] = json.loads(tail_text)
                    except Exception:
                        pass
                except Exception:
                    pass
        else:
            parsed = unpack_header_simple(header, endian_big=endian_big)
            print(f"[{now()}] Header parsed: {parsed}")
            body_raw = b""
            if parsed["stream_len"] > 0:
                body_raw = client.recv_exact(parsed["stream_len"])
            if body_raw:
                try:
                    body_json = json.loads(body_raw.decode("utf-8"))
                except Exception:
                    body_json = {"_raw": body_raw.hex()}
            else:
                body_json = {}

        print(f"[{now()}] Body parsed: {json.dumps(body_json, ensure_ascii=False)}")
        ret_code = body_json.get("ret_code", 0)
        err_msg = body_json.get("err_msg", "")
        print(f"[{now()}] ret_code={ret_code}, err_msg={err_msg!r}")
    except Exception as e:
        print(f"[{now()}] ERROR -> {e}")
    finally:
        client.close()
        print(f"[{now()}] Closed {ip}:{port}")


def main():
    parser = argparse.ArgumentParser(description="SEER TCP API test client")
    parser.add_argument("--ip", default=ROBOT_IP, help="Robot IP")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="Robot TCP port")
    parser.add_argument("--type", type=int, required=True, help="API message type (uint16)")
    parser.add_argument("--number", type=int, default=1, help="Request number (uint16)")
    parser.add_argument(
        "--json",
        default="{}",
        help="Request body JSON, e.g. '{\"foo\":1}'",
    )
    parser.add_argument(
        "--endian",
        choices=["big", "little"],
        default="big",
        help="Только для --proto simple: порядок байт заголовка",
    )
    parser.add_argument(
        "--proto",
        choices=["rbk4", "simple"],
        default="rbk4",
        help="rbk4: заголовок !BBHLH2sH2s (как в примере из доки); simple: таблица 报文结构",
    )
    args = parser.parse_args()

    try:
        body_obj = json.loads(args.json)
        if not isinstance(body_obj, dict):
            raise ValueError("Body JSON must be an object")
    except Exception as e:
        raise SystemExit(f"Invalid --json payload: {e}")

    send_request(
        ip=args.ip,
        port=args.port,
        msg_type=args.type,
        msg_number=args.number,
        body_obj=body_obj,
        proto=args.proto,
        endian_big=args.endian == "big",
    )


if __name__ == "__main__":
    main()