#!/usr/bin/env python3
"""
Проверка, почему «рука» (манипулятор) не реагирует на команды.

- `teleop_gripper.py` и GripManager — это гриппер/кисть, не суставы рук.
- Суставы: node ArmPlanning, `robotControl` / `getCurrentState` (см. док 机械臂 API).

Пример:
  python3 arm_diagnose.py --ip 192.168.192.7
"""

from __future__ import annotations

import argparse
import json
import sys

from arm_planning_tcp import call_once, status_from_response


def main() -> None:
    ap = argparse.ArgumentParser(description="ArmPlanning: состояние и подсказки")
    ap.add_argument("--ip", required=True)
    ap.add_argument("--port", type=int, default=19205)
    args = ap.parse_args()

    try:
        raw = call_once(
            args.ip,
            {"func_name": "getCurrentState", "command": "get_arm_state"},
            port=args.port,
        )
    except OSError as e:
        print(f"TCP error: {e}", file=sys.stderr)
        sys.exit(1)

    st = status_from_response(raw)
    # status_from_response уже даёт объект из tail["status"], напр. {"get_arm_state": "IDLE"}
    print(json.dumps(st if st is not None else {"status": None}, ensure_ascii=False, indent=2))

    state = None
    if isinstance(st, dict):
        state = st.get("get_arm_state")

    print("", file=sys.stderr)
    print("Подсказки (манипулятор ArmPlanning):", file=sys.stderr)
    if state == "IDLE":
        print(
            "  IDLE — можно слать MoveJ/MoveJP (robotControl) с velocity/acceleration.",
            file=sys.stderr,
        )
        print(
            '  Пример: python3 arm_planning_tcp.py --ip ... --request-json '
            '\'{"func_name":"robotControl","move_group_name":"right_arm",'
            '"move_right_joints":[...],"velocity":0.5,"acceleration":0.3}\'',
            file=sys.stderr,
        )
    elif state == "REALTIME":
        print(
            "  REALTIME — блокирующий MoveJ может быть отклонён; нужен поток "
            "dual_joints_cmd / real_time_mode по доке.",
            file=sys.stderr,
        )
    elif state == "INITIALIZING":
        print("  INITIALIZING — дождитесь окончания инициализации моторов.", file=sys.stderr)
    elif state in ("ERROR", "Emergency Stop!", "Emergency Stop"):
        print("  Авария/ошибка — сброс на пульте или в ПО перед MoveJ.", file=sys.stderr)
    elif state == "DRAG":
        print("  DRAG — выйдите из drag/teach, затем обычный MoveJ.", file=sys.stderr)
    elif state == "RUNNING":
        print("  RUNNING — движение по предыдущей команде; дождитесь завершения.", file=sys.stderr)
    elif state == "COMPLIANT":
        print("  COMPLIANT — режим импеданса; используйте compliant_control по доке.", file=sys.stderr)
    else:
        print(f"  Текущее состояние: {state!r}", file=sys.stderr)

    print("", file=sys.stderr)
    print(
        "Если нужен именно гриппер: teleop_gripper.py / test_hands.py (GripManager), "
        "не ArmPlanning.",
        file=sys.stderr,
    )


if __name__ == "__main__":
    main()
