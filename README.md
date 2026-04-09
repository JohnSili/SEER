# SEER Robot Control Scripts

Набор Python-скриптов для управления роботом SEER X1 Pro по Ethernet через TCP API (RBK4).

Поддерживаются:
- `ArmPlanning` (суставы рук, состояние, MoveJ)
- `GripManager` (гриппер/кисть)
- `Tracking` (ручное управление базой)

## Требования

- Linux / терминал с TTY
- Python 3.10+
- Сеть до робота (пример IP: `192.168.192.7`)
- Открытые порты (по фактической проверке): `19204`, `19205`, `19208`

> Во всех скриптах используется RBK4-фрейм TCP (`!BBHLH2sH2s`), который корректно работает с контроллером.

## Структура

- `arm_planning_tcp.py` — универсальный клиент `ArmPlanning` (one-shot вызовы + класс `ArmPlanningClient`)
- `teleop_arm.py` — телеоперация суставов рук (MoveJ), поддержка правой/левой, переключение `t`
- `teleop_gripper.py` — телеоперация гриппера (`GripManager.sendControl`)
- `teleop_arrows.py` — телеоперация шасси стрелками (`Tracking.manual`)
- `arm_diagnose.py` — быстрая диагностика состояния руки (`get_arm_state`)
- `test_hands.py` — one-shot пресеты для `GripManager`
- `test_seer.py` — низкоуровневая отладка TCP/RBK4

## Быстрый старт

### 1) Проверить состояние ArmPlanning

```bash
python3 arm_diagnose.py --ip 192.168.192.7
```

Ожидаемое рабочее состояние для MoveJ: `IDLE`.

### 2) One-shot команда правой руки (MoveJ)

```bash
python3 arm_planning_tcp.py --ip 192.168.192.7 --request-json '{"func_name":"robotControl","move_group_name":"right_arm","move_right_joints":[-1.78,1.57,1.57,1.75,0.3,1.0,0.0],"velocity":0.3,"acceleration":0.2}'
```

### 3) Телеоперация руки

```bash
python3 teleop_arm.py --ip 192.168.192.7 --arm right
```

Клавиши:
- `1..7` выбор сустава
- `↑/↓` или `w/s` изменение угла (рад)
- `[` `]` шаг
- `t` переключение активной руки (если доступны обе)
- `r` синхронизация с текущими углами робота
- `q` выход

### 4) Телеоперация гриппера

```bash
python3 teleop_gripper.py --ip 192.168.192.7 --band-name Gripper-000
```

Если нужно видеть состояние ArmPlanning на экране:

```bash
python3 teleop_gripper.py --ip 192.168.192.7 --band-name Gripper-000 --show-arm-state
```

### 5) Телеоперация базы

```bash
python3 teleop_arrows.py --ip 192.168.192.7
```

## Важная инфа

- Команды для рук в API — в радианах.
- `MoveJ` управляет суставами, а не прямолинейным движением TCP.

## Примечания по API

- Для `ArmPlanning` ответы часто содержат route-echo в JSON-части, а полезный payload приходит в tail (`status`).
- `arm_planning_tcp.py` по умолчанию печатает уже полезный `status`; используйте `--raw` для полного ответа.
