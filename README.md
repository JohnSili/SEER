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

### Окружение Python (protobuf для камеры)

Один раз из корня репозитория:

```bash
./scripts/bootstrap_venv.sh
source .venv/bin/activate
```

Либо вручную: `python3 -m venv .venv`, затем `pip install -r requirements.txt` и генерация `proto/messageV4_camera_pb2.py` командой из комментария в `requirements.txt`.

> Во всех скриптах используется RBK4-фрейм TCP (`!BBHLH2sH2s`), который корректно работает с контроллером.

## Структура

- `arm_planning_tcp.py` — универсальный клиент `ArmPlanning` (one-shot вызовы + класс `ArmPlanningClient`)
- `teleop_arm.py` — телеоперация суставов рук (MoveJ), поддержка правой/левой, переключение `t`
- `teleop_gripper.py` — телеоперация гриппера (`GripManager.sendControl`)
- `teleop_arrows.py` — телеоперация шасси стрелками (`Tracking.manual`)
- `arm_diagnose.py` — быстрая диагностика состояния руки (`get_arm_state`)
- `test_hands.py` — one-shot пресеты для `GripManager`
- `test_seer.py` — низкоуровневая отладка TCP/RBK4
- `device_camera_tcp.py` — `DeviceCamera.getCameraData` (порт `19204`, тип `1999`) → PNG (`protobuf` + `opencv`)
- `lidar_tcp.py` — проба лидара / `getLaserScan` на порту `19204` (несколько пресетов `NetProtocol`; точное имя — из вашей доки)
- `robot_info_tcp.py` — **查询机器人信息**, порт `19204`, тип **1000** (модель, `vehicle_id`, `version`, карта, `echoid`, …)
- `resource_manager_tcp.py` — **ResourceManager** (`serviceDispatcher`): список скриптов/MD5, параметры, скачивание/загрузка, удаление/переименование, `genericScript` start/stop/kill (`19204`/`1999` и `19205`/`2999`)

## Быстрый старт

### 0) Кадр с камеры (после `source .venv/bin/activate`)

```bash
python3 device_camera_tcp.py --ip 192.168.192.7 --name Camera-000 --type RGB --out rgb.png
# или все камеры + глубина:
python3 device_camera_tcp.py --ip 192.168.192.7 --name all --type RGBD --out-dir ./cam_out
```

### 0a) Информация о роботе (系统 API, тип 1000)

```bash
python3 robot_info_tcp.py --ip 192.168.192.7
python3 robot_info_tcp.py --ip 192.168.192.7 --raw
```

### 0b) Лидар / лазерный скан (подбор пресета под доку)

```bash
python3 lidar_tcp.py --ip 192.168.192.7 --preset net_getLaserScan
python3 lidar_tcp.py --ip 192.168.192.7 --preset net_getLaserScan --raw
```

Если ответ пустой или `40003` — откройте доку NetProtocol и задайте полное тело через `--request-json`.

### 0c) ResourceManager: скрипты и файлы (getScriptInfo / download / upload / genericScript)

Пути (`relative_path`, `file_path`) — как в доке робота (пример: `battery/standard/test.py` или `generic/...`).

```bash
# Список скриптов + MD5 (19204, 1999)
python3 resource_manager_tcp.py --ip 192.168.192.7 script-list

# Параметры (type=params)
python3 resource_manager_tcp.py --ip 192.168.192.7 script-params --path generic/battery/standard/test_config.json
python3 resource_manager_tcp.py --ip 192.168.192.7 script-params-reset --path generic/battery/standard/test_config.json

# Скачать скрипт (тело файла — в хвосте ответа; сохраняется в --out)
python3 resource_manager_tcp.py --ip 192.168.192.7 download --file-type script --path generic/battery/standard/test.py --out ./test.py

# Загрузить локальный файл (19205, 2999; тело в extra после JSON)
python3 resource_manager_tcp.py --ip 192.168.192.7 upload --file-type script --path generic/battery/standard/test.py --file ./test.py

python3 resource_manager_tcp.py --ip 192.168.192.7 remove --path generic/battery/standard/test.py
python3 resource_manager_tcp.py --ip 192.168.192.7 rename --from generic/battery/standard/test.py --to generic/battery/standard/test.bak.py

# Запуск / остановка уже лежащих на роботе скриптов (genericScript, порт 19205)
python3 resource_manager_tcp.py --ip 192.168.192.7 run --path battery/standard/test.py
python3 resource_manager_tcp.py --ip 192.168.192.7 stop --path battery/standard/test.py
python3 resource_manager_tcp.py --ip 192.168.192.7 kill --path battery/standard/test.py

# Залить локальный .py и сразу запустить
python3 resource_manager_tcp.py --ip 192.168.192.7 push-run --path battery/standard/my_task.py --file ./my_task.py
```

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
- `m` переключение режима `JOINT` / `CART`
- `1..7` выбор сустава
- `↑/↓` или `w/s` изменение угла (рад)
- `[` `]` шаг
- `t` переключение активной руки (если доступны обе)
- `r` синхронизация с текущими углами робота
- `q` выход

Локальная проверка self-collision теперь собрана в папке `self-collision-mesh`, поэтому достаточно:

```bash
python3 teleop_arm.py --ip 192.168.192.7 --self-collision-check
```

#### Self-Collision в `teleop_arm.py`

Проверка работает локально через `pybullet` перед отправкой шага в `ArmPlanning`:
- берётся геометрия и кинематика из `URDF`
- пары исключений берутся из `SRDF` (`disable_collisions`)
- используется набор ресурсов из `self-collision-mesh/`:
  - `robot_moveit_abs.urdf`
  - `wheeled_humanoid_v3_2.srdf`
  - `meshes/*.STL`

Если `pybullet` не установлен, включить проверку не получится:

```bash
source .venv/bin/activate
pip install pybullet
```

Что делает флаг `--self-collision-check`:
- перед каждым шагом в `JOINT` режиме локально выставляет текущую позу рук в PyBullet
- дополнительно подтягивает состояние талии с робота, чтобы корпус в локальной модели не оставался в нуле
- если находится пересечение или слишком близкая пара, команда не отправляется, а на экране показывается `blocked`

Полезные параметры:
- `--collision-distance 0.0` проверяет только проникновение; при значении больше нуля ловятся и близкие пары
- `--collision-ignore-pairs` добавляет свои пары `linkA:linkB`, которые нужно игнорировать поверх `SRDF`
- `--collision-srdf-include-default-arm` включает все `Default`-исключения из `SRDF` как в MoveIt; по умолчанию проверка строже для `arm* <-> base_link`

Ограничение: сейчас self-collision применяется только в режиме `JOINT`. В режиме `CART` декартовая команда отправляется без локальной collision-проверки.

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

### SSH на контроллер (если доступен)

Управление через **эти скрипты** идёт по **TCP API** (порты вроде `19204` / `19205`), это **не SSH**.

**SSH** — отдельный сервис на роботе (часто порт **22**), если его включили и выдали учётку:

1. ПК в одной сети с роботом, проверка: `nc -zv 192.168.192.7 22` или `ssh -v user@192.168.192.7`
2. Логин и пароль (или ключ) берутся **из документации / интегратора / поддержки SEER** — в публичной доке Robokit они обычно не «общие для всех».
3. На многих поставках SSH **закрыт** для заказчика; тогда работа только через Roboshop, RBK Studio и TCP API.

Точную процедуру «как включить SSH» нужно запрашивать у **SEER / поставщика** под вашу модель и прошивку.

## Важная инфа

- Команды для рук в API — в радианах.
- `MoveJ` управляет суставами, а не прямолинейным движением TCP.

## Примечания по API

- Для `ArmPlanning` ответы часто содержат route-echo в JSON-части, а полезный payload приходит в tail (`status`).
- `arm_planning_tcp.py` по умолчанию печатает уже полезный `status`; используйте `--raw` для полного ответа.
