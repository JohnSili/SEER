#!/usr/bin/env bash
# Создать .venv, установить зависимости, сгенерировать messageV4_camera_pb2.py
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

if [[ ! -d .venv ]]; then
  python3 -m venv .venv
fi
# shellcheck disable=SC1091
source .venv/bin/activate
python -m pip install -U pip
python -m pip install -r requirements.txt

python -m grpc_tools.protoc \
  -I=proto \
  --python_out=proto \
  proto/messageV4_camera.proto

echo "OK: .venv готов, сгенерирован proto/messageV4_camera_pb2.py"
echo "Активация: source .venv/bin/activate"
