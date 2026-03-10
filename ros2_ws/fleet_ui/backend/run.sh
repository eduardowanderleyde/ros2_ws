#!/bin/bash
# Rode a partir da raiz do workspace, com o ambiente ROS sourceado.
# Pré-requisito: sudo apt install python3.12-venv
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$SCRIPT_DIR/../../.." && pwd)"
cd "$WS"

if [ ! -d "$SCRIPT_DIR/venv" ] || ! "$SCRIPT_DIR/venv/bin/python" -c "import fastapi" 2>/dev/null; then
  echo "Criando/atualizando venv em $SCRIPT_DIR/venv ..."
  rm -rf "$SCRIPT_DIR/venv"
  python3 -m venv "$SCRIPT_DIR/venv" || { echo "Instale: sudo apt install python3.12-venv"; exit 1; }
  "$SCRIPT_DIR/venv/bin/pip" install -q fastapi "uvicorn[standard]" websockets pyyaml
fi

source /opt/ros/jazzy/setup.bash 2>/dev/null || true
source "$WS/install/setup.bash" 2>/dev/null || true

exec "$SCRIPT_DIR/venv/bin/python" "$SCRIPT_DIR/main.py"
