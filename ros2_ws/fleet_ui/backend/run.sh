#!/bin/bash
# Rode a partir da raiz do workspace, com o ambiente ROS sourceado.
# Pré-requisito: sudo apt install python3.12-venv
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$SCRIPT_DIR/../../.." && pwd)"
cd "$WS"

source /opt/ros/jazzy/setup.bash 2>/dev/null || true
source "$WS/install/setup.bash" 2>/dev/null || true

# Instala fastapi/uvicorn no Python do sistema se ainda não existir
if ! python3 -c "import fastapi" 2>/dev/null; then
  echo "Instalando fastapi e uvicorn no Python do sistema..."
  pip3 install --user -q fastapi "uvicorn[standard]" websockets pyyaml
fi

exec python3 "$SCRIPT_DIR/main.py"
