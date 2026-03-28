#!/usr/bin/env bash
# Diagnóstico rápido: /odom, TF map->base_link, /cmd_vel.
# Uso: com Gazebo+Nav2+fleet já a correr, noutro terminal:
#   bash scripts/diagnose_nav_fleet.sh
# Opcional: DIAG_OUT=/tmp/meus_diag.log bash scripts/diagnose_nav_fleet.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
INSTALL_SETUP="${WS_ROOT}/install/setup.bash"
OUT="${DIAG_OUT:-${WS_ROOT}/diagnostics/diag_$(date +%Y%m%d_%H%M%S).log}"

mkdir -p "$(dirname "$OUT")"

# Após exec, toda a saída já vai para o ficheiro + terminal.
log() { echo "$@"; }
section() {
  log ""
  log "======== $1 ========"
}

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ERRO: /opt/ros/jazzy/setup.bash não encontrado." >&2
  exit 1
fi
# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash
if [[ -f "$INSTALL_SETUP" ]]; then
  # shellcheck source=/dev/null
  source "$INSTALL_SETUP"
else
  echo "AVISO: $INSTALL_SETUP não existe — rode colcon build no workspace." >&2
fi

exec > >(tee -a "$OUT") 2>&1
log "Relatório: $OUT"
log "Data: $(date -Iseconds)"
log "ROS_DISTRO=${ROS_DISTRO:-}"

section "Tópicos relevantes (grep)"
ros2 topic list 2>&1 | grep -E '^/(odom|cmd_vel|map|scan|imu)' || true
log "(lista completa abaixo, primeiras 60 linhas)"
ros2 topic list 2>&1 | head -60

section "A — /odom (uma mensagem, timeout 5s)"
if ros2 topic list 2>&1 | grep -qx '/odom'; then
  ros2 topic echo /odom --once --timeout 5 2>&1 || log "[A] falhou ou sem mensagens no tempo."
else
  log "[A] SKIP: tópico /odom não existe (simulação/Nav2 provavelmente parados)."
fi

section "B — TF map -> base_link (até 6s, primeira transformação válida)"
if ros2 topic list 2>&1 | grep -qx '/tf' || ros2 topic list 2>&1 | grep -qx '/tf_static'; then
  timeout 6 ros2 run tf2_ros tf2_echo map base_link 2>&1 | head -40 || log "[B] timeout ou frame map/base_link indisponível."
else
  log "[B] SKIP: sem /tf"
fi

section "C — /cmd_vel (uma mensagem, timeout 8s)"
if ros2 topic list 2>&1 | grep -qx '/cmd_vel'; then
  ros2 topic echo /cmd_vel --once --timeout 8 2>&1 || log "[C] sem cmd_vel no intervalo (Nav2 pode não estar a comandar)."
else
  log "[C] SKIP: tópico /cmd_vel não existe."
fi

section "Serviços fleet (amostra)"
ros2 service list 2>&1 | grep -E 'collection|go_to_point|play_route|fleet' | head -20 || true

section "Fim"
log "Gravado em: $OUT"
echo ""
echo "Concluído. Copie o ficheiro acima ou: cat \"$OUT\""
