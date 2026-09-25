#!/bin/bash
# Derruba tudo o que o start.sh sobe: sessao tmux, nos ROS e Gazebo.
#
# O gz sim costuma sobreviver ao fechamento do tmux: fica orfao (pai = PID 1) e
# segura GPU e memoria. Por isso, alem de fechar o tmux, cada processo e cacado
# pelo nome e recebe SIGINT, depois SIGTERM e, se ainda resistir, SIGKILL.

SOCKET=DEM_Simulation

# Padroes da linha de comando (pgrep -f).
PATTERNS=(
  'ros2 launch (limo_cerrado_sim|ground_segmentation_ros2|terrain_modelling)'
  '(^| )gz sim'                                       # servidor e GUI do Gazebo
  '/lib/ros_gz_bridge/parameter_bridge'
  '/lib/ros_gz_sim/create'
  '/lib/robot_state_publisher/robot_state_publisher'
  '/lib/limo_cerrado_sim/livox_mid360_emulator'
  '/lib/depth_image_proc/point_cloud_xyzrgb_node'
  '/lib/ground_segmentation_ros2/ground_segmentation_ros2_node'
  '/lib/terrain_modelling/terrain_modelling_node'
  '/lib/teleop_twist_keyboard/teleop_twist_keyboard'
  '/lib/rviz2/rviz2'
  '/lib/rmw_zenoh_cpp/rmw_zenohd'                     # roteador zenoh subido pelo launch
)

pids() {
  for p in "${PATTERNS[@]}"; do pgrep -f -- "$p"; done | sort -u | grep -vx "$$"
}

# espera ate 5 s os processos sairem
wait_gone() {
  for _ in $(seq 10); do
    [ -z "$(pids)" ] && return 0
    sleep 0.5
  done
  return 1
}

# 1. tmux: Ctrl+C em cada painel, para os launches encerrarem os nos filhos
if tmux -L "$SOCKET" has-session 2>/dev/null; then
  echo "Fechando a sessao tmux ($SOCKET)..."
  for pane in $(tmux -L "$SOCKET" list-panes -a -F '#{pane_id}'); do
    tmux -L "$SOCKET" send-keys -t "$pane" C-c
  done
  sleep 3
  tmux -L "$SOCKET" kill-server 2>/dev/null
fi

# 2. o que sobrou: SIGINT -> SIGTERM -> SIGKILL
for sig in INT TERM KILL; do
  list=$(pids)
  [ -z "$list" ] && break
  echo "SIG$sig em $(echo "$list" | wc -l) processo(s)"
  kill -"$sig" $list 2>/dev/null
  wait_gone && break
done

# 3. conferencia
left=$(pids)
if [ -n "$left" ]; then
  echo "AINDA VIVOS:"
  ps -o pid,etime,args -p $(echo $left | tr ' ' ',')
  exit 1
fi
echo "Tudo derrubado."
