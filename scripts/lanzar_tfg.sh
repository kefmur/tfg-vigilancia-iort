#!/bin/bash
# ============================================================
#  lanzar_tfg.sh - Lanzamiento del sistema base del TFG
#  Jeffer Espinosa Murillo - Vigilancia IoRT
#  NOTA: el brain_node se lanza APARTE, a mano (ver README).
# ============================================================

# --- Configuración: ajusta estas rutas si cambian ---
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$HOME/ros2_ws/install/setup.bash"
MAPA="$HOME/ros2_ws/maps/maze_map.yaml"

SRC="source $ROS_SETUP && source $WS_SETUP"

echo "==================================================="
echo "  Lanzando sistema base TFG - Vigilancia IoRT"
echo "==================================================="
echo "  El brain_node se lanza APARTE una vez todo este"
echo "  arriba y hayas hecho el '2D Pose Estimate' en RViz."
echo "==================================================="

# --- Terminal 1: Simulacion + Nav2 + AMCL + RViz ---
gnome-terminal --tab --title="1-SIM+Nav2" -- bash -c \
"$SRC; ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py \
world:=maze localization:=true nav2:=true rviz:=true slam:=false \
map:=$MAPA; exec bash"

echo "Esperando 20s a que arranque la simulacion..."
sleep 20

# --- Terminal 2: Puente MQTT <-> ROS2 ---
gnome-terminal --tab --title="2-MQTT" -- bash -c \
"$SRC; ros2 run tfg_patrol mqtt_bridge; exec bash"
sleep 2

# --- Terminal 3: Nodo de vision YOLO ---
gnome-terminal --tab --title="3-YOLO" -- bash -c \
"$SRC; ros2 run tfg_patrol yolo_node; exec bash"
sleep 2

# --- Terminal 4: Visualizador del mapa ---
gnome-terminal --tab --title="4-MAPA" -- bash -c \
"$SRC; ros2 run tfg_patrol mapa_visualizador; exec bash"
sleep 2

# --- Terminal 5: Servidor de video MJPEG ---
gnome-terminal --tab --title="5-VIDEO" -- bash -c \
"$SRC; ros2 run web_video_server web_video_server; exec bash"
sleep 2

# --- Terminal 6: Node-RED (si no esta como servicio) ---
gnome-terminal --tab --title="6-NODERED" -- bash -c \
"node-red; exec bash"

echo "==================================================="
echo "  Sistema base lanzado."
echo "  AHORA:"
echo "   1. En RViz: '2D Pose Estimate' para situar el robot."
echo "   2. Lanza el brain_node en una terminal nueva:"
echo "      source $ROS_SETUP && source $WS_SETUP"
echo "      ros2 run tfg_patrol brain_node 2>&1 | tee ~/brain_log.txt"
echo "  Dashboard: http://localhost:1880/dashboard/page1"
echo "==================================================="
