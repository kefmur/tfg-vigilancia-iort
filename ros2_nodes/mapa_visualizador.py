#!/usr/bin/env python3
"""Renderiza el mapa OccupancyGrid + pose AMCL + waypoints como imagen
y la publica en /dashboard/mapa para consumo del dashboard via web_video_server."""

import os
import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import re

import paho.mqtt.client as mqtt
import json
import time as time_mod

RUTA_WAYPOINTS = os.path.expanduser("~/ros2_ws/maps/puntos_navegacion.txt")

PATRON_WP = re.compile(
    r'(?P<nombre>[^:]+):\s*'
    r'x=(?P<x>-?\d+\.?\d*),\s*'
    r'y=(?P<y>-?\d+\.?\d*),\s*'
    r'qz=(?P<qz>-?\d+\.?\d*),\s*'
    r'qw=(?P<qw>-?\d+\.?\d*)'
)


def cargar_waypoints(ruta):
    wps = {}
    if not os.path.isfile(ruta):
        return wps
    with open(ruta) as f:
        for linea in f:
            m = PATRON_WP.match(linea.strip())
            if m:
                nombre = m.group('nombre').strip()
                wps[nombre] = (float(m.group('x')), float(m.group('y')))
    return wps


class MapaVisualizador(Node):
    def __init__(self):
        super().__init__("mapa_visualizador")
        self.bridge = CvBridge()
        self.mapa = None
        self.resolution = 0.05
        self.origin = (0.0, 0.0)
        self.pose = None
        self.destino = None
        self.waypoints = cargar_waypoints(RUTA_WAYPOINTS)

        qos_map = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, "/map", self.cb_map, qos_map)
        self.create_subscription(PoseWithCovarianceStamped,
                                 "/amcl_pose", self.cb_pose, 10)

        self.pub = self.create_publisher(Image, "/dashboard/mapa", 10)
        self.create_timer(0.5, self.render)  # 2 Hz, suficiente
        self.get_logger().info("Mapa visualizador listo.")
        
        
        # Lista de detecciones recientes: cada item es dict con x, y, clase, ts
        self.detecciones = []
        self.DETECCION_TTL = 300.0  # segundos que se mantienen visibles las marcas
        
# Cliente MQTT para recibir las detecciones geolocalizadas
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_message = self._on_mqtt_message
        try:
            self.mqtt_client.connect('localhost', 1883, 60)
            self.mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().warn(f'MQTT no disponible: {e}')

    def cb_map(self, msg):
        h, w = msg.info.height, msg.info.width
        data = np.array(msg.data, dtype=np.int8).reshape((h, w))
        img = np.full((h, w, 3), 200, dtype=np.uint8)  # gris = desconocido
        img[data == 0] = (255, 255, 255)               # libre = blanco
        img[data >= 65] = (0, 0, 0)                    # ocupado = negro
        self.mapa = cv2.flip(img, 0)                   # ROS invierte Y
        self.resolution = msg.info.resolution
        self.origin = (msg.info.origin.position.x,
                       msg.info.origin.position.y)
        self.map_h = h

    def cb_pose(self, msg):
        p = msg.pose.pose
        yaw = 2.0 * math.atan2(p.orientation.z, p.orientation.w)
        self.pose = (p.position.x, p.position.y, yaw)

    def world_to_px(self, x, y):
        px = int((x - self.origin[0]) / self.resolution)
        py = self.map_h - int((y - self.origin[1]) / self.resolution)
        return px, py

    def render(self):
        if self.mapa is None: return
        img = self.mapa.copy()

        # Waypoints en azul (ya lo tenías)
        for nombre, (wx, wy) in self.waypoints.items():
            px, py = self.world_to_px(wx, wy)
            cv2.circle(img, (px, py), 6, (255, 0, 0), -1)
            cv2.putText(img, nombre, (px + 8, py - 8),cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)

    # --- NUEVO: detecciones en verde con la clase ---
        for det in self.detecciones:
            px, py = self.world_to_px(det['x'], det['y'])
            if det['contexto'] == 'alerta':
                # Cruz verde
                cv2.drawMarker(img, (px, py), (0, 200, 0),markerType=cv2.MARKER_TILTED_CROSS,markerSize=12, thickness=2)
                color_txt = (0,150,0)
                
            else:
                #Círculo amarillo más pequeño para detecciones oportunistas
                cv2.circle(img, (px,py), 5, (0, 200, 220), -1)
                cv2.circle(img,(px, py), 5, (0, 100, 110), 1)
                color_txt = (0,130,150)
            cv2.putText(img, det['clase'], (px + 6, py + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color_txt, 1)
    # --- fin nuevo ---

        # Pose del robot en rojo (ya lo tenías)
        if self.pose is not None:
            x, y, yaw = self.pose
            px, py = self.world_to_px(x, y)
            cv2.circle(img, (px, py), 8, (0, 0, 255), -1)
            tip = (int(px + 20 * math.cos(yaw)),int(py - 20 * math.sin(yaw)))
            cv2.arrowedLine(img, (px, py), tip, (0, 0, 255), 2, tipLength=0.4)

        # Resto igual...
        scale = 500.0 / img.shape[1]
        img = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
        ros_img = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        self.pub.publish(ros_img)


    def _on_mqtt_connect(self, client, userdata, flags, rc):
        client.subscribe('robot/deteccion')
        self.get_logger().info('Suscrito a robot/deteccion')

    def _on_mqtt_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode())
            self.detecciones.append({'x': data['x'],'y': data['y'],'clase': data.get('clase', '?'), 'contexto': data.get('contexto', 'alerta'),'ts': data.get('timestamp', time_mod.time())
        })
        # Limpiar detecciones antiguas
            ahora = time_mod.time()
            self.detecciones = [d for d in self.detecciones if ahora - d['ts'] < self.DETECCION_TTL]
        except Exception as e:
            self.get_logger().warn(f'Error parseando deteccion: {e}')
 
def main():
    rclpy.init()
    nodo = MapaVisualizador()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
