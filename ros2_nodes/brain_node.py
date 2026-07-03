#!/usr/bin/env python3
"""
brain_node: maquina de estados con prioridades, cola, fusion sensorial,
cooldown por zona, inspeccion activa 360 y patrullaje adaptativo.

Patrullaje adaptativo: tras completar una vuelta, las zonas se reordenan
segun el numero de alertas CONFIRMADAS recibidas en la ventana temporal
ADAPT_VENTANA.

Geolocalización: fusión visión + LiDAR. El ángulo viene del centro del
bbox de YOLO, la distancia se obtiene del LaserScan en ese ángulo.
"""

import os
import re
import math
import json
import time as time_mod
from enum import Enum
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import paho.mqtt.client as mqtt


RUTA_WAYPOINTS = os.path.expanduser("~/ros2_ws/maps/puntos_navegacion.txt")
#RUTA_WAYPOINTS = os.path.expanduser("~/ros2_ws/maps/puntos_navegacion_warehouse.txt")

TIEMPO_INSPECCION = 13.0
VELOCIDAD_ROTACION = 0.5
COOLDOWN_ZONA = 300.0

# --- Patrullaje adaptativo ---
ADAPT_VENTANA = 600.0   # ventana temporal de "actividad reciente" (segundos)
ADAPT_ACTIVO = True     # poner a False para desactivar el patrullaje adaptativo

# Intervalo mínimo entre detecciones oportunistas publicadas
DETECCION_PATRULLA_INTERVALO = 10.0

MQTT_BROKER = 'localhost'
MQTT_PORT = 1883

CLASES_RELEVANTES = ['person']

GRAVEDAD = {
    'flame': 3, 'fuego': 3, 'gas': 3, 'humo': 3,
    'pir': 2, 'intrusion': 2,
    'temperatura': 1, 'sonido': 1, 'vibracion': 1,
}
GRAVEDAD_DEFECTO = 1


class Estado(Enum):
    PATRULLANDO = 1
    INVESTIGANDO = 2
    INSPECCIONANDO = 3
    VOLVIENDO = 4


def cargar_waypoints(ruta):
    waypoints = {}
    patron = re.compile(
        r'(?P<nombre>[^:]+):\s*'
        r'x=(?P<x>-?\d+\.?\d*),\s*'
        r'y=(?P<y>-?\d+\.?\d*),\s*'
        r'qz=(?P<qz>-?\d+\.?\d*),\s*'
        r'qw=(?P<qw>-?\d+\.?\d*)'
    )
    with open(ruta, 'r') as f:
        for linea in f:
            m = patron.match(linea.strip())
            if m:
                nombre = m.group('nombre').strip()
                waypoints[nombre] = (
                    float(m.group('x')), float(m.group('y')),
                    float(m.group('qz')), float(m.group('qw'))
                )
    return waypoints


class BrainNode(Node):
    def __init__(self):
        super().__init__('brain_node')

        self.waypoints = cargar_waypoints(RUTA_WAYPOINTS)
        self.nombres_wp = list(self.waypoints.keys())
        self.orden_original = list(self.nombres_wp)
        self.get_logger().info(f'Cargados {len(self.waypoints)} waypoints: {self.nombres_wp}')

        self.navigator = BasicNavigator()

        # Estado de la máquina
        self.estado = Estado.PATRULLANDO
        self.indice_patrulla = 0
        self.zona_evento = None
        self.tipo_evento = None
        self.gravedad_actual = 0
        self.tiempo_inicio_inspeccion = None
        self.cola = []
        self.zonas_cooldown = {}

        # Pose AMCL y scan LiDAR
        self.pose_actual = None     # (x, y, yaw)
        self.ultimo_scan = None

        # Detecciones
        self.ultima_deteccion = 'nada'
        self.ultima_deteccion_publicada_ts = 0.0
        self.ultima_deteccion_evaluada = ''
        self._ultima_pose_pub = None

        # Historial para el patrullaje adaptativo
        self.historial_confirmadas = deque(maxlen=500)

        # Suscripciones
        self.create_subscription(String, '/eventos_iot', self.callback_evento, 10)
        self.create_subscription(String, '/yolo/detecciones', self.callback_yolo, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.callback_pose, 10)
        self.create_subscription(LaserScan, '/scan', self.callback_scan, 10)

        # Publicadores
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        # MQTT
        self.mqtt_client = mqtt.Client()
        try:
            self.mqtt_client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
            self.mqtt_client.loop_start()
            self.get_logger().info('Conectado a MQTT para publicar alertas.')
        except Exception as e:
            self.get_logger().warn(f'No pude conectar a MQTT: {e}')

        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 activo. Iniciando comportamiento...')

        self.timer = self.create_timer(0.5, self.bucle_principal)
        self.ir_a_waypoint(self.nombres_wp[self.indice_patrulla])

    # ---------- Callbacks de subscripción ----------

    def callback_pose(self, msg):
        p = msg.pose.pose
        yaw = 2.0 * math.atan2(p.orientation.z, p.orientation.w)
        self.pose_actual = (p.position.x, p.position.y, yaw)

    def callback_scan(self, msg):
        self.ultimo_scan = msg

    def callback_yolo(self, msg):
        self.ultima_deteccion = msg.data

    def callback_evento(self, msg):
        partes = msg.data.split(':')
        if len(partes) < 2:
            return
        zona = partes[0]
        tipo = partes[1]

        if zona not in self.waypoints:
            self.get_logger().warn(f'Zona "{zona}" no es waypoint. Ignorado.')
            return

        if self.zona_en_cooldown(zona):
            self.get_logger().info(f'Zona "{zona}" en cooldown. Evento ignorado.')
            return

        g = self.gravedad_de(tipo)
        self.get_logger().warn(f'>>> EVENTO: zona={zona}, tipo={tipo}, gravedad={g}')

        if self.estado == Estado.PATRULLANDO:
            self.atender_evento(zona, tipo, g)
            return

        if g > self.gravedad_actual:
            self.get_logger().warn(f'Mas grave ({g}>{self.gravedad_actual}). INTERRUMPO.')
            if self.zona_evento is not None:
                self.cola.append((self.gravedad_actual, self.zona_evento, self.tipo_evento))
            self.navigator.cancelTask()
            self.atender_evento(zona, tipo, g)
        else:
            self.get_logger().info(f'Gravedad {g} encolada.')
            self.cola.append((g, zona, tipo))

    # ---------- Utilidades ----------

    def crear_pose(self, nombre):
        x, y, qz, qw = self.waypoints[nombre]
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def ir_a_waypoint(self, nombre):
        if nombre not in self.waypoints:
            self.get_logger().error(f'Waypoint desconocido: {nombre}')
            return
        self.get_logger().info(f'Navegando a: {nombre}')
        try:
            self.mqtt_client.publish('robot/destino', nombre)
        except Exception:
            pass
        self.navigator.goToPose(self.crear_pose(nombre))

    def gravedad_de(self, tipo):
        return GRAVEDAD.get(tipo, GRAVEDAD_DEFECTO)

    def hay_deteccion_relevante(self):
        for clase in CLASES_RELEVANTES:
            if clase in self.ultima_deteccion:
                return True
        return False

    def zona_en_cooldown(self, zona):
        if zona not in self.zonas_cooldown:
            return False
        ahora = self.get_clock().now()
        return ahora < self.zonas_cooldown[zona]

    def marcar_cooldown_zona(self, zona):
        fin = self.get_clock().now() + Duration(seconds=COOLDOWN_ZONA)
        self.zonas_cooldown[zona] = fin
        self.get_logger().info(f'Zona "{zona}" en cooldown durante {COOLDOWN_ZONA:.0f}s')

    def rotar(self):
        twist = Twist()
        twist.angular.z = VELOCIDAD_ROTACION
        self.pub_cmd_vel.publish(twist)

    def detener(self):
        twist = Twist()
        self.pub_cmd_vel.publish(twist)

    def siguiente_de_cola(self):
        if not self.cola:
            return None
        self.cola.sort(key=lambda e: e[0], reverse=True)
        return self.cola.pop(0)

    # ---------- Geolocalización con fusión visión + LiDAR ----------

    def distancia_en_angulo(self, angulo_rel):
        """
        Devuelve la distancia del LiDAR en el ángulo relativo al frente del robot.
        Si no hay scan o la lectura es inválida, devuelve 1.5 como fallback.
        """
        if self.ultimo_scan is None:
            return 1.5

        s = self.ultimo_scan
        idx = int((angulo_rel - s.angle_min) / s.angle_increment)
        if idx < 0 or idx >= len(s.ranges):
            return 1.5

        d = s.ranges[idx]
        if math.isnan(d) or math.isinf(d):
            return 1.5
        if d < s.range_min or d > s.range_max:
            return 1.5

        return max(0.3, d - 0.3)

    def publicar_deteccion_geolocalizada(self, deteccion_str, contexto='alerta'):
        """
        Publica la posición global estimada de un objeto detectado.
        Usa fusión visión+LiDAR: ángulo desde YOLO, distancia desde el scan.
        """
        if self.pose_actual is None:
            self.get_logger().warn('No tengo pose AMCL, no publico geolocalización.')
            return

        # Parsear primera detección: "clase:conf:angulo,..."
        primera = deteccion_str.split(',')[0]
        partes = primera.split(':')
        clase = partes[0]

        if len(partes) >= 3:
            try:
                angulo = float(partes[2])
            except ValueError:
                angulo = 0.0
        else:
            angulo = 0.0

        distancia = self.distancia_en_angulo(angulo)

        x_r, y_r, yaw = self.pose_actual
        x_obj = x_r + distancia * math.cos(yaw + angulo)
        y_obj = y_r + distancia * math.sin(yaw + angulo)

        payload = json.dumps({
            'x': round(x_obj, 2),
            'y': round(y_obj, 2),
            'clase': clase,
            'zona': self.zona_evento if self.zona_evento else 'patrulla',
            'contexto': contexto,
            'distancia': round(distancia, 2),
            'angulo_rad': round(angulo, 3),
            'timestamp': time_mod.time(),
            'detalle': deteccion_str
        })

        try:
            self.mqtt_client.publish('robot/deteccion', payload, retain=False)
            self.get_logger().info(
                f'[GEO/{contexto}] {clase} en ({x_obj:.2f}, {y_obj:.2f}) '
                f'| dist={distancia:.2f}m, ang={math.degrees(angulo):+.1f}°'
            )
        except Exception as e:
            self.get_logger().warn(f'No pude publicar deteccion: {e}')

    def chequeo_deteccion_oportunista(self):
        """
        Si durante cualquier estado YOLO detecta una persona, marcarla en el mapa.
        Filtra por intervalo temporal y por distancia mínima desde la última.
        """
        if not self.hay_deteccion_relevante():
            return

        ahora = time_mod.time()
        if ahora - self.ultima_deteccion_publicada_ts < DETECCION_PATRULLA_INTERVALO:
            return

        if self.ultima_deteccion == self.ultima_deteccion_evaluada:
            return

        # Filtro de movimiento mínimo: 1 m desde la última publicada
        if self._ultima_pose_pub is not None and self.pose_actual is not None:
            dx = self.pose_actual[0] - self._ultima_pose_pub[0]
            dy = self.pose_actual[1] - self._ultima_pose_pub[1]
            if math.sqrt(dx * dx + dy * dy) < 1.0:
                return

        self.publicar_deteccion_geolocalizada(self.ultima_deteccion, contexto='patrulla')
        self.ultima_deteccion_publicada_ts = ahora
        self.ultima_deteccion_evaluada = self.ultima_deteccion
        self._ultima_pose_pub = self.pose_actual

    # ---------- Patrullaje adaptativo ----------

    def reordenar_patrulla_por_actividad(self):
        if not ADAPT_ACTIVO:
            return

        ahora_s = self.get_clock().now().nanoseconds / 1e9
        actividad = {nombre: 0 for nombre in self.orden_original}
        for ts, zona in self.historial_confirmadas:
            if ahora_s - ts <= ADAPT_VENTANA and zona in actividad:
                actividad[zona] += 1

        if all(v == 0 for v in actividad.values()):
            self.nombres_wp = list(self.orden_original)
            self.get_logger().info('Patrullaje adaptativo: sin actividad reciente, orden por defecto.')
            return

        def clave(nombre):
            return (-actividad[nombre], self.orden_original.index(nombre))

        self.nombres_wp = sorted(self.orden_original, key=clave)
        resumen = ', '.join(f'{z}={actividad[z]}' for z in self.nombres_wp)
        self.get_logger().info(f'Patrullaje adaptativo: actividad reciente -> {resumen}')
        self.get_logger().info(f'Nueva orden de patrullaje: {self.nombres_wp}')

    # ---------- Publicaciones MQTT ----------

    def publicar_alerta(self, zona, tipo, confirmada, detalle):
        estado_txt = 'CONFIRMADA' if confirmada else 'FALSO_POSITIVO'
        payload = f'{zona}:{tipo}:{estado_txt}:{detalle}'
        try:
            self.mqtt_client.publish(f'events/alert/{zona}', payload)
        except Exception as e:
            self.get_logger().warn(f'No pude publicar alerta MQTT: {e}')
        self.get_logger().warn(f'>>> ALERTA: {payload}')

        if confirmada:
            ahora_s = self.get_clock().now().nanoseconds / 1e9
            self.historial_confirmadas.append((ahora_s, zona))

    def publicar_estado(self):
        try:
            self.mqtt_client.publish('robot/estado', self.estado.name)
        except Exception:
            pass

    # ---------- Máquina de estados ----------

    def atender_evento(self, zona, tipo, gravedad):
        self.zona_evento = zona
        self.tipo_evento = tipo
        self.gravedad_actual = gravedad
        self.estado = Estado.INVESTIGANDO
        self.get_logger().info(f'Estado -> INVESTIGANDO (zona {zona}, gravedad {gravedad})')
        self.ir_a_waypoint(zona)

    def bucle_principal(self):
        self.publicar_estado()
        self.chequeo_deteccion_oportunista()
        if self.estado == Estado.PATRULLANDO:
            self.logica_patrullando()
        elif self.estado == Estado.INVESTIGANDO:
            self.logica_investigando()
        elif self.estado == Estado.INSPECCIONANDO:
            self.logica_inspeccionando()
        elif self.estado == Estado.VOLVIENDO:
            self.logica_volviendo()

    def logica_patrullando(self):
        if self.navigator.isTaskComplete():
            self.indice_patrulla += 1
            if self.indice_patrulla >= len(self.nombres_wp):
                self.get_logger().info('Vuelta de patrullaje completada.')
                self.reordenar_patrulla_por_actividad()
                self.indice_patrulla = 0
            self.ir_a_waypoint(self.nombres_wp[self.indice_patrulla])

    def logica_investigando(self):
        if self.navigator.isTaskComplete():
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info('Llegue a la zona. INSPECCIONANDO...')
                self.estado = Estado.INSPECCIONANDO
                self.tiempo_inicio_inspeccion = self.get_clock().now()
            else:
                self.get_logger().warn('No pude llegar. VOLVIENDO.')
                self.estado = Estado.VOLVIENDO

    def logica_inspeccionando(self):
        transcurrido = (self.get_clock().now() - self.tiempo_inicio_inspeccion).nanoseconds / 1e9

        if transcurrido < TIEMPO_INSPECCION:
            self.rotar()
            if self.hay_deteccion_relevante():
                self.get_logger().info(f'Deteccion durante rotacion: {self.ultima_deteccion}')
                self.detener()
                self.publicar_deteccion_geolocalizada(self.ultima_deteccion, contexto='alerta')
                self.publicar_alerta(self.zona_evento, self.tipo_evento, True, self.ultima_deteccion)
                self.marcar_cooldown_zona(self.zona_evento)
                self.estado = Estado.VOLVIENDO
            return

        self.detener()
        self.publicar_alerta(self.zona_evento, self.tipo_evento, False, 'sin deteccion visual')
        self.marcar_cooldown_zona(self.zona_evento)
        self.estado = Estado.VOLVIENDO

    def logica_volviendo(self):
        siguiente = self.siguiente_de_cola()
        if siguiente is not None:
            g, zona, tipo = siguiente
            if self.zona_en_cooldown(zona):
                self.get_logger().info(f'Cola: descarto {zona} (en cooldown).')
                self.logica_volviendo()
                return
            self.get_logger().info(f'Pendiente en cola: {zona} (gravedad {g}).')
            self.atender_evento(zona, tipo, g)
        else:
            self.get_logger().info('Cola vacia. Estado -> PATRULLANDO')
            self.estado = Estado.PATRULLANDO
            self.zona_evento = None
            self.tipo_evento = None
            self.gravedad_actual = 0
            self.ir_a_waypoint(self.nombres_wp[self.indice_patrulla])


def main():
    rclpy.init()
    nodo = BrainNode()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.navigator.lifecycleShutdown()
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
