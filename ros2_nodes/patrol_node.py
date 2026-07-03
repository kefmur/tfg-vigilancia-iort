#!/usr/bin/env python3
"""
Nodo de patrullaje para TFG.
Lee waypoints desde un fichero de texto y los recorre en bucle usando Nav2.

Formato del fichero (una línea por waypoint):
    nombre: x=-7.480, y=7.488, qz=-0.475, qw=0.880
"""

import os
import re
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


# Ruta al fichero de waypoints (la misma que usa pose_saver)
RUTA_WAYPOINTS = os.path.expanduser("~/ros2_ws/maps/puntos_navegacion.txt")


def cargar_waypoints(ruta):
    """
    Lee el fichero de waypoints y devuelve una lista de tuplas:
    [(nombre, x, y, qz, qw), ...]
    """
    waypoints = []
    patron = re.compile(
        r'(?P<nombre>[^:]+):\s*'
        r'x=(?P<x>-?\d+\.?\d*),\s*'
        r'y=(?P<y>-?\d+\.?\d*),\s*'
        r'qz=(?P<qz>-?\d+\.?\d*),\s*'
        r'qw=(?P<qw>-?\d+\.?\d*)'
    )

    with open(ruta, 'r') as f:
        for linea in f:
            linea = linea.strip()
            if not linea:
                continue
            m = patron.match(linea)
            if m:
                nombre = m.group('nombre').strip()
                x = float(m.group('x'))
                y = float(m.group('y'))
                qz = float(m.group('qz'))
                qw = float(m.group('qw'))
                waypoints.append((nombre, x, y, qz, qw))
    return waypoints


def crear_pose(navigator, x, y, qz, qw):
    """Crea un PoseStamped a partir de coordenadas y orientación (quaternion)."""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


def main():
    rclpy.init()
    navigator = BasicNavigator()

    # ---- Cargar waypoints desde el fichero ----
    if not os.path.exists(RUTA_WAYPOINTS):
        navigator.get_logger().error(
            f'No se encuentra el fichero de waypoints: {RUTA_WAYPOINTS}'
        )
        rclpy.shutdown()
        return

    waypoints_data = cargar_waypoints(RUTA_WAYPOINTS)

    if not waypoints_data:
        navigator.get_logger().error('El fichero no contiene waypoints válidos.')
        rclpy.shutdown()
        return

    navigator.get_logger().info(f'Cargados {len(waypoints_data)} waypoints:')
    for nombre, x, y, qz, qw in waypoints_data:
        navigator.get_logger().info(f'  - {nombre}: ({x:.2f}, {y:.2f})')

    # Esperar a que Nav2 esté completamente activo
    navigator.waitUntilNav2Active()
    navigator.get_logger().info('Nav2 activo. Iniciando patrullaje...')

    # Convertir a PoseStamped
    waypoints = [
        crear_pose(navigator, x, y, qz, qw)
        for (nombre, x, y, qz, qw) in waypoints_data
    ]

    # ---- Bucle infinito de patrullaje ----
    ciclo = 0
    try:
        while rclpy.ok():
            ciclo += 1
            navigator.get_logger().info(f'--- Ciclo de patrullaje #{ciclo} ---')

            navigator.followWaypoints(waypoints)

            while not navigator.isTaskComplete():
                feedback = navigator.getFeedback()
                if feedback:
                    idx = feedback.current_waypoint
                    nombre = waypoints_data[idx][0]
                    navigator.get_logger().info(
                        f'Dirigiéndose al waypoint {idx + 1}/{len(waypoints)}: {nombre}'
                    )
                rclpy.spin_once(navigator, timeout_sec=1.0)

            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                navigator.get_logger().info('Ciclo completado correctamente.')
            elif result == TaskResult.CANCELED:
                navigator.get_logger().warn('Patrullaje cancelado.')
                break
            elif result == TaskResult.FAILED:
                navigator.get_logger().error('Patrullaje fallido. Reintentando...')

    except KeyboardInterrupt:
        navigator.get_logger().info('Interrumpido por usuario.')
    finally:
        navigator.lifecycleShutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
