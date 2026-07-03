#!/usr/bin/env python3
"""
Puente MQTT -> ROS2.
Escucha eventos de sensores IoT por MQTT y los republica en un topic ROS2
(/eventos_iot) para que el brain_node los procese.

Formato del topic MQTT esperado: sensors/<zona>/<tipo>
  ejemplo: sensors/inicio/pir
El mensaje publicado en ROS2 es un String con formato "zona:tipo:payload"
  ejemplo: "inicio:pir:1"
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import paho.mqtt.client as mqtt


MQTT_BROKER = 'localhost'
MQTT_PORT = 1883
MQTT_TOPIC = 'sensors/#'


class MqttBridge(Node):
    def __init__(self):
        super().__init__('mqtt_bridge')

        # Publisher ROS2: aquí enviamos los eventos traducidos
        self.publisher = self.create_publisher(String, '/eventos_iot', 10)

        # Cliente MQTT
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        self.get_logger().info(f'Conectando a broker MQTT en {MQTT_BROKER}:{MQTT_PORT}...')
        self.mqtt_client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
        self.mqtt_client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info('Conectado al broker MQTT correctamente.')
            client.subscribe(MQTT_TOPIC)
            self.get_logger().info(f'Suscrito a: {MQTT_TOPIC}')
        else:
            self.get_logger().error(f'Fallo de conexión MQTT, código: {rc}')

    def on_message(self, client, userdata, msg):
        payload = msg.payload.decode('utf-8', errors='replace')
        topic = msg.topic
        self.get_logger().info(f'[MQTT] Topic: {topic} | Mensaje: {payload}')

        # Parsear topic: sensors/<zona>/<tipo>
        partes = topic.split('/')
        if len(partes) >= 3 and partes[0] == 'sensors':
            zona = partes[1]
            tipo = partes[2]
            # Construir mensaje ROS2: "zona:tipo:payload"
            evento = f'{zona}:{tipo}:{payload}'

            ros_msg = String()
            ros_msg.data = evento
            self.publisher.publish(ros_msg)
            self.get_logger().info(f'[ROS2] Publicado en /eventos_iot: {evento}')
        else:
            self.get_logger().warn(f'Topic con formato inesperado, ignorado: {topic}')

    def destroy_node(self):
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        super().destroy_node()


def main():
    rclpy.init()
    nodo = MqttBridge()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
