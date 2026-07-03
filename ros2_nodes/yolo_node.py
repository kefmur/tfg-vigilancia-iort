#!/usr/bin/env python3
"""
yolo_node: procesa la camara del robot con YOLO.
Se suscribe a /oakd/rgb/preview/image_raw (camara del TurtleBot4).
Publica:
  - /yolo/detecciones      -> texto con clases detectadas (formato clase:conf:angulo,...)
  - /yolo/imagen_anotada   -> imagen con cajas dibujadas (para el dashboard)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO


TOPIC_CAMARA = '/oakd/rgb/preview/image_raw'
MODELO = 'yolov8n.pt'
UMBRAL_CONFIANZA = 0.5
PROCESAR_CADA = 5

# Parámetros de la cámara para el cálculo del ángulo
FOV_H = 1.204   # rad (69° horizontal de la OAK-D)


class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        self.get_logger().info('Cargando modelo YOLO...')
        self.model = YOLO(MODELO)
        self.bridge = CvBridge()
        self.contador = 0
        self.ultimas_detecciones = ''

        self.sub = self.create_subscription(
            Image, TOPIC_CAMARA, self.callback_imagen, 10)
        self.pub = self.create_publisher(String, '/yolo/detecciones', 10)
        self.pub_imagen = self.create_publisher(Image, '/yolo/imagen_anotada', 10)
        self.get_logger().info(f'YOLO listo. Suscrito a {TOPIC_CAMARA}')

    def callback_imagen(self, msg):
        self.contador += 1
        if self.contador % PROCESAR_CADA != 0:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error al convertir imagen: {e}')
            return

        resultados = self.model(frame, conf=UMBRAL_CONFIANZA, device=0, verbose=False)

        # Ancho real del frame procesado
        ancho_img = frame.shape[1]

        # Construir el string de detecciones con ángulo
        detecciones_str = ""
        for result in resultados:
            for box in result.boxes:
                clase = result.names[int(box.cls)]
                conf = float(box.conf)
                xyxy = box.xyxy[0].tolist()                  # [x1, y1, x2, y2]
                cx_pixel = (xyxy[0] + xyxy[2]) / 2.0
                # Pixel a ángulo: en imagen x+ es derecha, en ROS yaw+ es izquierda
                angulo = -((cx_pixel - ancho_img / 2.0) / ancho_img * FOV_H)
                detecciones_str += f"{clase}:{conf:.2f}:{angulo:.3f},"
        detecciones_str = detecciones_str.rstrip(",")

        self.ultimas_detecciones = detecciones_str

        # Publicar texto
        msg_out = String()
        msg_out.data = detecciones_str if detecciones_str else 'nada'
        self.pub.publish(msg_out)

        # Publicar imagen anotada
        try:
            frame_anotado = resultados[0].plot()
            msg_img = self.bridge.cv2_to_imgmsg(frame_anotado, encoding='bgr8')
            msg_img.header = msg.header
            self.pub_imagen.publish(msg_img)
        except Exception as e:
            self.get_logger().warn(f'No pude publicar imagen anotada: {e}')

        if detecciones_str:
            self.get_logger().info(f'Detectado: {detecciones_str}')


def main():
    rclpy.init()
    nodo = YoloNode()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
