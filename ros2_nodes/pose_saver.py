import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import os

class PoseSaver(Node):
    def __init__(self):
        super().__init__('guardar_posicion_node')
        # Nos suscribimos al tópico de localización de Nav2
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.listener_callback,
            10)
        self.ultima_pose = None
        self.get_logger().info('Nodo iniciado. Mueve el robot y pulsa ENTER para guardar un punto.')

    def listener_callback(self, msg):
        # Guardamos la posición más reciente recibida
        self.ultima_pose = msg.pose.pose

    def guardar_en_archivo(self):
        if self.ultima_pose is None:
            print("Esperando a recibir la posición del robot (¿Nav2 está activo?)...")
            return

        nombre_punto = input("Introduce un nombre para este lugar (ej. cocina): ")
        
        # Formateamos los datos
        x = self.ultima_pose.position.x
        y = self.ultima_pose.position.y
        z = self.ultima_pose.orientation.z
        w = self.ultima_pose.orientation.w

        linea = f"{nombre_punto}: x={x:.3f}, y={y:.3f}, qz={z:.3f}, qw={w:.3f}\n"

        ruta = os.path.expanduser("~/ros2_ws/maps/puntos_navegacion.txt")
        with open(ruta, "a") as f:
            f.write(linea)
        
        print(f"✅ Guardado '{nombre_punto}' en puntos_navegacion.txt")

def main(args=None):
    rclpy.init(args=args)
    nodo = PoseSaver()

    # Hilo para capturar el teclado sin bloquear ROS
    import threading
    thread = threading.Thread(target=rclpy.spin, args=(nodo,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            input("\nPulsa ENTER para capturar la posición actual...")
            nodo.guardar_en_archivo()
    except KeyboardInterrupt:
        pass

    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
