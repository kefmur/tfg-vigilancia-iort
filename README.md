===========================================================
  TFG - Robotica movil e IoT para vigilancia inteligente
  Jefferson Espinosa Murillo (jem38@alu.ua.es)
  Universidad de Alicante - Grupo UniCAD
  Tutor: Francisco Antonio Pujol Lopez
===========================================================

Este documento explica como arrancar el sistema completo en
la Jetson Orin Nano Super y como esta organizado el proyecto.

-----------------------------------------------------------
0. RESUMEN DEL SISTEMA
-----------------------------------------------------------
El sistema integra tres capas comunicadas por MQTT:
  - Capa robotica : TurtleBot4 simulado (Ignition Fortress)
                    + ROS2 Humble + Nav2 + AMCL + YOLOv8.
  - Capa IoT      : 3 nodos ESP32 con sensores fisicos.
  - Capa supervis.: dashboard Node-RED con video en vivo.

Plataforma: NVIDIA Jetson Orin Nano Super, JetPack 6.2.1
(Ubuntu 22.04, CUDA 12.6), modo de potencia MAXN_SUPER.


-----------------------------------------------------------
1. REQUISITOS ANTES DE ARRANCAR
-----------------------------------------------------------
  - Broker MQTT Mosquitto activo (servicio systemd):
        sudo systemctl status mosquitto
    Si no esta activo:
        sudo systemctl start mosquitto

  - (Opcional) Modo de maximo rendimiento de la Jetson:
        sudo nvpmodel -m 0
        sudo jetson_clocks

  - Los 3 nodos ESP32 encendidos y conectados a la misma
    red WiFi que la Jetson (solo si se prueban los sensores
    fisicos reales; en simulacion no hacen falta).


-----------------------------------------------------------
2. ARRANQUE DEL SISTEMA BASE (script)
-----------------------------------------------------------
Ejecutar el script de lanzamiento:

        ~/lanzar_tfg.sh

Esto abre 6 pestanas de terminal en este orden:
   1. Simulacion + Nav2 + AMCL + RViz
   2. Puente MQTT <-> ROS2 (mqtt_bridge)
   3. Nodo de vision YOLO (yolo_node)
   4. Visualizador del mapa (mapa_visualizador)
   5. Servidor de video MJPEG (web_video_server)
   6. Node-RED (dashboard)

(Si Node-RED ya corre como servicio systemd, ignora la
 pestana 6 o quitala del script.)


-----------------------------------------------------------
3. PASO CRITICO: INICIALIZAR AMCL
-----------------------------------------------------------
Cuando RViz haya cargado el mapa:

   -> Pulsar el boton "2D Pose Estimate" en la barra de RViz.
   -> Hacer clic sobre el mapa en la posicion aproximada del
      robot y arrastrar en la direccion a la que mira.

Sin este paso, el frame "map" no se publica y Nav2 NO
funcionara. Es el error mas comun al arrancar.


-----------------------------------------------------------
4. LANZAR EL BRAIN_NODE (APARTE, a mano)
-----------------------------------------------------------
Una vez todo lo anterior esta arriba y AMCL inicializado,
abrir una terminal NUEVA y ejecutar:

    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    ros2 run tfg_patrol brain_node 2>&1 | tee ~/brain_log.txt

El "tee" guarda el log en ~/brain_log.txt (util para revisar
la secuencia de eventos, preemption, etc. tras la demo).

El robot deberia empezar a patrullar los waypoints.


-----------------------------------------------------------
5. ACCESO AL DASHBOARD
-----------------------------------------------------------
En el navegador de la Jetson:

    http://localhost:1880/dashboard/page1

Desde otro dispositivo de la misma red (si Node-RED esta con
uiHost: 0.0.0.0):

    http://<IP_de_la_Jetson>:1880/dashboard/page1


-----------------------------------------------------------
6. PROBAR SIN SENSORES FISICOS (eventos sinteticos)
-----------------------------------------------------------
Para disparar un evento a mano y ver reaccionar al robot,
en una terminal:

    # Evento PIR en la zona de entrada
    mosquitto_pub -h localhost -t "sensors/inicio/pir" -m "1"

    # Evento de fuego en la zona centro (gravedad 3)
    mosquitto_pub -h localhost -t "sensors/centro/fuego" -m "1"

Para ver todos los eventos que llegan al broker:

    mosquitto_sub -h localhost -t "sensors/#" -v


-----------------------------------------------------------
7. PARAMETROS CONFIGURABLES (brain_node.py)
-----------------------------------------------------------
Se editan en la cabecera de brain_node.py y luego recompilar:
    colcon build --packages-select tfg_patrol

  TIEMPO_INSPECCION            13.0 s   (barrido 360)
  VELOCIDAD_ROTACION           0.5 rad/s
  COOLDOWN_ZONA                120.0 s
  ADAPT_VENTANA                600.0 s
  ADAPT_ACTIVO                 True     (False en validacion)
  DETECCION_PATRULLA_INTERVALO 10.0 s
  CLASES_RELEVANTES            ['person']

NOTA: durante las pruebas de validacion se uso
ADAPT_ACTIVO = False para un patrullaje determinista.


-----------------------------------------------------------
8. ESTRUCTURA DEL PROYECTO
-----------------------------------------------------------
  ~/ros2_ws/src/tfg_patrol/   Paquete ROS2 (nodos Python)
  ~/ros2_ws/maps/             Mapas (maze_map.yaml/.pgm)
  ~/.node-red/flows.json      Flujo del dashboard Node-RED
  ~/firmware_esp32/           Firmware de los 3 nodos ESP32
  ~/lanzar_tfg.sh             Script de arranque del sistema
  ~/brain_log.txt             Log generado por el brain_node

  (Ajusta las rutas segun tu organizacion real.)


-----------------------------------------------------------
9. APAGADO ORDENADO
-----------------------------------------------------------
  - Ctrl+C en la terminal del brain_node primero.
  - Ctrl+C en el resto de terminales (o cerrarlas).
  - La simulacion (pestana 1) puede tardar un poco en cerrar.


-----------------------------------------------------------
10. NOTA SOBRE EL DESPLIEGUE EN ROBOT FISICO
-----------------------------------------------------------
El sistema se valido en simulacion. El despliegue sobre el
TurtleBot4 fisico quedo limitado por la red WiFi del
laboratorio (Nav2 requiere baja latencia / 5 GHz; la red
disponible degradaba el topic /scan). Ver Cap. 5 (Caso 8) y
las conclusiones de la memoria. Solucion futura: conectar la
Raspberry Pi del robot a una red de 5 GHz dedicada o usar
conexion Ethernet / computo a bordo.

===========================================================
