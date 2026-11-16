"""My CONTROL node.   sb-launch control:=telma_pipeline/control

  in : /sandbox/path      nav_msgs/Path (map)
       /sandbox/pose      Odometry (map -> base_link), twist.linear.x = speed
  out: /control_command   fs_msgs/ControlCommand: throttle [0,1], brake [0,1], steering [-1,1] (+ = RIGHT)

Everything inside compute() is yours. Wheelbase 1.627 m, max steer 22.4 deg.
"""
#Se genera un PNG por ejecución y muestra dos medidas para comparar las ganancias:
#RMSE de velocidad y RMS a la trayectoria
import numpy as np
import rclpy
from fs_msgs.msg import ControlCommand
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node

from sandbox_tools import contract
from sandbox_tools.node_utils import run
from sandbox_tools.geometry import Pose2D, world_to_body

#Imports para generar los gráficos
import csv
from datetime import datetime
from pathlib import Path as FilePath
import time

#Parámetros 
KP_STEERING = 0.4
KP_SPEED = 0.3
TARGET_SPEED = 2.0

#GRÁFICOS: Función para calcular la distancia entre el coche y el punto más cercano de la trayectoria
#Es el error que quiero representar para ver si el coche sigue bien la trayectoria
def distance_to_path(path_body):
    """Distancia del coche a la trayectoria, en metros."""
    if len(path_body) == 1:
        return float(np.linalg.norm(path_body[0]))

    starts = path_body[:-1]
    segments = path_body[1:] - starts
    lengths_squared = np.sum(segments**2, axis=1)

    # Proyección del origen (el coche) sobre cada segmento.
    projection = np.divide(
        -np.sum(starts * segments, axis=1),
        lengths_squared,
        out=np.zeros_like(lengths_squared),
        where=lengths_squared > 1e-12,
    )
    projection = np.clip(projection, 0.0, 1.0)

    closest_points = starts + projection[:, None] * segments

    return float(np.min(np.linalg.norm(closest_points, axis=1)))

'''
Cómo lo teníamos antes:
return 0.0, 0.0, 1.0

'''
def compute(path_body: np.ndarray, speed: float):
    #De los puntos que ve delante el coche, cogemos uno al que dirigirse
    ahead = path_body[path_body[:, 0] > 0.0]
    if len(ahead) == 0:
        return 0.0, 0.0, 1.0

    
    #Coordenadas: x cuánto por delante, y cuánto a la izquierda
    distances = np.linalg.norm(ahead, axis=1)
    #Elige el punto cuya distancia al coche sea más cercana a 3 m
    target = ahead[np.argmin(np.abs(distances - 3.0))]

    '''
    Control P velocidad:
    giro = 0.3*error lateral
    -Punto 1m a la izquierda -> giro +0.3
    -Punto 2m a la izquierda -> giro +0.6
    '''
    #CONTROL P DE DIRECCIÓN
    error_lateral = target[1]  
    kp_steering = KP_STEERING
    steering = np.clip(kp_steering * error_lateral, -1.0, 1.0)

    '''
    Control P de velocidad: 
    -Si va a 1m/s -> error 2-1=1 -> acelerador = 0.3
    -Si va a 3m/s -> error 2-3=-1 -> freno 0.3
    '''

    # CONTROL P DE VELOCIDAD
    target_speed = TARGET_SPEED  # m/s
    kp_speed = KP_SPEED
    effort = kp_speed * (target_speed - speed)

    throttle = np.clip(effort, 0.0, 1.0)
    brake = np.clip(-effort, 0.0, 1.0)

    return float(throttle), float(steering), float(brake)


class Control(Node):
    def __init__(self):
        super().__init__("control")
        self.path = self.pose = None

        # PREPARAR EL CSV: se hace una sola vez al arrancar.
        self.start_time = time.monotonic()
        self.last_flush = self.start_time

        folder = FilePath(__file__).resolve().parent / "control_results"
        folder.mkdir(parents=True, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        filename = (
            f"{timestamp}_Kdir_{KP_STEERING}_Kvel_{KP_SPEED}.csv"
        )

        self.log_file = open(
            folder / filename, "w", newline="", encoding="utf-8"
        )
        self.log_writer = csv.writer(self.log_file)
        self.log_writer.writerow([
            "time_s",
            "reference_speed",
            "actual_speed",
            "path_distance_m",
            "steering",
        ])
        self.log_file.flush()

        self.get_logger().info(
            f"Datos guardados en: {folder / filename}"
        )

        # Comunicación con el resto del sistema.
        self.create_subscription(
            Path,
            contract.PATH,
            lambda m: setattr(self, "path", m),
            10,
        )
        self.create_subscription(
            Odometry,
            contract.POSE,
            lambda m: setattr(self, "pose", m),
            10,
        )
        self.pub = self.create_publisher(
            ControlCommand, contract.CONTROL, 10
        )

        # Ejecutar tick() 40 veces por segundo.
        self.create_timer(1.0 / 40.0, self.tick)

    def tick(self):
        if self.path is None or self.pose is None or not self.path.poses:
            self.pub.publish(ControlCommand(brake=1.0))
            return

        pose = Pose2D.from_odometry(self.pose)
        pts = np.array([
            (p.pose.position.x, p.pose.position.y)
            for p in self.path.poses
        ])

        path_body = world_to_body(pts, pose)
        throttle, steering, brake = compute(path_body, pose.vx)

        # Enviar las órdenes al coche.
        # compute(): positivo = izquierda.
        # Simulador: positivo = derecha.
        self.pub.publish(ControlCommand(
            throttle=float(throttle),
            steering=-float(steering),
            brake=float(brake),
        ))

        # REGISTRAR DATOS: añadir una fila en cada tick.
        now = time.monotonic()

        self.log_writer.writerow([
            now - self.start_time,
            TARGET_SPEED,
            pose.vx,
            distance_to_path(path_body),
            steering,
        ])

        if now - self.last_flush >= 1.0:
            self.log_file.flush()
            self.last_flush = now

    def destroy_node(self):
        # Cerrar el archivo cuando se destruya el nodo.
        log_file = getattr(self, "log_file", None)
        if log_file is not None and not log_file.closed:
            log_file.close()

        return super().destroy_node()

def main():
    run(Control)


if __name__ == "__main__":
    main()

