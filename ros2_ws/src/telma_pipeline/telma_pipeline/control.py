"""My CONTROL node.   sb-launch control:=telma_pipeline/control

  in : /sandbox/path      nav_msgs/Path (map)
       /sandbox/pose      Odometry (map -> base_link), twist.linear.x = speed
  out: /control_command   fs_msgs/ControlCommand: throttle [0,1], brake [0,1], steering [-1,1] (+ = RIGHT)

Everything inside compute() is yours. Wheelbase 1.627 m, max steer 22.4 deg.
"""
import numpy as np
import rclpy
from fs_msgs.msg import ControlCommand
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node

from sandbox_tools import contract
from sandbox_tools.node_utils import run
from sandbox_tools.geometry import Pose2D, world_to_body

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
    kp_steering = 0.3
    steering = np.clip(kp_steering * error_lateral, -1.0, 1.0)

    '''
    Control P de velocidad: 
    -Si va a 1m/s -> error 2-1=1 -> acelerador = 0.3
    -Si va a 3m/s -> error 2-3=-1 -> freno 0.3
    '''

    # CONTROL P DE VELOCIDAD
    target_speed = 2.0  # m/s
    kp_speed = 0.3
    effort = kp_speed * (target_speed - speed)

    throttle = np.clip(effort, 0.0, 1.0)
    brake = np.clip(-effort, 0.0, 1.0)

    return float(throttle), float(steering), float(brake)


class Control(Node):
    def __init__(self):
        super().__init__("control")
        self.path = self.pose = None
        self.create_subscription(Path, contract.PATH, lambda m: setattr(self, "path", m), 10)
        self.create_subscription(Odometry, contract.POSE, lambda m: setattr(self, "pose", m), 10)
        self.pub = self.create_publisher(ControlCommand, contract.CONTROL, 10)
        self.create_timer(1.0 / 40.0, self.tick)

    def tick(self):
        if self.path is None or self.pose is None or not self.path.poses:
            self.pub.publish(ControlCommand(brake=1.0))
            return
        pose = Pose2D.from_odometry(self.pose)
        pts = np.array([(p.pose.position.x, p.pose.position.y) for p in self.path.poses])
        throttle, steering, brake = compute(world_to_body(pts, pose), pose.vx)
        # /control_command uses the car's convention, + = RIGHT (like IFSSIM and
        # IFS09-DV-PIPELINE). compute() gives + = left (y is left): flip it.
        self.pub.publish(ControlCommand(throttle=float(throttle), steering=-float(steering), brake=float(brake)))


def main():
    run(Control)


if __name__ == "__main__":
    main()
