import rclpy
from rclpy.node import Node

import tf_transformations

from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8
from geometry_msgs.msg import Twist

# Concept : Initialiser une rotation dans le bon sens lorsqu'une nouvelle rotation est publiée, puis vérifier régulièrement si le robot a etteint l'angle souhaité
# Vérification régulière avec un timer, et non à chaque maj de l'odométrie pour avoir un asservissement constant (moins précis mais plus régulier)


# Topic /state_robot : type UInt8
# msg.data = 0 : robot immobile, ready pour un nouveau déplacement
# msg.data = 1 : robot en rotation
# msg.data = 2 : robot en déplacement rectiligne

class CmdVelTurnPublisher(Node):

    def __init__(self):
        super().__init__('cmd_vel_turn_publisher')
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)  # publisher pour commander la vitesse des roues
        # subscriber pour récupérer la position à atteindre
        self.cmd_turn_subscriber = self.create_subscription(
            Float32,
            'cmd_turn',
            self.cmd_turn_callback,
            10)

        # Publisher/subscriber permettant de mettre à jour l'état de déplacement du robot
        self.state_robot_publisher = self.create_publisher(UInt8, 'state_robot', 10)
        self.state_robot_subscriber = self.create_subscription(
            UInt8, 
            'state_robot', 
            self.state_robot_callback, 
            10)
        self.state_robot_subscriber 

        # subscriber pour récupérer la position du robot
        self.odom_subscriber = self.create_subscription(
            Odometry,
            'odom',  # Nom du topic (modifiable selon ton cas)
            self.odom_callback,
            10)  # Taille de la queue
        self.odom_subscriber  # Empêche la suppression de la subscription par le garbage collector

        self.speedTurn = 0.8
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.yaw = 0.0  # yaw actuel (update avec l'odométrie)
        self.targetYaw = 0.0  # yaw objectif
        self.offset_reaching_yaw = 0.15  # Ecart max entre le yaw objectif et le yaw réel (arrêt de la rotation lorsque l'écart réel passe en dessous de cette valeur)

        self.state_robot = 0

    def update_state_robot(self, data):
        msg_state_robot = UInt8()
        msg_state_robot.data = data
        self.state_robot_publisher.publish(msg_state_robot)

    # Lorsque l'odométrie est mise à jour, on met à jour l'attribut yaw (en convertissant les quaternions en angle euler)
    def odom_callback(self, msg):
        qx, qy, qz, qw = msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
        roll, pitch, yaw = tf_transformations.euler_from_quaternion([qx, qy, qz, qw])
        self.yaw  = yaw

    # Initialisation de la rotation dans le bon sens
    def cmd_turn_callback(self, msg):
        self.get_logger().info(f'Reçu : x={msg.data}')
        self.targetYaw = msg.data
        
        msg_Twist = Twist()
        if msg.data-self.yaw>0:  # si la commande est > à l'angle actuel, on tourne dans le sens positif, sinon dans le sens négatif, à vitesse constante
            msg_Twist.angular.z = self.speedTurn
        else:
            msg_Twist.angular.z = -self.speedTurn
        
        # Vérification supplémentaire que le robot est bien arrêté avant de lancer une nouvelle rotation
        if (self.state_robot == 0):
            self.update_state_robot(1)  # Mise à jour pour signaler que le robot se met en mouvement de rotation
            self.cmd_vel_publisher.publish(msg_Twist)
        else:
            self.get_logger().info("Robot en mouvement, commande de rotation en attente")
    
    # mise à jour de l'état de déplacemment du robot 
    def state_robot_callback(self, msg):
        self.state_robot = msg.data
        self.get_logger().info(f"Changement d'état : {msg.data}")

    # callback, qui vérifie régulièrement si l'angle est atteint ou non, et arrête la rotation lorsque c'est le cas
    def timer_callback(self):
        if abs(self.yaw-self.targetYaw)<self.offset_reaching_yaw and self.state_robot == 1:
            msg = Twist()
            msg.angular.z = 0.0
            self.update_state_robot(0)  # Mise à jour pour signifier que le robot devient arrêté
            self.cmd_vel_publisher.publish(msg)

        self.get_logger().info(f"yaw={self.yaw}, target_yaw={self.targetYaw}")

def main(args=None):
    rclpy.init(args=args)

    cmd_vel_turn_publisher = CmdVelTurnPublisher()

    rclpy.spin(cmd_vel_turn_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cmd_vel_turn_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()