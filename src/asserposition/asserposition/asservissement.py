import rclpy
from rclpy.node import Node

import tf_transformations
import math

from std_msgs.msg import Float32
from std_msgs.msg import UInt8
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

# Concept du noeud : gérer la rotation puis le déplacement du robot, en publiant d'abord l'angle à atteindre sur /cmd_turn
# puis en publiant la distance à parcourir sur /cmd_dist
# Objectif : le robot s'oriente d'abord vers le point à atteindre, puis il y va en ligne droite


# Topic /state_robot : type UInt8 (permet de savoir depuis ce noeud où en est la rotation et l'avancement rectiligne du robot, 
#                                  pour n'envoyer la nouvelle commande que lorsque le robot a fini son déplacement)
# msg.data = 0 : robot immobile, prêt pour un nouveau déplacement
# msg.data = 1 : robot en rotation
# msg.data = 2 : robot en déplacement rectiligne

class Asservissement(Node):

    def __init__(self):
        super().__init__('asservissement_position')
        self.new_pos_subscriber = self.create_subscription(
            Point,
            'target_position',
            self.new_pose_callback,
            10)
        self.new_pos_subscriber  # prevent unused variable warning

         # subscriber pour récupérer la position du robot
        self.odom_subscriber = self.create_subscription(
            Odometry,
            'odom',  # Nom du topic (modifiable selon ton cas)
            self.odom_callback,
            10)  # Taille de la queue
        self.odom_subscriber  # Empêche la suppression de la subscription par le garbage collector

        # subscriber pour récupérer l'état de déplacement du robot
        self.state_robot_subscriber = self.create_subscription(
            UInt8, 
            'state_robot', 
            self.state_robot_callback, 
            10)
        self.state_robot_subscriber 

        # Publisher pour la distance et l'angle de rotation
        self.cmd_turn_publisher = self.create_publisher(Float32, 'cmd_turn', 10)
        self.cmd_dist_publisher = self.create_publisher(Float32, 'cmd_dist', 10)

        self.yaw = 0.0
        self.posX = 0.0 # position actuel
        self.posY = 0.0
        self.targetPoseX = 0.0 # objectif
        self.targetPoseY = 0.0
        self.distance = 0.0
        self.state_robot = 0

    def new_pose_callback(self, msg):
        self.get_logger().info(f'Reçu : x={msg.x}, y={msg.y}')
        self.targetPoseX = msg.x
        self.targetPoseY = msg.y
        # Calcul d'une distance euclidienne entre 2 points en 2D
        self.distance = math.sqrt((self.targetPoseX-self.posX)*(self.targetPoseX-self.posX)+(self.targetPoseY-self.posY)*(self.targetPoseY-self.posY))
        # Calcul de l'angle entre la direction X de la map et la droite passant par le robot et l'objectif (rad) 
        # (c'est l'angle de rotation absolu par rapport au start)
        # Objectif : atteindre l'angle en question
        self.target_yaw = math.atan((self.targetPoseY-self.posY)/(self.targetPoseX-self.posX))
        msg = Float32()
        msg.data = self.target_yaw
        while(self.state_robot != 0):
            self.get_logger().info("Robot en mouvement, commande de rotation en attente")
            # permet d'attendre 100 ms (éviter de spam), sans bloquer le reste du noeud 
            # (time.sleep bloque tout le noeud, ce qui empêcherai les callback de changer la valeur de state_robot)
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))
        self.cmd_turn_publisher.publish(msg)
        # On force le noeud à attendre pour être sur que le noeud gérant la rotation ait le temps de changer la valeur 
        # de /state_robot avant de lancer la séquence d'avancement rectiligne
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1))

        # Il y a un problème ici, le robot tourne durant 1 secondes et n'attend pas la fin de la rotation pour avancer
        # On attend que le robot ait fini sa rotation avant d'avancer
        while(self.state_robot != 0):
            self.get_logger().info("Robot en mouvement, commande de rotation en attente")
            # permet d'attendre 100 ms (éviter de spam), sans bloquer le reste du noeud 
            # (time.sleep bloque tout le noeud, ce qui empêcherai les callback de changer la valeur de state_robot)
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))
        msg_cmd_dist = Float32()
        msg_cmd_dist.data = self.distance
        self.cmd_dist_publisher.publish(msg_cmd_dist)
        

    
    # maj du yaw, et de la position absolue (X, Y)
    def odom_callback(self, msg):
        qx, qy, qz, qw = msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
        roll, pitch, yaw = tf_transformations.euler_from_quaternion([qx, qy, qz, qw])
        self.yaw  = yaw
        self.posX = msg.pose.pose.position.x
        self.posY = msg.pose.pose.position.y
        #self.get_logger().info(f'Reçu de odom : x={self.posX}, y={self.posY}, yaw={self.yaw}')

    # mise à jour de l'état de déplacemment du robot
    def state_robot_callback(self, msg):
        self.state_robot = msg.data
        self.get_logger().info(f"Changement d'état : {msg.data}")

def main(args=None):
    rclpy.init(args=args)

    asservissement_position = Asservissement()

    rclpy.spin(asservissement_position)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    asservissement_position.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()