import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

import numpy as np

# Objectif du noeud : on lui donne en entrée la distance à parcourir, et il commande la vitesse de rotation des roues pour 
# faire avancer le robot en ligne droite, avec une phase d'accélération, et de décélération, pour que le robot parcours 
# la distance passée en entrée

# Le terme rampe ici désignera une fonction composé d'une fonction affine croissante passant par l'origine (pour x de 0 à t1), 
# puis d'un plateau (pour x de t1 à t2), et d'une fonction affine décroissante (pour x de t2 à t3)

class CmdVelRampPublisher(Node):

    def __init__(self):
        super().__init__('cmd_vel_ramp_publisher')  #Nom du noeud entre ''
        self.publisher_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription_cmd_dist = self.create_subscription(
            Float32,
            'cmd_dist',
            self.cmd_dist_callback,
            10)
            
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.distance = 0.0  # distance à parcourir
        # Paramètre de la rampe
        self.t1=0.0
        self.t1=0.0
        self.t1=0.0
        self.beta=0.0
        self.acceleration=0.25
        self.vmax=0.2

        self.circonfRoue = 0.19
        self.t=0
        self.run = False
    """
    def timer_callback(self):
        msg = Twist()
        msg.linear.x = (self.i%2) / 2
        self.publisher_cmd_vel.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.linear.x)
        self.i += 1
    """

    

    def cmd_dist_callback(self, msg):
        self.get_logger().info('Receiving : "%s"' % msg.data)
        self.distance = float(msg.data)
        self.create_ramp()
        self.i=0
        self.run=True

    def conversion_metre_rads(self, value):
        return (value*17)/(10*np.pi*self.circonfRoue)

    def create_ramp(self):
        """
        Paramétrage d'une rampe
        v(t) = {
            acceleration*t       , 0<=t<=t1
            vmaw                 , t1<=t<=t2
            -acceleration*t+beta , t2<=t<=t3
        }
        Avec 
        - v(t) : vitesse en fonction du temps (vitesse décrivant une rampe croissante, puis un plateau et une rampe décroissante, jusqu'à 0)
        - acceleration : ceof dir des rampes
        - vmax : vitesse maximale, atteinte lors du plateau
        - beta : ordonnée à l'origine dans le cas de la rampe décroissante (nulle pour la rampe croissante, mais non nulle pour la rampe descendante)
        - t1 : instant de transition entre fin de rampe croissante et début du plateau
        - t2 : instant de transition entre fin de plateau et début de rampe décroissante
        - t3 : fin de rampe décroissante (v(t3)=0 dans la théorie)
        """
        self.t1 = float(self.vmax/self.acceleration)
        self.t2 = float(self.distance/self.vmax)
        self.t3 = float(self.t2+self.t1)
        self.beta = float(self.acceleration*self.t3)
    
    def ramp(self, time):
        cmd_vel_metre = 0.0
        if self.t1==0 or self.t2==0 or self.t3==0 or self.beta==0:
            self.create_ramp()
        if self.acceleration==0 or self.vmax==0 or self.distance==0:
            self.get_logger().info('Données manquantes (accélération, vitesse max ou distance)')
        else:
            if time<self.t1:
                cmd_vel_metre = float(self.acceleration*time)
            elif time<self.t2:
                cmd_vel_metre = float(self.vmax)
            elif time<self.t3:
                cmd_vel_metre = float(-self.acceleration*time+self.beta)
            else:
                cmd_vel_metre = 0.0
            
            if cmd_vel_metre<0:
                return 0.0
            elif cmd_vel_metre>self.vmax:
                return float(vmax)
            else:
                return cmd_vel_metre

    def runDist(self):  # Fonction qui va publier les valeurs de vitesses permettant de parcourir 90% de la distance prévu
        cmd_vel_rad = self.conversion_metre_rads(self.ramp(self.t))
        msg = Twist()
        msg.linear.x=cmd_vel_rad
        self.publisher_cmd_vel.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.linear.x)
        self.t+=1/10
        if self.t>1 and cmd_vel_rad==0:
            self.run=False
    
    def timer_callback(self):
        if self.run:
            self.runDist()

def main(args=None):
    rclpy.init(args=args)

    cmd_vel_ramp_publisher = CmdVelRampPublisher()

    rclpy.spin(cmd_vel_ramp_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cmd_vel_ramp_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()