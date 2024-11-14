import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from icm20948 import ICM20948
import numpy as np

class NodeImu(Node):
    def __init__(self):
        super().__init__('Imu_readings')
        self.period = 0.04  # Période entre les callbacks (25 Hz)
        
        # Publisher pour les données brutes de l'IMU
        self.publisher_imu_ = self.create_publisher(Imu, 'imu/data_raw', 10)
        
        # Publisher pour les données du magnétomètre
        self.publisher_mag_ = self.create_publisher(MagneticField, 'imu/mag', 10)

        # Initialiser le capteur ICM20948 IMU
        self.sensor = ICM20948()  # Initialiser l'IMU ICM20948

        # Timer pour les callbacks périodiques
        self.timer_ = self.create_timer(self.period, self.timer_callbacks)

        # Initialiser les messages ROS pour l'IMU et le magnétomètre
        self.imu_msg = Imu()
        self.mag_msg = MagneticField()

        self.get_logger().info('Nœud IMU initialisé')

    def timer_callbacks(self):
        """
        Publier les données brutes de l'IMU et du magnétomètre à intervalles réguliers
        """
        # Lecture des données de l'accéléromètre, du gyroscope et du magnétomètre
        ax, ay, az, gx, gy, gz = self.sensor.read_accelerometer_gyro_data()
        mx, my, mz = self.sensor.read_magnetometer_data()

        # Remplir le message IMU avec les données brutes
        self.imu_msg.linear_acceleration.x = ax * 9.81  # Accélération en m/s^2
        self.imu_msg.linear_acceleration.y = ay * 9.81
        self.imu_msg.linear_acceleration.z = az * 9.81
        
        self.imu_msg.angular_velocity.x = np.radians(gx)  # Gyroscope en radians/s
        self.imu_msg.angular_velocity.y = np.radians(gy)
        self.imu_msg.angular_velocity.z = np.radians(gz)
        
        # Marquer les covariances comme non disponibles pour l'orientation
        self.imu_msg.orientation_covariance[0] = -1  # Orientation non calculée par le capteur directement

        # Publier les données brutes de l'IMU
        self.publisher_imu_.publish(self.imu_msg)

        # Remplir le message MagneticField avec les données du magnétomètre
        self.mag_msg.magnetic_field.x = mx  # Magnétomètre en Tesla (attention à l'unité)
        self.mag_msg.magnetic_field.y = my
        self.mag_msg.magnetic_field.z = mz

        # Covariances du magnétomètre (mettre une estimation si connue)
        self.mag_msg.magnetic_field_covariance[0] = 0.01
        self.mag_msg.magnetic_field_covariance[4] = 0.01
        self.mag_msg.magnetic_field_covariance[8] = 0.01

        # Publier les données du magnétomètre
        self.publisher_mag_.publish(self.mag_msg)

def main(args=None):
    rclpy.init(args=args)
    node = NodeImu()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
