import rclpy
import socket
import time
from rclpy.node import Node
from std_msgs.msg import Int16

class PAMISubscriber(Node):
	def __init__(self):
		super().__init__('pami_subscriber')
		self.envoye = False
		self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		self.socket.bind((host,port))
		self.get_logger().info('Communication PAMI démarrée')
		self.socket.listen(2)
		self.conn = []
		self.addr = []
		
		for i in range(2):
			conn, address = self.socket.accept()
			self.conn.append(conn)
			self.addr.append(address)
			self.get_logger().info('Un connexion a réussi')
			self.get_logger().info('Adresse : "%s"' % address[0])
			self.get_logger().info('Adresse : "%s" ; Ping' % address[0])

			data = "ping"
			data = data.encode('utf-8')
			conn.send(data)
			data = conn.recv(1024)
			data = data.decode("utf-8")
			if data == "pingReturn":
				self.get_logger().info('Adresse : "%s" ; Ping => OK' % address[0])

			#conn.close()
			time.sleep(0.2)
		self.subscription = self.create_subscription(Int16, '/timer', self.listener_callback, 10)
		self.subscription  # prevent unused variable warning

	def listener_callback(self, msg):
		self.get_logger().info('Timer : "%d"' % msg.data)
		if msg.data > 90 and self.envoye == False:
			for conn, addr in zip(self.conn, self.addr):
				data = "startSequence"
				data = data.encode('utf-8')
				conn.send(data)
				self.get_logger().info('Adresse : "%s" ; Ordre de demarrage envoye' % addr[0])
				#data = conn.recv(1024).decode("utf-8")
				#print(f"{addr[0]} sent {data}")
				#conn.close()
				time.sleep(0.2)
			#data = self.esp1_conn.recv(1024)
			#data = data.decode('utf-8')
			#if data == "startSequenceReturn":
				#self.get_logger().info('Adresse : "%s" ; Ordre de lancement confirme' % self.esp1_address[0])
			self.envoye = True

host, port = ('', 5566)

def main(args=None):
	rclpy.init(args=args)
	pami_subscriber = PAMISubscriber()
	rclpy.spin(pami_subscriber)
	socket.close()
	pami_subscriber.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
