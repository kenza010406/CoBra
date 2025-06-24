#!/usr/bin/env python3
# ui_controller.py - RPi4 - COMMUNICATION CLAIRE
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CobraUIController(Node):
    def __init__(self):
        super().__init__('cobra_ui_controller')
        
        # Publishers - ENVOI vers RPI5
        self.to_rpi5_publisher = self.create_publisher(String, '/rpi4_to_rpi5', 10)
        
        # Subscribers - RÉCEPTION depuis RPI5  
        self.from_rpi5_subscriber = self.create_subscription(String, '/rpi5_to_rpi4', self.rpi5_message_callback, 10)
        
        self.get_logger().info('🚀 CoBra UI Controller - RPi4 - COMMUNICATION CLAIRE')
        self.get_logger().info('📤 Envoi vers RPI5: /rpi4_to_rpi5')
        self.get_logger().info('📥 Réception depuis RPI5: /rpi5_to_rpi4')
    
    def rpi5_message_callback(self, msg):
        """Messages reçus du RPI5"""
        message = msg.data
        self.get_logger().info(f'📥 RPI5→RPI4: {message}')
        
        # Ici on peut traiter les messages du RPI5 si nécessaire
        # Par exemple logger les changements de statut
        if 'POSITION:' in message:
            self.get_logger().info('📍 Position mise à jour')
        elif 'CAN:' in message:
            self.get_logger().info('🔌 Statut CAN mis à jour')
        elif 'MOTORS:' in message:
            self.get_logger().info('⚙️ Statut moteurs mis à jour')

def main(args=None):
    rclpy.init(args=args)
    controller = CobraUIController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('⏹️ Arrêt UI Controller - RPi4')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

