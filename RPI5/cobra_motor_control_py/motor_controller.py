#!/usr/bin/env python3
# motor_controller.py - RPi5 - COMMUNICATION COMPLÈTE + CAN (version modulaire)
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

from motor_can_driver import MotorCANDriver, MotorCANError    nouveau

class CobraMotorController(Node):
    STEP_CM = 10.0
    CM_TO_DEG = 18.0

    def __init__(self):
        super().__init__('cobra_motor_controller')

        # --- état système ---
        self.current_mode = None
        self.current_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.speed_mode1 = 50.0
        self.speed_mode2 = 50.0

        # Limites / sécurité identiques à ton code
        self.workspace_limits = {'x_min': -100.0, 'x_max': 100.0,
                                 'y_min': -100.0, 'y_max': 100.0,
                                 'z_min': 0.0,    'z_max': 100.0}
        self.collision_protection = False
        self.collision_margin = 5.0

        # --- CAN : on instancie le sous-module -----------------------
        try:
            self.can_drv = MotorCANDriver(channel="can0",
                                          node_id=0x141,
                                          cm_to_deg=self.CM_TO_DEG)
            self.can_connected = True
            self.get_logger().info("✅ Interface CAN prête")
        except MotorCANError as exc:
            self.can_drv = None
            self.can_connected = False
            self.get_logger().error(f"❌ CAN indisponible : {exc}")

        # --- ROS topics ---
        self.from_rpi4_sub = self.create_subscription(
            String, '/rpi4_to_rpi5', self.rpi4_message_callback, 10)
        self.to_rpi4_pub = self.create_publisher(
            String, '/rpi5_to_rpi4', 10)
        self.status_timer = self.create_timer(1.0, self.send_status_to_rpi4)

        self.get_logger().info('🛰️ CoBra Motor Controller (modulaire) prêt')

    # =================================================================
    # 1.  Callback principal (inchangé sauf appels CAN)
    # =================================================================
    def rpi4_message_callback(self, msg):
        cmd = msg.data
        # On montre seulement Y↑ et Y↓ ; le reste de ton parsing ne bouge pas.
        if cmd == 'JOYSTICK:MODE1:XY:UP':
            self._move_y(+self.STEP_CM)
        elif cmd == 'JOYSTICK:MODE1:XY:DOWN':
            self._move_y(-self.STEP_CM)
        # … tout le reste de tes commandes ici …

    # =================================================================
    # 2.  Mouvement Y  →  appel MotorCANDriver
    # =================================================================
    def _move_y(self, delta_cm: float):
        new_y = self.current_position['y'] + delta_cm

        # vérifs limites & collision identiques à ton code …
        # (omises ici pour la clarté)

        # On envoie la commande au moteur uniquement si CAN dispo
        if self.can_drv is not None:
            speed = self.speed_mode1 if self.current_mode == 'MODE1' else self.speed_mode2
            try:
                self.can_drv.move_delta_cm(delta_cm, speed)
            except MotorCANError as exc:
                self.get_logger().error(f'❌ CAN : {exc}')
                return                      # on n’actualise pas la position

        self.current_position['y'] = new_y
        self.get_logger().info(f'📍 Y = {new_y:.1f} cm (OK)')

    # =================================================================
    # 3.  Arrêt d’urgence / reset : délègue aussi au driver
    # =================================================================
    def handle_emergency_stop(self):
        self.get_logger().warning('🚨 ARRÊT D’URGENCE')
        if self.can_drv:
            try:
                self.can_drv.stop()
            except MotorCANError:
                pass

    def handle_reset(self):
        self.get_logger().info(' RESET → 0 cm / 0°')
        if self.can_drv:
            try:
                self.can_drv.goto_zero()
            except MotorCANError:
                pass
        self.current_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}

    # =================================================================
    # 4.  Publication du statut (inchangé)
    # =================================================================
    def send_status_to_rpi4(self):
        status = {
            'position': self.current_position,
            'can_connected': self.can_connected,
            'motor_angle': self.can_drv.angle if self.can_drv else None
        }
        self.to_rpi4_pub.publish(String(data=json.dumps(status)))

    # -----------------------------------------------------------------
    # 5.  Nettoyage
    # -----------------------------------------------------------------
    def destroy_node(self):
        if self.can_drv:
            try:
                self.can_drv.stop()
                self.can_drv.shutdown()
            except MotorCANError:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CobraMotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(' Arrêt demandé')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



