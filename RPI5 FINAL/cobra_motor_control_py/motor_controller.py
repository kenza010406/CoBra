#!/usr/bin/env python3
# motor_controller.py - RPi5 - COMMUNICATION COMPLÈTE + CAN (version modifiée pour canusb)

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
  
from cobra_motor_control_py.motor_can_driver import MotorCANDriver, MotorCANError

class CobraMotorController(Node):
    STEP_CM = 5.0
    CM_TO_DEG = 18.0

    def __init__(self):
        super().__init__('cobra_motor_controller')

        self.current_mode = None
        self.current_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.speed_mode1 = 50.0
        self.speed_mode2 = 50.0

        self.workspace_limits = {'x_min': -100.0, 'x_max': 100.0,
                                 'y_min': -100.0, 'y_max': 100.0,
                                 'z_min': 0.0,    'z_max': 100.0}
        self.collision_protection = False
        self.collision_margin = 5.0

        try:
            self.can_drv = MotorCANDriver(node_id=0x141,
                                          cm_to_deg=self.CM_TO_DEG)
            self.can_connected = True
            self.get_logger().info("✅ Interface CAN prête")
        except MotorCANError as exc:
            self.can_drv = None
            self.can_connected = False
            self.get_logger().error(f"❌ Interface CAN indisponible : {exc}")

        self.from_rpi4_sub = self.create_subscription(
            String, '/rpi4_to_rpi5', self.rpi4_message_callback, 10)
        self.to_rpi4_pub = self.create_publisher(
            String, '/rpi5_to_rpi4', 10)
        self.status_timer = self.create_timer(1.0, self.send_status_to_rpi4)

        self.get_logger().info('🛰️ CoBra Motor Controller (modulaire) prêt')

    def rpi4_message_callback(self, msg):
        cmd = msg.data
        self.get_logger().info(f"📩 Commande reçue : {cmd}")

        if cmd.startswith('JOYSTICK:'):
            self.handle_joystick(cmd)
        elif cmd == 'EMERGENCY_STOP':
            self.handle_emergency_stop()
        elif cmd in ['RESET_POSITION', 'RESET', 'REINIT']:
            self.handle_reset()
        elif cmd == 'QUIT_TO_ZERO' :
            self.handle_quit_to_zero()
        elif cmd.startswith('MODE:'):
            self.handle_mode_change(cmd)
        else:
            self.get_logger().warning(f"❓ Commande inconnue : {cmd}")

    def handle_joystick(self, cmd):
        self.get_logger().info(f"🎮 Commande joystick reçue : {cmd}")
        parts = cmd.split(':')

        if len(parts) < 4:
            self.get_logger().warning("❌ Format joystick invalide")
            return

        _, mode, plane, direction = parts

        if mode == 'MODE1':
            self.current_mode = 'MODE1'
            speed = self.speed_mode1
        elif mode == 'MODE2':
            self.current_mode = 'MODE2'
            speed = self.speed_mode2
        else:
            self.get_logger().warning(f"❌ Mode inconnu : {mode}")
            return

        delta = self.STEP_CM

        if plane == 'XY':
            if direction == 'UP':
                self._move_y(+delta)
            elif direction == 'DOWN':
                self._move_y(-delta)
        elif plane == 'Z':
            if direction == 'UP':
                self._move_z(+delta)
            elif direction == 'DOWN':
                self._move_z(-delta)
        else:
            self.get_logger().warning(f"❌ Plan inconnu : {plane}")

    def handle_mode_change(self, cmd):
        parts = cmd.split(':')
        if len(parts) != 2:
            self.get_logger().warning("❌ Format de commande mode invalide")
            return

        mode = parts[1]
        if mode == '1':#!/usr/bin/env python3
# motor_controller.py - RPi5 - COMMUNICATION COMPLÈTE + CAN (version modifiée pour canusb)

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
  
from cobra_motor_control_py.motor_can_driver import MotorCANDriver, MotorCANError

class CobraMotorController(Node):
    STEP_CM = 5.0
    CM_TO_DEG = 18.0

    def __init__(self):
        super().__init__('cobra_motor_controller')

        self.current_mode = None
        self.current_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.speed_mode1 = 50.0
        self.speed_mode2 = 50.0

        self.workspace_limits = {'x_min': -100.0, 'x_max': 100.0,
                                 'y_min': -100.0, 'y_max': 100.0,
                                 'z_min': 0.0,    'z_max': 100.0}
        self.collision_protection = False
        self.collision_margin = 5.0

        try:
            self.can_drv = MotorCANDriver(node_id=0x141,
                                          cm_to_deg=self.CM_TO_DEG)
            self.can_connected = True
            self.get_logger().info("✅ Interface CAN prête")
        except MotorCANError as exc:
            self.can_drv = None
            self.can_connected = False
            self.get_logger().error(f"❌ Interface CAN indisponible : {exc}")

        self.from_rpi4_sub = self.create_subscription(
            String, '/rpi4_to_rpi5', self.rpi4_message_callback, 10)
        self.to_rpi4_pub = self.create_publisher(
            String, '/rpi5_to_rpi4', 10)
        self.status_timer = self.create_timer(1.0, self.send_status_to_rpi4)

        self.get_logger().info('🛰️ CoBra Motor Controller (modulaire) prêt')

    def rpi4_message_callback(self, msg):
        cmd = msg.data
        self.get_logger().info(f"📩 Commande reçue : {cmd}")

        if cmd.startswith('JOYSTICK:'):
            self.handle_joystick(cmd)
        elif cmd == 'EMERGENCY_STOP':
            self.handle_emergency_stop()
        elif cmd in ['RESET_POSITION', 'RESET', 'REINIT']:
            self.handle_reset()
        elif cmd == 'QUIT_TO_ZERO' :
            self.handle_quit_to_zero()
        elif cmd.startswith('MODE:'):
            self.handle_mode_change(cmd)
        else:
            self.get_logger().warning(f"❓ Commande inconnue : {cmd}")

    def handle_joystick(self, cmd):
        self.get_logger().info(f"🎮 Commande joystick reçue : {cmd}")
        parts = cmd.split(':')

        if len(parts) < 4:
            self.get_logger().warning("❌ Format joystick invalide")
            return

        _, mode, plane, direction = parts

        if mode == 'MODE1':
            self.current_mode = 'MODE1'
            speed = self.speed_mode1
        elif mode == 'MODE2':
            self.current_mode = 'MODE2'
            speed = self.speed_mode2
        else:
            self.get_logger().warning(f"❌ Mode inconnu : {mode}")
            return

        delta = self.STEP_CM

        if plane == 'XY':
            if direction == 'UP':
                self._move_y(+delta)
            elif direction == 'DOWN':
                self._move_y(-delta)
        elif plane == 'Z':
            if direction == 'UP':
                self._move_z(+delta)
            elif direction == 'DOWN':
                self._move_z(-delta)
        else:
            self.get_logger().warning(f"❌ Plan inconnu : {plane}")

    def handle_mode_change(self, cmd):
        parts = cmd.split(':')
        if len(parts) != 2:
            self.get_logger().warning("❌ Format de commande mode invalide")
            return

        mode = parts[1]
        if mode == '1':
            self.current_mode = 'MODE1'
        elif mode == '2':
            self.current_mode = 'MODE2'
        else:
            self.get_logger().warning(f"❌ Mode inconnu : {mode}")
            return

        self.get_logger().info(f"🔄 Changement de mode : {self.current_mode}")
        self.handle_quit_to_zero()

    def _move_y(self, delta_cm: float):
        new_y = self.current_position['y'] + delta_cm

        if self.can_drv is not None:
            speed = self.speed_mode1 if self.current_mode == 'MODE1' else self.speed_mode2
            try:
                self.can_drv.move_delta_cm(delta_cm, speed)
            except MotorCANError as exc:
                self.get_logger().error(f'❌ CAN : {exc}')
                return

        self.current_position['y'] = new_y
        angle_target = self.can_drv.angle if self.can_drv else 0.0
        self.get_logger().info(f'📍 Y = {new_y:.1f} cm (Angle ≈ {angle_target:.1f}°) ✅')

    def _move_z(self, delta_cm: float):
        new_z = self.current_position['z'] + delta_cm

        if self.can_drv is not None:
            speed = self.speed_mode1 if self.current_mode == 'MODE1' else self.speed_mode2
            try:
                self.can_drv.move_delta_cm(delta_cm, speed)
            except MotorCANError as exc:
                self.get_logger().error(f'❌ CAN : {exc}')
                return

        self.current_position['z'] = new_z
        angle_target = self.can_drv.angle if self.can_drv else 0.0
        self.get_logger().info(f'📍 Z = {new_z:.1f} cm (Angle ≈ {angle_target:.1f}°) ✅')

    def handle_emergency_stop(self):
        self.get_logger().warning('🚨 ARRÊT D’URGENCE demandé')
        if self.can_drv:
            try:
                self.can_drv.stop()
                self.get_logger().info('🛑 Moteur arrêté avec succès')
            except MotorCANError as e:
                self.get_logger().error(f'❌ Échec arrêt moteur : {e}')

    def handle_reset(self):
        self.get_logger().info('♻️ Réinitialisation demandée')
        if self.can_drv:
            try:
                self.can_drv.goto_zero()
                self.get_logger().info('🔄 Moteur retourné à 0°')
            except MotorCANError as e:
                self.get_logger().error(f'❌ Échec retour à 0° : {e}')
        self.current_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}

    def handle_quit_to_zero(self):
        self.get_logger().info('🔁 Retour à 0 demandé (changement de mode)')
        if self.can_drv:
            try:
                self.can_drv.goto_zero()
                self.get_logger().info('🔄 Moteur retourné à 0°')
            except MotorCANError as e:
                self.get_logger().error(f'❌ Échec retour à 0° : {e}')
        self.current_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}

    def send_status_to_rpi4(self):
        status = {
            'position': self.current_position,
            'can_connected': self.can_connected,
            'motor_angle': self.can_drv.angle if self.can_drv else None
        }
        self.to_rpi4_pub.publish(String(data=json.dumps(status)))

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

            self.current_mode = 'MODE1'
        elif mode == '2':
            self.current_mode = 'MODE2'
        else:
            self.get_logger().warning(f"❌ Mode inconnu : {mode}")
            return

        self.get_logger().info(f"🔄 Changement de mode : {self.current_mode}")
        self.handle_quit_to_zero()

    def _move_y(self, delta_cm: float):
        new_y = self.current_position['y'] + delta_cm

        if self.can_drv is not None:
            speed = self.speed_mode1 if self.current_mode == 'MODE1' else self.speed_mode2
            try:
                self.can_drv.move_delta_cm(delta_cm, speed)
            except MotorCANError as exc:
                self.get_logger().error(f'❌ CAN : {exc}')
                return

        self.current_position['y'] = new_y
        angle_target = self.can_drv.angle if self.can_drv else 0.0
        self.get_logger().info(f'📍 Y = {new_y:.1f} cm (Angle ≈ {angle_target:.1f}°) ✅')

    def _move_z(self, delta_cm: float):
        new_z = self.current_position['z'] + delta_cm

        if self.can_drv is not None:
            speed = self.speed_mode1 if self.current_mode == 'MODE1' else self.speed_mode2
            try:
                self.can_drv.move_delta_cm(delta_cm, speed)
            except MotorCANError as exc:
                self.get_logger().error(f'❌ CAN : {exc}')
                return

        self.current_position['z'] = new_z
        angle_target = self.can_drv.angle if self.can_drv else 0.0
        self.get_logger().info(f'📍 Z = {new_z:.1f} cm (Angle ≈ {angle_target:.1f}°) ✅')

    def handle_emergency_stop(self):
        self.get_logger().warning('🚨 ARRÊT D’URGENCE demandé')
        if self.can_drv:
            try:
                self.can_drv.stop()
                self.get_logger().info('🛑 Moteur arrêté avec succès')
            except MotorCANError as e:
                self.get_logger().error(f'❌ Échec arrêt moteur : {e}')

    def handle_reset(self):
        self.get_logger().info('♻️ Réinitialisation demandée')
        if self.can_drv:
            try:
                self.can_drv.goto_zero()
                self.get_logger().info('🔄 Moteur retourné à 0°')
            except MotorCANError as e:
                self.get_logger().error(f'❌ Échec retour à 0° : {e}')
        self.current_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}

    def handle_quit_to_zero(self):
        self.get_logger().info('🔁 Retour à 0 demandé (changement de mode)')
        if self.can_drv:
            try:
                self.can_drv.goto_zero()
                self.get_logger().info('🔄 Moteur retourné à 0°')
            except MotorCANError as e:
                self.get_logger().error(f'❌ Échec retour à 0° : {e}')
        self.current_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}

    def send_status_to_rpi4(self):
        status = {
            'position': self.current_position,
            'can_connected': self.can_connected,
            'motor_angle': self.can_drv.angle if self.can_drv else None
        }
        self.to_rpi4_pub.publish(String(data=json.dumps(status)))

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
