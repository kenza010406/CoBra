import struct
from cobra_motor_control_py.can_interface import send_can_frame

class MotorCANError(RuntimeError):
    pass

class MotorCANDriver:
    def __init__(self, node_id: int = 0x141, cm_to_deg: float = 18.0):
        self._node_id = node_id
        self._cm_to_deg = cm_to_deg
        self._angle_curr = 0.0

    @property
    def angle(self) -> float:
        return self._angle_curr

    def move_delta_cm(self, delta_cm: float, speed_percent: float) -> None:
        speed_dps = int(max(0, min(100, speed_percent)) * 6)  # °/s
        target_deg = self._angle_curr + delta_cm * self._cm_to_deg
        self._send_angle_command(target_deg, speed_dps)
        self._angle_curr = target_deg

    def goto_zero(self, speed_percent: float = 100) -> None:
        self._send_angle_command(0.0, int(speed_percent * 6))
        self._angle_curr = 0.0

    def stop(self) -> None:
        self._send_angle_command(self._angle_curr, 0)

    def _send_angle_command(self, angle_deg: float, speed_dps: int) -> None:
        angle_lsb = int(angle_deg * 100)                 # 0.01°/LSB
        speed_lsb = max(0, min(60000, speed_dps))

        # Format little-endian: 00 | speed(2) | angle_lo(2) | angle_hi(2)
        payload = struct.pack('<BHH', 0x00, speed_lsb, angle_lsb & 0xFFFF)
        angle_high = (angle_lsb >> 16) & 0xFFFF
        payload += struct.pack('<H', angle_high)

        frame_id = f"{self._node_id:X}"
        data_hex = "A4" + payload.hex()

        if not send_can_frame(frame_id, data_hex):
            raise MotorCANError("Échec d'envoi de la trame CAN")
