#!/usr/bin/env python3
# motor_can_driver.py
"""
Couche d’accès CAN pour le moteur X6-40.
Elle regroupe toute la logique d’initialisation, d’envoi de trames
et de nettoyage.  Le contrôleur principal n’a plus qu’à instancier
MotorCANDriver et appeler ses méthodes.

Si quelque chose rate, la classe lève MotorCANError : le contrôleur
décide alors de la stratégie (logger, retry, etc.).
"""

from __future__ import annotations
import subprocess
import struct
import can        # python-can
from typing import Optional


class MotorCANError(RuntimeError):
    """Exception générique levée en cas d’erreur CAN."""


class MotorCANDriver:
    """
    Parameters
    ----------
    channel : str
        Interface CAN (par ex. "can0").
    bitrate : int
        Vitesse en bit/s (500 000 chez toi).
    node_id : int
        ID CAN du moteur.
    cm_to_deg : float
        Conversion cm  →  degrés moteur (18 °/cm dans ton code).
    """

    __slots__ = ("_bus", "_node_id", "_cm_to_deg", "_angle_curr")

    def __init__(
        self,
        channel: str = "can0",
        bitrate: int = 500_000,
        node_id: int = 0x141,
        cm_to_deg: float = 18.0,
    ) -> None:
        self._node_id = node_id
        self._cm_to_deg = cm_to_deg
        self._angle_curr = 0.0              # stocké localement
        self._bus: Optional[can.Bus] = None
        self._setup_interface(channel, bitrate)

    # ------------------------------------------------------------------
    # API exposée au contrôleur ROS ------------------------------------
    # ------------------------------------------------------------------
    @property
    def angle(self) -> float:
        """Angle courant (cache local)."""
        return self._angle_curr

    def move_delta_cm(self, delta_cm: float, speed_percent: float) -> None:
        """
        Déplace le moteur d’un delta linéaire exprimé en centimètres.
        Conversion : 1 cm → cm_to_deg °.
        """
        speed_dps = int(max(0, min(100, speed_percent)) * 6)  # % → °/s
        target_deg = self._angle_curr + delta_cm * self._cm_to_deg
        self._send_angle_command(target_deg, speed_dps)
        self._angle_curr = target_deg      # maj locale seulement si OK

    def goto_zero(self, speed_percent: float = 100) -> None:
        """Retourne à 0° (calibration)."""
        self._send_angle_command(0.0, int(speed_percent * 6))
        self._angle_curr = 0.0

    def stop(self) -> None:
        """Arrêt immédiat (envoie l’angle courant, vitesse 0)."""
        self._send_angle_command(self._angle_curr, 0)

    def shutdown(self) -> None:
        """Ferme proprement le bus CAN."""
        if self._bus is not None:
            self._bus.shutdown()
            self._bus = None

    # ------------------------------------------------------------------
    # Initialisation et bas niveau -------------------------------------
    # ------------------------------------------------------------------
    def _setup_interface(self, channel: str, bitrate: int) -> None:
        for cmd in (["sudo", "ip", "link", "set", channel, "down"],
                    ["sudo", "ip", "link", "set", channel, "up",
                     "type", "can", "bitrate", str(bitrate)]):
            subprocess.run(cmd, check=False, capture_output=True)

        try:
            self._bus = can.interface.Bus(channel, bustype="socketcan")
        except Exception as exc:  # pragma: no cover
            raise MotorCANError(f"Impossible d’ouvrir {channel}") from exc

    def _send_angle_command(self, angle_deg: float, speed_dps: int) -> None:
        """Construit puis transmet la trame 0xA4 « move to pos ». 0,01 °/LSB."""
        if self._bus is None:
            raise MotorCANError("Bus CAN non initialisé")

        angle_lsb = int(angle_deg * 100)                 # 32 b
        speed_lsb = max(0, min(60_000, speed_dps))       # 0-60000

        # Little-endian : <B  H  H  I  => total 7 octets utiles
        payload = struct.pack("<BHHI", 0x00, speed_lsb,
                              angle_lsb & 0xFFFF,        # low 16 b
                              (angle_lsb >> 16) & 0xFFFF)

        msg = can.Message(arbitration_id=self._node_id,
                          data=bytes([0xA4]) + payload,
                          is_extended_id=False)
        try:
            self._bus.send(msg)
        except can.CanError as exc:  # pragma: no cover
            raise MotorCANError("Échec d’envoi trame CAN") from exc
