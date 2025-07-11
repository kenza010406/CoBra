import subprocess
import glob

CANUSB_PATH = "/home/cobra/USB-CAN-A"  # ← adapte ce chemin si besoin

def find_usb_can():
    ports = glob.glob('/dev/ttyUSB*')
    return ports[0] if ports else None

def send_can_frame(frame_id: str, data_hex: str, bitrate=1000000):
    device = find_usb_can()
    if not device:
        print("❌ Aucun périphérique USB-CAN détecté.")
        return False

    cmd = [
        "./canusb",
        "-d", device,
        "-s", str(bitrate),
        "-i", frame_id,
        "-j", data_hex,
        "-n", "1"
    ]
    try:
        result = subprocess.run(cmd, cwd=CANUSB_PATH)
        return result.returncode == 0
    except Exception as e:
        print(f"Erreur CAN : {e}")
        return False
