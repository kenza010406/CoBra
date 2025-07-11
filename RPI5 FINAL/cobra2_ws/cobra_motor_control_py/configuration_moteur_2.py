from cobra_motor_control_py.can_interface  import send_can_frame
frame_id="300"
data_hex="7900010000000002"
success = send_can_frame(frame_id, data_hex)
if success:
	print(":✅ Moteur 2 configuré avec ID = 2 (0x142)")
else:
	print(":❌ Echec de la configuration du Moteur 2")
