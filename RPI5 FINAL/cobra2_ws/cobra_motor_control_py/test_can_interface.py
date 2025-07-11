from can_interface import send_can_frame

# Exemple : envoyer une trame CAN (adapter l'ID et les données à ton moteur)
frame_id = "141"               # ou "0x141" selon ton convertisseur
data_hex = "A4001400881300"    # une trame valide de rotation ou mouvement

success = send_can_frame(frame_id, data_hex)

if success:
    print("✅ Trame envoyée avec succès")
else:
    print("❌ Échec de l'envoi")
