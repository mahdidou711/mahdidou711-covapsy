# Modules pour capteur ultrason SRF10 communicant en I2C

import smbus2
import threading
import time

# Variables partagées
sonar_data = {"arriere_cm": None}
sonar_lock = threading.Lock()


def sonar_thread_func(stop_event: threading.Event):
    """Boucle infinie de lecture du SRF10 et mise à jour thread-safe."""
    bus_number = 1
    address = 0x70  # Adresse I2C par défaut du SRF10 (configurable via commande dédiée).

    while not stop_event.is_set():
        try:
            # Ouverture du bus à chaque itération : permet la reconnexion automatique
            # après une OSError prolongée (ex: vibrations arrachant temporairement le bus).
            # Context manager garantit bus.close() même si une OSError est levée.
            with smbus2.SMBus(bus_number) as bus:

                # Déclencher une mesure en cm
                bus.write_byte_data(address, 0x00, 0x51)

                # Délai obligatoire de 70ms pour la réalisation de la mesure
                time.sleep(0.07)

                # Lecture du résultat sur 16 bits
                high = bus.read_byte_data(address, 0x02)
                low  = bus.read_byte_data(address, 0x03)

            distance_cm = (high << 8) | low

            # Mise à jour de la donnée protégée par Lock
            with sonar_lock:
                sonar_data["arriere_cm"] = distance_cm

            # Pause courte pour laisser respirer le bus entre deux mesures.
            time.sleep(0.01)

        except OSError:
            # Retourner None de façon silencieuse en cas d'erreur I2C
            with sonar_lock:
                sonar_data["arriere_cm"] = None
            time.sleep(0.07)


def get_sonar_arriere():
    """Accesseur thread-safe pour récupérer la dernière distance en cm."""
    with sonar_lock:
        return sonar_data["arriere_cm"]
