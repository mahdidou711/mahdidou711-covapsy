"""Lecture du sonar arrière SRF10."""

import smbus2  # accès I2C au SRF10
import threading  # lock et stop_event
import time  # temporisations de mesure

sonar_data = {"arriere_cm": None}  # dernière distance arrière en cm
sonar_lock = threading.Lock()  # protège la donnée partagée


def sonar_thread_func(stop_event: threading.Event):
    """Boucle de mesure arrière."""
    bus_number = 1  # bus I2C principal du Raspberry Pi
    address = 0x70  # adresse I2C par défaut du SRF10

    while not stop_event.is_set():
        try:
            # Ouvre le bus uniquement le temps d'une mesure complète.
            with smbus2.SMBus(bus_number) as bus:  # ouverture du bus à chaque boucle
                # Nouvelle mesure en cm.
                bus.write_byte_data(address, 0x00, 0x51)  # commande "range in cm"
                time.sleep(0.07)  # délai de conversion SRF10
                high = bus.read_byte_data(address, 0x02)  # octet haut
                low = bus.read_byte_data(address, 0x03)  # octet bas

            # Recompose la distance 16 bits renvoyée par le capteur.
            distance_cm = (high << 8) | low  # reconstruction 16 bits en cm

            with sonar_lock:
                sonar_data["arriere_cm"] = distance_cm  # publication thread-safe

            time.sleep(0.01)  # petite pause entre deux mesures

        except Exception as e:
            print(f"[sonar] erreur thread : {type(e).__name__}: {e}")  # diagnostic simple
            with sonar_lock:
                sonar_data["arriere_cm"] = None  # donnée invalide si erreur bus
            time.sleep(0.07)  # évite une boucle d'erreur trop rapide


def get_sonar_arriere():
    """Retourne la dernière distance arrière."""
    with sonar_lock:
        return sonar_data["arriere_cm"]  # lecture thread-safe
