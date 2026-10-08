import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Diagnostic : laisser les blocs série commentés.
# Arduino : décommenter le code des blocs 1 à 5 (les deux parties du bloc 4).
# Fermer l'ancien programme série avant de lancer ce récepteur.
# Le thread de lecture reste daemon=True, comme dans le code d'origine.


# --- BLOC 1/5 : imports et paramètres du programme d'origine ---
import serial
import threading
import time
#
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200
# --- FIN BLOC 1/5 ---


# --- BLOC 2/5 : lecture des réponses, en parallèle de ROS ---
def read_from_arduino(ser):
    while True:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(f'\nArduino says: {line}')
        except Exception as e:
            print(f'\nRead error: {e}')
            break
# --- FIN BLOC 2/5 ---
# Seul l'affichage "Command > " a été retiré de la lecture d'origine :
# la saisie au clavier se trouve maintenant dans le programme émetteur.


class CommandReceiver(Node):
    VALID_COMMANDS = {'start', 'stand', 'stop', 'status'}

    def __init__(self):
        super().__init__('command_receiver')
        self.subscription = self.create_subscription(
            String, 'command', self.listener_callback, 10)

    def listener_callback(self, msg):
        command = msg.data.strip().lower()

        if command not in self.VALID_COMMANDS:
            self.get_logger().warning(
                f'Commande refusée : "{command}". '
                'Commandes autorisées : start, stand, stop, status.')
            return

        self.get_logger().info(f'Commande acceptée : "{command}"')

        # --- BLOC 3/5 : même envoi que dans le programme d'origine ---
        self.ser.write((command + '\n').encode('utf-8'))
        # --- FIN BLOC 3/5 ---


def main(args=None):
    rclpy.init(args=args)
    node = None

    # --- BLOC 4/5 : variables pour fermer la connexion même en cas d'erreur ---
    ser = None
    # --- FIN BLOC 4/5 (première partie) ---

    try:
        node = CommandReceiver()

        # --- BLOC 4/5 (suite) : ouverture et démarrage de la lecture ---
        try:
            ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        except Exception as e:
            print('Could not open serial port.')
            print(e)
            print('Check if CP2102 is connected and if the port is /dev/ttyUSB0.')
            return
        #
        time.sleep(2)
        node.ser = ser
        print('Connected to Arduino through CP2102.')
        #
        reader_thread = threading.Thread(
            target=read_from_arduino, args=(ser,), daemon=True)
        reader_thread.start()
        # --- FIN BLOC 4/5 ---

        print('Récepteur prêt : start, stand, stop, status. Ctrl+C pour quitter.')
        # ROS remplace la boucle input() : chaque message appelle la callback.
        # quit ferme seulement l'émetteur ; il n'est pas transmis ici.
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            # --- BLOC 5/5 : fermer le port, comme dans le programme d’origine ---
            if ser is not None:
                ser.close()
            # --- FIN BLOC 5/5 ---
            pass  # Garde le bloc try valide en mode diagnostic.
        finally:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
