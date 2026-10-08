import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CommandPublisher(Node):
    def __init__(self):
        super().__init__('command_publisher')
        self.publisher_ = self.create_publisher(String, 'command', 10)

    def publish_command(self, command):
        """Publier le texte saisi, sans communiquer directement avec l'Arduino."""
        msg = String()
        msg.data = command
        self.publisher_.publish(msg)
        self.get_logger().info(f'Commande publiée : "{msg.data}"')

def main(args=None):
    # Initialiser ROS, puis créer notre nœud émetteur.
    rclpy.init(args=args)
    node = CommandPublisher()

    print('Commandes : start, stand, stop, status, quit')
    print('quit ferme cet émetteur uniquement, sans arrêter le robot.')

    try:
        while rclpy.ok():
            command = input('Command > ').strip()

            if command.lower() == 'quit':
                break

            # Une ligne vide ne doit pas produire de message.
            if not command:
                continue

            # Entrée déclenche un envoi : aucun timer n'est nécessaire.
            node.publish_command(command)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print('Émetteur fermé.')

if __name__ == '__main__':
    main()
