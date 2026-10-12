import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import String

class CommandPublisher(Node):
    def __init__(self):
        super().__init__('command_publisher')
        # Publie les commandes texte sur le topic ROS command.
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
    print('quit demande stand à l’Arduino via le récepteur, puis ferme les deux nœuds ROS.')

    try:
        while rclpy.ok():
            # Attend une saisie clavier avant chaque envoi.
            command = input('Command > ').strip()

            if command.lower() == 'quit':
                node.publish_command('quit')
                # Attend la livraison ROS avant de détruire le publisher.
                if node.publisher_.get_subscription_count() == 0:
                    node.get_logger().warning('Aucun récepteur ROS connecté : fermeture distante non confirmée.')
                elif not node.publisher_.wait_for_all_acked(Duration(seconds=2)):
                    node.get_logger().warning('Livraison de quit non confirmée après 2 secondes.')
                break

            # Une ligne vide ne doit pas produire de message.
            if not command:
                continue

            # Entrée déclenche un envoi : aucun timer n'est nécessaire.
            node.publish_command(command)

    except KeyboardInterrupt:
        pass
    finally:
        # Libère le nœud et ferme ROS, même après Ctrl+C.
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print('Émetteur fermé.')

if __name__ == '__main__':
    main()
