import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

# Coppelia ZeroMQ Remote API
from coppeliasim_zmqremoteapi_client import *


class DockingSetter(Node):
    """Ponte do modo de docking, nos dois sentidos.

      /myRobot/docking_mode (Int32) --> sinal <h>Docking  (pedido do ROS 2)
      sinal <h>DockingRequest      --> /myRobot/docking_mode  (checkbox
                                       "docking" do joystick da cena)

    O segundo sentido só funciona com a versão modificada do python_controler
    (coppeliasim/python_controler.py, instalada com apply_scene_patch.py), que é
    quem escreve <h>DockingRequest quando o checkbox muda.
    """
    def __init__(self):
        super().__init__('docking_Setter')

        # Republica no tópico os cliques no checkbox "docking" da cena.
        self.docking_publisher = self.create_publisher(Int32, '/myRobot/docking_mode', 10)
        self.last_request = None
        self.last_written = None    # último valor escrito em <h>Docking

        # Subscribe to the docking mode topic
        self.subscription = self.create_subscription(
            Int32,
            '/myRobot/docking_mode',
            self.docking_mode_callback,
            10
        )
        
        # Connect to CoppeliaSim
        try:
            self.client = RemoteAPIClient()
            self.sim = self.client.getObject('sim')
            self.robotHandle = self.sim.getObject('/myRobot')
            
            self.get_logger().info('Connected to CoppeliaSim successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to CoppeliaSim: {e}')
            return

        self.request_signal = str(self.robotHandle) + "DockingRequest"
        self.timer = self.create_timer(0.1, self.poll_checkbox)

        self.get_logger().info('ROS2 → CoppeliaSim Docking setter started.')

    def poll_checkbox(self):
        """Publica em /myRobot/docking_mode quando o checkbox da cena muda."""
        try:
            request = self.sim.getInt32Signal(self.request_signal)
        except Exception as e:
            self.get_logger().error(f'Failed to read {self.request_signal}: {e}')
            return
        if request is None or request == self.last_request:
            return
        first = self.last_request is None
        self.last_request = request
        if first and request == 0:
            # Estado inicial desmarcado: nada a pedir.
            return
        self.get_logger().info(
            f'Checkbox "docking" da cena: {"marcado" if request else "desmarcado"}; '
            f'publicando docking_mode = {request}')
        self.docking_publisher.publish(Int32(data=1 if request else 0))

    def docking_mode_callback(self, msg):
        docking_state = msg.data
        # O autodocking republica o modo a cada 1 s. Só escrevemos na cena
        # quando ele MUDA: reescrever o mesmo 0 bem no instante de um clique no
        # checkbox desmarcaria o checkbox e engoliria o clique.
        if docking_state == self.last_written:
            return
        try:
            self.sim.setInt32Signal(str(self.robotHandle)+"Docking", docking_state)
            self.last_written = docking_state
            # O python_controler vai sincronizar o checkbox e o DockingRequest
            # com este valor; anotamos para não republicar o eco.
            self.last_request = 1 if docking_state else 0
            self.get_logger().info(f"Setting docking state to: {docking_state}")
        except Exception as e:
            self.get_logger().error(f'Failed to set docking state in CoppeliaSim: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = DockingSetter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()