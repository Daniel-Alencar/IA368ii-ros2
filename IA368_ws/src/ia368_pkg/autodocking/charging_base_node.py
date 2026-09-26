import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
# Coppelia ZeroMQ Remote API
from coppeliasim_zmqremoteapi_client import *

class ChargingBaseBridge(Node):
    """Publica o beacon IR da base de carga (intensidade e ângulo relativo).

    Protocolo da cena "Evaluation scene3.2_students.ttt" (script /chargingBase/beacon):
      1. o cliente escreve o sinal int "Beacon" = handle do robô (pedido);
      2. a cada passo, se o robô estiver no alcance do feixe, o script responde
         nos sinais float <h>StrengthSignal e <h>RelativeAngle;
      3. fora do alcance ele não escreve nada, então apagamos os sinais depois de
         ler, para não republicar um valor velho.

    A especificação cita os nomes <h>signalStrength/<h>relativeAngle; se a cena
    escrever esses, eles também são aceitos.
    """
    def __init__(self):
        super().__init__('charging_base_bridge')

        # Create publishers for charging base state
        self.strengthSignal_publisher = self.create_publisher(Float32,'/myRobot/charging_base/strengthSignal', 10)
        self.relativeAngle_publisher = self.create_publisher(Float32,'/myRobot/charging_base/relativeAngle', 10)

        # Connect to CoppeliaSim
        try:
            self.client = RemoteAPIClient()
            self.sim = self.client.getObject('sim')
            self.robotHandle = self.sim.getObject('/myRobot')

            self.get_logger().info('Connected to CoppeliaSim successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to CoppeliaSim: {e}')
            return

        # Nomes dos sinais: (usado pela cena, citado na especificação)
        h = str(self.robotHandle)
        self.strength_names = (h + "StrengthSignal", h + "signalStrength")
        self.angle_names = (h + "RelativeAngle", h + "relativeAngle")

        # Timer to publish charging base data every 10Hz
        self.timer = self.create_timer(0.1, self.publish_charging_base_sensor)

        self.get_logger().info('ROS2 → CoppeliaSim Charging base bridge started.')

    def read_and_clear(self, names):
        """Devolve o primeiro sinal float existente entre names (e o apaga), ou None."""
        for name in names:
            value = self.sim.getFloatSignal(name)
            if value is not None:
                self.sim.clearFloatSignal(name)
                return value
        return None

    def publish_charging_base_sensor(self):
        try:
            # Pede ao beacon a leitura para este robô
            self.sim.setInt32Signal("Beacon", self.robotHandle)

            # Sem sinal = robô fora do alcance do feixe: não publica nada
            strengthSignal = self.read_and_clear(self.strength_names)
            if strengthSignal is not None:
                strengthSignal_msg = Float32()
                strengthSignal_msg.data = float(strengthSignal)
                self.strengthSignal_publisher.publish(strengthSignal_msg)

            relativeAngle = self.read_and_clear(self.angle_names)
            if relativeAngle is not None:
                relativeAngle_msg = Float32()
                relativeAngle_msg.data = float(relativeAngle)
                self.relativeAngle_publisher.publish(relativeAngle_msg)

        except Exception as e:
            self.get_logger().error(f'Error reading charging base: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ChargingBaseBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
