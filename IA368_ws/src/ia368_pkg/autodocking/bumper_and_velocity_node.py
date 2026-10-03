"""
Ponte (bridge) ROS 2 <-> CoppeliaSim para o bumper e para a velocidade do robô.

Este nó faz dois papéis, em sentidos opostos:

  1. Velocidade (ROS 2 -> CoppeliaSim)
     Recebe comandos Twist em /myRobot/cmd_vel (velocidade linear v e
     angular w do robô), converte em velocidades das duas rodas pela
     cinemática inversa do robô diferencial e aplica direto nas juntas
     das rodas com sim.setJointTargetVelocity().
     Para isso funcionar, o script /myRobot/python_controler da cena precisa
     da versão modificada (coppeliasim/python_controler.py), que não
     sobrescreve as juntas quando o joystick está parado.

  2. Bumper (CoppeliaSim -> ROS 2)
     Lê periodicamente o sensor de força /myRobot/forceSensor (o
     "para-choque" do robô) e publica a força/torque medidos em
     /myRobot/bumper como geometry_msgs/Wrench. Força alta = colisão.

Fluxo:
    /myRobot/cmd_vel --> [cmd_vel_callback] --> setJointTargetVelocity --> rodas
    forceSensor      --> [publish_bumper]   --> /myRobot/bumper
"""
import math

import rclpy
from rclpy.node import Node
# Twist: comando de velocidade (linear.x = v [m/s], angular.z = w [rad/s])
# Wrench: força (force.x/y/z [N]) + torque (torque.x/y/z [N.m])
from geometry_msgs.msg import Twist, Wrench
# Cliente da ZeroMQ Remote API: permite chamar as funções "sim.*" do
# CoppeliaSim a partir de um processo Python externo (este nó).
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


class BumperAndVelocityBridge(Node):
    def __init__(self):
        # Nome do nó no grafo ROS 2 (aparece em `ros2 node list`)
        super().__init__('bumper_and_velocity_bridge')

        # --- Parâmetros ROS 2 -------------------------------------------------
        # Podem ser alterados sem mexer no código, por exemplo:
        #   ros2 run ia368_pkg bumper_and_velocity_node --ros-args -p wheel_base:=0.2
        #
        # Caminho das juntas (motores) das rodas na cena. Vazio = o nó procura
        # sozinho, dentro de /myRobot, juntas com "left"/"right" no nome.
        self.declare_parameter('left_motor', '')
        self.declare_parameter('right_motor', '')
        # Distância entre as rodas (L) [m]. 0.0 = medir na própria cena a
        # distância entre as duas juntas.
        self.declare_parameter('wheel_base', 0.0)
        # Raio da roda (r) [m]: 0,05 m nesta cena (o mesmo valor usado pelo
        # python_controler e medido pelo script de odometria da cena).
        self.declare_parameter('wheel_radius', 0.05)
        # Sentido das juntas: nesta cena as juntas estão montadas de forma que
        # velocidade NEGATIVA faz o robô andar para a frente (o python_controler
        # também inverte o sinal). -1.0 = inverter; 1.0 = usar como está.
        self.declare_parameter('joint_direction', -1.0)
        # Por quanto tempo [s] o último cmd_vel também é reescrito nos sinais de
        # override <h>leftVel/<h>rightVel (ver cmd_vel_callback). 0 = desliga.
        self.declare_parameter('override_hold', 0.5)

        # --- Interface ROS 2 ------------------------------------------------
        # Publisher: envia as leituras do bumper. O "10" é o tamanho da fila
        # (QoS depth): quantas mensagens ficam guardadas se o assinante
        # estiver atrasado.
        self.bumper_publisher = self.create_publisher(Wrench, '/myRobot/bumper', 10)
        # Subscriber: toda vez que chegar um Twist em /myRobot/cmd_vel, o ROS 2
        # chama self.cmd_vel_callback(msg) automaticamente.
        self.subscription = self.create_subscription(Twist, '/myRobot/cmd_vel', self.cmd_vel_callback, 10)

        # --- Conexão com o CoppeliaSim ----------------------------------------
        try:
            # Conecta ao CoppeliaSim (por padrão localhost:23000). O CoppeliaSim
            # precisa estar aberto, com a cena carregada.
            self.client = RemoteAPIClient()
            # Objeto que expõe a API "sim" (sim.getObject, sim.readForceSensor...)
            self.sim = self.client.getObject('sim')
            # "Handles" são números inteiros que identificam objetos da cena.
            # Toda função sim.* recebe o handle do objeto em que vai agir.
            self.robotHandle = self.sim.getObject('/myRobot')
            self.forceSensorHandle = self.sim.getObject('/myRobot/forceSensor')
            self.leftMotor, self.rightMotor = self.find_wheel_joints()

            self.get_logger().info('Connected to CoppeliaSim successfully')
        except Exception as e:
            # Cena errada/fechada ou objeto inexistente: registra o erro e
            # sai do construtor sem criar o timer (o nó fica "vazio").
            self.get_logger().error(f'Failed to connect to CoppeliaSim: {e}')
            return

        # --- Geometria das rodas ----------------------------------------------
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.joint_direction = self.get_parameter('joint_direction').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.override_hold = self.get_parameter('override_hold').value
        h = str(self.robotHandle)
        self.left_signal, self.right_signal = h + 'leftVel', h + 'rightVel'
        # Último comando, em m/s NA RODA (a unidade dos sinais de override), e
        # o instante em que chegou.
        self.last_wheel_vel = None
        self.last_cmd_time = 0.0
        if self.wheel_base <= 0.0:
            # getObjectPosition(A, B) = posição de A no referencial de B.
            # A posição da junta esquerda vista da junta direita é o vetor que
            # liga as duas rodas; o comprimento dele é a distância entre elas (L).
            self.wheel_base = math.dist(self.sim.getObjectPosition(self.leftMotor, self.rightMotor), (0, 0, 0))
        # Mostra no log o que foi detectado, para conferir se faz sentido.
        # getObjectAlias(h, 1) = caminho completo do objeto na cena.
        self.get_logger().info(
            f'Wheel joints: {self.sim.getObjectAlias(self.leftMotor, 1)} / '
            f'{self.sim.getObjectAlias(self.rightMotor, 1)}, '
            f'wheel_base={self.wheel_base:.3f} m, wheel_radius={self.wheel_radius:.4f} m')

        # Dá "play" na simulação se ela estiver parada.
        if self.sim.getSimulationState() == self.sim.simulation_stopped:
            self.sim.startSimulation()

        # Timer: chama self.publish_bumper a cada 0,05 s (20 Hz). O bumper é
        # lido por "polling" porque o CoppeliaSim não avisa o ROS 2 sozinho.
        self.timer = self.create_timer(0.05, self.publish_bumper)

        self.get_logger().info('ROS2 ↔ CoppeliaSim Bumper and velocity bridge started.')

    def find_wheel_joints(self):
        """Devolve (handle da junta esquerda, handle da junta direita)."""
        # 1º caso: o usuário informou os caminhos pelos parâmetros.
        left_path = self.get_parameter('left_motor').value
        right_path = self.get_parameter('right_motor').value
        if left_path and right_path:
            return self.sim.getObject(left_path), self.sim.getObject(right_path)

        # 2º caso: busca automática. getObjectsInTree lista todos os objetos
        # abaixo de /myRobot na hierarquia, filtrando só juntas (joints).
        # O último argumento (0) inclui o próprio objeto base na busca.
        joints = self.sim.getObjectsInTree(self.robotHandle, self.sim.object_joint_type, 0)
        # Separa pelo nome: getObjectAlias(j) devolve o nome curto (ex.: "leftMotor").
        left = [j for j in joints if 'left' in self.sim.getObjectAlias(j).lower()]
        right = [j for j in joints if 'right' in self.sim.getObjectAlias(j).lower()]
        # Tem que achar exatamente uma de cada; senão é ambíguo (ou não existe)
        # e o usuário precisa dizer quais são pelos parâmetros.
        if len(left) != 1 or len(right) != 1:
            aliases = [self.sim.getObjectAlias(j, 1) for j in joints]
            raise RuntimeError(
                f'could not identify the wheel joints among {aliases}; '
                'set the left_motor and right_motor parameters')
        return left[0], right[0]

    def cmd_vel_callback(self, msg):
        """Chamado a cada Twist recebido: converte (v, w) em velocidade das rodas."""
        linVel = msg.linear.x   # v: velocidade para frente [m/s]
        rotVel = msg.angular.z  # w: velocidade de giro em torno do eixo vertical [rad/s]

        # Cinemática inversa do robô diferencial.
        # Ao girar com velocidade w, cada roda está a L/2 do centro, então
        # ganha (roda externa) ou perde (roda interna) uma velocidade L/2 * w
        # em relação ao centro do robô:
        #     velocidade linear da roda direita = v + (L/2) * w
        #     velocidade linear da roda esquerda = v - (L/2) * w
        # A junta recebe velocidade ANGULAR [rad/s], então divide pelo raio
        # (v_roda = r * phi_ponto  =>  phi_ponto = v_roda / r).
        # Ex.: w > 0 (girar para a esquerda) -> roda direita mais rápida.
        # Por fim, joint_direction corrige o sentido de montagem das juntas.
        rightVel = self.joint_direction * (linVel + self.wheel_base / 2 * rotVel) / self.wheel_radius
        leftVel = self.joint_direction * (linVel - self.wheel_base / 2 * rotVel) / self.wheel_radius
        self.last_wheel_vel = (linVel - self.wheel_base / 2 * rotVel,
                               linVel + self.wheel_base / 2 * rotVel)
        self.last_cmd_time = self.now()
        try:
            # Define a velocidade-alvo de cada motor; o motor da junta no
            # CoppeliaSim aplica torque para atingir e manter essa velocidade
            # até receber um novo valor. Ou seja: o robô continua andando com o
            # último cmd_vel; para pará-lo, publique um Twist zerado.
            self.sim.setJointTargetVelocity(self.leftMotor, leftVel)
            self.sim.setJointTargetVelocity(self.rightMotor, rightVel)
            self.write_override()
        except Exception as e:
            self.get_logger().error(f'Error setting wheel velocities: {e}')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def write_override(self):
        """Repete o último cmd_vel nos sinais <h>leftVel/<h>rightVel.

        Compatibilidade com a cena SEM o patch do python_controler: o script
        original escreve o valor do joystick (zero) nas juntas a CADA passo e
        apaga o setJointTargetVelocity acima — o robô não sai do lugar, embora
        o autodocking mude de estado. Esse mesmo script, porém, dá prioridade
        aos sinais de override (que ele lê e apaga a cada passo), então os
        reescrevemos enquanto o comando for recente. A versão modificada do
        python_controler também os aceita, então isto funciona nas duas.

        Só por `override_hold` segundos depois do último cmd_vel: se fosse para
        sempre, o joystick da cena nunca mais mandaria no robô.
        """
        if (self.override_hold <= 0.0 or self.last_wheel_vel is None
                or self.now() - self.last_cmd_time > self.override_hold):
            return
        left, right = self.last_wheel_vel
        self.sim.setFloatSignal(self.left_signal, left)
        self.sim.setFloatSignal(self.right_signal, right)

    def publish_bumper(self):
        """Chamado pelo timer (20 Hz): lê o sensor de força e publica em /myRobot/bumper."""
        try:
            self.write_override()
        except Exception as e:
            self.get_logger().error(f'Error writing velocity override: {e}')
        try:
            # readForceSensor devolve:
            #   result:       bits de estado (bit 0 = há dados válidos)
            #   forceVector:  [fx, fy, fz] em N, no referencial do sensor
            #   torqueVector: [tx, ty, tz] em N.m, no referencial do sensor
            # O sensor liga o para-choque ao corpo do robô: com o robô livre a
            # força fica perto de zero; numa batida o impacto passa pelo
            # sensor e a leitura sobe.
            result, forceVector, torqueVector = self.sim.readForceSensor(self.forceSensorHandle)
            # Empacota a leitura numa mensagem Wrench (força + torque).
            # float() porque os campos da mensagem ROS 2 exigem float.
            msg = Wrench()
            msg.force.x, msg.force.y, msg.force.z = (float(f) for f in forceVector)
            msg.torque.x, msg.torque.y, msg.torque.z = (float(t) for t in torqueVector)
            self.bumper_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error reading bumper: {e}')


def main(args=None):
    # Inicializa o ROS 2 neste processo.
    rclpy.init(args=args)
    node = BumperAndVelocityBridge()

    try:
        # spin() fica em loop processando eventos: mensagens recebidas
        # (cmd_vel_callback) e disparos do timer (publish_bumper).
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ctrl+C encerra o loop.
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
