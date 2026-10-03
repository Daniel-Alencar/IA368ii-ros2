"""Docking autônomo do /myRobot, feito INTEIRAMENTE fora do CoppeliaSim.

Diferente da atividade anterior (em que o comportamento morava num script da
cena), aqui a lógica roda como um nó ROS 2 comum. Este arquivo não importa
`coppeliasim_zmqremoteapi_client`, não conhece handles e não sabe o que é um
"sinal" do CoppeliaSim: ele só assina e publica tópicos. Toda a tradução é
trabalho da ponte (battery_node, charging_base_node, docking_node e
bumper_and_velocity_node), que já está pronta.

Interface ROS 2 usada (e só ela):

    ENTRADAS                                        SAÍDAS
    /myRobot/charging_base/strengthSignal  Float32   /myRobot/cmd_vel      Twist
    /myRobot/charging_base/relativeAngle   Float32   /myRobot/docking_mode Int32
    /myRobot/battery_state            BatteryState
    /myRobot/bumper                        Wrench

Máquina de estados:

      de qualquer estado, docking OFF  ──▶  IDLE
      de qualquer estado, carregando   ──▶  DOCKED

      ┌──────┐ docking ON  ┌────────┐   beacon    ┌──────────┐ carregando ┌────────┐
      │ IDLE │────────────▶│ SEARCH │────────────▶│ APPROACH │───────────▶│ DOCKED │
      └──────┘             └────────┘             └──────────┘            └────────┘
       teleop               gira no   ◀───────────   │     │               parado,
       livre                 lugar    beacon perdido │     │ beacon         na base
                               ▲         (e longe)   │     │ perdido
                               │                     │     ▼ (mas perto)
                     ┌─────────┴┐                    │  ┌───────┐
                     │  SWEEP   │◀──┐                │  │ FINAL │ empurrão às
                     └──────────┘   │                │  └───┬───┘ cegas
                      varre em      │   para-choque  │      │
                      ESPIRAL       │                ▼      ▼
                      (abre o raio) │            ┌─────────────┐ carregou ┌────────┐
                                    │            │   CONTACT   │─────────▶│ DOCKED │
                                    │            └─────────────┘          └────────┘
                                    │                   │ não carregou em
                                    │             ┌────────┐ contact_wait s
                                    └─────────────│ BACKUP │◀──────────────┘
                                       recua e    └────────┘
                                       inverte o lado da busca

Comportamento pedido no enunciado, estado por estado:

  - docking ligado e beacon detectado  -> APPROACH, usando o relativeAngle;
  - beacon perdido durante o docking   -> SEARCH, girando no lugar até reencontrá-lo;
  - robô carregando                    -> DOCKED, parado; quando o docking é
                                          desligado volta para IDLE e a
                                          teleoperação fica livre de novo.

Os estados SWEEP, FINAL, CONTACT e BACKUP não estão no enunciado; eles existem
porque sem eles o docking trava na prática, e os três últimos vêm de ler o
script da cena (coppeliasim/beacon.lua):

  - SWEEP: a detecção do beacon é posicional (o feixe da base tem que conter o
    dockingSensor), então girar no lugar quase não ajuda a reencontrá-lo — é
    preciso trocar de posição. A varredura é uma ESPIRAL de raio crescente, e
    não um arco de raio fixo, porque um arco de raio fixo é um círculo que
    refaz o próprio caminho em vez de explorar.
  - FINAL: perto da base, o feixe passa a detectar o para-choque em vez do
    dockingSensor, e o beacon se cala — o `getBeaconInfo` só responde se o
    objeto detectado for exatamente o dockingSensor. Perder o sinal assim
    significa "cheguei", não "me perdi": girar jogaria fora um docking pronto.
  - CONTACT: o para-choque dispara ~1 s antes de a carga aparecer (a bateria da
    cena é atualizada a 1 Hz), então o robô para e espera antes de desistir.
  - BACKUP: sai de um obstáculo (ou de um encaixe torto) e tenta de novo pelo
    outro lado, recuando um pouco mais a cada tentativa seguida.

Quatro detalhes desta cena que explicam decisões do código:

  1. **Em IDLE o nó não publica cmd_vel.** O bumper_and_velocity_node não tem
     prazo de validade: ele repassa o último comando e o robô fica com ele. Se
     este nó ficasse publicando Twist zerado em IDLE, a teleoperação (joystick
     da cena ou teleop_twist_keyboard) não conseguiria mover o robô. Então em
     IDLE ficamos calados, e mandamos um único Twist zerado ao ENTRAR em IDLE,
     para o robô não herdar o último comando do docking.

  2. **O beacon só fala quando o robô está dentro do feixe.** O
     charging_base_node não publica nada fora do feixe (em vez de publicar
     zero), então "perdi o beacon" aqui é medido por TEMPO: sem mensagem nova
     por `signal_timeout` segundos, consideramos o sinal perdido.

  3. **O ângulo é medido no referencial do dockingSensor, não do robô.** Nesta
     cena os dois coincidem — a frente do robô é o eixo +y dele, e o
     dockingSensor está no nariz com o +x apontando para lá —, então
     `relativeAngle` já é o erro de direção do robô e `angle_target = 0`, como
     diz o enunciado. Mas confira isso se trocar de robô: se o sensor estiver
     montado torto, o robô persegue um ponto ao lado da base e **perde o sinal
     justamente quando "alinha"**, porque girar esconde o sensor atrás do corpo.
     Ver `angle_target` e coppeliasim/beacon.lua.

  4. **O estado de carga do /myRobot/battery_state não é confiável.** O sinal
     `<h>Charging` da cena é apagado pelo script /myRobot/battery assim que ele
     o lê, e muitas vezes a ponte chega tarde. Por isso a carga é inferida
     também pela bateria SUBINDO (ver update_charging).

Execução:

    ros2 launch ia368_pkg autodocking.launch.py      # ponte + este nó
    ros2 run ia368_pkg autodocking_node              # só este nó

Para testar à mão, sem esperar a bateria baixar:

    ros2 topic pub --once /myRobot/docking_mode std_msgs/msg/Int32 "{data: 1}"
    ros2 topic pub --once /myRobot/docking_mode std_msgs/msg/Int32 "{data: 0}"
"""
import math
import time

import rclpy
from rclpy.node import Node
# Twist: comando de velocidade do robô (linear.x = v [m/s], angular.z = w [rad/s])
# Wrench: leitura do para-choque (força [N] + torque [N.m])
from geometry_msgs.msg import Twist, Wrench
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32, Int32


# Nomes dos estados. Strings (e não números) para o log sair legível.
IDLE = 'IDLE'          # docking desligado: não comandamos nada, teleop livre
SEARCH = 'SEARCH'      # girando no lugar à procura do feixe do beacon
SWEEP = 'SWEEP'        # girar não achou: varre a vizinhança em espiral
APPROACH = 'APPROACH'  # beacon na mão: alinha pelo ângulo e avança
FINAL = 'FINAL'        # perdeu o beacon encostando na base: empurra em frente
CONTACT = 'CONTACT'    # encostou em algo com a base perto: para e espera a carga
BACKUP = 'BACKUP'      # bateu em algo sem carregar: recua e tenta de novo
DOCKED = 'DOCKED'      # na base, carregando: parado


def wrap_to_pi(angle):
    """Normaliza um ângulo para o intervalo -pi..pi.

    Necessário porque o erro de alinhamento é uma diferença de ângulos: sem
    normalizar, girar 350° "para a esquerda" pareceria melhor que girar 10°
    para a direita. O truque do atan2(sin, cos) faz isso sem ifs.
    """
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, limit):
    """Satura `value` em -limit..+limit."""
    return max(-limit, min(limit, value))


class AutoDocking(Node):
    def __init__(self):
        # Nome do nó no grafo ROS 2 (aparece em `ros2 node list`)
        super().__init__('autodocking')

        # =====================================================================
        #  Parâmetros
        # =====================================================================
        # Todos ajustáveis sem recompilar, por exemplo:
        #   ros2 run ia368_pkg autodocking_node --ros-args -p battery_low:=50.0

        # --- Ritmo -----------------------------------------------------------
        # 10 Hz. Não aumente muito: a ponte só conversa com o CoppeliaSim ~20
        # vezes por segundo (cada chamada da Remote API custa ~12 ms com a
        # simulação rodando), então publicar mais rápido só forma fila.
        self.declare_parameter('control_rate', 10.0)

        # --- Quando ligar e desligar o docking sozinho -----------------------
        # A bateria desta cena gasta 1 % por segundo simulado e começa em 100 %:
        # são ~100 s até o robô parar.
        self.declare_parameter('auto_dock_on_low_battery', True)
        # 95 %: o docking liga ~5 s depois do início da simulação, e o robô tem
        # ~95 s para achar a base. Com a bateria gastando 1 % por segundo, a
        # BUSCA do feixe (que pode passar de 45 s, conforme a posição inicial e
        # os obstáculos) é o que mais consome, então quanto antes começar,
        # melhor.
        self.declare_parameter('battery_low', 95.0)    # % para ligar o docking
        self.declare_parameter('release_when_full', True)
        # 100 %, e não 95 %: battery_low precisa ficar ABAIXO de battery_full
        # (senão o modo oscila), e a bateria da cena satura em exatamente 100.
        self.declare_parameter('battery_full', 100.0)  # % para desligar o docking

        # --- Beacon ----------------------------------------------------------
        # Sem mensagem nova por este tempo, consideramos o beacon perdido (o
        # charging_base_node simplesmente para de publicar fora do feixe).
        self.declare_parameter('signal_timeout', 0.5)  # s
        # Sem /myRobot/battery_state por este tempo, a ponte caiu (ou a
        # simulação parou): paramos o robô em vez de seguir com o último
        # comando, que a ponte manteria nas juntas indefinidamente.
        self.declare_parameter('data_timeout', 2.0)    # s
        # Valor de relativeAngle que significa "base exatamente à frente do
        # robô". O /chargingBase/beacon calcula
        #
        #     relativeAngle = atan2(base - sensor) - orientação DO SENSOR
        #
        # usando a pose de /myRobot/dockingSensor, e não a do robô. Vale a pena
        # conferir que os dois coincidem, porque é fácil errar aqui. Medido
        # nesta cena:
        #
        #   * a FRENTE do robô é o eixo +y dele (mandando as duas rodas para
        #     frente, o deslocamento é +0,373 m em y e +0,002 m em x);
        #   * o dockingSensor está em (0, +0,09) m — 9 cm à frente, no nariz —
        #     com yaw +90°, ou seja o +x DELE aponta para o +y do robô.
        #
        # Logo o "à frente" do sensor é o "à frente" do robô, e relativeAngle já
        # É o erro de direção do robô: angle_target = 0, como diz o enunciado.
        #
        # Se trocar de robô ou de cena, meça de novo: angle_target = -(yaw do
        # dockingSensor em relação à frente do robô).
        self.declare_parameter('angle_target', 0.0)  # rad
        # Sentido do ângulo. Se o robô girar PARA LONGE da base em vez de ir
        # na direção dela, troque para -1.0.
        self.declare_parameter('angle_sign', 1.0)
        # Acima desta intensidade estamos encostando na base: anda devagar
        # para encaixar sem empurrá-la. A escala vem do beacon da cena:
        #
        #     signalStrength = 1 - distância / volume_range,  volume_range = 2 m
        #
        # então 0,85 = 0,30 m da base. Deixe alto: a bateria gasta 1 % por
        # segundo, e rastejar longe da base custa carga à toa (com 0,6 = 0,80 m
        # medimos 15 s de rastejo e 15 % de bateria em um teste).
        self.declare_parameter('strength_slow', 0.85)

        # --- Ganhos e velocidades --------------------------------------------
        # Limites da cena, de referência: 0,5 m/s e 90 deg/s (1,57 rad/s).
        self.declare_parameter('k_angle', 1.5)          # ganho P do alinhamento
        self.declare_parameter('max_angular', 1.0)      # rad/s
        self.declare_parameter('approach_speed', 0.25)  # m/s indo para a base
        self.declare_parameter('final_speed', 0.05)     # m/s já perto da base
        # Erro de alinhamento acima do qual o robô gira PARADO antes de avançar.
        self.declare_parameter('align_threshold', 0.35)  # rad (~20°)

        # --- Busca -----------------------------------------------------------
        # Girar no lugar é o que o enunciado pede quando o sinal é perdido, e é
        # a resposta certa quando o robô só está mal apontado. Mas nesta cena a
        # detecção do beacon é POSICIONAL (o feixe da base tem que conter o
        # dockingSensor), então girar quase não ajuda a reencontrá-lo: mantemos
        # a volta curta e barata, e a busca de verdade é o SWEEP.
        self.declare_parameter('search_angular_speed', 1.0)  # rad/s girando
        self.declare_parameter('search_spin_time', 3.0)      # s
        # SWEEP: varredura em ESPIRAL DE ARQUIMEDES. O feixe da base é um
        # corredor estreito (medido nesta cena: uma única direção, mais estreita
        # que +-30°, até ~1,2-1,5 m), e para entrar nele o robô precisa trocar
        # de POSIÇÃO.
        #
        # Por que espiral e não arco de raio fixo: um arco de raio fixo é um
        # CÍRCULO — ele volta ao ponto de partida e refaz o mesmo caminho, então
        # não explora, oscila. A espiral abre o raio a cada volta.
        #
        # Por que de ARQUIMEDES: o espaçamento entre duas voltas vizinhas é
        # constante (sweep_spacing). A versão anterior fazia o raio crescer a
        # uma taxa constante NO TEMPO (r = r0 + k*t); como cada volta demora
        # 2*pi*r/v, o raio crescia ~7x por volta (era uma espiral LOGARÍTMICA):
        # o robô dava ~1 volta, já passava de sweep_max_radius e recomeçava de
        # outro ponto — na prática, laços soltos sem cobrir a vizinhança. Para o
        # espaçamento por volta ser constante, dr/dtheta = spacing / (2*pi), e
        # com dtheta/dt = v/r:
        #
        #     dr/dt = sweep_spacing * v / (2*pi*r)
        #     w     = v / r                        (limitado a sweep_angular_max)
        #
        # Começa apertada em volta do robô (r0 = v / w_max) e vai abrindo. Ao
        # passar de sweep_max_radius a espiral recomeça, invertendo o lado.
        #
        # Por que a espiral acha o corredor sem precisar cobrir área: o corredor
        # sai RADIALMENTE da base, então qualquer laço que CIRCUNDE a base o
        # cruza. Como a espiral abre o raio, em algum momento ela passa a
        # circundar a base — e aí acha. O espaçamento só precisa ser menor que
        # o alcance do feixe (~1,2 m) para nenhuma volta "pular" o corredor.
        #
        # Tempo para chegar ao raio R: t = pi * (R² - r0²) / (spacing * v).
        # Com os valores abaixo: ~55 s e ~2,7 voltas até 1,5 m (a 1 % de bateria
        # por segundo, por isso o docking liga cedo, em battery_low = 95 %).
        self.declare_parameter('sweep_speed', 0.25)          # m/s
        self.declare_parameter('sweep_angular_max', 1.0)     # rad/s (giro mais fechado)
        self.declare_parameter('sweep_spacing', 0.5)         # m entre voltas vizinhas
        self.declare_parameter('sweep_max_radius', 1.5)      # m, quando recomeça

        # --- Para-choque -----------------------------------------------------
        # O sensor de força mede o peso do para-choque mesmo com o robô livre,
        # então a referência de "parado" é medida no início (baseline) e o
        # limiar vale para o DESVIO em relação a ela.
        self.declare_parameter('bumper_threshold', 1.0)      # N acima da baseline
        self.declare_parameter('bumper_calibration_samples', 20)
        # Ao encostar na base, o para-choque dispara na hora, mas a carga só
        # aparece até ~1 s depois (a bateria da cena é atualizada a 1 Hz). Sem
        # esta espera o robô recuaria de um encaixe que deu certo.
        # Chegando perto, o feixe da base passa a detectar o para-choque em
        # vez do dockingSensor e o beacon PARA de responder (o
        # /chargingBase/beacon só responde se o objeto detectado for exatamente
        # o dockingSensor). Perder o sinal assim não é "me perdi", é "cheguei":
        # em vez de girar, o robô empurra em frente por este tempo.
        self.declare_parameter('final_push_time', 3.0)       # s
        self.declare_parameter('contact_wait', 3.0)          # s
        self.declare_parameter('backup_speed', 0.12)         # m/s de ré
        self.declare_parameter('backup_turn', 0.5)           # rad/s ao recuar
        self.declare_parameter('backup_time', 1.5)           # s
        # Cada tentativa frustrada recua mais que a anterior (x1, x1,5, x2...),
        # para o robô contornar o obstáculo em vez de voltar a bater no mesmo
        # ponto. Limitado a `backup_escalation_max` tentativas.
        self.declare_parameter('backup_escalation', 0.5)
        self.declare_parameter('backup_escalation_max', 4)

        # Lê tudo de uma vez para atributos (parâmetros são lidos uma só vez;
        # mudá-los em execução pede um `add_on_set_parameters_callback`).
        p = lambda name: self.get_parameter(name).value
        self.control_rate = p('control_rate')
        self.auto_dock = p('auto_dock_on_low_battery')
        self.battery_low = p('battery_low')
        self.release_when_full = p('release_when_full')
        self.battery_full = p('battery_full')
        self.signal_timeout = p('signal_timeout')
        self.data_timeout = p('data_timeout')
        self.angle_target = p('angle_target')
        self.angle_sign = p('angle_sign')
        self.strength_slow = p('strength_slow')
        self.k_angle = p('k_angle')
        self.max_angular = p('max_angular')
        self.approach_speed = p('approach_speed')
        self.final_speed = p('final_speed')
        self.align_threshold = p('align_threshold')
        self.search_angular_speed = p('search_angular_speed')
        self.search_spin_time = p('search_spin_time')
        self.sweep_speed = p('sweep_speed')
        self.sweep_angular_max = p('sweep_angular_max')
        self.sweep_spacing = p('sweep_spacing')
        self.sweep_max_radius = p('sweep_max_radius')
        self.bumper_threshold = p('bumper_threshold')
        self.bumper_calibration_samples = p('bumper_calibration_samples')
        self.final_push_time = p('final_push_time')
        self.contact_wait = p('contact_wait')
        self.backup_speed = p('backup_speed')
        self.backup_turn = p('backup_turn')
        self.backup_time = p('backup_time')
        self.backup_escalation = p('backup_escalation')
        self.backup_escalation_max = p('backup_escalation_max')

        # =====================================================================
        #  Interface ROS 2
        # =====================================================================
        # O "10" de cada chamada é a profundidade da fila (QoS depth).
        self.cmd_publisher = self.create_publisher(Twist, '/myRobot/cmd_vel', 10)
        self.docking_publisher = self.create_publisher(Int32, '/myRobot/docking_mode', 10)

        self.create_subscription(
            Float32, '/myRobot/charging_base/strengthSignal', self.on_strength, 10)
        self.create_subscription(
            Float32, '/myRobot/charging_base/relativeAngle', self.on_angle, 10)
        self.create_subscription(
            BatteryState, '/myRobot/battery_state', self.on_battery, 10)
        self.create_subscription(Wrench, '/myRobot/bumper', self.on_bumper, 10)
        # Assinamos TAMBÉM o tópico em que publicamos, para que o modo de
        # docking possa ser ligado/desligado à mão com `ros2 topic pub`. Receber
        # a própria mensagem é inofensivo: o callback só copia o valor que já
        # está no estado interno.
        self.create_subscription(Int32, '/myRobot/docking_mode', self.on_docking_mode, 10)

        # =====================================================================
        #  Estado observado (os callbacks só guardam; quem decide é o timer)
        # =====================================================================
        self.battery = None             # % (0 a 100), None até a 1ª mensagem
        self.last_battery = None        # amostra anterior, para ver se subiu
        self.battery_status = None      # power_supply_status da mensagem
        self.battery_time = 0.0         # monotonic da última mensagem de bateria
        self.data_lost_warned = False
        self.strength = 0.0             # intensidade do beacon
        self.relative_angle = None      # rad; None enquanto nunca vimos o beacon
        self.beacon_time = 0.0          # monotonic da última leitura do beacon
        self.charging_time = 0.0        # monotonic da última evidência de carga
        self.charging = False
        # Para-choque: a baseline é a média das primeiras leituras.
        self.bumper_samples = []
        self.bumper_baseline = None
        self.bumper_force = 0.0         # desvio atual em relação à baseline [N]

        # =====================================================================
        #  Estado interno da máquina de estados
        # =====================================================================
        self.docking_mode = False       # espelho do que foi publicado/recebido
        self.auto_dock_armed = True     # evita religar o docking que o usuário desligou
        self.state = IDLE
        self.state_time = time.monotonic()
        # Sentido do giro na busca: +1 = esquerda. Começa para a esquerda e
        # passa a apontar para o lado onde o beacon foi visto por último.
        self.search_direction = 1.0
        # Raio atual da espiral do SWEEP [m]. None = começar uma espiral nova.
        # Fica guardado através do BACKUP, para uma batida no meio da varredura
        # não jogar fora o raio já aberto.
        self.sweep_r = None
        self.last_step_time = time.monotonic()
        self.dt = 1.0 / self.control_rate
        # Quantas vezes encostamos em algo sem conseguir carregar, seguidas.
        self.contact_failures = 0
        self.last_docking_publish = 0.0
        self.last_status_log = 0.0

        # Guarda contra uma configuração que se auto-anula: com
        # battery_low >= battery_full o mesmo nível de bateria pede para LIGAR
        # (está abaixo de low) e para DESLIGAR (está acima de full) o docking, e
        # o modo oscilaria a cada ciclo do laço.
        if self.release_when_full and self.battery_low >= self.battery_full:
            corrected = self.battery_full - 5.0
            self.get_logger().error(
                f'battery_low ({self.battery_low:.0f} %) >= battery_full '
                f'({self.battery_full:.0f} %) faria o modo de docking oscilar; '
                f'usando battery_low = {corrected:.0f} %.')
            self.battery_low = corrected

        self.timer = self.create_timer(1.0 / self.control_rate, self.step)

        self.get_logger().info(
            'Autodocking iniciado. '
            f'Liga o docking abaixo de {self.battery_low:.0f} % de bateria '
            f'({"ativo" if self.auto_dock else "desativado"}); '
            f'desliga acima de {self.battery_full:.0f} % '
            f'({"ativo" if self.release_when_full else "desativado"}).')

    # =========================================================================
    #  Callbacks: guardam o que chegou, sem decidir nada
    # =========================================================================

    def on_strength(self, msg):
        """Intensidade do beacon IR: quanto maior, mais perto da base."""
        self.strength = msg.data
        self.beacon_time = time.monotonic()

    def on_angle(self, msg):
        """Direção da base em relação ao robô, em rad (0 = base à frente)."""
        self.relative_angle = msg.data
        self.beacon_time = time.monotonic()

    def on_battery(self, msg):
        """Nível da bateria.

        A ponte (battery_node) preenche `percentage` em 0..100, mas a convenção
        da mensagem BatteryState do ROS 2 é 0..1. Aceitamos as duas: valor acima
        de 1 já está em porcentagem; abaixo, multiplicamos por 100. (O caso
        ambíguo é uma bateria realmente abaixo de 1 %, perto do fim da vida —
        irrelevante aqui, porque o docking liga muito antes disso.)
        """
        percentage = msg.percentage
        self.battery = percentage if percentage > 1.0 else percentage * 100.0
        self.battery_status = msg.power_supply_status
        self.battery_time = time.monotonic()

    def on_bumper(self, msg):
        """Para-choque: guarda o DESVIO da força em relação à baseline.

        As primeiras `bumper_calibration_samples` leituras (robô paradinho no
        início) formam a baseline, porque o sensor de força sustenta o
        para-choque e mede o peso dele mesmo sem colisão nenhuma.
        """
        magnitude = math.sqrt(msg.force.x ** 2 + msg.force.y ** 2 + msg.force.z ** 2)

        if self.bumper_baseline is None:
            self.bumper_samples.append(magnitude)
            if len(self.bumper_samples) >= self.bumper_calibration_samples:
                self.bumper_baseline = sum(self.bumper_samples) / len(self.bumper_samples)
                self.get_logger().info(
                    f'Para-choque calibrado: baseline {self.bumper_baseline:.2f} N, '
                    f'colisão acima de {self.bumper_threshold:.2f} N de desvio.')
            return

        self.bumper_force = abs(magnitude - self.bumper_baseline)

    def on_docking_mode(self, msg):
        """Modo de docking vindo do tópico (nosso ou de um `ros2 topic pub`)."""
        requested = msg.data != 0
        if requested != self.docking_mode:
            self.docking_mode = requested
            self.get_logger().info(
                f'Modo de docking: {"LIGADO" if requested else "DESLIGADO"} (pelo tópico)')
            if not requested:
                # Usuário desligou: não religamos por bateria baixa até ela
                # subir de novo (senão o nó brigaria com o comando dele).
                self.auto_dock_armed = False

    # =========================================================================
    #  Leituras derivadas
    # =========================================================================

    def signal_detected(self):
        """True se o beacon foi lido há menos de `signal_timeout` segundos.

        Fora do feixe o charging_base_node não publica nada, então a ausência
        de mensagens — e não um valor zero — é o que indica sinal perdido.
        """
        return (self.relative_angle is not None
                and time.monotonic() - self.beacon_time <= self.signal_timeout)

    def angle_error(self):
        """Erro de alinhamento: 0 = apontando para a base, >0 = girar à esquerda."""
        return self.angle_sign * wrap_to_pi(self.relative_angle - self.angle_target)

    def bumper_hit(self):
        """True se o para-choque está sendo pressionado (já calibrado)."""
        return self.bumper_baseline is not None and self.bumper_force > self.bumper_threshold

    def update_charging(self):
        """Decide se o robô está carregando, juntando duas evidências.

        1. `power_supply_status` da mensagem: é a evidência direta, mas chega
           só às vezes, porque o script /myRobot/battery apaga o sinal
           `<h>Charging` da cena antes da ponte conseguir lê-lo.
        2. A bateria SUBINDO: se o nível cresceu de uma amostra para a outra,
           está carregando — não há outra forma de a bateria subir nesta cena.

        A bateria da cena é atualizada a 1 Hz e este laço roda a 10 Hz, então a
        evidência 2 aparece em 1 de cada 10 ciclos; por isso ela é mantida por
        `hold` segundos. Já a bateria CAINDO desliga a carga na hora.
        """
        now = time.monotonic()
        hold = 3.0  # s de validade de uma evidência de carga

        if self.battery_status in (BatteryState.POWER_SUPPLY_STATUS_CHARGING,
                                   BatteryState.POWER_SUPPLY_STATUS_FULL):
            self.charging_time = now

        if self.battery is not None and self.last_battery is not None:
            if self.battery > self.last_battery:
                self.charging_time = now
            elif self.battery < self.last_battery:
                self.charging_time = 0.0  # descarregando: decide na hora
        self.last_battery = self.battery

        self.charging = (now - self.charging_time) <= hold

    # =========================================================================
    #  Modo de docking
    # =========================================================================

    def update_docking_mode(self):
        """Liga o docking com a bateria baixa e desliga com ela cheia."""
        if self.battery is None:
            return

        # Re-arma o gatilho quando a bateria volta a um nível confortável, para
        # que o próximo ciclo de descarga ligue o docking de novo.
        if self.battery > self.battery_low:
            self.auto_dock_armed = True

        if (self.auto_dock and self.auto_dock_armed
                and not self.docking_mode and self.battery <= self.battery_low):
            self.get_logger().warn(
                f'Bateria em {self.battery:.0f} %: ligando o modo de docking.')
            self.set_docking_mode(True)

        elif (self.release_when_full and self.docking_mode
                and self.battery >= self.battery_full):
            self.get_logger().info(
                f'Bateria em {self.battery:.0f} %: desligando o modo de docking.')
            self.set_docking_mode(False)

    def set_docking_mode(self, enabled):
        """Muda o modo e anuncia no tópico (o docking_node leva para a cena)."""
        self.docking_mode = enabled
        self.publish_docking_mode()

    def publish_docking_mode(self):
        """Publica o modo atual em /myRobot/docking_mode."""
        self.docking_publisher.publish(Int32(data=1 if self.docking_mode else 0))
        self.last_docking_publish = time.monotonic()

    # =========================================================================
    #  Máquina de estados
    # =========================================================================

    def enter(self, state):
        """Troca de estado, zerando o cronômetro do estado e registrando no log."""
        if state == self.state:
            return
        self.get_logger().info(f'{self.state} -> {state}')
        self.state = state
        self.state_time = time.monotonic()

    def state_elapsed(self):
        """Segundos desde a entrada no estado atual."""
        return time.monotonic() - self.state_time

    def data_lost(self):
        """True se a ponte parou de publicar a bateria.

        É o nosso "sinal de vida" da ponte: ela publica a bateria a 10 Hz sem
        parar, enquanto o beacon só aparece dentro do feixe. Vale também antes
        da primeira mensagem (battery_time = 0), para o nó não sair comandando
        o robô enquanto a ponte ainda não subiu.
        """
        return time.monotonic() - self.battery_time > self.data_timeout

    def step(self):
        """Chamado pelo timer (10 Hz): observa, decide e comanda."""
        now = time.monotonic()
        # dt real do laço (limitado, para uma pausa longa não dar um salto).
        self.dt = min(now - self.last_step_time, 0.5)
        self.last_step_time = now

        # Ponte muda ou simulação parada: para o robô e não decide nada. Sem
        # isto o robô sairia com o último comando para sempre, porque o
        # bumper_and_velocity_node não tem prazo de validade.
        if self.data_lost():
            if not self.data_lost_warned and self.battery_time > 0.0:
                self.get_logger().warn(
                    'Sem /myRobot/battery_state: a ponte caiu ou a simulação parou. '
                    'Parando o robô e soltando o controle.')
                self.data_lost_warned = True
            elif self.battery_time == 0.0 and now - self.last_status_log >= 5.0:
                # Nunca recebemos a bateria: sem ela o docking não liga sozinho.
                self.last_status_log = now
                self.get_logger().warn(
                    'Aguardando /myRobot/battery_state: o docking só liga depois '
                    'da primeira leitura de bateria. O battery_node está rodando '
                    'e a simulação está em execução?')
            # IDLE manda o Twist zerado e depois se cala, que é o que queremos:
            # para o robô sem travar a teleoperação, caso só este caminho tenha
            # caído.
            self.enter(IDLE)
            cmd = self.run_idle()
            if cmd is not None:
                self.cmd_publisher.publish(cmd)
            return
        self.data_lost_warned = False

        self.update_charging()
        self.update_docking_mode()

        # Republica o modo de docking de vez em quando, para que ele seja
        # visível em `ros2 topic echo` a qualquer momento.
        if time.monotonic() - self.last_docking_publish >= 1.0:
            self.publish_docking_mode()

        # Transições que valem em qualquer estado ativo.
        if not self.docking_mode:
            # Docking desligado: largamos o controle do robô. A transição para
            # IDLE manda um Twist zerado (ver run_idle), e depois ficamos
            # calados para não atropelar a teleoperação.
            self.enter(IDLE)
        elif self.charging:
            # Chegou na base: o enunciado pede parar e esperar.
            self.enter(DOCKED)
        elif self.state == DOCKED and self.battery is not None and self.battery >= 99.5:
            # Na base com a bateria cheia: ela satura em 100 e para de subir,
            # então a evidência de carga "bateria subindo" some. Isso NÃO quer
            # dizer que saímos da base; continuamos em DOCKED.
            pass
        elif self.state in (IDLE, DOCKED):
            # Docking ligado (e não carregando): começa a procurar. Se o beacon
            # já estiver visível, vai direto para a aproximação.
            self.sweep_r = None  # busca nova: espiral nova
            self.enter(APPROACH if self.signal_detected() else SEARCH)

        cmd = {
            IDLE: self.run_idle,
            SEARCH: self.run_search,
            SWEEP: self.run_sweep,
            APPROACH: self.run_approach,
            FINAL: self.run_final,
            CONTACT: self.run_contact,
            BACKUP: self.run_backup,
            DOCKED: self.run_docked,
        }[self.state]()

        # cmd é None quando o estado não quer comandar o robô (só em IDLE).
        if cmd is not None:
            self.cmd_publisher.publish(cmd)

        self.log_status()

    def run_idle(self):
        """Docking desligado: teleoperação livre.

        Publica UM Twist zerado ao entrar no estado (para o robô não continuar
        com o último comando do docking, já que a ponte não tem prazo de
        validade) e depois fica calado, para não brigar com o joystick da cena
        nem com o teleop_twist_keyboard.
        """
        if self.state_elapsed() < 1.0 / self.control_rate * 3:
            return Twist()
        return None

    def run_search(self):
        """Gira no lugar à procura do feixe do beacon.

        É o comportamento que o enunciado pede quando o sinal é perdido. Gira
        para o lado onde o beacon foi visto por último (search_direction).
        """
        if self.signal_detected():
            self.enter(APPROACH)
            return self.run_approach()

        if self.state_elapsed() > self.search_spin_time:
            # A volta no lugar não achou: o robô está fora do corredor do feixe.
            # Trocar de posição é a única saída -> varredura em espiral.
            self.enter(SWEEP)
            return self.run_sweep()

        cmd = Twist()
        cmd.angular.z = self.search_angular_speed * self.search_direction
        return cmd

    def sweep_r0(self):
        """Raio inicial da espiral: o giro mais fechado permitido."""
        return self.sweep_speed / self.sweep_angular_max

    def run_sweep(self):
        """Varre a vizinhança em ESPIRAL, procurando o feixe da base.

        Esta é a busca de verdade. Girar no lugar não reencontra o feixe porque
        a detecção é posicional: o feixe da base tem que conter o dockingSensor,
        e girar só faz o sensor descrever um circulozinho de 9 cm. Um arco de
        raio fixo também não serve — é um círculo, que refaz o próprio caminho.
        A espiral abre o raio a cada volta, cobrindo área nova continuamente.
        """
        if self.signal_detected():
            self.enter(APPROACH)
            return self.run_approach()

        if self.bumper_hit():
            self.enter(BACKUP)
            return self.run_backup()

        if self.sweep_r is None:
            self.sweep_r = self.sweep_r0()

        if self.sweep_r > self.sweep_max_radius:
            # Espiral esgotada: recomeça do raio pequeno, pelo outro lado, com
            # uma olhada girando no lugar no meio.
            self.sweep_r = None
            self.search_direction = -self.search_direction
            self.enter(SEARCH)
            return self.run_search()

        radius = self.sweep_r
        cmd = Twist()
        cmd.linear.x = self.sweep_speed
        cmd.angular.z = min(self.sweep_angular_max,
                            self.sweep_speed / radius) * self.search_direction

        # Espiral de Arquimedes: o raio cresce sweep_spacing por volta.
        self.sweep_r += self.sweep_spacing * self.sweep_speed / (2.0 * math.pi * radius) * self.dt
        return cmd

    def run_approach(self):
        """Vai até a base usando o relativeAngle (e a intensidade como distância).

        Controle proporcional no ângulo. Com o robô muito desalinhado, gira
        PARADO (v = 0): avançar torto só afasta do feixe. Alinhado, a velocidade
        cai com o cosseno do erro e, perto da base (intensidade alta), é limitada
        a `final_speed` para encaixar devagar.
        """
        # O para-choque é testado ANTES da perda de sinal: encostando na base
        # as duas coisas acontecem juntas, e o contato é a informação melhor.
        if self.bumper_hit():
            # Encostou em algo enquanto seguia o beacon. Pode ser a própria
            # base (encaixe!) ou um obstáculo no caminho: para e espera para
            # descobrir, em vez de recuar de um docking que deu certo.
            self.enter(CONTACT)
            return self.run_contact()

        if not self.signal_detected():
            if self.strength >= self.strength_slow:
                # Perdeu o sinal MUITO perto da base: é o para-choque tapando o
                # dockingSensor, não o robô se perdendo. Empurra em frente.
                self.enter(FINAL)
                return self.run_final()
            # Sinal perdido longe da base: a próxima varredura começa do zero,
            # centrada aqui (onde o feixe acabou de ser visto).
            self.sweep_r = None
            # Guarda para que lado a base estava, para girar para o lado certo.
            if self.relative_angle is not None:
                error = self.angle_error()
                self.search_direction = 1.0 if error >= 0.0 else -1.0
            self.enter(SEARCH)
            return self.run_search()

        error = self.angle_error()
        cmd = Twist()
        cmd.angular.z = clamp(self.k_angle * error, self.max_angular)

        if abs(error) > self.align_threshold:
            cmd.linear.x = 0.0                      # gira parado até alinhar
        else:
            cmd.linear.x = self.approach_speed * math.cos(error)
            if self.strength >= self.strength_slow:
                cmd.linear.x = min(cmd.linear.x, self.final_speed)
        return cmd

    def run_final(self):
        """Empurrão final às cegas, quando o beacon se apaga junto à base.

        Perto do encaixe o feixe da base passa a ver o para-choque em vez do
        dockingSensor, e o beacon para de responder. Girar aqui seria jogar
        fora um docking quase pronto, então o robô segue em frente devagar até
        encostar (-> CONTACT), carregar (-> DOCKED, decidido no step) ou
        esgotar `final_push_time`.
        """
        if self.bumper_hit():
            self.enter(CONTACT)
            return self.run_contact()

        if self.signal_detected():
            # Reapareceu: volta a usar o ângulo.
            self.enter(APPROACH)
            return self.run_approach()

        if self.state_elapsed() > self.final_push_time:
            self.enter(CONTACT)
            return self.run_contact()

        cmd = Twist()
        cmd.linear.x = self.final_speed
        return cmd

    def run_contact(self):
        """Encostou em algo seguindo o beacon: para e espera `contact_wait`.

        Se era a base, a carga começa e o step() manda para DOCKED antes de a
        espera acabar. Se não começou, era obstáculo (ou um encaixe torto): aí
        recua e tenta de novo.
        """
        if self.state_elapsed() > self.contact_wait:
            self.contact_failures += 1
            self.get_logger().warn(
                f'Encostei em algo e a carga não começou (tentativa '
                f'{self.contact_failures}): recuando para tentar de novo.')
            self.enter(BACKUP)
            return self.run_backup()
        return Twist()

    def backup_duration(self):
        """Tempo de ré desta tentativa, crescendo a cada falha seguida."""
        failures = min(max(self.contact_failures - 1, 0), self.backup_escalation_max)
        return self.backup_time * (1.0 + self.backup_escalation * failures)

    def run_backup(self):
        """Recua (girando um pouco) e volta a procurar, pelo outro lado."""
        if self.state_elapsed() > self.backup_duration():
            # Inverte o sentido da busca: insistir no mesmo lado tende a bater
            # no mesmo obstáculo.
            self.search_direction = -self.search_direction
            self.enter(SEARCH)
            return self.run_search()

        cmd = Twist()
        cmd.linear.x = -self.backup_speed
        cmd.angular.z = self.backup_turn * self.search_direction
        return cmd

    def run_docked(self):
        """Na base, carregando: parado.

        Chegar aqui significa que o encaixe deu certo, então o contador de
        tentativas frustradas volta a zero.

        Continua publicando Twist zerado (e não None) porque aqui queremos
        mesmo segurar o robô na base: qualquer comando antigo que ainda
        estivesse nas juntas morre neste zero.
        """
        self.contact_failures = 0
        return Twist()

    # =========================================================================
    #  Log
    # =========================================================================

    def log_status(self):
        """Uma linha de status por segundo, para acompanhar o docking."""
        now = time.monotonic()
        if now - self.last_status_log < 1.0:
            return
        self.last_status_log = now

        battery = f'{self.battery:.0f}%' if self.battery is not None else '--'
        if self.signal_detected():
            beacon = f'força {self.strength:.2f}, erro {math.degrees(self.angle_error()):+.0f}°'
        else:
            beacon = 'sem sinal'
        self.get_logger().info(
            f'[{self.state}] bateria {battery} | '
            f'docking {"ON" if self.docking_mode else "OFF"} | '
            f'{"carregando" if self.charging else "descarregando"} | '
            f'beacon: {beacon} | bumper {self.bumper_force:.2f} N')


def main(args=None):
    rclpy.init(args=args)
    node = AutoDocking()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
