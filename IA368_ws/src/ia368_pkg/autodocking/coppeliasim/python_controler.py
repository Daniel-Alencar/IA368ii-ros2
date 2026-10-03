"""Child script (Python) do /myRobot: joystick e ponte para comandos externos.

Este arquivo é o script embutido na cena "Evaluation scene3.2_students.ttt",
pendurado em /myRobot/python_controler. Editar aqui NÃO muda a cena: para
instalar esta versão na cena, rode apply_scene_patch.py (mesma pasta) com a
cena aberta no CoppeliaSim.

O que ele faz, a cada passo de simulação:

    sliders do joystick ──┐
                          ├──▶ (rightVel, leftVel) ──▶ setJointTargetVelocity
    sinais <h>leftVel  ───┘                            nos dois motores
           <h>rightVel     (override externo, tem prioridade)

E, de quebra, atualiza os rótulos da janela do joystick com a bateria e a
odometria, e sincroniza o checkbox "docking" com o modo de docking (ver abaixo).

<h> é o handle do robô na cena, então os sinais se chamam "84leftVel",
"84Docking" e assim por diante.

Os sinais de override são de USO ÚNICO: sysCall_actuation os lê e os APAGA.
Quem quiser comandar por eles precisa reescrevê-los a cada passo.

MODIFICAÇÃO (ponte ROS 2 direta nas juntas):
  Na versão original este script escrevia nas juntas a CADA passo, com o valor
  do joystick (0 quando parado). Isso apagava qualquer comando externo feito
  com setJointTargetVelocity, como o da ponte ROS 2 (bumper_and_velocity_node),
  que a especificação pede. Agora ele só escreve nas juntas quando:
    - o joystick está em uso (ou acabou de voltar a zero, para parar o robô);
    - chegou um override pelos sinais <h>leftVel/<h>rightVel;
    - a bateria acabou (o robô para, mesmo sob comando externo).
  Fora desses casos, os motores ficam com quem os comandou por último.

MODIFICAÇÃO (checkbox "docking" -> ROS 2):
  O checkbox "docking" da janela do joystick agora escreve o sinal int
  <h>DockingRequest (1 marcado, 0 desmarcado). O docking_node o lê e publica
  em /myRobot/docking_mode, então clicar no checkbox liga/desliga o docking do
  autodocking_node sem esperar a bateria baixar. No sentido contrário, o sinal
  <h>Docking (escrito pelo docking_node) agora marca E desmarca o checkbox,
  para ele sempre mostrar o modo atual.
"""

import math

# Conversões de ângulo. Note que a cena usa 3.14, e não math.pi: as contas de
# max_rotVel herdam esse arredondamento.
deg2rad = 3.14 / 180.
rad2deg = 180. / 3.14


# =============================================================================
#  Inicialização
# =============================================================================

def sysCall_init():
    sim = require('sim')

    # Handles dos objetos da cena. self.* sobrevive entre as chamadas dos
    # sysCall_*; variáveis locais comuns, não.
    self.robotHandle = sim.getObject("/myRobot")
    self.rightMotorHandle = sim.getObject("/rightMotor")
    self.leftMotorHandle = sim.getObject("/leftMotor")
    self.proximitySensorHandle = sim.getObject("/proximitySensor")
    print("python_controller: Keyboard listener started... (press ESC to stop simulation)")

    global ui, simUI, linVel, rotVel, L, max_linVel, max_rotVel, wheelradius
    global joystickWasActive

    # Comando atual do joystick, em m/s e rad/s no referencial do robô.
    linVel = 0
    rotVel = 0
    # Joystick estava em uso no passo anterior? Usado para mandar um último
    # comando zero quando ele volta ao centro (ver sysCall_actuation).
    joystickWasActive = False

    # Parâmetros geométricos, fixos aqui (a cena real mede 0.05 m e 0.2 m).
    wheelradius = 0.05   # m, raio da roda
    L = 0.2              # m, distância entre as duas rodas

    # Limites de fundo de escala dos sliders.
    max_linVel = 0.5             # m/s
    max_rotVel = 90 * deg2rad    # rad/s

    # Load the simUI plugin
    sim = require('sim')
    simUI = require('simUI')

    # Define the UI
    #
    # Cada widget se liga a uma função deste arquivo pelo atributo on-click ou
    # on-change; os ids numéricos são usados depois para atualizar os rótulos:
    #   1 = slider de rotação     3000 = rótulo da velocidade angular
    #   2 = slider linear         4000 = rótulo da velocidade linear
    #  10 = checkbox de docking   4100 = bateria      4200 = odometria
    xml = '''
    <ui title="joystick" closeable="true" resizable="true" activate="true" layout="vbox">
        
        <!-- Top button -->
        <button text="STOP" on-click="stopButtonPressed" />

        <!-- Vertical slider centered -->
        <group layout="hbox" flat="true">

            <label text="   0 m/s  " id="4000" word-wrap="false" />
            <button text="0 Lin Vel" on-click="linVelButtonPressed" />
            <vslider id="2" minimum="-50" maximum="50" value="0" on-change="vslider_changed" />
            <checkbox id="10" text="docking" on-change="dockingButtonPressed" />            
            <stretch />
        </group>

        <!-- Horizontal slider with button at left -->
        <group layout="hbox" flat="true">
            <label text="   0 deg/s  " id="3000" word-wrap="false" />
            <button text="0 rot vel" on-click="rotButtonPressed" />
            <hslider id="1" minimum="-50" maximum="50" value="0" on-change="hslider_changed" />
        </group>
        <group layout="hbox" flat="true">
            <label text="battery: --- % "id="4100" word-wrap="false"   />
            <label text="odometry (x,y,theta): (_,_,_) "id="4200" word-wrap="false"   />
        </group>

    </ui>
    '''
    ui = simUI.create(xml)


# =============================================================================
#  Callbacks da interface do joystick
# =============================================================================

def vslider_changed(ui, id, newVal):
    """Slider vertical: velocidade linear, de -50..50 para -max..+max m/s."""
    global linVel
    linVel = (newVal) / 50 * max_linVel
    #print(f"python_controller: vertical slider value: {newVal}", linVel )
    simUI.setLabelText(ui, 4000, str(round(linVel, 2)) + " m/s ")


def hslider_changed(ui, id, newVal):
    """Slider horizontal: velocidade angular. O sinal é invertido para que
    arrastar para a direita gire o robô para a direita."""
    global rotVel
    rotVel = -(newVal) / 50 * max_rotVel
    #print(f"python_controller: Horizontal slider value: {newVal}", rotVel  )
    simUI.setLabelText(ui, 3000, str(round(rotVel, 1)) + " deg/s ")


def stopButtonPressed(ui, id):
    """Botão STOP: zera os dois eixos e devolve os sliders ao centro."""
    global linVel, rotVel
    linVel = 0
    rotVel = 0
    #print("python_controller: stopping")
    simUI.setSliderValue(ui, 1, 0)
    simUI.setSliderValue(ui, 2, 0)
    simUI.setLabelText(ui, 3000, "0 deg/s ")
    simUI.setLabelText(ui, 4000, "0 m/s ")


def linVelButtonPressed(ui, id):
    """Zera só a velocidade linear."""
    global linVel
    linVel = 0
    #print("python_controller: lin vel is 0")
    simUI.setSliderValue(ui, 2, 0)
    simUI.setLabelText(ui, 4000, "0 m/s ")


def rotButtonPressed(ui, id):
    """Zera só a velocidade angular."""
    global rotVel
    rotVel = 0
    print("python_controller: rot vel is 0")
    simUI.setSliderValue(ui, 1, 0)
    simUI.setLabelText(ui, 3000, "0 deg/s ")


def dockingButtonPressed(ui, id, newVal):
    """Checkbox "docking": anuncia a mudança de modo na cena.

    newVal vem do Qt: 2 = marcado, 0 = desmarcado. O broadcastMsg é recebido
    por sysCall_msg de outros scripts — nesta cena, porém, ninguém o escuta: o
    comportamento de docking é o que o aluno deve implementar.
    """
    val = False
    if newVal == 2:
        val = True
    #print(f"python_controller: docking mode {val}")
    msg = {'id': 'dockingMode', 'data': [val]}
    sim.broadcastMsg(msg)
    # MODIFICAÇÃO: deixa o pedido visível para a ponte ROS 2 (docking_node).
    sim.setInt32Signal(str(self.robotHandle) + "DockingRequest", 1 if val else 0)


# =============================================================================
#  Laço de atuação, chamado a cada passo de simulação
# =============================================================================

def sysCall_actuation():
    global joystickWasActive

    # inverse kinematic equations
    #
    # Cada roda está a L/2 do centro, então sua velocidade linear é a do robô
    # mais ou menos a contribuição do giro. Resultado em m/s NA RODA.
    rightVel = linVel + L / 2 * rotVel
    leftVel = linVel - L / 2 * rotVel

    # check if for external motor control commands
    #
    # Override externo: quem escrever estes sinais manda, no lugar do joystick.
    # São de uso único — lidos e apagados aqui, precisam ser reescritos a cada
    # passo por quem estiver comandando de fora (por exemplo a ponte ROS 2 em
    # motor_mode=signal).
    rightVelExt = sim.getFloatSignal(str(self.robotHandle) + "rightVel")
    leftVelExt = sim.getFloatSignal(str(self.robotHandle) + "leftVel")
    if leftVelExt is not None:
        sim.clearFloatSignal(str(self.robotHandle) + "leftVel")
        leftVel = leftVelExt
        #print ('overridden left vel')
    if rightVelExt is not None:
        sim.clearFloatSignal(str(self.robotHandle) + "rightVel")
        rightVel = rightVelExt
        #print ('overridden right vel')

    # check and update battery status
    #
    # Quem escreve <h>Battery é o script /myRobot/battery, a cada 1 s.
    batt = sim.getFloatSignal(str(self.robotHandle) + "Battery")
    if batt is not None:
        simUI.setLabelText(ui, 4100, "Battery: " + str(round(batt, 2)) + " % ")
    if batt == 0:
        #battery is dead
        rightVel = 0
        leftVel = 0

    #check and set external docking mode setting
    #
    # MODIFICAÇÃO: a versão original só reagia a 1 (marcava o checkbox). Agora
    # 0 também desmarca, senão o checkbox continuaria marcado depois de o
    # autodocking desligar o docking, e o <h>DockingRequest = 1 que ele
    # mantém religaria o docking. O pedido é atualizado aqui mesmo (e não só
    # no callback) para não depender de o Qt disparar o evento.
    forceDocking = sim.getInt32Signal(str(self.robotHandle) + "Docking")
    if forceDocking is not None:
        sim.clearInt32Signal(str(self.robotHandle) + "Docking")
        simUI.setCheckboxValue(ui, 10, 2 if forceDocking == 1 else 0, True)
        sim.setInt32Signal(str(self.robotHandle) + "DockingRequest", 1 if forceDocking == 1 else 0)

    # check and update odometry
    #
    # O /myRobot/odometry publica (x, y, theta) numa propriedade do objeto, e
    # não num sinal. Abaixo, as tentativas anteriores de transporte que o autor
    # deixou registradas.
    #odomPack=sim.getStringSignal(str(self.robotHandle)+"Odometry")
    #odomPack = sim.getStringProperty(self.robotHandle, "customData.Odometry", {'noError' : True})
    #print("python_controler :",odomPack,self.robotHandle)
    #odomPack = sim.getBufferProperty(sim.handle_app, str(self.robotHandle)+"Odometry", {'noError' : True})
    #if odomPack:
    #    odom = sim.unpackTable(odomPack)
    odomPack = sim.getBufferProperty(self.robotHandle, "customData.Odometry", {'noError': True})

    if odomPack:
        odom = sim.unpackTable(odomPack)
        odomStr = f"({odom[0]:.2f}, {odom[1]:.2f}, {math.degrees(odom[2]):.2f})"
        #print("python_controler: ",odomStr, odom)
        simUI.setLabelText(ui, 4200, f"odometry (x,y,theta): " + odomStr)
        #simUI.setLabelText(ui,4200,"odometry (x,y,theta): (: "+str(round(batt,2))+") ")

    # Repetição do corte por bateria vazia, já feito acima. Inofensivo, mas
    # necessário se o bloco da odometria acima vier a alterar as velocidades.
    if batt == 0:
        #battery is dead
        rightVel = 0
        leftVel = 0

    # update motors veloicity
    #
    # MODIFICAÇÃO: só escreve nas juntas se este script tiver algo a comandar;
    # senão preserva o último comando externo (ponte ROS 2). O passo em que o
    # joystick volta a zero ainda escreve, para o robô parar.
    joystickActive = linVel != 0 or rotVel != 0
    externalOverride = leftVelExt is not None or rightVelExt is not None
    writeMotors = joystickActive or joystickWasActive or externalOverride or batt == 0
    joystickWasActive = joystickActive

    # Duas conversões de uma vez: m/s -> rad/s dividindo pelo raio, e o sinal
    # negativo porque, do jeito que as juntas estão montadas nesta cena,
    # velocidade negativa faz o robô andar para a frente.
    if writeMotors:
        sim.setJointTargetVelocity(self.rightMotorHandle, -rightVel / wheelradius)
        sim.setJointTargetVelocity(self.leftMotorHandle, -leftVel / wheelradius)

    # Leitura do sensor de proximidade. O resultado não é usado: os três "nil"
    # são apenas nomes descartáveis (em Python, nil não é palavra reservada).
    state, dist, nil, nil, nil = sim.readProximitySensor(self.proximitySensorHandle)

    # Check if any key was pressed
    #_, keys, _ = sim.getSimulatorMessage()
    #key=chr(keys[0])

    #if key == 'w':
    #    print("Key pressed:", key)
    #pass


def sysCall_sensing():
    # put your sensing code here
    pass


def sysCall_cleanup():
    simUI.destroy(ui)
    pass
