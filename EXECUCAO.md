# Como executar os projetos ROS 2 do IA368ii

Guia prático para rodar cada um dos projetos do pacote `ia368_pkg` (workspace `IA368_ws`) junto com o CoppeliaSim.

## Sumário

- [Situação dos projetos: o que falta implementar](#situação-dos-projetos-o-que-falta-implementar)
0. [Preparação (uma vez só)](#0-preparação-uma-vez-só)
1. [Detecção 3D com YOLO](#1-detecção-3d-com-yolo)
2. [Autodocking (ponte Remote API ↔ ROS 2)](#2-autodocking-ponte-remote-api--ros-2)
3. [Controle de posição](#3-controle-de-posição)
4. [Pega banana](#4-pega-banana)
5. [SLAM com slam_toolbox (LiDAR)](#5-slam-com-slam_toolbox-lidar)
6. [SLAM RGB-D com RTAB-Map (Kinect)](#6-slam-rgb-d-com-rtab-map-kinect)
7. [Navegação com Nav2](#7-navegação-com-nav2)
8. [Resumo rápido](#resumo-rápido)

---

## Situação dos projetos: o que falta implementar

Nem todo projeto funciona "de ponta a ponta" só com o launch. Em alguns, o aluno ainda precisa escrever parte do código (a lógica de controle, ou a própria ponte com o CoppeliaSim).

| Projeto | Situação | O que o aluno precisa implementar |
|---|---|---|
| 1. YOLO 3D | ✅ Completo | Nada. Basta rodar. |
| 2. Autodocking | ✅ Completo (ponte + docking autônomo) | Nada. As duas atividades da [especificação](IA368_ws/src/ia368_pkg/autodocking/Especifications.md) estão implementadas: a ponte ROS 2 ↔ CoppeliaSim e o nó externo `autodocking_node` com o comportamento de docking. |
| 3. Controle de posição | ⚠️ **Incompleto** | A lei de controle em `control_law()` do `position_control_node_students.py`. |
| 4. Pega banana | ⚠️ **Incompleto** | Um **nó novo** que use as detecções para mover o robô até a banana. |
| 5. SLAM Toolbox | ✅ Completo | Nada. O mapeamento é feito pelo `slam_toolbox`; o robô é dirigido por teleoperação. |
| 6. RTAB-Map | ✅ Completo | Nada. O mapeamento é feito pelo `rtabmap`; o robô é dirigido por teleoperação. |
| 7. Nav2 | ✅ Completo | Nada. O Nav2 planeja e controla; as metas são enviadas pelo RViz. Os parâmetros em `config/nav2_params_*.yaml` podem ser ajustados. |

Nos projetos incompletos, os detalhes (tópicos disponíveis e o que o nó do aluno deve fazer) estão na seção de cada um, no bloco **"O que falta implementar"**.

---

## 0. Preparação (uma vez só)

### Dependências Python comuns

Todos os nós conversam com o CoppeliaSim pela ZMQ Remote API:

```bash
pip install coppeliasim-zmqremoteapi-client
# Ubuntu 24.04:
pip install coppeliasim-zmqremoteapi-client --break-system-packages
```

### Dependências ROS 2 (apenas para SLAM/Nav2)

```bash
sudo apt install ros-$ROS_DISTRO-slam-toolbox \
                 ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-bringup \
                 ros-$ROS_DISTRO-rtabmap-ros
```

### Compilar o workspace

```bash
cd IA368ii/IA368_ws
colcon build --packages-select ia368_pkg
source install/setup.bash   # bash
source install/setup.zsh    # zsh
```

> **Todo terminal novo precisa desse `source`**, senão o `ros2 launch` responde `Package 'ia368_pkg' not found`. Use o arquivo do seu shell (`setup.bash` ou `setup.zsh`); misturar os dois não funciona. Para não repetir, adicione ao `~/.bashrc`/`~/.zshrc`, **depois** do `source /opt/ros/<distro>/setup.*`:
>
> ```bash
> source /caminho/para/IA368ii/IA368_ws/install/setup.zsh
> ```

> Se aparecer `TypeError: canonicalize_version() got an unexpected keyword argument 'strip_trailing_zero'`, faça `pip install --upgrade setuptools==70.0.0` e compile de novo.

### Fluxo geral de cada projeto

1. Abra o **CoppeliaSim** e carregue a cena (`.ttt`) do projeto.
2. Em um terminal novo: `cd IA368ii/IA368_ws && source install/setup.bash` (ou `setup.zsh`).
3. Rode o `ros2 launch` correspondente. A maioria dos nós chama `sim.startSimulation()` sozinha, então não é preciso apertar *play*.
4. Para encerrar: `Ctrl+C` no terminal e pare a simulação no CoppeliaSim.

> **Importante:** os launches de SLAM/Nav2 usam caminhos relativos (`config/...`). Rode esses comandos **a partir da pasta `IA368_ws`**, senão os arquivos de parâmetros não são encontrados.

---

## 1. Detecção 3D com YOLO

**Código:** [yolo_detector/](IA368_ws/src/ia368_pkg/yolo_detector/)
**Cena:** [tf_scene.ttt](IA368_ws/src/ia368_pkg/yolo_detector/tf_scene.ttt)

### Dependências

Ubuntu 22.04:

```bash
pip install ultralytics
```

Ubuntu 24.04:

```bash
pip install ultralytics --no-deps --break-system-packages
pip install polars requests torchvision ultralytics-thop opencv-python numpy==1.26.4 --break-system-packages
pip install torch --break-system-packages
```

O modelo `yolo11n-seg.pt` é baixado automaticamente na primeira execução, no diretório de onde o launch foi chamado.

### Execução

```bash
ros2 launch ia368_pkg yolo_detection.launch.py dummy:=0
```

| Argumento | Valores | Efeito |
|---|---|---|
| `dummy` | `0` (padrão) / `1` | `1` também sobe o `dummy_creation_node`, que cria *dummies* na cena nas posições detectadas |

**Nós (namespace `yolo_detector`):** `kinect_node`, `tf_node`, `yolo_node` e `dummy_creation_node` (opcional).

**Tópicos úteis:**

- `/rgb/image`, `/depth/image`: imagens do Kinect
- `/yolo/annotated`: imagem com as detecções
- `/yolo/object_3d_point`: marcador 3D do objeto detectado

Para visualizar: `rqt_image_view /yolo/annotated` ou `rviz2`.

---

## 2. Autodocking (ponte Remote API ↔ ROS 2 + docking autônomo)

**Código:** [autodocking/](IA368_ws/src/ia368_pkg/autodocking/)
**Cena:** [Evaluation scene3.2_students.ttt](IA368_ws/src/ia368_pkg/autodocking/coppeliasim/robot/Evaluation%20scene3.2_students.ttt) (também disponível no [Google Drive](https://drive.google.com/file/d/1kWkmB_3bF3PY6_6VbIV730ZlBTSmm2EU/view?usp=sharing)). Coloque-a no seu diretório `roomba docking`.

### Execução

```bash
# ponte + docking autônomo (as duas atividades)
ros2 launch ia368_pkg autodocking.launch.py

# só a ponte (primeira atividade, ou para dirigir o robô à mão)
ros2 launch ia368_pkg remoteAPI_ROS2_bridge.launch.py
```

As duas atividades estão descritas em [Especifications.md](IA368_ws/src/ia368_pkg/autodocking/Especifications.md): a primeira é a **ponte** ROS 2 ↔ CoppeliaSim, a segunda é o **nó externo de docking autônomo**, que usa só os tópicos da ponte. O fluxo de dados completo e as decisões de projeto estão em [FLUXO_COMUNICACAO.md](IA368_ws/src/ia368_pkg/autodocking/FLUXO_COMUNICACAO.md).

**Nós e interface ROS 2:**

| Nó | Sinal/objeto no CoppeliaSim | Tópico ROS 2 | Direção |
|---|---|---|---|
| `battery_node` | `<handle>Battery` (float) e `<handle>Charging` (int) | `/myRobot/battery_state` (`BatteryState`; carga em `power_supply_status`) | Sim → ROS |
| `charging_base_node` | `<handle>StrengthSignal` (float) | `/myRobot/charging_base/strengthSignal` (`Float32`) | Sim → ROS |
| `charging_base_node` | `<handle>RelativeAngle` (float) | `/myRobot/charging_base/relativeAngle` (`Float32`) | Sim → ROS |
| `docking_node` | `<handle>Docking` (int) | `/myRobot/docking_mode` (`Int32`) | ROS → Sim |
| `bumper_and_velocity_node` | juntas das rodas (`sim.setJointTargetVelocity`) | `/myRobot/cmd_vel` (`Twist`) | ROS → Sim |
| `bumper_and_velocity_node` | `/myRobot/forceSensor` | `/myRobot/bumper` (`Wrench`) | Sim → ROS |

O `bumper_and_velocity_node` encontra sozinho as juntas das rodas (as juntas com `left`/`right` no nome dentro de `/myRobot`) e mede a distância entre elas para usar como `wheel_base`. No início ele registra no log quais juntas e qual `wheel_base` foram usados. Se a detecção falhar ou os valores estiverem errados, defina os parâmetros:

```bash
ros2 run ia368_pkg bumper_and_velocity_node --ros-args \
    -p left_motor:=/myRobot/leftMotor -p right_motor:=/myRobot/rightMotor \
    -p wheel_base:=0.2 -p wheel_radius:=0.0975
```

### Testar a ponte (o que a especificação pede para demonstrar)

```bash
ros2 topic echo /myRobot/battery_state                  # bateria + estado de carga
ros2 topic echo /myRobot/charging_base/strengthSignal   # intensidade do beacon IR
ros2 topic echo /myRobot/charging_base/relativeAngle    # ângulo até a base
ros2 topic pub --once /myRobot/docking_mode std_msgs/msg/Int32 "{data: 1}"   # ativa docking (0 desativa)
ros2 topic pub /myRobot/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.3}}"
```

O beacon só responde **se for chamado**, e só quando o robô está dentro do feixe IR: o `charging_base_node` escreve o sinal `Beacon = <handle>` a cada ciclo, e o script `/chargingBase/beacon` responde nos sinais do robô. Fora do feixe o nó não publica nada — nos dois tópicos acima o `echo` fica em silêncio, e isso é normal, não é erro.

### Correções feitas em relação ao código original

O código original da ponte não atendia a especificação em dois pontos (já corrigidos nesta cópia):

- **`charging_base_node`:** não pedia a leitura ao beacon e não apagava os sinais depois de ler, então republicava um valor velho para sempre depois de o robô sair do feixe. Agora ele escreve `Beacon = <handle>` a cada ciclo, aceita os dois pares de nomes de sinal (`StrengthSignal`/`RelativeAngle`, que é o que esta cena escreve, e `signalStrength`/`relativeAngle`, citados na especificação) e apaga o que leu.
- **`bumper_and_velocity_node`:** gravava os sinais intermediários `rightVel`/`leftVel` em vez de chamar `sim.setJointTargetVelocity()` nas juntas, o que a especificação proíbe. Também ativava `sim.setStepping(True)` sem chamar `sim.step()` (o que pode congelar a simulação) e rodava em espera ativa (CPU a 100%). Foi reescrito como um nó ROS 2 normal, com timer.

### Docking autônomo (`autodocking_node`)

[autodocking.py](IA368_ws/src/ia368_pkg/autodocking/autodocking.py) é o controlador da segunda atividade. Ele fala **só ROS 2**: não importa `coppeliasim_zmqremoteapi_client`, não conhece handles nem sinais da cena. Assina `/myRobot/charging_base/strengthSignal`, `/myRobot/charging_base/relativeAngle`, `/myRobot/battery_state` e `/myRobot/bumper`, e publica `/myRobot/cmd_vel` e `/myRobot/docking_mode`.

O comportamento é uma máquina de estados:

| Estado | O que faz |
|---|---|
| `IDLE` | docking desligado: **não publica `cmd_vel`**, para deixar a teleoperação livre |
| `SEARCH` | gira no lugar até reencontrar o beacon |
| `APPROACH` | alinha pelo `relativeAngle` e avança em direção à base |
| `FINAL` | o beacon se apagou já encostando na base: empurra em frente às cegas |
| `CONTACT` | encostou em algo: para e espera para ver se a carga começa |
| `BACKUP` | não era a base: recua e tenta de novo pelo outro lado |
| `SWEEP` | a volta no lugar não achou nada: varre a vizinhança em **espiral** |
| `DOCKED` | na base, carregando: parado |

O docking liga sozinho com 95 % de bateria (logo no início da simulação) e desliga quando ela chega a 100 %, devolvendo o robô para a teleoperação. Para testar sem esperar a bateria baixar:

```bash
ros2 topic pub --once /myRobot/docking_mode std_msgs/msg/Int32 "{data: 1}"   # liga
ros2 topic pub --once /myRobot/docking_mode std_msgs/msg/Int32 "{data: 0}"   # desliga
```

O nó registra uma linha de status por segundo, que é a forma mais rápida de ver o que ele está pensando:

```
[APPROACH] bateria 36% | docking ON | descarregando | beacon: força 0.51, erro +2° | bumper 0.00 N
```

> **Confira o referencial do ângulo.** O beacon da cena mede o `relativeAngle` no referencial do `/myRobot/dockingSensor`, não do corpo do robô. Nesta cena os dois coincidem (medido: a frente do robô é o eixo **+y** dele, e o `dockingSensor` está no nariz com o `+x` apontando para lá), então `angle_target = 0` como diz o enunciado. O sintoma de `angle_target` errado é traiçoeiro: o robô acha o beacon, gira para alinhar e **perde o sinal exatamente quando o erro chega a zero**, repetindo isso para sempre. Se você vir isso no log, é o `angle_target`. Se o robô girar para o lado contrário ao da base, é o `angle_sign:=-1.0`. Os demais parâmetros estão na seção 4.5 do [FLUXO_COMUNICACAO.md](IA368_ws/src/ia368_pkg/autodocking/FLUXO_COMUNICACAO.md).

**Limitação principal, e o parâmetro que decide entre funcionar e não funcionar:** o feixe da base é um corredor estreito, e o `beacon.lua` só responde se o `dockingSensor` for o objeto **mais próximo** dentro dele (o sofá, as paredes e o corpo do robô competem). Fora desse corredor o beacon não publica nada e a busca é cega — o `SWEEP` em espiral acha o feixe porque qualquer laço que circunde a base cruza o corredor, mas os obstáculos fazem o custo variar muito. Medido no sistema completo, de uma pose a 0,5 m da base e fora do corredor: o docking ligou com 60 % de bateria, achou o beacon com 41 % e encaixou com 38 % — **~22 % de bateria**. Como a bateria gasta 1 % por segundo simulado, `battery_low = 40` (o padrão antigo) dava um orçamento do tamanho do custo e o robô morria no meio da busca. Por isso o padrão agora é **95 %**: o docking liga ~5 s depois do início e o robô tem quase toda a bateria para achar a base. Para demonstrar com segurança, leve o robô para perto da base com a teleoperação e só então ative o `docking_mode`; a aproximação em si custa ~3 %. Detalhes na seção 4.6 do [FLUXO_COMUNICACAO.md](IA368_ws/src/ia368_pkg/autodocking/FLUXO_COMUNICACAO.md).

Os scripts em [autodocking/coppeliasim/](IA368_ws/src/ia368_pkg/autodocking/coppeliasim/) **não precisam ser adicionados à cena**: os `.lua` (`beacon.lua`, que define todo o protocolo, `battery.lua`, `dockingSensor_sensorScript.lua` e `odometry.lua`) são cópias de referência de scripts que **já estão dentro** do `.ttt`. A única exceção é o `python_controler.py`: a cena original traz uma versão que reescreve as juntas a cada passo com o valor do joystick (zero) e apaga o `cmd_vel` da ponte. Instale a versão modificada uma vez, com a cena aberta e a simulação parada:

```bash
cd IA368_ws/src/ia368_pkg/autodocking/coppeliasim
python3 apply_scene_patch.py --save
```

Sintoma de que falta o patch: o robô não se move com `ros2 topic pub /myRobot/cmd_vel ...`, embora o log do autodocking mostre os estados mudando.

---

## 3. Controle de posição

**Código:** [position_control/](IA368_ws/src/ia368_pkg/position_control/)
**Cena:** [Exercise_position_control.ttt](IA368_ws/src/ia368_pkg/position_control/Exercise_position_control.ttt)

### O que falta implementar

⚠️ No template original do repositório, o método `control_law()` de [position_control_node_students.py](IA368_ws/src/ia368_pkg/position_control/position_control_node_students.py) tem um `# TODO` que devolve `vu = 0` e `omega = 0`. Com o template, o robô não se move.

O aluno deve implementar o controlador de Siegwart:

- **Task 1:** a lei de controle com os ganhos `Krho`, `Kalpha` e `Kbeta`.
- **Task 2:** as opções `backwardAllowed` (andar de ré quando o alvo está atrás) e `useconstantSpeed`/`constantSpeed` (velocidade linear constante).

O template também tem um bug na leitura de parâmetros: `useconstantSpeed` sobrescreve `self.constantSpeed`. A solução de referência (`position_control_node_solution.py`) está no `.gitignore` e não vem no repositório.

> Nesta cópia local, as duas tasks já foram implementadas (e o bug corrigido). A explicação da implementação e da teoria está em [CONTROLADOR_SIEGWART.md](IA368_ws/src/ia368_pkg/position_control/CONTROLADOR_SIEGWART.md).

### Execução

Com o controlador implementado, recompile (`colcon build --packages-select ia368_pkg`) e rode:

```bash
ros2 launch ia368_pkg position_control.launch.py
```

**Nós:**

- `odom_node`: publica `/myRobot/odom`
- `target_node`: publica o alvo (`/myRobotTarget` da cena) em `/myRobot/goal` (`Pose2D`)
- `pos_control_node`: o seu controlador. Lê odom + goal e publica `/myRobot/cmd_vel`
- `vel_node`: aplica `/myRobot/cmd_vel` nos motores `leftMotor`/`rightMotor`

Mova o `myRobotTarget` na cena durante a simulação para testar o controlador.

> **Problema comum:** se o robô andar sozinho sem comando do ROS 2, zere a *Target velocity* em *Dynamic properties* dos motores `rightMotor` e `leftMotor`.

---

## 4. Pega banana

**Código:** [yolo_detector/](IA368_ws/src/ia368_pkg/yolo_detector/) + [velocity_node.py](IA368_ws/src/ia368_pkg/position_control/velocity_node.py)
**Cena:** [pega_banana.ttt](IA368_ws/src/ia368_pkg/yolo_detector/pega_banana.ttt)

### Dependências

As mesmas da [Detecção YOLO](#1-detecção-3d-com-yolo).

### Execução

```bash
ros2 launch ia368_pkg pega_banana.launch.py dummy:=0
```

**Nós:** `kinect_node`, `tf_node`, `yolo_node`, `dummy_creation_node` (opcional; todos no namespace `yolo_detector`) e `vel_node` (namespace raiz, escuta `/myRobot/cmd_vel`).

### O que falta implementar

⚠️ O launch sobe a **percepção** (Kinect + YOLO) e o **atuador** (`vel_node`), mas nenhum nó publica em `/myRobot/cmd_vel`. Ou seja, o robô detecta os objetos mas não se move. O aluno precisa criar um nó (e registrá-lo no `setup.py` e no launch) que:

- **leia a posição da banana**, pelo tópico `/yolo/object_3d_point` (`Marker`, no frame `camera_color_optical_frame`, com a classe em `marker.text`) ou pelas TFs `object_<classe>` publicadas pelo `yolo_node`;
- **filtre a classe da banana** (no COCO, usado pelo `yolo11n-seg.pt`, `banana` é a classe 46, então `class_46`);
- **publique** `/myRobot/cmd_vel` para aproximar o robô da banana. O controlador de posição do projeto 3 pode ser reaproveitado aqui.

Além disso, o `yolo_node` só busca `./Bowl` e `./Cup` na cena; a linha do `./banana` está comentada em [yolo_3d_detection.py](IA368_ws/src/ia368_pkg/yolo_detector/yolo_3d_detection.py).

---

## 5. SLAM com slam_toolbox (LiDAR)

**Código:** [slam_toolbox/](IA368_ws/src/ia368_pkg/slam_toolbox/)
**Cena:** [p3_slam_toolbox.ttt](IA368_ws/src/ia368_pkg/slam_toolbox/p3_slam_toolbox.ttt)
**Parâmetros:** [mapper_params_online_async.yaml](IA368_ws/config/mapper_params_online_async.yaml)

### Execução

```bash
cd IA368ii/IA368_ws            # obrigatório (caminho relativo do config)
ros2 launch ia368_pkg slam_toolbox.launch.py
```

**O que sobe:**

- `lidar_node`: publica `/myRobot/scan`
- `tf_node_slam_toolbox`: TF `odom → base_link` a partir do ground truth da cena
- `vel_node`: com remapeamento `/myRobot/cmd_vel → /cmd_vel`
- TF estática `base_link → laser_link`
- `slam_toolbox` (`online_async_launch.py`) em modo `mapping`

### Visualizar e dirigir o robô

```bash
rviz2    # Fixed Frame = map; adicione Map (/map), LaserScan (/myRobot/scan) e TF
ros2 run teleop_twist_keyboard teleop_twist_keyboard   # publica em /cmd_vel
```

Para salvar o mapa: `ros2 run nav2_map_server map_saver_cli -f meu_mapa`, ou use o painel *SlamToolboxPlugin* no RViz.

---

## 6. SLAM RGB-D com RTAB-Map (Kinect)

**Código:** [rtabmap/](IA368_ws/src/ia368_pkg/rtabmap/)
**Cenas:** [p3dx_rtabmap.ttt](IA368_ws/src/ia368_pkg/rtabmap/p3dx_rtabmap.ttt) ou [p3dx_rtabmap_perfect_odom.ttt](IA368_ws/src/ia368_pkg/rtabmap/p3dx_rtabmap_perfect_odom.ttt) (odometria perfeita)

### Execução

```bash
ros2 launch ia368_pkg rtabmap_rgbd.launch.py
```

**O que sobe:**

- `kinect_node_rtabmap_rgb`: publica `/rgb/image`, `/depth/image`, `/rgb/camera_info`
- `odom_node`: com remapeamento `/myRobot/odom → /odom`
- `tf_node_rtabmap_rgb`: TFs do robô/Kinect
- TF estática `base_link → kinect`
- `rtabmap` (iniciado com `-d`, que **apaga o banco de dados anterior** a cada execução)
- `rtabmap_viz`: interface de visualização do RTAB-Map

Os parâmetros do RTAB-Map estão definidos direto no launch. O arquivo [rtabmap_params.yaml](IA368_ws/config/rtabmap_params.yaml) não é usado por ele.

Para mover o robô, publique em `/myRobot/cmd_vel` (por exemplo, `ros2 run ia368_pkg vel_node` em outro terminal + `teleop_twist_keyboard --ros-args -r /cmd_vel:=/myRobot/cmd_vel`).

---

## 7. Navegação com Nav2

**Código:** [nav2/ground_truth_node.py](IA368_ws/src/ia368_pkg/nav2/ground_truth_node.py) + tudo do [slam_toolbox](#5-slam-com-slam_toolbox-lidar)
**Cena:** [p3dx_nav2.ttt](IA368_ws/src/ia368_pkg/nav2/p3dx_nav2.ttt). A cena [home.ttt](home.ttt), na raiz do repositório, é a do projeto final.
**Parâmetros:** [nav2_params_humble.yaml](IA368_ws/config/nav2_params_humble.yaml) / [nav2_params_jazzy.yaml](IA368_ws/config/nav2_params_jazzy.yaml)
**RViz:** [nav2_viz.rviz](IA368_ws/config/nav2_viz.rviz)

### Execução

Escolha o launch de acordo com a sua distro ROS 2:

```bash
cd IA368ii/IA368_ws            # obrigatório (caminhos relativos dos configs)

# ROS 2 Humble (Ubuntu 22.04)
ros2 launch ia368_pkg nav2_humble.launch.py

# ROS 2 Jazzy (Ubuntu 24.04)
ros2 launch ia368_pkg nav2_jazzy.launch.py
```

**O que sobe:**

1. Todo o `slam_toolbox.launch.py` (LiDAR, TFs, `vel_node`, slam_toolbox)
2. `ground_truth_node`: publica `/myRobot/odom`
3. Após **10 s** (`TimerAction`), o Nav2 (`nav2_bringup/navigation_launch.py`) com os parâmetros da distro

### Enviar metas

Em outro terminal, também dentro de `IA368_ws`:

```bash
rviz2 -d config/nav2_viz.rviz
```

Use o botão **Nav2 Goal** (ou *2D Goal Pose*) para clicar no destino do robô. O Nav2 publica em `/cmd_vel`, que o `vel_node` repassa aos motores.

---

## Resumo rápido

| Projeto | Situação | Cena (`.ttt`) | Comando |
|---|---|---|---|
| YOLO 3D | ✅ | `yolo_detector/tf_scene.ttt` | `ros2 launch ia368_pkg yolo_detection.launch.py dummy:=0` |
| Autodocking | ✅ | `autodocking/coppeliasim/robot/Evaluation scene3.2_students.ttt` | `ros2 launch ia368_pkg autodocking.launch.py` |
| Controle de posição | ⚠️ falta `control_law()` | `position_control/Exercise_position_control.ttt` | `ros2 launch ia368_pkg position_control.launch.py` |
| Pega banana | ⚠️ falta o nó de controle | `yolo_detector/pega_banana.ttt` | `ros2 launch ia368_pkg pega_banana.launch.py dummy:=0` |
| SLAM Toolbox* | ✅ | `slam_toolbox/p3_slam_toolbox.ttt` | `ros2 launch ia368_pkg slam_toolbox.launch.py` |
| RTAB-Map | ✅ | `rtabmap/p3dx_rtabmap.ttt` | `ros2 launch ia368_pkg rtabmap_rgbd.launch.py` |
| Nav2* | ✅ | `nav2/p3dx_nav2.ttt` ou `home.ttt` | `ros2 launch ia368_pkg nav2_{humble,jazzy}.launch.py` |

\* Executar de dentro de `IA368_ws`.

As cenas ficam em `IA368_ws/src/ia368_pkg/`, exceto `home.ttt`, que está na raiz do repositório.

