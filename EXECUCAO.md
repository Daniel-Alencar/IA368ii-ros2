# Como executar os projetos ROS 2 do IA368ii

Guia prático para rodar cada um dos projetos do pacote `ia368_pkg` (workspace `IA368_ws`) junto com o CoppeliaSim.

## Sumário

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

## 2. Autodocking (ponte Remote API ↔ ROS 2)

**Código:** [autodocking/](IA368_ws/src/ia368_pkg/autodocking/)
**Cena:** [Evaluation scene3.2_students.ttt](IA368_ws/src/ia368_pkg/autodocking/Evaluation%20scene3.2_students.ttt) (também disponível no [Google Drive](https://drive.google.com/file/d/1kWkmB_3bF3PY6_6VbIV730ZlBTSmm2EU/view?usp=sharing)). Coloque-a no seu diretório `roomba docking`.

### Execução

```bash
ros2 launch ia368_pkg remoteAPI_ROS2_bridge.launch.py
```

**Nós:** `battery_node`, `bumper_and_velocity_node`, `charging_base_node`, `docking_node`.

**Tópicos úteis:**

- `/myRobot/battery_state` (`BatteryState`)
- `/myRobot/bumper` (`Wrench`)
- `/myRobot/charging_base/strengthSignal` e `/myRobot/charging_base/relativeAngle` (`Float32`)
- `/myRobot/cmd_vel` (`Twist`): comando de velocidade para o robô

Teste manual do movimento:

```bash
ros2 topic pub /myRobot/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
```

---

## 3. Controle de posição

**Código:** [position_control/](IA368_ws/src/ia368_pkg/position_control/)
**Cena:** [Exercise_position_control.ttt](IA368_ws/src/ia368_pkg/position_control/Exercise_position_control.ttt)

### Execução

Implemente o controlador em [position_control_node_students.py](IA368_ws/src/ia368_pkg/position_control/position_control_node_students.py), recompile (`colcon build --packages-select ia368_pkg`) e rode:

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

| Projeto | Cena (`.ttt`) | Comando |
|---|---|---|
| YOLO 3D | `yolo_detector/tf_scene.ttt` | `ros2 launch ia368_pkg yolo_detection.launch.py dummy:=0` |
| Autodocking | `autodocking/Evaluation scene3.2_students.ttt` | `ros2 launch ia368_pkg remoteAPI_ROS2_bridge.launch.py` |
| Controle de posição | `position_control/Exercise_position_control.ttt` | `ros2 launch ia368_pkg position_control.launch.py` |
| Pega banana | `yolo_detector/pega_banana.ttt` | `ros2 launch ia368_pkg pega_banana.launch.py dummy:=0` |
| SLAM Toolbox* | `slam_toolbox/p3_slam_toolbox.ttt` | `ros2 launch ia368_pkg slam_toolbox.launch.py` |
| RTAB-Map | `rtabmap/p3dx_rtabmap.ttt` | `ros2 launch ia368_pkg rtabmap_rgbd.launch.py` |
| Nav2* | `nav2/p3dx_nav2.ttt` ou `home.ttt` | `ros2 launch ia368_pkg nav2_{humble,jazzy}.launch.py` |

\* Executar de dentro de `IA368_ws`.

As cenas ficam em `IA368_ws/src/ia368_pkg/`, exceto `home.ttt`, que está na raiz do repositório.
