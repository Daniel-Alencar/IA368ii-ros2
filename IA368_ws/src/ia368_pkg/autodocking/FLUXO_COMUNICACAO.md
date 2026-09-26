# Fluxo de dados: CoppeliaSim ↔ ROS 2 (autodocking)

Os nós ROS 2 desta pasta são uma **ponte**: leem sinais da cena e publicam em tópicos, ou recebem tópicos e escrevem na cena. A comunicação com o CoppeliaSim usa a ZeroMQ Remote API (`sim.*`).

Os sinais da cena têm o handle do robô no nome. Nesta cena, `/myRobot` tem handle `84`, então `<h>Battery` vira `84Battery`.

## Da cena para o ROS 2 (sensores)

| Quem escreve na cena | Sinal / objeto | Nó ROS 2 | Tópico |
|---|---|---|---|
| script `/myRobot/battery` | `<h>Battery` | `battery_node` | `/myRobot/battery_state` |
| script `/chargingBase/beacon` | `<h>Charging` | `battery_node` | `/myRobot/battery_state` (`power_supply_status`) |
| script `/chargingBase/beacon` | `<h>StrengthSignal` | `charging_base_node` | `/myRobot/charging_base/strengthSignal` |
| script `/chargingBase/beacon` | `<h>RelativeAngle` | `charging_base_node` | `/myRobot/charging_base/relativeAngle` |
| sensor `/myRobot/forceSensor` | (lido com `sim.readForceSensor`) | `bumper_and_velocity_node` | `/myRobot/bumper` |

O beacon só responde **se for chamado**: o `charging_base_node` escreve o sinal `Beacon = <h>`, e o script da base escreve a intensidade e o ângulo, mas apenas se o robô estiver dentro do feixe.

## Do ROS 2 para a cena (comandos)

| Tópico | Nó ROS 2 | O que faz na cena |
|---|---|---|
| `/myRobot/docking_mode` | `docking_node` | escreve `<h>Docking`. O `python_controler` só reage a `1`, marcando o checkbox "docking". |
| `/myRobot/cmd_vel` | `bumper_and_velocity_node` | converte `(v, ω)` em velocidade das rodas e chama `sim.setJointTargetVelocity` nas juntas `leftMotor`/`rightMotor` |

## Diagrama

```
        CoppeliaSim (scripts da cena)                        ROS 2
 ┌──────────────────────────────────────┐
 │ battery ─────── <h>Battery ─────────────▶ battery_node ──▶ /myRobot/battery_state
 │ beacon ──────── <h>Charging ────────────▶ battery_node
 │ beacon ◀─────── Beacon = <h> ─────────────── charging_base_node (pedido)
 │ beacon ──────── <h>StrengthSignal ──────▶ charging_base_node ──▶ .../strengthSignal
 │                 <h>RelativeAngle ───────▶ charging_base_node ──▶ .../relativeAngle
 │ forceSensor ────────────────────────────▶ bumper_and_velocity ──▶ /myRobot/bumper
 │                                      │
 │ python_controler ◀── <h>Docking ─────────── docking_node ◀── /myRobot/docking_mode
 │ juntas das rodas ◀── setJointTargetVelocity ── bumper_and_velocity ◀── /myRobot/cmd_vel
 └──────────────────────────────────────┘
```

## Scripts da cena que não passam pelo ROS 2

- **`python_controler`:** o joystick da cena. Também escreve nas juntas das rodas; para não apagar o comando do `cmd_vel`, a cena precisa da versão modificada ([coppeliasim/python_controler.py](coppeliasim/python_controler.py)), que só escreve quando o joystick está em uso.
- **`odometry`:** estima `(x, y, θ)` pelos encoders e grava na propriedade `customData.Odometry` do robô. Nenhum nó ROS 2 lê esse valor.
