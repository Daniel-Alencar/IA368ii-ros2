# Fluxo de dados: CoppeliaSim ↔ ROS 2 ↔ autodocking

Este diretório tem dois tipos de nó, e a diferença entre eles é o ponto principal:

- a **ponte** (`battery_node`, `charging_base_node`, `docking_node`,
  `bumper_and_velocity_node`) é o único processo que fala as duas línguas: lê
  sinais da cena pela ZeroMQ Remote API (`sim.*`) e os publica em tópicos, e faz
  o inverso com os comandos;
- o **controlador** ([autodocking.py](autodocking.py)) só fala ROS 2. Ele não
  importa `coppeliasim_zmqremoteapi_client`, não conhece handles e não sabe o
  que é um "sinal" da cena. Assina quatro tópicos e publica dois.

```
┌──────────────────────────┐   ┌──────────────────────┐   ┌────────────────────┐
│      CoppeliaSim         │   │       a ponte        │   │    autodocking     │
│  (processo do simulador) │   │   (4 nós ROS 2)      │   │   (nó ROS 2)       │
│                          │◀─▶│                      │◀─▶│                    │
│  scripts da cena:        │   │  o único que fala    │   │  só fala ROS 2     │
│   battery, beacon,       │   │  as duas línguas     │   │                    │
│   odometry, python_      │   │                      │   │                    │
│   controler              │   │                      │   │                    │
└──────────────────────────┘   └──────────────────────┘   └────────────────────┘
        ▲                              ▲                          ▲
        │      ZeroMQ Remote API       │       tópicos ROS 2      │
        └──────── TCP :23000 ──────────┘◀───── (DDS) ─────────────┘
```

Os sinais da cena têm o handle do robô no nome. Nesta cena, `/myRobot` tem
handle `84`, então `<h>Battery` vira `84Battery`. Não decore o `84`: ele muda se
a cena mudar, e cada nó descobre o seu sozinho.

---

## 1. Da cena para o ROS 2 (sensores)

| Quem escreve na cena | Sinal / objeto | Nó da ponte | Tópico |
|---|---|---|---|
| script `/myRobot/battery` | `<h>Battery` | `battery_node` | `/myRobot/battery_state` |
| script `/chargingBase/beacon` | `<h>Charging` | `battery_node` | `/myRobot/battery_state` (`power_supply_status`) |
| script `/chargingBase/beacon` | `<h>StrengthSignal` | `charging_base_node` | `/myRobot/charging_base/strengthSignal` |
| script `/chargingBase/beacon` | `<h>RelativeAngle` | `charging_base_node` | `/myRobot/charging_base/relativeAngle` |
| sensor `/myRobot/forceSensor` | (lido com `sim.readForceSensor`) | `bumper_and_velocity_node` | `/myRobot/bumper` |

O beacon só responde **se for chamado**: o `charging_base_node` escreve o sinal
`Beacon = <h>`, e o script da base escreve a intensidade e o ângulo, mas apenas
se o robô estiver dentro do feixe.

## 2. Do ROS 2 para a cena (comandos)

| Tópico | Nó da ponte | O que faz na cena |
|---|---|---|
| `/myRobot/docking_mode` | `docking_node` | escreve `<h>Docking`. O `python_controler` só reage a `1`, marcando o checkbox "docking". |
| `/myRobot/cmd_vel` | `bumper_and_velocity_node` | converte `(v, ω)` em velocidade das rodas e chama `sim.setJointTargetVelocity` nas juntas `leftMotor`/`rightMotor` |

## 3. Diagrama completo

```
        CoppeliaSim (scripts da cena)                ROS 2
 ┌──────────────────────────────────────┐
 │ battery ─────── <h>Battery ─────────────▶ battery_node ──▶ /myRobot/battery_state ──┐
 │ beacon ──────── <h>Charging ────────────▶ battery_node                              │
 │ beacon ◀─────── Beacon = <h> ─────────────── charging_base_node (pedido)            │
 │ beacon ──────── <h>StrengthSignal ──────▶ charging_base_node ──▶ .../strengthSignal─┤
 │                 <h>RelativeAngle ───────▶ charging_base_node ──▶ .../relativeAngle ─┤
 │ forceSensor ────────────────────────────▶ bumper_and_velocity ──▶ /myRobot/bumper ──┤
 │                                      │                                              ▼
 │                                      │                                      ┌─────────────┐
 │ python_controler ◀── <h>Docking ────────── docking_node ◀── /docking_mode ◀──│ autodocking │
 │ juntas das rodas ◀── setJointTarget... ─── bumper_and_velocity ◀── /cmd_vel ◀│             │
 └──────────────────────────────────────┘                                      └─────────────┘
```

---

## 4. O controlador: autodocking.py

Rode a ponte e o controlador juntos:

```bash
ros2 launch ia368_pkg autodocking.launch.py
```

ou separados, que é melhor para depurar:

```bash
ros2 launch ia368_pkg remoteAPI_ROS2_bridge.launch.py
ros2 run ia368_pkg autodocking_node
```

### 4.1 Máquina de estados

```
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
                      de outro lugar│            ┌─────────────┐ carregou ┌────────┐
                                    │            │   CONTACT   │─────────▶│ DOCKED │
                                    │            └─────────────┘          └────────┘
                                    │                   │ não carregou em
                                    │             ┌────────┐ contact_wait s
                                    └─────────────│ BACKUP │◀──────────────┘
                                       recua e    └────────┘
                                       inverte o lado da busca
```

| Estado | O que faz | No enunciado? |
|---|---|---|
| `IDLE` | docking desligado: **não publica `cmd_vel`**, teleoperação livre | sim |
| `SEARCH` | gira no lugar até reencontrar o beacon | sim |
| `APPROACH` | alinha pelo `relativeAngle` e avança | sim |
| `DOCKED` | na base, carregando: parado | sim |
| `SWEEP` | a volta no lugar não achou nada: varre a vizinhança em **espiral** | não |
| `FINAL` | o beacon se apagou já encostando na base: empurra em frente às cegas | não |
| `CONTACT` | encostou em algo seguindo o beacon: para e espera para ver se a carga começa | não |
| `BACKUP` | não era a base: recua e tenta de novo pelo outro lado | não |

Os quatro últimos não estão no enunciado; sem eles o docking trava na prática
(ver [4.3](#43-as-armadilhas-desta-cena)).

### 4.2 Quem liga e desliga o docking

O nó liga o modo de docking sozinho quando a bateria cai abaixo de
`battery_low` (95 % por padrão) e o desliga quando ela chega a `battery_full`
(100 %), devolvendo o robô para a teleoperação. A bateria desta cena gasta **1 %
por segundo simulado** e começa em 100 %: são ~100 s de autonomia, e é por isso
que o limiar é generoso.

O nó também **assina** o `/myRobot/docking_mode` em que publica, para dar para
ligar e desligar o docking à mão, sem esperar a bateria baixar:

```bash
ros2 topic pub --once /myRobot/docking_mode std_msgs/msg/Int32 "{data: 1}"
ros2 topic pub --once /myRobot/docking_mode std_msgs/msg/Int32 "{data: 0}"
```

Desligar à mão **desarma** o gatilho automático até a bateria subir de novo
acima de `battery_low` — senão o nó religaria o docking no ciclo seguinte e
brigaria com o comando do usuário.

### 4.3 As armadilhas desta cena

Estas decisões do código não são estilo; sem elas o nó não funciona. As quatro
primeiras saem de ler os scripts da própria cena, cujas cópias de referência
estão em [coppeliasim/](coppeliasim/) — em especial
[beacon.lua](coppeliasim/beacon.lua), que define todo o protocolo.

**0. Confira o referencial do ângulo — e meça, não suponha.** O `beacon.lua`
calcula

```lua
local roombaOri = sim.getObjectOrientation(dockingSensorHandle, -1)
local relativeAngle = math.atan2(dy, dx) - roombaOri[3]
```

ou seja, o ângulo é medido no referencial do **`/myRobot/dockingSensor`**, e não
do corpo do robô. Nesta cena os dois felizmente coincidem, mas só dá para saber
medindo:

| Medida na cena | Valor |
|---|---|
| frente do robô | o eixo **+y** dele (mandando as rodas para frente, o deslocamento foi +0,373 m em y e +0,002 m em x) |
| `dockingSensor` no referencial do robô | posição `(0, +0,09)` m — no nariz — e yaw **+90°** |
| logo, o `+x` do sensor aponta para | o `+y` do robô, isto é, para a **frente** |

Então `relativeAngle` já **é** o erro de direção do robô, e `angle_target = 0`,
exatamente como diz o enunciado.

O detalhe importante é o **sintoma de quando esse parâmetro está errado**, porque
é um erro que não parece um erro: o robô acha o beacon, gira para alinhar, e
**perde o sinal justamente quando o erro chega a zero** — e aí volta a girar, num
laço infinito. Foi o que aconteceu num teste com `angle_target = -90°`:

```
[APPROACH] beacon: força 0.72, erro +104°
[APPROACH] beacon: força 0.72, erro  +53°
[APPROACH] beacon: força 0.72, erro  +10°
APPROACH -> SEARCH          <-- perdeu o sinal ao alinhar
```

A causa é geométrica: alinhar com o alvo errado tira o `dockingSensor` do
corredor estreito do feixe (ver [4.6](#46-limitações-conhecidas)). Se você vir
esse padrão no log — força constante e erro indo a zero antes de perder o sinal —
é o `angle_target`, não o `angle_sign`.

1. **Em `IDLE` o nó fica calado.** O `bumper_and_velocity_node` não tem prazo de
   validade: ele repassa o último `cmd_vel` e o robô fica com ele. Se o nó
   publicasse `Twist` zerado continuamente em `IDLE`, a teleoperação (o joystick
   da cena ou o `teleop_twist_keyboard`) não conseguiria mover o robô. Então ele
   manda **um** `Twist` zerado ao entrar em `IDLE` (para o robô não herdar o
   último comando do docking) e depois se cala. Medido: com a bateria cheia, o nó
   publica **zero** mensagens em `/myRobot/cmd_vel`.

2. **O beacon não publica "zero", ele simplesmente para de publicar.** Fora do
   feixe o `charging_base_node` não manda nada, então "perdi o beacon" é medido
   por **tempo**: sem mensagem nova por `signal_timeout` (0,5 s), o sinal está
   perdido. Não há valor sentinela para testar.

3. **O estado de carga do `/myRobot/battery_state` não é confiável.** O sinal
   `<h>Charging` da cena é apagado pelo script `/myRobot/battery` assim que ele
   o lê, e muitas vezes a ponte chega tarde — o `power_supply_status` fica em
   `DISCHARGING` mesmo com o robô carregando. Então o nó junta duas evidências:
   o `power_supply_status`, quando chega, **e a bateria subindo**, que nesta cena
   só pode significar carga.

   > Lição geral: sempre que um script da cena **apaga** um sinal depois de ler,
   > esse sinal vira um evento de uso único, e quem chega em segundo lugar não vê
   > nada. Vale para `Charging`, `Docking` e `leftVel`/`rightVel`.

4. **Chegando perto, o beacon se cala — e isso significa "cheguei".** O
   `getBeaconInfo` só responde se o objeto detectado pelo feixe for
   **exatamente** o `dockingSensor`:

   ```lua
   if detectedObjectHandle == dockingSensorHandle then
       return signalStrength, relativeAngle
   else
       return nil, nil
   ```

   Junto à base o para-choque passa à frente do sensor e o beacon emudece. Um nó
   ingênuo entenderia "perdi o sinal" e giraria, jogando fora um docking pronto.
   Por isso, perder o sinal com `strength >= strength_slow` (ou seja, perto) leva
   ao `FINAL`, um empurrão em frente às cegas, e não ao `SEARCH`. Pelo mesmo
   motivo o `APPROACH` testa o **para-choque antes** da perda de sinal: junto à
   base as duas coisas acontecem no mesmo instante, e o contato é a informação
   melhor.

5. **O para-choque dispara ~1 s antes da carga.** A bateria da cena é
   atualizada a 1 Hz, e a carga é verificada pelo beacon também a 1 Hz. Por isso
   o `CONTACT`: encostar em algo **não** manda recuar na hora, manda parar e
   esperar `contact_wait` (3 s). Sem isso o robô recuaria de um encaixe que deu
   certo.

### 4.4 Convenções de unidade

| Grandeza | Unidade | Observação |
|---|---|---|
| `relativeAngle` | rad, −π…π | `0` = base à frente do `dockingSensor`, que nesta cena aponta para a frente do robô — então `0` = base à frente do robô (ver [4.3](#43-as-armadilhas-desta-cena)) |
| `strengthSignal` | 0…1 | `1 − distância / volume_range`, com `volume_range = 2 m`. Então 0,85 ≈ 0,30 m e 0,95 ≈ 0,10 m |
| `battery_state.percentage` | **0…100** | a ponte publica em %, mas a convenção da mensagem `BatteryState` do ROS 2 é 0…1; o nó aceita as duas (acima de 1 já está em %) |
| `cmd_vel.linear.x` | m/s | positivo = para a frente (a ponte cuida do sinal das juntas) |
| `cmd_vel.angular.z` | rad/s | positivo = girar à esquerda |
| `bumper` | N | o sensor mede o peso do para-choque mesmo sem colisão, então o nó calibra uma *baseline* nas primeiras 20 leituras e compara o **desvio** |

### 4.5 Parâmetros que você provavelmente vai querer mexer

```bash
ros2 run ia368_pkg autodocking_node --ros-args \
    -p battery_low:=95.0 -p angle_sign:=1.0 -p approach_speed:=0.25
```

| Parâmetro | Padrão | Para que serve |
|---|---|---|
| `angle_target` | 0.0 | valor de `relativeAngle` que significa "base à frente do robô". É `−(yaw do dockingSensor em relação à frente do robô)`; aqui dá 0 |
| `angle_sign` | 1.0 | sentido do ângulo; **se o robô girar para longe da base, ponha −1.0** |
| `approach_speed` / `final_speed` | 0.25 / 0.05 | m/s indo para a base e já encostando |
| `strength_slow` | 0.85 | intensidade acima da qual anda devagar |
| `align_threshold` | 0.35 rad | acima disso gira parado antes de avançar |
| `search_angular_speed` / `search_spin_time` | 1.0 rad/s / 3.0 s | a volta no lugar que o enunciado pede |
| `sweep_speed` / `sweep_spacing` / `sweep_max_radius` | 0.25 m/s / 0.5 m / 1.5 m | a varredura em **espiral de Arquimedes**: `dr/dt = spacing·v / (2π·r)`, `w = v/r` (espaçamento constante entre voltas; ~55 s e ~2,7 voltas até 1,5 m) |
| `battery_low` / `battery_full` | 95.0 / 100.0 | % que liga e desliga o docking |
| `final_push_time` | 3.0 s | empurrão às cegas quando o beacon se cala junto à base |
| `contact_wait` | 3.0 s | espera, após encostar, para ver se a carga começa |
| `bumper_threshold` | 1.0 N | desvio da baseline que conta como colisão |
| `data_timeout` | 2.0 s | sem `battery_state` por este tempo, o nó para o robô |

Os dois primeiros a conferir são `angle_target` e `angle_sign`: eles dependem de
como a cena mede o ângulo, e são a diferença entre o robô ir para a base ou
passar ao lado dela. O log do nó mostra o **erro já corrigido**, em graus, a cada
segundo — se o erro vai a zero e o robô ainda não chega na base, é o
`angle_target` que está errado:

```
[APPROACH] bateria 36% | docking ON | descarregando | beacon: força 0.51, erro +2° | bumper 0.00 N
```

### 4.6 A busca do feixe, e o orçamento de bateria

**O feixe da base é um corredor estreito, e o portão de detecção é mais duro do
que parece.** O `beacon.lua` faz:

```lua
state, distance, detectPoint, detectedObjectHandle = sim.readProximitySensor(beaconHandle)
if detectedObjectHandle == dockingSensorHandle then ...
```

`readProximitySensor` devolve o objeto **mais próximo** dentro do volume. Não
basta o `dockingSensor` estar no feixe: ele tem de ser o primeiro da fila, e o
sofá, as paredes e o próprio corpo do robô competem por essa posição. (Cuidado
ao depurar: `sim.checkProximitySensor(feixe, sensor)` testa só aquele objeto e é
bem mais permissivo que o portão real — foi uma pegadinha em que eu caí ao medir
isto.)

**Por que ESPIRAL e não arco de raio fixo.** Um arco de raio fixo é um círculo:
ele volta ao ponto de partida e refaz o mesmo caminho, então não explora,
oscila. A espiral abre o raio a cada volta. E o que faz isso funcionar sem
precisar cobrir área é a geometria: **o corredor sai radialmente da base, então
qualquer laço que circunde a base o cruza.** Conforme a espiral abre, em algum
momento ela passa a circundar a base — e aí acha.

**Por que espiral de ARQUIMEDES.** A primeira versão fazia o raio crescer a uma
taxa constante no tempo (`r = r0 + k·t`). Como cada volta demora `2π·r/v`, o
raio crescia ~7x por volta — era uma espiral *logarítmica*: o robô dava ~1
volta, já passava de `sweep_max_radius`, girava no lugar e recomeçava de outro
ponto. Na prática, laços soltos que não cobriam a vizinhança (o sintoma de "a
busca em espiral não funciona"). Agora o raio é integrado com
`dr/dt = sweep_spacing·v / (2π·r)`, o que dá espaçamento constante
(`sweep_spacing`) entre voltas vizinhas, e o raio já aberto é mantido através de
um BACKUP (uma batida no sofá não faz a espiral recomeçar do zero).

Cobrir área de verdade seria inviável: um disco de raio 1 m com 0,2 m de
espaçamento entre voltas dá ~21 m de caminho, ~70 s a 0,3 m/s, e o robô só tem
~100 s de bateria.

**O que realmente limita é a bateria, e a medida é em bateria, não em segundos.**
Com os quatro nós da ponte conversando com o simulador, a simulação roda a
~0,6x do tempo real, então cronômetro de parede engana. A bateria gasta 1 % por
segundo **simulado**, e é a métrica certa. Medido nesta cena, numa execução do
sistema completo a partir de uma pose a 0,5 m da base e fora do corredor:

```
docking ligou com 60%  ->  beacon achado com 41%  ->  encaixou com 38%
                           (19% na busca)            (3% na aproximação)
```

Ou seja, ~22 % de bateria do momento em que o docking liga até encaixar. Com os 40 %
usados antes, o orçamento ficava do tamanho do custo e o robô morria no meio da
busca — era exatamente o sintoma de "não consegue fazer o docking". O padrão
agora é `battery_low = 95` (e `battery_full = 100`, que precisa ficar acima): o
docking liga ~5 s depois do início e o robô tem quase toda a bateria para a
busca.

**Variação.** Isoladamente, todos os padrões (arco e espiral, vários ajustes)
acham o feixe em 4-8 s a partir dessa pose. A variação grande entre execuções do
nó inteiro vem dos **obstáculos**, não do padrão de busca: um BACKUP no sofá
custa vários por cento e muda o centro da espiral. Não vale apertar mais os
números da espiral; o que reduz a variação é começar mais perto da base.

**Recomendações práticas:**

- Deixe `battery_low` generoso (o padrão é 95 %). É o parâmetro que decide entre
  funcionar e não funcionar.
- Para demonstrar com segurança, leve o robô para perto da base com a
  teleoperação e então ative o `docking_mode` — é o cenário que o enunciado
  descreve ("se o modo de docking está ON e o sinal é detectado"). A aproximação
  em si é a parte confiável: ~3 % de bateria.
- **Não há desvio de obstáculos.** O para-choque é um sensor de contato: o nó
  recua e tenta de novo, recuando um pouco mais a cada tentativa seguida, mas
  não sabe por onde contornar. O que resolveria é publicar o
  `/myRobot/proximitySensor` (e a odometria) na ponte, o que sai do escopo desta
  atividade — o enunciado manda o controlador usar só os tópicos existentes.

---

## 5. Ritmo: por que tudo é mais devagar do que parece

| O quê | Frequência |
|---|---|
| Passo da simulação | 20 Hz |
| Bateria e verificação de carga do lado da cena | 1 Hz |
| Resposta do beacon | a cada passo, se for pedida |
| Leitura dos sensores pela ponte | 10 Hz |
| Laço de controle do `autodocking` | 10 Hz |

A bateria gasta **1 % por segundo** e carrega **1 % por segundo** (medido em
[coppeliasim/battery.lua](coppeliasim/battery.lua): `energy_decay_rate` e
`energy_charge_rate` = 1, a cada 1000 ms). Começando em 100 %, a autonomia é de
~100 s; e carregar até os 100 % que soltam o robô leva, no máximo, o tempo que
faltava de bateria (ex.: ~40 s se encaixar com 60 %).

O limite duro: **com a simulação rodando, cada chamada da Remote API custa
~12 ms**. Ela espera a vez entre passos de simulação, o que dá um teto de ~80
chamadas por segundo para a ponte inteira. Por isso publicar `cmd_vel` a 100 Hz
não acelera nada — só forma fila. Entre 10 e 20 Hz é o ponto certo.

---

## 6. Depurando: teste um elo de cada vez

A cadeia é longa, então não pergunte "por que não funciona?" — pergunte "até
onde funciona?".

```bash
# 1. Os nós estão vivos e os tópicos existem?
ros2 node list
ros2 topic list | grep myRobot

# 2. A cena está produzindo dados? (elo CoppeliaSim → ponte)
ros2 topic echo /myRobot/battery_state --field percentage
ros2 topic echo /myRobot/charging_base/relativeAngle

# 3. O controlador está comandando? (elo autodocking → ponte)
ros2 topic echo /myRobot/cmd_vel
ros2 topic echo /myRobot/docking_mode

# 4. O robô obedece? (elo ponte → cena)
ros2 topic pub -r 10 /myRobot/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}}'
```

- Se o passo 2 não mostra nada, o problema está na cena: simulação parada,
  script desabilitado, ou o robô fora do feixe do beacon (normal!).
- Se o 2 funciona e o 3 fica em `IDLE` para sempre, o docking não ligou: veja a
  bateria, ou ligue à mão com o `ros2 topic pub` da seção 4.2.
- Se o 3 publica mas o robô não anda, olhe a bateria (em 0 % o
  `python_controler` para os motores) e confirme que a cena está com a versão
  modificada do `python_controler` (ver seção 7).
- Se o robô anda mas **vira para longe da base**, é o `angle_sign`.

---

## 7. Scripts da cena que não passam pelo ROS 2

- **`python_controler`:** o joystick da cena. Também escreve nas juntas das
  rodas; para não apagar o comando do `cmd_vel`, a cena precisa da versão
  modificada ([coppeliasim/python_controler.py](coppeliasim/python_controler.py)),
  que só escreve quando o joystick está em uso. Instale-a com
  [coppeliasim/apply_scene_patch.py](coppeliasim/apply_scene_patch.py).
- **`odometry`:** estima `(x, y, θ)` pelos encoders e grava na propriedade
  `customData.Odometry` do robô. Nenhum nó ROS 2 lê esse valor — o docking desta
  atividade não precisa de odometria, porque navega pelo beacon.

A pasta [coppeliasim/](coppeliasim/) tem **cópias de referência** dos scripts que
estão dentro da cena, extraídas com a Remote API. Editá-las não muda a cena
(exceto o `python_controler.py`, que o `apply_scene_patch.py` instala); elas
estão ali para ler:

| Arquivo | Script da cena | Por que importa |
|---|---|---|
| [beacon.lua](coppeliasim/beacon.lua) | `/chargingBase/beacon` | define **todo** o protocolo do beacon: a convenção do ângulo, a escala da intensidade, e os dois gates (detectar o `dockingSensor` para responder; 0,1 m para carregar) |
| [battery.lua](coppeliasim/battery.lua) | `/myRobot/battery` | 1 %/s de gasto e de carga; **apaga** o sinal `<h>Charging` |
| [dockingSensor_sensorScript.lua](coppeliasim/dockingSensor_sensorScript.lua) | `/myRobot/dockingSensor/sensorScript` | documenta a convenção do ângulo e gira a seta de retorno visual |
| [python_controler.py](coppeliasim/python_controler.py) | `/myRobot/python_controler` | o joystick; **esta é a versão modificada** que a cena precisa |
| [odometry.lua](coppeliasim/odometry.lua) | `/myRobot/odometry` | odometria por encoders (não usada por esta atividade) |
