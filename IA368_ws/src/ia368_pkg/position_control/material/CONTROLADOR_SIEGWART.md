# Controle de posição de Siegwart: implementação e fundamentos

Este documento explica o controlador implementado em [position_control_node_students.py](position_control_node_students.py): a cinemática do robô, a lei de controle da **Task 1**, as extensões da **Task 2** (movimento de ré e velocidade constante) e o motivo de cada escolha.

Referência: R. Siegwart, I. Nourbakhsh, D. Scaramuzza, *Introduction to Autonomous Mobile Robots*, 2ª ed., seção 3.6.2.4 ("Feedback control").

---

## 1. O robô: cinemática de tração diferencial

O robô da cena é um Pioneer P3-DX: duas rodas motrizes independentes e uma roda-boba (*caster*). Na cena, os parâmetros são:

| Símbolo | Significado | Valor |
|---|---|---|
| $r$ | raio da roda | 0,0975 m |
| $L$ | distância entre as rodas | 0,331 m |

### Cinemática direta (usada pelo `odom_node`)

Das velocidades angulares das rodas $\dot\varphi_R, \dot\varphi_L$ obtemos a velocidade linear $v$ e a angular $\omega$ do robô:

$$
v = \frac{r(\dot\varphi_R + \dot\varphi_L)}{2}, \qquad
\omega = \frac{r(\dot\varphi_R - \dot\varphi_L)}{L}
$$

Integrando no referencial do mundo, temos o **modelo uniciclo**:

$$
\dot x = v\cos\theta, \qquad \dot y = v\sin\theta, \qquad \dot\theta = \omega
$$

### Cinemática inversa (usada pelo `vel_node`)

O controlador produz $(v, \omega)$, e o `vel_node` converte para as velocidades das rodas:

$$
\dot\varphi_R = \frac{v + \frac{L}{2}\omega}{r}, \qquad
\dot\varphi_L = \frac{v - \frac{L}{2}\omega}{r}
$$

### A restrição não-holonômica

As rodas não deslizam lateralmente, e isso impõe

$$
\dot x \sin\theta - \dot y \cos\theta = 0
$$

Ou seja, o robô só se move na direção para a qual está apontando. Ele tem 3 graus de liberdade de configuração $(x, y, \theta)$, mas apenas 2 entradas $(v, \omega)$.

**Consequência importante:** pelo teorema de Brockett (1983), **não existe** uma realimentação de estado suave e invariante no tempo, $u = k(x, y, \theta)$, que estabilize esse sistema em um ponto. Um controlador proporcional ingênuo em $(x, y, \theta)$ não funciona. O controlador de Siegwart contorna o teorema mudando para **coordenadas polares**: essa transformação é descontínua em $\rho = 0$, então a lei de controle, vista nas coordenadas originais, não é suave na origem.

---

## 2. Coordenadas polares em relação ao alvo

O estado do robô é descrito em relação ao alvo $(x_g, y_g, \theta_g)$:

```
                         alvo (θ_g)
                            ●──────►
                          ╱
                    ρ   ╱
                      ╱  ↖ direção robô→alvo: atan2(Δy, Δx)
                    ╱
         robô ●───────►  θ (orientação do robô)
                  α = ângulo entre a frente do robô e a direção do alvo
```

| Variável | Definição | Significado físico |
|---|---|---|
| $\rho$ | $\sqrt{\Delta x^2 + \Delta y^2}$ | distância até o alvo |
| $\alpha$ | $\operatorname{atan2}(\Delta y, \Delta x) - \theta$ | quanto o robô precisa girar para **apontar** para o alvo |
| $\beta$ | $\theta_g - \theta - \alpha$ | quanto falta, depois de apontar para o alvo, para atingir a **orientação final** $\theta_g$ |

com $\Delta x = x_g - x$ e $\Delta y = y_g - y$. Todos os ângulos são normalizados em $[-\pi, \pi]$.

Chegar ao alvo com a orientação certa equivale a levar $(\rho, \alpha, \beta) \to (0, 0, 0)$.

> **Diferença em relação ao template.** O livro assume o alvo na origem com $\theta_g = 0$, o que dá $\beta = -\theta - \alpha$. O template usava essa fórmula, mas o alvo da cena tem orientação arbitrária e a condição de parada exige $|\theta - \theta_g| < $ `angle_threshold`. Por isso a implementação usa $\beta = \theta_g - \theta - \alpha$. Isso equivale a expressar o problema no referencial do alvo, onde a orientação final é 0.

### Dinâmica em coordenadas polares

Derivando as definições e substituindo o modelo uniciclo, para $\alpha \in I_1 = (-\pi/2, \pi/2]$ (alvo à frente):

$$
\begin{aligned}
\dot\rho  &= -v\cos\alpha \\
\dot\alpha &= \frac{v\sin\alpha}{\rho} - \omega \\
\dot\beta  &= -\frac{v\sin\alpha}{\rho}
\end{aligned}
$$

Interpretação física:
- **$\dot\rho$:** só a componente de $v$ na direção do alvo ($v\cos\alpha$) diminui a distância.
- **$\dot\alpha$:** a componente lateral ($v\sin\alpha$) faz a linha de visada girar, com efeito maior quanto mais perto do alvo ($1/\rho$). Girar o robô ($\omega$) corrige $\alpha$ diretamente.
- **$\dot\beta$:** $\beta$ só muda pelo movimento de translação. O robô não consegue corrigir a orientação final girando parado; ele precisa **fazer uma curva** enquanto se aproxima. É a restrição não-holonômica aparecendo.

---

## 3. Task 1: a lei de controle

### Implementação

```python
vu    = direction * self.Krho * rho                 # [m/s]
omega = self.Kalpha * alpha + self.Kbeta * beta     # [rad/s]
```

$$
v = k_\rho\,\rho, \qquad \omega = k_\alpha\,\alpha + k_\beta\,\beta
$$

- **$v = k_\rho\,\rho$:** o robô anda mais rápido longe do alvo e desacelera suavemente até parar.
- **$k_\alpha\,\alpha$:** gira o robô para apontar para o alvo.
- **$k_\beta\,\beta$ (com $k_\beta < 0$):** gira no sentido oposto, "abrindo" a curva para que o robô chegue ao alvo já alinhado com $\theta_g$.

### Sistema em malha fechada

Substituindo a lei de controle na dinâmica polar:

$$
\begin{aligned}
\dot\rho  &= -k_\rho\,\rho\cos\alpha \\
\dot\alpha &= k_\rho\sin\alpha - k_\alpha\alpha - k_\beta\beta \\
\dot\beta  &= -k_\rho\sin\alpha
\end{aligned}
$$

O termo $1/\rho$ some: como $v$ é proporcional a $\rho$, a razão $v/\rho = k_\rho$ é constante e o sistema não tem singularidade ao se aproximar do alvo.

### Estabilidade (linearização em torno do alvo)

Para $\alpha$ pequeno, $\cos\alpha \approx 1$ e $\sin\alpha \approx \alpha$:

$$
\begin{bmatrix}\dot\rho\\ \dot\alpha\\ \dot\beta\end{bmatrix}
=
\begin{bmatrix}
-k_\rho & 0 & 0\\
0 & -(k_\alpha - k_\rho) & -k_\beta\\
0 & -k_\rho & 0
\end{bmatrix}
\begin{bmatrix}\rho\\ \alpha\\ \beta\end{bmatrix}
$$

O polinômio característico é

$$
(\lambda + k_\rho)\left(\lambda^2 + (k_\alpha - k_\rho)\lambda - k_\rho k_\beta\right) = 0
$$

Todos os autovalores têm parte real negativa (critério de Routh-Hurwitz) se e somente se:

$$
\boxed{k_\rho > 0, \qquad k_\beta < 0, \qquad k_\alpha - k_\rho > 0}
$$

O livro também dá uma condição de **estabilidade forte**, que garante que $\alpha$ permaneça em $I_1$ durante todo o trajeto (o robô nunca "troca de lado" e tem que dar ré):

$$
k_\alpha + \tfrac{5}{3}k_\beta - \tfrac{2}{\pi}k_\rho > 0
$$

### Ganhos escolhidos

Os ganhos padrão do template ($0{,}1$ para os três) **violam** duas condições ($k_\beta > 0$ e $k_\alpha - k_\rho = 0$). A implementação usa o exemplo do livro ($k_\rho=3$, $k_\alpha=8$, $k_\beta=-1{,}5$) escalado por 0,1 para velocidades compatíveis com o Pioneer:

| Ganho | Valor | Verificação |
|---|---|---|
| $k_\rho$ | 0,3 | $> 0$ ✓ |
| $k_\alpha$ | 0,8 | $k_\alpha - k_\rho = 0{,}5 > 0$ ✓ |
| $k_\beta$ | −0,15 | $< 0$ ✓ |
| Estabilidade forte | | $0{,}8 - 0{,}25 - 0{,}19 = 0{,}36 > 0$ ✓ |

Autovalores do sistema linearizado:

$$
\lambda_1 = -0{,}30, \qquad \lambda_{2,3} = \frac{-0{,}5 \pm \sqrt{0{,}25 - 0{,}18}}{2} \approx -0{,}12,\ -0{,}38
$$

São todos reais e negativos: a aproximação é **superamortecida**, sem oscilação em torno do alvo. A constante de tempo mais lenta é $1/0{,}12 \approx 8$ s, o que dita quanto tempo o robô leva para alinhar a orientação final.

**Efeito de cada ganho, na prática:**
- **$k_\rho$ maior:** o robô anda mais rápido, mas a velocidade inicial ($k_\rho\rho$) pode saturar os motores quando o alvo está longe.
- **$k_\alpha$ maior:** o robô aponta para o alvo mais agressivamente.
- **$|k_\beta|$ maior:** a orientação final é corrigida mais cedo, com curvas mais abertas.

---

## 4. Task 2a: movimento de ré (`backwardAllowed`)

### O problema

As equações acima valem para $\alpha \in I_1$, com o alvo à frente. Se o alvo está atrás ($\alpha \in I_2 = (-\pi, -\pi/2] \cup (\pi/2, \pi]$), andando só para a frente o robô precisa primeiro dar meia-volta, o que gera uma trajetória longa.

### A ideia física

Um robô diferencial é simétrico: andar de ré é o mesmo que andar para a frente com a "frente" redefinida como a traseira. Definimos um **robô virtual** com orientação $\theta' = \theta + \pi$. Para ele, o alvo que estava atrás agora está à frente.

### Como os ângulos mudam

Para o robô virtual:

$$
\alpha' = \operatorname{atan2}(\Delta y, \Delta x) - (\theta + \pi) = \alpha - \pi \equiv \alpha + \pi \pmod{2\pi}
$$

A orientação final também muda. Se o robô virtual chega com $\theta' = \theta_g$, o robô real chega com $\theta = \theta_g - \pi$, que é o lado errado. O robô virtual precisa mirar $\theta'_g = \theta_g + \pi$:

$$
\beta' = \theta'_g - \theta' - \alpha' = (\theta_g + \pi) - (\theta + \pi) - (\alpha - \pi) = \beta + \pi
$$

As velocidades do robô real a partir do virtual:
- **$v = -v'$:** andar para a frente com o virtual é andar de ré com o real.
- **$\omega = \omega'$:** o sentido de rotação é o mesmo nos dois referenciais.

### Implementação

```python
direction = 1.0
if self.backwardAllowed and not (-math.pi / 2 < alpha <= math.pi / 2):
    direction = -1.0
    alpha = self.normalize_angle(alpha + math.pi)
    beta = self.normalize_angle(beta + math.pi)
...
vu = direction * self.Krho * rho
```

A mesma lei de controle da Task 1 é aplicada ao robô virtual, e só o sinal de $v$ é invertido. Toda a análise de estabilidade da seção 3 continua valendo.

**Limitação:** o modo é escolhido a cada iteração pelo valor de $\alpha$. Se $\alpha$ oscilar em torno de $\pm\pi/2$, o controlador pode alternar entre frente e ré, e $\beta$ salta $\pi$ na troca, gerando um degrau em $\omega$. Com os ganhos acima (estabilidade forte), $\alpha$ tende a ficar em um único intervalo; se aparecer esse vaivém, a solução é adicionar histerese na troca de modo.

---

## 5. Task 2b: velocidade constante (`useconstantSpeed`)

### O problema

Com $v = k_\rho\rho$, o robô é rápido longe do alvo e muito lento perto dele. Em vários cenários é desejável andar em velocidade constante $v_c$ (`constantSpeed`), sem perder o comportamento de convergência.

### A ideia física: preservar a curvatura

A **forma** da trajetória no plano depende apenas da **curvatura**:

$$
\kappa = \frac{\omega}{v} = \frac{1}{R_{\text{curva}}}
$$

Se $v$ e $\omega$ forem multiplicados pelo mesmo fator positivo $s$, a curvatura não muda. O robô percorre **exatamente o mesmo caminho**, apenas mais rápido ou mais devagar.

Matematicamente: o sistema em malha fechada $\dot X = f(X)$ e o sistema $\dot X = s(X)\,f(X)$, com $s(X) > 0$, têm as **mesmas órbitas** (curvas no espaço de estados); só a parametrização no tempo muda ($dt' = dt/s$). Estabilidade e convergência são propriedades das órbitas, então são preservadas.

### Implementação

Escolhemos $s$ para que a velocidade linear seja exatamente $v_c$:

$$
s = \frac{v_c}{k_\rho\,\rho} \quad\Rightarrow\quad
v = v_c, \qquad \omega = s\,(k_\alpha\alpha + k_\beta\beta)
$$

```python
if self.useconstantSpeed and rho > self.dist_threshold:
    scale = self.constantSpeed / (self.Krho * rho)
    vu = direction * self.constantSpeed
    omega *= scale
```

Isso funciona junto com o modo de ré: `direction` mantém o sinal de $v$.

### Por que a condição `rho > dist_threshold`

O fator $s = v_c/(k_\rho\rho)$ cresce sem limite quando $\rho \to 0$, e com ele $\omega \propto 1/\rho$. Perto do alvo isso significa:
- rotações muito rápidas, que os motores não conseguem executar (saturação);
- com o controle discreto (20 Hz) e a odometria a 10 Hz, um $\omega$ grande entre duas amostras gira o robô demais e causa *overshoot*.

Por isso, dentro do raio `dist_threshold` o controlador volta à lei proporcional da Task 1, que desacelera suavemente e corrige a orientação final.

---

## 6. Condição de parada

Em `control_loop`, o robô para ($v = \omega = 0$) quando:

$$
\rho < \texttt{dist\_threshold}\ (0{,}1\text{ m}) \quad\text{e}\quad |\theta - \theta_g| < \texttt{angle\_threshold}\ (0{,}1\text{ rad} \approx 5{,}7°)
$$

Os limiares são necessários porque a convergência é assintótica: em teoria $\rho$ só chega a zero em tempo infinito, e ruído de odometria faria o robô "tremer" em torno do alvo.

---

## 7. Validação

A lei de controle foi simulada fora do CoppeliaSim, integrando o modelo uniciclo (seção 1) com passo de 50 ms, nas 4 combinações de opções e em 4 cenários, incluindo alvos atrás do robô e orientações finais opostas:

| Início $(x, y, \theta)$ | Alvo $(x, y, \theta)$ | Normal | Ré | Vel. const. | Ré + vel. const. |
|---|---|---|---|---|---|
| (0, 0, 0) | (2, 1, 0,5) | 15,6 s | 15,6 s | 26,9 s | 26,9 s |
| (0, 0, 0) | (−2, 0,5, 0) | 34,6 s | **18,1 s** | 59,2 s | 27,6 s |
| (0, 0, 1,5) | (1,5, −1,5, −2,0) | 12,3 s | 27,7 s | 33,4 s | 40,6 s |
| (1, 1, 3,0) | (−1, −2, 1,0) | 33,9 s | 33,9 s | 69,5 s | 69,5 s |

Todos os 16 casos convergiram dentro dos limiares. Observações:
- **Alvo atrás do robô (2ª linha):** a ré reduz o tempo quase pela metade, porque o robô não precisa dar meia-volta.
- **Velocidade constante:** com $v_c = 0{,}1$ m/s, os tempos são maiores, já que no modo proporcional o robô começa bem mais rápido ($k_\rho\rho \approx 0{,}67$ m/s a 2,2 m). O caminho percorrido é o mesmo.
- **Ré mais lenta na 3ª linha:** o modo de ré nem sempre é mais rápido. Aqui o alvo está atrás, mas a orientação final pede chegar de frente, e a ré obriga uma manobra mais longa para alinhar $\theta_g$.

No CoppeliaSim o resultado será diferente: o robô tem inércia e atrito, os motores saturam e as rodas podem patinar. Os ganhos podem precisar de ajuste fino.

---

## 8. Cuidados no uso real

- **Referenciais:** o `odom_node` integra a pose a partir de $(0, 0, 0)$, enquanto o `target_node` publica o alvo em coordenadas do **mundo** da cena. Os dois só coincidem se o robô começar na origem do mundo com $\theta = 0$. Caso contrário, o robô vai para um ponto deslocado.
- **Deriva da odometria:** a pose vem da integração das velocidades das rodas, então o erro acumula com o tempo (deslizamento, discretização). Em trajetos longos, o robô para perto, mas não exatamente sobre o alvo visual.
- **Taxas diferentes:** o controlador roda a 20 Hz, a odometria a 10 Hz e o `vel_node` aplica comandos a cada 0,4 s (2,5 Hz). Esse último é o gargalo: ganhos muito altos combinados com esse atraso podem causar oscilações que o modelo contínuo não prevê.

---

## 9. Como ajustar os parâmetros

Os parâmetros são declarados no nó ROS 2 e podem ser alterados sem editar o código:

```bash
ros2 run ia368_pkg pos_control_node --ros-args \
    -p Krho:=0.3 -p Kalpha:=0.8 -p Kbeta:=-0.15 \
    -p backwardAllowed:=true \
    -p useconstantSpeed:=true -p constantSpeed:=0.2
```

Ao mudar os ganhos, confira sempre as condições de estabilidade da seção 3.
