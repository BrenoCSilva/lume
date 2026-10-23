# Sincronização Temporal e Compensação de Latência (`consume_motion_command_time`)

A função [`consume_motion_command_time()`](file:///home/lume/astro/src/obstacle_avoider/obstacle_avoider_main.cpp#L387-L465) é a primeira etapa crítica executada dentro do [`obstacle_avoider_timer_handler()`](file:///home/lume/astro/src/obstacle_avoider/obstacle_avoider_main.cpp#L1300). Ela é responsável por manter a fila de comandos de movimento rigorosamente sincronizada com o relógio do veículo, resolvendo os dois maiores desafios da robótica móvel em malha fechada: **a latência de cálculo do planejador (MPP)** e **o avanço temporal contínuo da fila de comandos durante a condução**.

---

## 1. Por que essa função é indispensável?

Em um veículo autônomo, o planejador de trajetória ([MPP - Model Predictive Planner](file:///home/lume/astro/src/mpp/)) e o controlador de segurança ([Obstacle Avoider](file:///home/lume/astro/src/obstacle_avoider/)) rodam em frequências e processos distintos:

1. **O MPP opera em escala de centenas de milissegundos:**  
   Ele precisa integrar equações de clothoides, calcular envelopes de colisão, resolver otimizações e gerar até 400 comandos à frente. Esse cálculo consome tipicamente entre $100\text{ ms}$ e $350\text{ ms}$.
2. **O relógio do robô não para:**  
   Enquanto o MPP calculava o novo plano, o caminhão continuou se deslocando fisicamente pelo asfalto.
3. **O problema do atraso acumulado:**  
   Se o Obstacle Avoider recebesse o novo plano e simplesmente executasse o primeiro comando (`Cmd[0]`), o veículo estaria executando uma ordem que foi pensada para onde ele estava **no passado**. O resultado seria erro de perseguição de trajetória (*tracking error*), solavancos no volante e desvios de rota.

> [!IMPORTANT]
> **Missão da Função:**  
> `consume_motion_command_time()` garante que o comando na posição zero da fila (`Cmd[0]`) corresponda **exatamente ao instante presente no relógio do veículo**, descartando fatias obsoletas do passado e fatiando o comando ativo no tempo.

---

## 2. Anatomia dos Comandos de Movimento

Cada elemento do vetor `motion_commands_vector[i]` é da estrutura `astro_robot_and_trailers_motion_command_t`:

```cpp
typedef struct
{
    double v;       // Velocidade linear longitudinal desejada (m/s)
    double phi;     // Ângulo de esterçamento do rodado dianteiro (radianos)
    double time;    // Duração deste degrau de comando (dt em segundos!)
} astro_robot_and_trailers_motion_command_t;
```

> [!NOTE]
> O campo `.time` **NÃO é uma data absoluta (timestamp)**. Ele é um **intervalo de tempo ($\Delta t$)**, como $0.05\text{ s}$ ($50\text{ ms}$) ou $0.02\text{ s}$ ($20\text{ ms}$).  
> Um vetor com 400 comandos de $0.05\text{ s}$ projeta um horizonte de predição de:
> $$T_{\text{horizonte}} = 400 \times 0{,}05\text{ s} = 20{,}0\text{ segundos no futuro}$$

---

## 3. O Algoritmo Passo a Passo e a Lógica Matemática

O código de [`consume_motion_command_time()`](file:///home/lume/astro/src/obstacle_avoider/obstacle_avoider_main.cpp#L387-L465) inicia obtendo o relógio atual do robô e o carimbo de referência:

```cpp
static void
consume_motion_command_time(int motion_command_vetor, FILE *out_act)
{
    int i, j;
    double current_time, time_consumed;

    if (use_timestamp_from_audit)
        current_time = timestamp_of_motion_commands_vector[motion_command_vetor];
    else
        current_time = astro_get_time();

    time_consumed = astro_robot_ackerman_motion_command_time_of_last_update;
```

* **`current_time`**: O relógio do sistema no exato instante presente (`astro_get_time()`).
* **`time_consumed`**: Inicia com `astro_robot_ackerman_motion_command_time_of_last_update`. Essa variável guarda:
  * O timestamp em que o MPP gerou o plano (quando chega um novo plano do MPP); OU
  * O timestamp do último consumo efetuado (quando o timer periódico está rodando).

---

### Etapa 1: A Varredura de Término dos Comandos

```cpp
    for (i = 0; i < num_motion_commands_in_vector[motion_command_vetor]; i++)
    {
        time_consumed += motion_commands_vector[motion_command_vetor][i].time;

        if (time_consumed > current_time)
        {
            break;
        }
    }
```

A cada iteração do `for`, a pergunta matemática que está sendo feita é:
> **"Em que segundo exato do relógio o comando $i$ vai terminar de ser executado?"**

* `time_consumed` começa no instante de referência inicial (`time_of_last_update`).
* A cada comando somado (`time_consumed += Cmd[i].time`), `time_consumed` avança para o instante em que **aquele comando termina**.
* O teste `if (time_consumed > current_time)` pergunta:
  > **"O término deste comando $i$ está no futuro (depois do agora)?"**
  * **Se for NÃO** (`time_consumed <= current_time`): Esse comando já expirou no passado! O carro já deveria tê-lo executado. O loop continua para o próximo comando.
  * **Se for SIM** (`time_consumed > current_time`): **`break`!** Encontramos o primeiro comando cujo término ainda está no futuro. **É este comando que o carro deve estar executando EXATAMENTE AGORA!**

---

### Exemplo Prático com Linha do Tempo

Imagine o seguinte cenário de condução real:

1. **O MPP gera um plano:**
   * O MPP tira uma foto dos sensores às **`10:20:30.000`** e começa a calcular a trajetória.
   * `timestamp = 10:20:30.000`.
   * Cada comando do plano dura $\Delta t = 0{,}050\text{ s}$ ($50\text{ ms}$).

2. **A latência do MPP:**
   * O MPP demora **$120\text{ ms}$** para calcular as clothoides, resolver otimizações e transmitir a mensagem via IPC/ROS.
   * Quando a mensagem chega no Obstacle Avoider, o relógio real do robô (`current_time = astro_get_time()`) já avançou e marca:
     $$t_{\text{atual}} = \mathbf{10:20:30.120}$$

3. **O que acontece dentro do loop:**
   * `time_consumed` inicia em `10:20:30.000`.
   * **$i = 0$:** `time_consumed += 0.050` $\to \mathbf{10:20:30.050}$.  
     *Termina em 30.050s, mas já são 30.120s!* $\to$ **Já passou! (Passado)**
   * **$i = 1$:** `time_consumed += 0.050` $\to \mathbf{10:20:30.100}$.  
     *Termina em 30.100s, mas já são 30.120s!* $\to$ **Já passou! (Passado)**
   * **$i = 2$:** `time_consumed += 0.050` $\to \mathbf{10:20:30.150}$.  
     *Termina em 30.150s, e o relógio marca 30.120s!* $\to$ Término no futuro!  
     **`break` no $i = 2$!**

```
Linha do Tempo:
30.000s               30.050s               30.100s        30.120s        30.150s
  |------ Cmd[0] ------|------ Cmd[1] ------|------ Cmd[2] ---*---|-- Cmd[3] --|
  [==== Descartado ===][==== Descartado ===][  Consumido  | Resta ]
                                                          ^
                                                      AGORA (current_time = 30.120s)
```

---

### Etapa 2: Reindexação e Descarte da Fila

Ao sair do loop, o índice `i` aponta para o comando que deve ser executado no presente:

```cpp
    if (i < num_motion_commands_in_vector[motion_command_vetor])
    {
        // 1. Move o comando vigente (Cmd[i]) para a posição zero da fila
        motion_commands_vector[motion_command_vetor][0] = motion_commands_vector[motion_command_vetor][i];

        // 2. Ajusta a duração residual que resta do Cmd[0]
        motion_commands_vector[motion_command_vetor][0].time = time_consumed - current_time;

        // 3. Desloca todos os comandos futuros para a esquerda
        for (j = 1; (j + i) < num_motion_commands_in_vector[motion_command_vetor]; j++)
        {
            motion_commands_vector[motion_command_vetor][j] = motion_commands_vector[motion_command_vetor][j + i];
        }

        num_motion_commands_in_vector[motion_command_vetor] = j;
        astro_robot_ackerman_motion_command_time_of_last_update = current_time;
```

#### O Ajuste Milimétrico do Tempo Residual:
No nosso exemplo, o `Cmd[2]` terminaria em `10:20:30.150`, mas o instante presente é `10:20:30.120`. Logo, esse degrau já foi executado por $20\text{ ms}$, restando apenas:
$$\Delta t_{\text{restante}} = t_{\text{fim}} - t_{\text{atual}} = 30{,}150 - 30{,}120 = \mathbf{0{,}030\text{ s}}\ (30\text{ ms})$$

O código grava exatamente $0{,}030\text{ s}$ no `Cmd[0].time`, descartando os comandos `Cmd[0]` e `Cmd[1]`. O `Cmd[2]` passa a ser o novo `Cmd[0]`, sincronizado com o instante exato do caminhão.

---

### Etapa 3: Tratamento de Esgotamento Total (*Failsafe*)

Se o tempo decorrido for maior do que a soma de **todos** os comandos da fila (por exemplo, se o MPP travar e parar de enviar planos):

```cpp
    else
    {
        num_motion_commands_in_vector[motion_command_vetor] = 1;
        motion_commands_vector[motion_command_vetor][0].v = 0.0;
        motion_commands_vector[motion_command_vetor][0].time = 1.0;

        if (!printed_total_consumed && out_act)
        {
            printed_total_consumed = true;
            fprintf(out_act, "  \033[1;31m[CONSUMO TOTAL]\033[0m O tempo consumiu todos os comandos! Fila esgotada -> Forcando Cmd[0] v = 0.00 m/s\n\n");
            fflush(out_act);
        }
    }
```
O sistema protege o veículo forçando a velocidade imediatamente a $0{,}00\text{ m/s}$, impedindo que o caminhão prossiga sem planejamento válido.

---

## 4. Diagrama de Sequência e Linha do Tempo

A relação temporal entre a emissão do plano pelo MPP e a compensação de latência pelo Obstacle Avoider é ilustrada abaixo:

```mermaid
sequenceDiagram
    autonumber
    participant MPP as Planejador MPP
    participant IPC as Barramento IPC / ROS
    participant OA as Obstacle Avoider
    participant Clock as Relógio Real (astro_get_time)

    Note over MPP,Clock: Instante t = 10:20:30.000 (Foto dos Sensores)
    MPP->>MPP: Inicia otimização das Clothoides (Duração = 150 ms)
    Note over Clock: O tempo avança: 10:20:30.050 ... 30.100 ... 30.150
    MPP->>IPC: Publica mensagem com timestamp = 10:20:30.000
    IPC->>OA: Mensagem recebida no OA
    Note over OA,Clock: Relógio atual do OA = 10:20:30.150! (Latência = 150 ms)
    OA->>OA: consume_motion_command_time() detecta que Cmd[0], Cmd[1] e Cmd[2] expiraram!
    OA->>OA: Descarta os 3 comandos obsoletos
    OA->>OA: Define Cmd[3] como o novo Cmd[0] (Sincronizado com o Presente)
```

---

## 5. Comportamento Contínuo nos Ciclos Periódicos do Timer

O que acontece nos ciclos seguintes, enquanto o MPP ainda não enviou outro plano?

O `obstacle_avoider_timer_handler()` é chamado periodicamente a cada $\approx 20\text{ ms}$ ($0{,}020\text{ s}$):

1. **Ciclo seguinte ($20\text{ ms}$ depois, $t = 10:20:30.140$):**
   * `time_consumed` inicia em `10:20:30.120`.
   * $i = 0$: `time_consumed += Cmd[0].time` ($0{,}030\text{ s}$) $\to 10:20:30.150$.
   * Como $10:20:30.150 > 10:20:30.140$, ocorre o **`break` no $i = 0$**.
   * Nenhum comando é descartado! O `Cmd[0]` continua ativo, mas sua duração residual é atualizada para:
     $$\Delta t = 30{,}150 - 30{,}140 = \mathbf{0{,}010\text{ s}}\ (10\text{ ms})$$
   * `time_of_last_update` é atualizado para `10:20:30.140`.

2. **Segundo ciclo ($40\text{ ms}$ depois, $t = 10:20:30.160$):**
   * `time_consumed` inicia em `10:20:30.140`.
   * $i = 0$: `time_consumed += Cmd[0].time` ($0{,}010\text{ s}$) $\to 10:20:30.150 \le 10:20:30.160$ (O `Cmd[0]` expirou!).
   * $i = 1$: `time_consumed += Cmd[1].time` ($0{,}050\text{ s}$) $\to 10:20:30.200 > 10:20:30.160$ (**`break` no $i = 1$**).
   * O comando antigo é descartado, `Cmd[1]` vira o novo `Cmd[0]`, e o log avisa:
     ```text
     [CONSUMO TEMPO] O tempo passou! Descartou 1 comando(s)...
     ```

---

## 6. Estudo de Caso Real: Transição de Plano e o Descarte de 70 Comandos

Durante um teste em veículo real, a inspeção da fila com prints detalhados registrou exatamente a transição entre um plano antigo e a chegada de um novo plano do MPP:

### 6.1 Fase 1: Condução Normal e Consumo Degrau a Degrau no Plano Inicial
O robô trafegava executando um plano com degraus de $\Delta t = 0{,}060\text{ s}$ ($60\text{ ms}$). Observe como a fila inicial começa em $v \approx 0{,}09\text{ m/s}$ com o primeiro comando quase expirando (`time = 0.004056s`), e a cada ciclo o timer descarta 1 ou 2 comandos conforme o tempo passa, avançando a velocidade suavemente:

#### Início da Fila (Velocidade em $0{,}09\text{ m/s}$):
```text
plano atual:
 0 --> v = 0.097019 | time = 0.004056  <-- Quase expirando (restam apenas 4 ms!)
 1 --> v = 0.099197 | time = 0.060000
 2 --> v = 0.101375 | time = 0.060000
 3 --> v = 0.103554 | time = 0.060000
 ...
 507 --> v = 1.199176 | time = 0.060000
soma_tempo: 30.445634

[CONSUMO TEMPO] O tempo passou! Descartou 1 comando(s) executado(s) (v_antiga=0.09 m/s). Novo Cmd[0]: v=0.10 m/s (Restam 507 cmds na fila)
```

#### Poucos Ciclos Depois (Velocidade em $0{,}10\text{ m/s}$):
O antigo `Cmd[0]` expirou. O novo comando do topo passa a ser executado e a fila reduz:
```text
plano atual:
 0 --> v = 0.101375 | time = 0.047110  <-- Novo início do plano (com tempo fatiado após avanço)
 1 --> v = 0.103554 | time = 0.060000
 2 --> v = 0.105732 | time = 0.060000
 ...
soma_tempo: 30.287110

[CONSUMO TEMPO] O tempo passou! Descartou 1 comando(s) executado(s) (v_antiga=0.10 m/s). Novo Cmd[0]: v=0.10 m/s (Restam 504 cmds na fila)
```

#### Imediatamente Antes da Chegada do Novo Plano (Velocidade em $0{,}11\text{ m/s}$):
O caminhão continuou acelerando suavemente até $0{,}11\text{ m/s}$, restando 499 comandos na fila:
```text
plano atual:
 0 --> v = 0.112266 | time = 0.034136  <-- Velocidade atual de cruzeiro do robô
 1 --> v = 0.114444 | time = 0.060000
 2 --> v = 0.116623 | time = 0.060000
 ...
 499 --> v = 1.199176 | time = 0.060000
soma_tempo: 29.974136

[CONSUMO TEMPO] O tempo passou! Descartou 1 comando(s) executado(s) (v_antiga=0.11 m/s). Novo Cmd[0]: v=0.11 m/s (Restam 499 cmds na fila)
```
* O veículo estava fisicamente em movimento a **$v = 0{,}11\text{ m/s}$**.

---

### 6.2 Fase 2: Chegada do Novo Plano do MPP (Antes do Consumo)
Imediatamente a seguir, o MPP enviou um novo plano completo com **474 comandos** (índices `0` a `473`). 

> [!CAUTION]
> **Esclarecimento Crucial: O Descarte é 100% TEMPORAL (Não por Velocidade):**  
> A função `consume_motion_command_time()` **não consulta a velocidade ($v$)** para decidir quais comandos descartar. Ela não "busca um comando que tenha a velocidade atual do robô". O critério de descarte é **estritamente cronológico baseado no relógio**:
> * Se o instante de término do comando for anterior ao relógio atual ($t_{\text{fim}} \le t_{\text{current_time}}$), o comando pertence ao passado e é descartado.
> * A velocidade no comando 70 coincidir com a velocidade real do carro ($0{,}11\text{ m/s}$) é uma **consequência física**: o robô acelerou por $1{,}40\text{ s}$ no mundo real e o MPP também planejou uma aceleração ao longo desses mesmos $1{,}40\text{ s}$.

Ao somar o timestamp base do plano ($T_{\text{base}}$) com a duração $\Delta t$ de cada degrau, a linha do tempo absoluta de cada comando fica evidente:

| Índice | Velocidade ($v$) | Duração ($\Delta t$) | Timestamp de Término ($t_{\text{fim}} = T_{\text{base}} + \sum \Delta t$) | Comparação com o Relógio Atual ($t_{\text{atual}} = T_{\text{base}} + 1{,}388\text{ s}$) | Decisão do OA |
| :---: | :---: | :---: | :---: | :---: | :---: |
| `Cmd[0]` | $-0{,}001\text{ m/s}$ | $0{,}020\text{ s}$ | $T_{\text{base}} + 0{,}020\text{ s}$ | $T_{\text{base}} + 0{,}020 \le t_{\text{atual}}$ | **PASSADO** (Expirado!) |
| `Cmd[1]` | $+0{,}001\text{ m/s}$ | $0{,}020\text{ s}$ | $T_{\text{base}} + 0{,}040\text{ s}$ | $T_{\text{base}} + 0{,}040 \le t_{\text{atual}}$ | **PASSADO** (Expirado!) |
| `Cmd[2]` | $+0{,}002\text{ m/s}$ | $0{,}020\text{ s}$ | $T_{\text{base}} + 0{,}060\text{ s}$ | $T_{\text{base}} + 0{,}060 \le t_{\text{atual}}$ | **PASSADO** (Expirado!) |
| ... | ... | ... | ... | ... | ... |
| `Cmd[68]` | $+0{,}110\text{ m/s}$ | $0{,}020\text{ s}$ | $T_{\text{base}} + 1{,}380\text{ s}$ | $T_{\text{base}} + 1{,}380 \le t_{\text{atual}}$ | **PASSADO** (Expirado!) |
| `Cmd[69]` | $+0{,}112\text{ m/s}$ | $0{,}020\text{ s}$ | $T_{\text{base}} + 1{,}400\text{ s}$ | $T_{\text{base}} + 1{,}400 > t_{\text{atual}}$ | **PRESENTE!** (`break` no $i = 70$) |
| `Cmd[70]` | $+0{,}113\text{ m/s}$ | $0{,}020\text{ s}$ | $T_{\text{base}} + 1{,}420\text{ s}$ | $T_{\text{base}} + 1{,}420 > t_{\text{atual}}$ | Futuro (Ativo) |

#### Saída do Log com Timestamp Acumulado e Status:
```text
plano atual (base_timestamp = 1729689630.000000 | current_time = 1729689631.387885 | latencia = 1.388s):
   0 --> v = -0.000999 | dt = 0.020000 | t_fim = 1729689630.020000 [PASSADO - EXPIRADO]
   1 --> v =  0.000635 | dt = 0.020000 | t_fim = 1729689630.040000 [PASSADO - EXPIRADO]
   2 --> v =  0.002268 | dt = 0.020000 | t_fim = 1729689630.060000 [PASSADO - EXPIRADO]
   ...
  68 --> v =  0.110088 | dt = 0.020000 | t_fim = 1729689631.380000 [PASSADO - EXPIRADO]
  69 --> v =  0.111721 | dt = 0.020000 | t_fim = 1729689631.400000 [FUTURO - ATIVO -> BREAK!]
  70 --> v =  0.113355 | dt = 0.020000 | t_fim = 1729689631.420000 [FUTURO - ATIVO]
```

---

### 6.3 Fase 3: A Compensação de Latência em Ação
O MPP demorou cerca de **$1{,}40\text{ segundos}$** calculando essa nova rota ($70 \times 0{,}020\text{ s} = 1{,}40\text{ s}$). Durante esse tempo, o caminhão continuou andando no mundo real.

Ao executar `consume_motion_command_time()`, a função detectou que os primeiros 70 comandos já pertenciam ao passado e realizou o descarte em massa:

```text
[CONSUMO TEMPO] O tempo passou! Descartou 70 comando(s) executado(s) (v_antiga=-0.00 m/s). Novo Cmd[0]: v=0.11 m/s (Restam 404 cmds na fila)
```
* **Contagem de comandos:** $474\text{ comandos originais} - 70\text{ descartados} = \mathbf{404\text{ comandos restantes}}$.

---

### 6.4 Fase 4: O Plano Sincronizado Resultante
No ciclo imediatamente seguinte, o print confirma que o antigo `Cmd[70]` assumiu a posição zero, perfeitamente sincronizado com a velocidade real do veículo ($0{,}11\text{ m/s}$):

```text
plano atual (após compensação):
 0 --> v = 0.113355 | time = 0.012115  <-- Antigo Cmd[70], com tempo residual fatiado!
 1 --> v = 0.114988 | time = 0.020000  <-- Antigo Cmd[71]
 2 --> v = 0.116622 | time = 0.060000  <-- Antigo Cmd[72]
 3 --> v = 0.121523 | time = 0.060000
 ...
 403 --> v = 2.077500 | time = 0.006491
soma_tempo: 24.098606
```

E a fila voltou ao consumo normal ciclo a ciclo:
```text
[CONSUMO TEMPO] O tempo passou! Descartou 3 comando(s)... Novo Cmd[0]: v=0.12 m/s (Restam 401 cmds na fila)
[CONSUMO TEMPO] O tempo passou! Descartou 1 comando(s)... Novo Cmd[0]: v=0.13 m/s (Restam 400 cmds na fila)
```

> [!TIP]
> **Impacto na Condução:**  
> Se esse descarte de 70 comandos não existisse, o caminhão receberia um comando de frear bruscamente até $0{,}00\text{ m/s}$ (o início obsoleto do plano de 1.4s atrás), gerando um tranco violento na transmissão e desestabilizando o engate das carretas. A compensação de latência garantiu uma transição $100\%$ contínua e suave.

---

## 7. Diagrama de Fluxo Interno da Função

```mermaid
flowchart TD
    A["consume_motion_command_time()"] --> B["current_time = astro_get_time()<br/>time_consumed = time_of_last_update"]
    B --> C["Loop i = 0 até N comandos"]
    C --> D["time_consumed += cmd[i].time"]
    D --> E{"time_consumed &gt; current_time?<br/>(O fim deste cmd está no futuro?)"}
    E -- Não (Cmd no passado) --> C
    E -- Sim (Cmd no presente) --> F["Interrompe loop (break em i)"]

    F --> G{"i &lt; N comandos?"}
    G -- Sim --> H["Cmd[0] = Cmd[i]<br/>Cmd[0].time = time_consumed - current_time"]
    H --> I["Desloca comandos restantes j = 1..N-i<br/>time_of_last_update = current_time"]
    I --> J{"i > 0?"}
    J -- Sim --> K["Imprime [CONSUMO TEMPO]<br/>Descartou i comandos"]
    J -- Não --> L["Fim do ciclo (Cmd[0] ainda ativo com tempo fatiado)"]
    K --> L

    G -- Não (Fila Esgotada) --> M["Cmd[0].v = 0.00 m/s<br/>Cmd[0].time = 1.0s"]
    M --> N["Imprime [CONSUMO TOTAL]<br/>Failsafe ativado"]
    N --> L
```
