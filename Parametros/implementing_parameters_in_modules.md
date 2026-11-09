# 04 — Revisão das Mudanças: Desvinculação DBC vs. JAUS no `publish_odometry()`

Este documento registra a análise passo a passo das alterações feitas em [`simulator_ackerman_can_main.cpp`](file:///home/lume/astro/src/simulator_ackerman/simulator_ackerman_can_main.cpp) para desvincular o envio de telemetria CAN do modo legado/JAUS (`switch (vehicle_id)`) quando um arquivo `.dbc` está carregado.

---

## 1. Objetivo da Refatoração

Antes dessa mudança:
* **Na leitura (`update_state_from_can`)**: A separação já era exclusiva — ou processava via DBC (`process_frame_from_dbc_mode`) e dava `return`, ou caía no `switch (vehicle_id)`.
* **No envio (`publish_odometry`)**: Os dois modos rodavam misturados. `Gear` e `RPM` iam pela fila do DBC, mas `send_can_frame_v_lume()` e `send_can_frame_phi()` continuavam sendo chamados fora do `if` principal, dependendo de checagens manuais do nome do arquivo (`"oj403"`, `"ojAstro_fullDBC"`) ou caindo no `switch (vehicle_id)`.

O objetivo das novas mudanças é permitir que **qualquer veículo** funcione dinamicamente passando apenas o caminho do `.dbc`, sem depender de código chumbado (`hardcoded`) por veículo.

---

## 2. Parte 1: A Mudança em `send_can_frame_v_lume()`

### Como era antes:
Dentro de `send_can_frame_v_lume()`, havia blocos manuais verificando o nome do arquivo (`if (name == "oj403")`, `else if (name == "ojAstro_fullDBC")`) para montar os bytes do pacote `0x201` na mão. Se fosse qualquer outro `.dbc`, ele caía no `switch (vehicle_id)` do modo legado.

### Como ficou agora ([linhas 494-505](file:///home/lume/astro/src/simulator_ackerman/simulator_ackerman_can_main.cpp#L494-L505)):
```cpp
if (dbc_can_process.valid_parser == true)
{
    // Modo DBC: Velocity/VelocityEncoder sao publicados em publish_odometry() a partir do DBC (sinal fisico ou
    // derivado invertido). O frame legado por vehicle_id so e usado se o VelocityEncoder for derivado e nao inversivel.
    if (!dbc_read_signal_is_derived_only("VelocityEncoder") || dbc_derived_signal_is_writable("VelocityEncoder"))
        return;
}
```
Se o modo DBC estiver ativo e o DBC souber publicar a velocidade, a função **aborta imediatamente (`return`)**, evitando o duplo acionamento (DBC + JAUS em paralelo) e delegando toda a responsabilidade para o `publish_odometry()`.

---

## 3. Os Dois Tipos de Sinais em um Arquivo `.dbc`

Para entender a condição do `if` acima, é fundamental distinguir as **duas formas** como um sinal de leitura (`read`) pode ser declarado nos arquivos `.dbc` do Astro:

### Tipo A: Sinal Físico Padrão (`SG_`)
Exemplo no [`ojAstro_fullDBC.dbc:23`](file:///home/lume/astro/src/astro_hal/dbc_file_example/ojAstro_fullDBC.dbc#L23-L24):
```dbc
BO_ 0x201 read: 2 DRIVER
 SG_ VelocityEncoder : 0|16@1+ (-0.008764419, 87.64419) [-300|300] "" CONTROLLER
```
* Começa com a tag **`SG_`**.
* É cadastrado diretamente na tabela hash **`composite_to_signal`** do HAL com a chave `"read::velocityencoder"`.
* O HAL sabe **ler e escrever** nele diretamente usando `(scale, offset)` via `build_can_frames_batch()`.

### Tipo B: Sinal Derivado por Expressão (`CM_ ... ASTRO_EXPR`)
Exemplo no [`oj403.dbc:20`](file:///home/lume/astro/src/astro_hal/dbc_file_example/oj403.dbc#L20-L24):
```dbc
BO_ 0x201 read: 8 CONTROLLER
 SG_ WheelSpeed1 : 16|16@1+ (1,0) [0|65535] "" CONTROLLER
 SG_ WheelSpeed2 : 32|16@1+ (1,0) [0|65535] "" CONTROLLER
CM_ BO_ 0x201 "ASTRO_CONST CAR_SPEED_FACTOR = 0.0017";
CM_ BO_ 0x201 "ASTRO_EXPR VelocityEncoder = (* 0.857 (* CAR_SPEED_FACTOR (mean WheelSpeed1 WheelSpeed2)))";
```
* **Não existe** `SG_ VelocityEncoder` nesse arquivo! Os sinais físicos reais nos bytes do pacote são `SG_ WheelSpeed1` e `SG_ WheelSpeed2`.
* O `VelocityEncoder` existe apenas como uma **fórmula matemática (`ASTRO_EXPR`)** dentro de um comentário `CM_`.
* Por não ser um `SG_`, ele **não está** na hash `composite_to_signal`, mas sim na estrutura interna de expressões derivadas do parser (`has_derived_signal`).
* Originalmente, o HAL só sabia **ler** uma `ASTRO_EXPR` (avaliando a fórmula), mas não sabia fazer a conta inversa para **escrever** nela.

---

## 4. Por que `dbc_read_signal_is_derived_only()` NÃO é contraditório?

Olhando a implementação de [`dbc_read_signal_is_derived_only()`](file:///home/lume/astro/src/simulator_ackerman/simulator_ackerman_can_main.cpp#L343-L361):

```cpp
// Retorna true se o DBC carregado define fisicamente o sinal "read::<name>" (case-insensitive).
static bool
dbc_has_read_signal(const char *name)
{
    if (!dbc_can_process.valid_parser)
        return (false);

    std::string key = astro_hal_can_interactions::to_lowercase(std::string("read::") + name);

    return (dbc_can_process.composite_to_signal.count(key) > 0);
}

// Sinal "read" definido no DBC apenas como ASTRO_EXPR (derivado): o HAL so sabe le-lo, nao escreve-lo.
static bool
dbc_read_signal_is_derived_only(const char *name)
{
    return (dbc_can_process.valid_parser &&
        !dbc_has_read_signal(name) &&
        astro_hal_can_interactions::has_signal(dbc_can_process, "read", name));
}
```

À primeira vista, parece contraditório exigir `!dbc_has_read_signal(name)` (ser `false`) e ao mesmo tempo `has_signal(...)` (ser `true`), já que ambos parecem consultar `composite_to_signal`.

Porém, ao inspecionar [`astro_hal_can_interactions::has_signal()`](file:///home/lume/astro/include/astro/astro_hal_can_interactions.hpp#L655-L681), vemos que ele consulta **dois lugares diferentes em sequência**:

```cpp
inline bool
has_signal(const astro_hal_can_interactions_t &dbc,
        const std::string &direction,
        const std::string &signal_name)
{
    ...
    // 1º LUGAR: Olha na hash de sinais físicos (SG_)
    if (dbc.composite_to_signal.count(composite_key) > 0)
    {
        return (true);
    }

    // 2º LUGAR: Só consulta ASTRO_EXPR quando o sinal físico não existe!
    return (Libdbc::detail::has_derived_signal(dbc.parser, direction, signal_name));
}
```

### Fluxo passo a passo da comparação:

| Função | Onde procura? | Retorno para `SG_ VelocityEncoder` (`ojAstro_fullDBC.dbc`) | Retorno para `ASTRO_EXPR VelocityEncoder` (`oj403.dbc`) |
| :--- | :--- | :---: | :---: |
| `dbc_has_read_signal(name)` | **Apenas no 1º Lugar** (`composite_to_signal` $\to$ `SG_`) | `true` | `false` |
| `!dbc_has_read_signal(name)` | Inverte o resultado acima (Garante que **não é `SG_`**) | `false` | `true` |
| `has_signal(dbc, "read", name)` | **1º Lugar (`SG_`) OU 2º Lugar (`ASTRO_EXPR`)** | `true` (achou no 1º) | `true` (achou no 2º!) |
| **`dbc_read_signal_is_derived_only(name)`** | **NÃO está no 1º (`!SG_`) E está no 2º (`ASTRO_EXPR`)** | **`false`** | **`true`** |

Portanto, a expressão `!dbc_has_read_signal(name) && has_signal(...)` significa literalmente:
> *"Retorne `true` se, e somente se, o sinal **NÃO** for um `SG_` físico na hash `composite_to_signal`, **MAS** existir cadastrado como uma fórmula `ASTRO_EXPR` no DBC."*

---

## 5. Os 4 Cenários da Condição de Retorno em `send_can_frame_v_lume()`

Voltando à [linha 502](file:///home/lume/astro/src/simulator_ackerman/simulator_ackerman_can_main.cpp#L502-L503), quando estamos com DBC ativo (`valid_parser == true`):
```cpp
if (!dbc_read_signal_is_derived_only("VelocityEncoder") || dbc_derived_signal_is_writable("VelocityEncoder"))
    return;
```

Fazendo o teste de mesa da lógica booleana, temos exatamente **4 cenários possíveis**:

### Cenário 1: É um sinal físico `SG_` (ex: `ojAstro_fullDBC.dbc`)
* `dbc_has_read_signal("VelocityEncoder")` $\to$ `true`
* Como `dbc_read_signal_is_derived_only` exige `!dbc_has_read_signal`, ela já retorna **`false`**.
* No `if (!dbc_read_signal_is_derived_only(...))`: o `!false` vira **`true`**.
* **Resultado:** Entra no `if`, dá **`return`** imediato e deixa o `publish_odometry()` enviar pelo DBC via `build_can_frames_batch()`.

### Cenário 2: NÃO é físico, É derivativo (`ASTRO_EXPR`) e É inversível (ex: `oj403.dbc`)
* `!dbc_has_read_signal("VelocityEncoder")` $\to$ `true`
* `has_signal("read", "VelocityEncoder")` $\to$ `true`
* `dbc_read_signal_is_derived_only` retorna **`true`**.
* No `if`: a 1ª parte (`!true`) dá **`false`**, então ele avalia a 2ª parte (`|| dbc_derived_signal_is_writable`), que dá **`true`**.
* **Resultado:** Entra no `if`, dá **`return`** imediato e deixa o `publish_odometry()` inverter a equação e enviar pelo DBC!

### Cenário 3: NÃO é físico e NEM derivativo (não existe `VelocityEncoder` no `.dbc`)
Imagine um veículo cujo `.dbc` usa apenas `SG_ Velocity` e não declara `VelocityEncoder`:
1. `!dbc_has_read_signal("VelocityEncoder")` $\to$ `true` (não é físico).
2. `has_signal("read", "VelocityEncoder")` $\to$ **`false`** (não existe nem como físico nem como `ASTRO_EXPR`).
3. Em `dbc_read_signal_is_derived_only`: `true && false` resulta em **`false`**.
4. No `if (!dbc_read_signal_is_derived_only(...))`: o `!false` vira **`true`**!
* **Resultado:** **Também dá `return`** e **NÃO** vai para o hardcode do JAUS!
* **Por que isso é importante?** Porque se foi carregado o `.dbc` de um veículo que não possui `VelocityEncoder` (usa apenas `Velocity`), o simulador não deve injetar um frame `0x201` legado do Astro na rede CAN.

### Cenário 4 (O ÚNICO caso em que cai no hardcode mesmo com DBC):
Para o `if` falhar nas duas pontas e deixar a execução avançar para o `switch (vehicle_id)` legado:
1. A 1ª ponta (`!dbc_read_signal_is_derived_only`) precisa ser **`false`** $\to$ ou seja, `VelocityEncoder` **existe** no `.dbc` como sinal derivativo (`ASTRO_EXPR`).
2. E a 2ª ponta (`dbc_derived_signal_is_writable`) também precisa ser **`false`** $\to$ ou seja, a equação `ASTRO_EXPR` é complexa/não-linear (ex: dividida em múltiplos bytes independentes) e **NÃO pode ser invertida** automaticamente.

---

### Tabela Verdade Resumida (Modo DBC Ativo)

| Como o sinal está no `.dbc`? | `is_derived_only()` | `!is_derived_only()` | `is_writable()` | Ação em `send_can_frame_v_lume()` |
| :--- | :---: | :---: | :---: | :--- |
| **1. É físico (`SG_`)** | `false` | **`true`** | *(nem avalia)* | **`return`** (Envia pelo DBC em lote) |
| **2. Não existe no `.dbc`** | `false` | **`true`** | *(nem avalia)* | **`return`** (Ignora, carro não usa esse sinal) |
| **3. É `ASTRO_EXPR` inversível** | `true` | `false` | **`true`** | **`return`** (Inverte equação e envia pelo DBC) |
| **4. É `ASTRO_EXPR` NÃO inversível** | `true` | `false` | `false` | **Cai no hardcode (`switch vehicle_id`)** |

---

## 6. Parte 2: A Velocidade na `fila_de_envio` do `publish_odometry()`

Em [`publish_odometry()` (linhas 888-900)](file:///home/lume/astro/src/simulator_ackerman/simulator_ackerman_can_main.cpp#L888-L900), após verificar e adicionar `Gear` e `RPM`, o código prepara o envio da velocidade:

```cpp
// O vehicle_driver multiplica Velocity/VelocityEncoder por -1 quando a marcha lida e re
// (speed_signal em vehicle_driver_dbc.cpp), entao o DBC leva a velocidade em modulo na re.
double v_dbc = (driver_outputs.current_gear < 0) ? fabs(simulator_config->v) : simulator_config->v;

if (dbc_has_read_signal("Velocity"))
    fila_de_envio.push_back({"read", "Velocity", v_dbc});
if (dbc_has_read_signal("VelocityEncoder"))
    fila_de_envio.push_back({"read", "VelocityEncoder", v_dbc});
```

### 6.1 Por que usar `fabs(simulator_config->v)` na marcha ré (`current_gear < 0`)?

1. **Comportamento do sensor real e do `vehicle_driver`**:
   No veículo físico, os sensores de roda/tacógrafo medem apenas a rapidez do giro da roda, enviando **sempre valores positivos ($\ge 0$)** na rede CAN, mesmo quando o caminhão dá ré.
   Por isso, o `vehicle_driver` ([`vehicle_driver_dbc.cpp:768-775`](file:///home/lume/astro/src/vehicle_driver/vehicle_driver_dbc.cpp#L768-L775)) olha para a marcha atual e define um multiplicador de sinal (`speed_signal`):
   ```cpp
   if ((current_gear_status == -1) || (current_gear_status == -2)) // Marcha Ré
       speed_signal = -1.0;
   else
       speed_signal = 1.0;
   ```
   E ao ler a velocidade da CAN ([linhas 817](file:///home/lume/astro/src/vehicle_driver/vehicle_driver_dbc.cpp#L817) e [864](file:///home/lume/astro/src/vehicle_driver/vehicle_driver_dbc.cpp#L864)), multiplica por `speed_signal`:
   ```cpp
   double velocity_frame = speed_signal * astro_hal_can_interactions::read_signal(..., "read", "Velocity", ...);
   ```

2. **Evitando inversão dupla de sinal**:
   Na física interna do simulador, quando o veículo dá ré, `simulator_config->v` já é **negativo** (ex: `-2.0 m/s`).
   * Se o simulador enviasse `-2.0` na CAN, o `vehicle_driver` faria `(-1.0) * (-2.0) = +2.0 m/s` (achando que o caminhão está indo para frente!), além de causar saturação em DBCs cujos sinais de roda são `unsigned` (`[0|65535]`).
   * Aplicando `fabs(simulator_config->v)` quando `driver_outputs.current_gear < 0`, o simulador envia `+2.0` na rede CAN, imitando o sensor real, e o `vehicle_driver` reconstrói `-2.0 m/s` corretamente.

---

### 6.2 Por que `Velocity` e `VelocityEncoder` recebem o mesmo valor `v_dbc`?

No mundo físico (em $\text{m/s}$), ambos representam exatamente a **mesma grandeza física**: a velocidade linear do veículo. A diferença está apenas na origem física do sensor no caminhão e na codificação binária do `.dbc`:

1. **Origem física (e uso no `vehicle_driver`)**:
   * **`Velocity` (ex: ID `0x8000022C`)**: Velocímetro original da central eletrônica (ECU/tacógrafo).
   * **`VelocityEncoder` (ex: ID `0x201`)**: Encoder externo de roda (ex: encoder "Tramontina" no Astro) instalado para maior resolução em baixas velocidades.
   * No [`vehicle_driver_dbc.cpp`](file:///home/lume/astro/src/vehicle_driver/vehicle_driver_dbc.cpp#L812-L874), ambos atualizam a mesma variável interna `vehicle_driver_dbc.current_velocity_read`.

2. **Codificação binária diferente feita pelo HAL**:
   Embora ambos recebam `v_dbc` em $\text{m/s}$ na `fila_de_envio` (ex: `5.0 m/s`), o `build_can_frames_batch()` consulta o `.dbc` e aplica o `(scale, offset)` específico de cada um. Por exemplo, no [`ojAstro_fullDBC.dbc`](file:///home/lume/astro/src/astro_hal/dbc_file_example/ojAstro_fullDBC.dbc#L17-L24):
   * **`SG_ Velocity : 48|16@1+ (0.0010850694444, 0)`** $\implies \text{raw} = \frac{5.0 - 0}{0.001085} = \mathbf{4608}$ (gravado nos bytes 6 e 7 do frame `0x8000022C`).
   * **`SG_ VelocityEncoder : 0|16@1+ (-0.008764419, 87.64419)`** $\implies \text{raw} = \frac{5.0 - 87.644}{-0.008764} = \mathbf{9429}$ (gravado nos bytes 0 e 1 do frame `0x201`).

Assim, o simulador só precisa informar a velocidade física `v_dbc` em $\text{m/s}$, e o HAL se encarrega de empacotar cada sinal com sua respectiva matemática e CAN ID!

---

## 7. Parte 3: O Cálculo de `phi_to_dbc_steering_angle()`

Na [linha 902](file:///home/lume/astro/src/simulator_ackerman/simulator_ackerman_can_main.cpp#L902-L903), ao adicionar o ângulo de direção (`SteeringAngle`) na `fila_de_envio`, o código chama [`phi_to_dbc_steering_angle()`](file:///home/lume/astro/src/simulator_ackerman/simulator_ackerman_can_main.cpp#L473-L478):

```cpp
static double
phi_to_dbc_steering_angle(double phi, double v)
{
    double L = simulator_config->distance_between_front_and_rear_axles;

    return ((1.0 + v * v * simulator_config->understeer_coeficient2) * atan(L * tan(-phi)));
}
```

### Por que essa fórmula existe?
Ela é a **função inversa algébrica exata** da conta que o `vehicle_driver` faz ao ler `SteeringAngle` da rede CAN em [`vehicle_driver_dbc.cpp:883-890`](file:///home/lume/astro/src/vehicle_driver/vehicle_driver_dbc.cpp#L883-L890):

```cpp
// No vehicle_driver_dbc.cpp:
double phi_can = astro_hal_can_interactions::read_signal(dbc_can_process_, "read", "SteeringAngle", frame, is_match);

double curvature = tan(phi_can / (1.0 + v * v * config->understeer_coeficient2)) / config->distance_between_front_and_rear_axles;
vehicle_driver_dbc.dbc_assistant.steering_angle_sensor = -atan(curvature);
```

No veículo real, o sensor da barra de direção tem sinal invertido em relação ao referencial trigonométrico do Astro e sofre efeito de subesterçamento (*understeer*, $1 + v^2 k$) proporcional à distância entre eixos ($L$).

Se o simulador enviasse `simulator_config->phi` diretamente no sinal `SteeringAngle`, o `vehicle_driver` dividiria o valor por $L \approx 4.76\text{ m}$ e inverteria seu sinal. Isolando $\phi_{\text{can}}$ da equação do `vehicle_driver`:

1. **Equação do `vehicle_driver`**:
   $$\phi = -\arctan\left( \frac{\tan\left(\frac{\phi_{\text{can}}}{1 + v^2 k}\right)}{L} \right)$$
2. **Isolando $\phi_{\text{can}}$**:
   $$\tan(-\phi) = \frac{\tan\left(\frac{\phi_{\text{can}}}{1 + v^2 k}\right)}{L} \implies \arctan(L \cdot \tan(-\phi)) = \frac{\phi_{\text{can}}}{1 + v^2 k}$$
   $$\phi_{\text{can}} = (1 + v^2 k) \cdot \arctan(L \cdot \tan(-\phi))$$

Assim, o simulador "pré-compensa" o valor enviado na CAN para que o `vehicle_driver`, ao aplicar sua fórmula de leitura, recupere exatamente o `phi` original do simulador!

---

## 8. Parte 4: Por que `SafeStop` e `ManualOverride` são zerados (`0.0`)?

Nas [linhas 906-909](file:///home/lume/astro/src/simulator_ackerman/simulator_ackerman_can_main.cpp#L906-L909) e [957-958](file:///home/lume/astro/src/simulator_ackerman/simulator_ackerman_can_main.cpp#L957-L958):

```cpp
// Dentro do modo DBC:
if (dbc_has_read_signal("SafeStop"))
    fila_de_envio.push_back({"read", "SafeStop", 0.0});
if (dbc_has_read_signal("ManualOverride"))
    fila_de_envio.push_back({"read", "ManualOverride", 0.0});
...
// Fora do modo DBC (Legado):
if (!dbc_can_process.valid_parser)
    send_can_frame_manual_override_and_safe_stop();
```

### 1. Significado físico no `vehicle_driver`
Esses dois sinais representam as botoeiras de segurança do painel ([`vehicle_driver_dbc.cpp:898-940`](file:///home/lume/astro/src/vehicle_driver/vehicle_driver_dbc.cpp#L898-L940)):
* **`SafeStop = 0.0`**: Botão de parada de emergência **liberado** (`1` acionaria freio de emergência total). Além disso, cada pacote recebido atualiza o *heartbeat* `time_override_initial = timestamp` no `vehicle_driver`.
* **`ManualOverride = 0.0`**: Chave seletora em **Modo Autônomo** (`1` indicaria que um motorista humano assumiu o volante físico).

Para que o veículo simulado possa operar em modo autônomo sem travas de emergência, ambos devem ser transmitidos periodicamente com valor **`0.0`**.

### 2. Desvinculação da função legada
Antes dessa mudança, [`send_can_frame_manual_override_and_safe_stop()`](file:///home/lume/astro/src/simulator_ackerman/simulator_ackerman_can_main.cpp#L731-L761) era chamada incondicionalmente no final de `publish_odometry()`, enviando frames zerados com CAN IDs fixos (`0x18FF01FE` ou `0x600`) via `switch (vehicle_id)`.
Agora, com `if (!dbc_can_process.valid_parser)`, a função legada só roda no modo JAUS/legado, enquanto no modo DBC esses dois sinais vão empacotados dinamicamente na `fila_de_envio`!

---

## 9. Parte 5: Por que existem dois blocos de envio no `publish_odometry()` (`build_can_frames_batch` vs. `derived_signals`)?

Em [`publish_odometry()` (linhas 914-949)](file:///home/lume/astro/src/simulator_ackerman/simulator_ackerman_can_main.cpp#L914-L949), vemos que o envio via DBC é dividido em duas etapas complementares e **mutuamente exclusivas**:

```cpp
// ETAPAS 1-3: Envio em lote dos sinais físicos (SG_)
frames_prontos = astro_hal_can_interactions::build_can_frames_batch(dbc_can_process, fila_de_envio);
for (auto& lote : frames_prontos)
    astro_hal_can_interactions::send_message_frame(...);

// ETAPA 4: Sinais derivados (ASTRO_EXPR): escritos por inversao da expressao lida do proprio DBC.
struct { const char *name; double value; } derived_signals[] = {
    {"Velocity", v_dbc},
    {"VelocityEncoder", v_dbc},
    {"SteeringAngle", phi_to_dbc_steering_angle(simulator_config->phi, simulator_config->v)}
};

for (auto &d : derived_signals)
{
    struct can_frame frame;
    std::string anchor;

    if (dbc_derived_signal_is_writable(d.name) && dbc_build_derived_frame(d.name, d.value, &frame, &anchor))
        astro_hal_can_interactions::send_message_frame(dbc_can_process, "read", anchor, &frame);
}
```

### 1. Por que `build_can_frames_batch()` não envia os sinais derivados?
A função `build_can_frames_batch()` do HAL opera exclusivamente sobre a tabela hash **`composite_to_signal`**, que armazena apenas sinais físicos declarados com **`SG_`**.
* Por isso, ao preencher a `fila_de_envio` (Etapa 1), cada sinal é filtrado por `if (dbc_has_read_signal(...))`.
* Se um sinal derivado (`CM_ ... ASTRO_EXPR`) fosse colocado na `fila_de_envio`, o `build_can_frames_batch()` não o encontraria na hash `composite_to_signal` e lançaria uma exceção de erro.

### 2. O que aconteceria sem a Etapa 4 (`derived_signals`)?
Em arquivos `.dbc` como o [`oj403.dbc`](file:///home/lume/astro/src/astro_hal/dbc_file_example/oj403.dbc#L20-L33):
* `VelocityEncoder` **não é `SG_`** (é uma fórmula `ASTRO_EXPR` sobre `WheelSpeed1` e `WheelSpeed2`).
* `SteeringAngle` **não é `SG_`** (é uma fórmula `ASTRO_EXPR` sobre `SteerRaw`).

Portanto, para o `oj403.dbc`, `dbc_has_read_signal("VelocityEncoder")` e `dbc_has_read_signal("SteeringAngle")` retornam **`false`**, deixando ambos de fora da `fila_de_envio`. Sem a Etapa 4, o simulador nunca enviaria esses dois sinais para o `oj403`!

### 3. Exclusividade Mútua (Nenhum sinal é enviado duas vezes)
Um sinal jamais entra nas duas etapas ao mesmo tempo:

| Sinal no `.dbc` | Entra no `build_can_frames_batch()` (Etapas 1-3)? | Entra no loop `derived_signals` (Etapa 4)? |
| :--- | :---: | :---: |
| **`VelocityEncoder` no `ojAstro_fullDBC.dbc`** (`SG_`) | **SIM** (`dbc_has_read_signal == true`) | **NÃO** (`is_derived_only == false`) |
| **`VelocityEncoder` no `oj403.dbc`** (`ASTRO_EXPR`) | **NÃO** (`dbc_has_read_signal == false`) | **SIM** (`is_writable == true`) |
| **`SteeringAngle` no `oj403.dbc`** (`ASTRO_EXPR`) | **NÃO** (`dbc_has_read_signal == false`) | **SIM** (`is_writable == true`) |

* **Etapas 1 a 3**: Empacotam e enviam todos os sinais **`SG_`** do `.dbc`.
* **Etapa 4**: Inverte a equação matemática e envia todos os sinais **`ASTRO_EXPR`** do `.dbc`.

---

## 10. Parte 6: Inicialização e Validação Antecipada em `load_and_validate_dbc()`

Na função `main()` ([linha 3251](file:///home/lume/astro/src/simulator_ackerman/simulator_ackerman_can_main.cpp#L3247-L3252)), o carregamento do `.dbc` foi encapsulado em [`load_and_validate_dbc()`](file:///home/lume/astro/src/simulator_ackerman/simulator_ackerman_can_main.cpp#L3150-L3202), que realiza uma inspeção completa nas tabelas hash antes da simulação começar:

```cpp
if (dbc_path != NULL)
{
    load_and_validate_dbc(dbc_path, argv[1]);
}
```

### O que `load_and_validate_dbc()` faz passo a passo:

1. **Carregamento seguro (`try / catch`)**:
   Verifica se o arquivo `.dbc` existe e tem permissão de leitura (`access(path, R_OK)`), e executa `process_dbc_file()` e `register_socket_to_interface()` dentro de um bloco `try / catch`, encerrando com mensagem amigável caso haja erro de sintaxe no `.dbc`.
2. **Validação dos sinais obrigatórios de comando (`write`)**:
   Verifica com `has_signal(dbc_can_process, "write", name)` se os três sinais de esforço (`ThrottleEffort`, `BrakesEffort`, `SteeringEffort`) estão presentes no `.dbc`. Caso falte algum deles, o simulador interrompe a inicialização imediatamente com `astro_die()`, evitando que a simulação rode sem que o veículo possa se mover.
3. **Diagnóstico antecipado dos sinais de telemetria (`read`)**:
   Percorre a lista `{"Velocity", "VelocityEncoder", "SteeringAngle", "Gear", "RPM", "SafeStop", "ManualOverride"}` e imprime no terminal o comportamento que o simulador adotará para cada sinal:
   * Se o sinal não existir no `.dbc`: avisa que não será publicado.
   * Se for `ASTRO_EXPR` e for inversível (`dbc_derived_signal_is_writable(name) == true`): informa que será publicado por inversão matemática da expressão.
   * Se for `ASTRO_EXPR` e não puder ser invertido: avisa se usará o frame legado por `vehicle_id` (caso de `VelocityEncoder` / `SteeringAngle`) ou se não será publicado.
4. **Pré-aquecimento do Cache de Inversão (`cache`)**:
   Ao chamar [`dbc_derived_signal_is_writable(name)`](file:///home/lume/astro/src/simulator_ackerman/simulator_ackerman_can_main.cpp#L451-L466) durante a partida na linha 3190, o simulador já executa o teste de inversão matemática uma única vez na inicialização e grava o resultado booleano (`true`/`false`) no `static std::unordered_map<std::string, bool> cache`. Quando o loop de tempo real (`publish_odometry`) começa a rodar, a consulta a `dbc_derived_signal_is_writable()` é instantânea ($O(1)$), lendo diretamente do cache!
