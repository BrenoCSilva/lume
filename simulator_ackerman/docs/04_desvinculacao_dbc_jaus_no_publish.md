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

## 6. Parte 2: A Marcha (`Gear`) e a Velocidade na `fila_de_envio` do `publish_odometry()`

### 6.0 Desacoplamento do `read::Gear` via `get_dbc_read_gear_value()`
Internamente, o simulador usa `current_gear` (baseado na tabela `gear_states[]` do Actros: `0x38 = 56` para Ré, `0x3C = 60` para Neutro e `0x44 = 68` para 1ª marcha) na física de aceleração (`compute_v_from_throttle_and_brakes_efforts`). Porém, na rede CAN, diferentes caminhões possuem códigos `VAL_` diferentes para `read::Gear`:
- No **Actros (`oj403.dbc`)**: `"gear_1"` = `68` (`0x44`), `"reverse_1"` = `56` (`0x38`), `"neutral"` = `60` (`0x3C`).
- No **Atego (`ojAstro_fullDBC.dbc`)**: `"gear_1"` = `62` (`0x3E`), `"reverse_1"` = `59` (`0x3B`), `"neutral"` = `60` (`0x3C`).

Para não enviar `0x44` (`68`) mocado e causar `ASTRO_ERROR_CODE_110` (`Gear read by DBC and unknown!`) no `vehicle_driver` ao usar o `ojAstro_fullDBC.dbc`, a função [`get_dbc_read_gear_value()`](file:///home/lume/astro/src/simulator_ackerman/simulator_ackerman_can_main.cpp#L365-L393) traduz o ID lógico (`driver_outputs.current_gear`: `-2..12`) para o nome de estado (`"neutral"`, `"reverse_1"`, `"gear_1"`, etc.) e busca o valor correspondente em `dbc_can_process.state_to_val_map["read::gear"]`.

---

Em [`publish_odometry()` (linhas 919-931)](file:///home/lume/astro/src/simulator_ackerman/simulator_ackerman_can_main.cpp#L919-L931), após verificar e adicionar `Gear` e `RPM`, o código prepara o envio da velocidade:

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

## 10. Parte 6: Como Funcionam `dbc_derived_signal_is_writable()` e `dbc_build_derived_frame()`

Na Etapa 4 de [`publish_odometry()` (linhas 935-949)](file:///home/lume/astro/src/simulator_ackerman/simulator_ackerman_can_main.cpp#L935-L949), para cada sinal candidato (`Velocity`, `VelocityEncoder`, `SteeringAngle`), o `if` executa duas funções em sequência:

```cpp
if (dbc_derived_signal_is_writable(d.name) && dbc_build_derived_frame(d.name, d.value, &frame, &anchor))
    astro_hal_can_interactions::send_message_frame(dbc_can_process, "read", anchor, &frame);
```

1. **Primeiro pergunta (`dbc_derived_signal_is_writable`)**: *"Esse sinal existe como fórmula `ASTRO_EXPR` no `.dbc` e pode ser invertido matematicamente?"*
2. **Se `true`, manda montar (`dbc_build_derived_frame`)**: *"Pegue o valor físico real (`d.value`) e preencha os bytes do `frame` para envio na rede CAN!"*

---

### 10.1 Função 1: `dbc_derived_signal_is_writable(const char *name)`

Definida nas [linhas 451-466](file:///home/lume/astro/src/simulator_ackerman/simulator_ackerman_can_main.cpp#L451-L466):

```cpp
static bool
dbc_derived_signal_is_writable(const char *name)
{
    static std::unordered_map<std::string, bool> cache;

    auto it = cache.find(name);

    if (it != cache.end())
        return (it->second);

    struct can_frame frame;
    std::string anchor;
    bool ok = dbc_build_derived_frame(name, 0.1, &frame, &anchor);
    cache[name] = ok;

    return (ok);
}
```

#### O que ela faz passo a passo:
1. **Tabela de memória (`cache`)**: Mantém um `static std::unordered_map<std::string, bool> cache` durante toda a execução do programa. Se o sinal `name` já foi testado antes (`it != cache.end()`), retorna imediatamente `true` ou `false` sem recalcular nada.
2. **Teste único com corpo de prova (`0.1`)**: Na primeira vez que é chamada para um sinal, cria um `struct can_frame frame` **local e descartável** (que nunca é enviado à rede CAN) e chama `dbc_build_derived_frame(name, 0.1, &frame, &anchor)`.
   * **Por que `0.1`?** Porque `0.1` (seja `0.1 m/s` ou `0.1 rad`) é um valor diferente de zero (evitando que bytes zerados passem por coincidência) e pequeno o suficiente para caber nos limites `[min|max]` de qualquer sensor veicular sem saturar.
   * O resultado (`ok = true` ou `false`) é salvo em `cache[name]` para todas as consultas futuras.

---

### 10.2 Função 2: `dbc_build_derived_frame()` (Dividida em 3 Blocos)

Definida nas [linhas 412-446](file:///home/lume/astro/src/simulator_ackerman/simulator_ackerman_can_main.cpp#L412-L446), seu objetivo é receber o nome do sinal derivado (`sig_name`, ex: `"VelocityEncoder"`) e o valor físico desejado (`target`, ex: `5.0 m/s`), descobrir qual valor bruto `x` gravar nos sinais físicos da mensagem e preencher o `frame`.

#### Bloco 1: Localizar a mensagem no `.dbc` ([linhas 414-423](file:///home/lume/astro/src/simulator_ackerman/simulator_ackerman_can_main.cpp#L414-L423))
```cpp
uint32_t id = 0;

if (!dbc_read_signal_is_derived_only(sig_name) ||
    !Libdbc::detail::get_derived_signal_message_id(dbc_can_process.parser, "read", sig_name, id))
    return (false);

const Libdbc::Message *msg = dbc_find_message_by_id(id);

if ((msg == NULL) || msg->get_signals().empty())
    return (false);
```
* Verifica se `sig_name` é de fato um sinal derivado (`ASTRO_EXPR`), descobre o CAN ID (`id`) da mensagem onde essa fórmula foi declarada e obtém o ponteiro `msg` (por exemplo, no `oj403.dbc`, `id = 0x201`, contendo os sinais físicos `WheelSpeed1` e `WheelSpeed2`).

#### Bloco 2: "Regra de Três" para encontrar `x` ([linhas 425-435](file:///home/lume/astro/src/simulator_ackerman/simulator_ackerman_can_main.cpp#L425-L435))
A função auxiliar [`dbc_eval_derived_with_all_signals(msg, sig_name, valor, &tmp, &resultado)`](file:///home/lume/astro/src/simulator_ackerman/simulator_ackerman_can_main.cpp#L386-L406) escreve `valor` em todos os sinais físicos da mensagem (`WheelSpeed1 = valor`, `WheelSpeed2 = valor`) e usa `read_signal()` do HAL para avaliar quanto a fórmula `ASTRO_EXPR` retorna em `resultado`.

```cpp
struct can_frame tmp;
double f0, f1, y;

if (!dbc_eval_derived_with_all_signals(msg, sig_name, 0.0, &tmp, &f0) ||
    !dbc_eval_derived_with_all_signals(msg, sig_name, 1.0, &tmp, &f1))
    return (false);

if (fabs(f1 - f0) < 1e-12)
    return (false);

double x = (target - f0) / (f1 - f0);
```
* Como as expressões usuais são afins/lineares ($f(x) = a \cdot x + b$), basta avaliar dois pontos para descobrir toda a reta:
  1. **Ponto 1 (`x = 0.0`)**: Grava `0.0` nos sinais físicos e obtém **`f0`** ($f(0) = b$).
  2. **Ponto 2 (`x = 1.0`)**: Grava `1.0` nos sinais físicos e obtém **`f1`** ($f(1) = a + b \implies a = f_1 - f_0$).
  3. **Regra de Três (`linha 435`)**: Para que o resultado da fórmula atinja `target`, o valor `x` a ser escrito nos sinais físicos é:
     $$x = \frac{\text{target} - f_0}{f_1 - f_0}$$

#### Bloco 3: Gravar `x` no `frame` final e tirar a "Prova Real" ([linhas 437-445](file:///home/lume/astro/src/simulator_ackerman/simulator_ackerman_can_main.cpp#L437-L445))
```cpp
if (!dbc_eval_derived_with_all_signals(msg, sig_name, x, frame, &y))
    return (false);

if (fabs(y - target) > 2e-3 + 1e-3 * fabs(target))
    return (false);

*anchor = msg->get_signals().front().name;

return (true);
```
* **Linha 437**: Grava o `x` calculado dentro do `frame` definitivo e lê de volta o resultado da fórmula em **`y`**.
* **Linha 440 (Prova Real)**: Confere se o valor `y` lido de volta a partir dos bytes reais do `frame` bateu com o `target` desejado (dentro da tolerância `0.002 + 0.1%`).
  * **Se bateu**: O `frame` está pronto! Guarda o nome do primeiro sinal físico em `*anchor` (necessário para `send_message_frame()` identificar a interface CAN) e retorna **`true`**.
  * **Se não bateu** (ex: expressão multi-byte como `SteerB0 + 256*SteerB1 + 3584*SteerB2` no `ojAstro_fullDBC.dbc`, onde gravar o mesmo `x` nos 3 bytes arredonda e gera erro alto): a prova real falha e retorna **`false`**!

---

## 11. Parte 7: Inicialização e Validação Antecipada em `load_and_validate_dbc()`

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

---

## 12. Parte 8: Correção da Realimentação Positiva e Estabilização de Ângulo (`phi_stabilization`)

### 12.1 Sintoma e Diagnóstico do Erro 302 (`ASTRO_ERROR_CODE_302`)
Durante os testes com o arquivo [`oj403.dbc`](file:///home/lume/astro/src/astro_hal/dbc_file_example/oj403.dbc), o simulador apresentou um comportamento anômalo:
1. O veículo saía da pista mesmo em linha reta.
2. O planejador de movimento (`mpp` — *Model Predictive Planner*) ficava preso disparando continuamente:
   ```text
   ASTRO_ERROR_REG(302, 4, "phi_stabilization")
   ```
3. Nos logs de envio e leitura CAN, o ângulo de direção físico convergira para o batente mecânico máximo negativo (`phi = -0.622 rad`), enquanto o alvo do PID de esforço pedia valores cada vez mais positivos (`SteerRaw` com alvo `1.197 rad`), criando um ciclo vicioso de realimentação positiva.

### 12.2 Causa Raiz: Inversão de Sinal em `phi_to_dbc_steering_angle()`
No [`oj403.dbc`](file:///home/lume/astro/src/astro_hal/dbc_file_example/oj403.dbc#L27), a mensagem `0x2A` define a expressão derivada de esterçamento:
```dbc
CM_ BO_ 0x2A "ASTRO_EXPR SteeringAngle = (/ SteerRaw 507000.0)";
```
Como essa expressão possui um único sinal físico linear (`SteerRaw`), `dbc_derived_signal_is_writable("SteeringAngle")` retornou `true` e o simulador passou a utilizar a função [`phi_to_dbc_steering_angle()`](file:///home/lume/astro/src/simulator_ackerman/simulator_ackerman_can_main.cpp#L513-L521) para converter o ângulo físico `phi` no alvo CAN a ser empacotado.

No código legado do simulador, a função continha:
```cpp
// VERSÃO ANTERIOR (COM BUG):
static double
phi_to_dbc_steering_angle(double phi, double v)
{
    return ((1.0 + v * v * simulator_config->understeer_coeficient2) * atan(simulator_config->distance_between_front_and_rear_axles * tan(-phi)));
}
```

A inclusão de `-phi` e de `atan(L * tan(-phi))` foi motivada por um equívoco ao ler [`handle_read_steering_angle_signal()`](file:///home/lume/astro/src/vehicle_driver/vehicle_driver_dbc.cpp#L883-L890) no `vehicle_driver`:
1. O `vehicle_driver` convertia o `phi_can` lido para curvatura XGV:
   $$\text{curvature} = \frac{\tan\left(\frac{\phi_{\text{can}}}{1 + v^2 u}\right)}{L}, \quad \text{steering\_angle\_sensor} = -\arctan(\text{curvature})$$
2. Logo em seguida, em [`report_curvature_message_handler()`](file:///home/lume/astro/src/vehicle_driver/vehicle_driver_main.cpp#L3448-L3467), o `vehicle_driver` desfazia a conta através de sua função inversa:
   $$\text{raw\_phi} = \text{get\_phi\_from\_curvature}(-\tan(\text{steering\_angle\_sensor})) \cdot (1 + v^2 u) \equiv +\phi_{\text{can}}$$
   E aplicava a calibração do robô: $\text{filtered\_phi} = \text{raw\_phi} \cdot \text{robot\_phi\_multiplier} + \text{phi\_bias}$.

Dessa forma:
* Ao colocar `-phi` no simulador, o sinal de ângulo chegava invertido ao `vehicle_driver`.
* Quando o `mpp` solicitava uma pequena correção para a direita durante a fase parada de alinhamento (`phi_stabilization`), o simulador enviava ângulo para a esquerda.
* O PID do driver comandava mais esforço no sentido oposto para compensar, saturando o volante em `-0.622 rad`. Como o erro angular nunca zerava, o `mpp` mantinha ativo o erro `302` indefinidamente.

### 12.3 Correção Definitiva
A função foi corrigida em [`simulator_ackerman_can_main.cpp`](file:///home/lume/astro/src/simulator_ackerman/simulator_ackerman_can_main.cpp#L509-L521) para compensar exclusivamente a escala do robô (`robot_phi_multiplier`), preservando o sinal positivo correto:
```cpp
// No vehicle_driver, handle_read_steering_angle_signal() aplica -atan(curvature(phi_can)) e logo em seguida
// report_curvature_message_handler() aplica a inversa get_phi_from_curvature(-tan(...)) e multiplica por
// robot_phi_multiplier. Portanto, para o driver recuperar o phi exato do simulador, basta dividir por robot_phi_multiplier.
static double
phi_to_dbc_steering_angle(double phi, double v)
{
	(void) v;

	if (fabs(robot_phi_multiplier) > 1e-6)
		return (phi / robot_phi_multiplier);

	return (phi);
}
```

---

## 13. Parte 9: Suporte a Ponto Flutuante IEEE-754 (`SIG_VALTYPE_ : 1`) nos Esforços de Atuação

### 13.1 Descrição do Problema
No arquivo [`oj403.dbc`](file:///home/lume/astro/src/astro_hal/dbc_file_example/oj403.dbc#L37), a mensagem `0x2E` (`SteeringEffort`) é declarada com o tipo especial `SIG_VALTYPE_`:
```dbc
BO_ 0x2E write: 8 DRIVER
 SG_ SteeringEffort : 0|32@1+ (1,0) [-100|100] "%" CONTROLLER
SIG_VALTYPE_ 0x2E SteeringEffort : 1;
```
A diretiva `SIG_VALTYPE_ ... : 1;` estipula no padrão DBC da Vector que o campo de 32 bits codifica um número de ponto flutuante de precisão simples no padrão **IEEE-754**, e não um número inteiro sem sinal (`uint32`).

Antes da correção, a rotina padrão `read_signal()` do HAL desempacotava os 32 bits como inteiro e aplicava os fatores de escala lineares. Como consequência, uma palavra binária IEEE-754 (por exemplo, `0xBF800000` correspondente a `-1.0f`) era interpretada como o inteiro `3212836864`, gerando valores absurdos de esterçamento (ex: `Steer: -6601130.24%`) que quebravam a física do simulador.

### 13.2 Implementação no HAL e no Simulador
A decodificação foi implementada no método principal [`read_signal()`](file:///home/lume/astro/src/utilities/astrohpp/astro_hal_can_interactions.hpp#L745-L756):
```cpp
const Libdbc::Signal *sig = result.second;

// Espelha physical_to_raw(): sinais com SIG_VALTYPE_ : 1 carregam um float IEEE-754 de 32 bits,
// nao um inteiro. Sem isso os bits do float seriam lidos como int32 (ex.: 0x2E SteeringEffort no oj403.dbc).
if ((sig->size == 32) && Libdbc::detail::is_float32_signal(dbc_parser.parser, msg->id(), sig->name))
{
    const uint32_t raw_bits = static_cast<uint32_t>(unpack_can_frame(sig, frame));
    float raw_float = 0.0f;
    __builtin_memcpy(&raw_float, &raw_bits, sizeof(raw_float));

    return (static_cast<double>(raw_float) * sig->factor + sig->offset);
}
```

E sincronizada entre:
* [`src/utilities/astrohpp/astro_hal_can_interactions.hpp`](file:///home/lume/astro/src/utilities/astrohpp/astro_hal_can_interactions.hpp) (código-fonte primário dos headers utilitários).
* [`include/astro/astro_hal_can_interactions.hpp`](file:///home/lume/astro/include/astro/astro_hal_can_interactions.hpp) (cópia distribuída para inclusão global).

Com essa alteração, o simulador passa a receber os esforços de direção, acelerador e freio com precisão exata tanto em DBCs inteiros quanto em DBCs com tipos IEEE-754 flutuantes.
