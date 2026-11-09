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

## 5. Resumo da Condição de Retorno em `send_can_frame_v_lume()`

Voltando à [linha 502](file:///home/lume/astro/src/simulator_ackerman/simulator_ackerman_can_main.cpp#L502-L503):
```cpp
if (!dbc_read_signal_is_derived_only("VelocityEncoder") || dbc_derived_signal_is_writable("VelocityEncoder"))
    return;
```

1. **Se `!dbc_read_signal_is_derived_only("VelocityEncoder")` for `true`**:
   Significa que `VelocityEncoder` é um sinal físico normal `SG_` (ex: `ojAstro_fullDBC.dbc`) ou o DBC usa apenas `SG_ Velocity`. O `publish_odometry()` vai empacotá-lo normalmente no `build_can_frames_batch()`. Logo, `send_can_frame_v_lume()` dá **`return`** e não faz nada.
2. **Se for `false`, avalia `|| dbc_derived_signal_is_writable("VelocityEncoder")`**:
   Significa que `VelocityEncoder` é uma fórmula `ASTRO_EXPR` (ex: `oj403.dbc`). Se o novo inversor matemático conseguir inverter essa fórmula, o `publish_odometry()` vai enviá-lo na etapa de sinais derivados. Logo, `send_can_frame_v_lume()` também dá **`return`** e não faz nada.
