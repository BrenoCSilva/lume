# Arquitetura e Fluxo Geral do Obstacle Avoider

No [`main()`](file:///home/lume/astro/src/obstacle_avoider/obstacle_avoider_main.cpp#L2327-L2364) do `obstacle_avoider`, existem duas abordagens arquiteturais estruturadas no código através da diretiva `#ifdef USE_MAIN_LOOP_TO_PUBLISH`:

```c
#ifdef USE_MAIN_LOOP_TO_PUBLISH
    double astro_obstacle_avoider_collision_avoidance_frequency;
    astro_obstacle_avoider_collision_avoidance_frequency = astro_obstacle_avoider_initialize(argc, argv);

    while (1)
    {
        obstacle_avoider_timer_handler();
        check_message_absence_timeout_timer_handler();
        astro_ipc_sleep(1.0 / astro_obstacle_avoider_collision_avoidance_frequency);
    }
#else
    astro_obstacle_avoider_initialize(argc, argv);

    // Timers periódicos comentados:
    // astro_ipc_addPeriodicTimer(1.0 / freq, (TIMER_HANDLER_TYPE) obstacle_avoider_timer_handler, NULL);
    // astro_ipc_addPeriodicTimer(1.0 / freq, (TIMER_HANDLER_TYPE) check_message_absence_timeout_timer_handler, NULL);

    astro_ipc_loop(shutdown_obstacle_avoider);
#endif
```

### Modos de Operação: Síncrono vs Orientado a Eventos

1. **Modo Síncrono (`USE_MAIN_LOOP_TO_PUBLISH` ativo):**
   - O módulo roda em laço fechado no `while (1)` síncrono com `astro_ipc_sleep()`;
   - Executa estritamente na frequência configurada no `.ini` (`robot_collision_avoidance_frequency`, ex: 20 vezes por segundo / a cada $0{,}05\text{ s}$);
   - Nesse modo, os handlers de mensagens (`mpp`, `localize`, `mapa`) apenas guardam os dados no buffer e **não chamam** o [`obstacle_avoider_timer_handler()`](file:///home/lume/astro/src/obstacle_avoider/obstacle_avoider_main.cpp#L1356);
   - O watchdog [`check_message_absence_timeout_timer_handler()`](file:///home/lume/astro/src/obstacle_avoider/obstacle_avoider_main.cpp#L1461) roda ativamente a cada ciclo para monitorar se sensores ou comandos pararam de chegar.

2. **Modo Reativo / Orientado a Eventos (`#else` - Padrão em Produção):**
   - O `#define USE_MAIN_LOOP_TO_PUBLISH` permanece desativado e os timers periódicos do IPC ficam comentados;
   - O módulo entrega o controle para a escuta assíncrona do `astro_ipc_loop()`;
   - **O fluxo opera exclusivamente através dos seus handlers de mensagens**, disparando a avaliação de colisão no momento exato em que os dados de planejamento e percepção chegam.

---

## 1. Os Três Gatilhos de Entrada (Handlers)

A validação de colisão e a emissão de comandos para a base são disparadas por **3 eventos principais**:

```mermaid
flowchart TD
    M1["Mensagem MPP<br/>(astro_robot_ackerman_motion_command)"] --> H1["robot_ackerman_motion_command<br/>_message_handler()"]
    M2["Mensagem Localize<br/>(astro_localize_ackerman_globalpos)"] --> H2["localize_ackerman_globalpos<br/>_message_handler()"]
    M3["Mensagem Mapa de Distâncias<br/>(astro_obstacle_distance_mapper_map)"] --> H3["astro_obstacle_distance_mapper<br/>_on_distance_map_message_handler()"]

    H1 -->|"Imediato (Sem atraso)"| CORE["obstacle_avoider_timer_handler()"]
    H2 --> LIM{"can_execute_obstacle<br/>_avoider_handler()?<br/>(Intervalo &gt; 70 ms)"}
    H3 --> LIM

    LIM -- Sim --> CORE
    LIM -- Não --> IGNORE["Ignora disparo no ciclo<br/>(Evita sobrecarga de CPU)"]
```

### Detalhamento dos Handlers:

| Handler | Mensagem de Entrada | Papel no Módulo | Condição de Disparo |
| :--- | :--- | :--- | :--- |
| [`robot_ackerman_motion_command_message_handler`](file:///home/lume/astro/src/obstacle_avoider/obstacle_avoider_main.cpp#L1330) | `astro_robot_ackerman_motion_command_message` | Recebe o vetor de comandos $(v, \phi, \Delta t)$ gerado pelo MPP. Armazena no buffer circular de trajetórias. | **Imediato:**  |
| [`localize_ackerman_globalpos_message_handler`](file:///home/lume/astro/src/obstacle_avoider/obstacle_avoider_main.cpp#L1553) | `astro_localize_ackerman_globalpos_message` |Atualiza a localização (GPS/pose da cabine e semirreboques) e a odometria atual (velocidade v e ângulos das rodas ϕ ). | **Condicional:** Controlado por limitador de taxa (~14 Hz). |
| [`astro_obstacle_distance_mapper_on_distance_map_message_handler`](file:///home/lume/astro/src/obstacle_avoider/obstacle_avoider_main.cpp#L1588) | `astro_obstacle_distance_mapper_map_message` | Atualiza a matriz de distâncias aos obstáculos. | **Condicional:** Controlado por limitador de taxa (~14 Hz). |

---

## 2. Controle de frequência do handler GPS e mapa de distâncias

Como o `localize` (GPS + Odometria) e o mapa de distâncias publicam mensagens em frequências elevadas um filtro de suas mensagens é realizado para evitar sobrecarga de processamento.

Para solucionar isso, a função [`can_execute_obstacle_avoider_handler()`](file:///home/lume/astro/src/obstacle_avoider/obstacle_avoider_main.cpp#L1004-L1027) atua como um filtro das mensagens:

```c
int can_execute_obstacle_avoider_handler()
{
    if (param_only_mpp == 1)
        return (0);

    double current_time = astro_get_time();

    if ((current_time - executed_obstacle_avoider_handler_last_timestamp) > 0.07) // 70 milissegundos (~14,3 Hz)
    {
        executed_obstacle_avoider_handler_last_timestamp = current_time;
        return (1);
    }
    return (0);
}
```

* **Comportamento para o handler do mpp:** Quando o MPP envia um novo plano, ele **não consulta** essa função e dispara o `obstacle_avoider_timer_handler()` na hora. A janela de 70 ms serve apenas para controlar rechecagens baseadas em odometria e mapas.
* **Flag `-only_mpp on`:** Se ativada, bloqueia todas as chamadas vindas do GPS e do mapa, limitando o OA a agir apenas mediante novos planos.

---

## 3. Diagrama Macro Integrado de Fluxo

O sistema opera de forma orientada a eventos. O fluxo geral de avaliação e os blocos principais que alteram o plano seguem o pipeline abaixo:

```mermaid
flowchart TD
    subgraph Gatilhos ["1. Gatilhos de Entrada "]
        T1["Novo Plano MPP"] --> H1["Handler MPP<br/>(robot_ackerman_motion_command)"]
        T2["Novo Globalpos GPS"] --> H2["Handler Localize<br/>(localize_ackerman_globalpos)"]
        T3["Novo Mapa Distâncias"] --> H3["Handler Mapa Distâncias<br/>(obstacle_distance_mapper)"]
    end

    subgraph Emergencia ["2. Simulação de Emergência (Potential Collision)"]
        H2 --> PC["Simulação de Frenagem<br/><b style='color:#4fc3f7'>set_trajectory_from_deceleration()</b><br/>Volante travado em phi + Torricelli"]
        PC --> EXT["Extensão de Segurança<br/><b style='color:#4fc3f7'>set_extension_trajectory()</b><br/>Projeção de N metros à frente"]
        EXT --> ACC["Atualiza Aceleração de Emergência<br/><b style='color:#ffb74d'>global_acc_to_avoid_potential_collision</b>"]
    end

    H1 -->|"Imediato"| OA_TIMER["Handler Central de Decisão<br/><b style='color:#4fc3f7'>obstacle_avoider_timer_handler()</b>"]
    ACC -->|"Frequência max ~14 Hz"| OA_TIMER
    H3 -->|"Frequência max ~14 Hz"| OA_TIMER

    subgraph PipelineOA ["3. Obstacle_avoider_timer_handler()"]
        OA_TIMER --> CONSUME["Consumo Temporal da Fila<br/><b style='color:#4fc3f7'>consume_motion_command_time()</b><br/>Sincroniza o plano com o relógio atual"]
        CONSUME --> TRAT_PC["Tratamento Potential Collision<br/><b style='color:#ff5252'>potential_collision_treatment()</b><br/>Filtro 5 ciclos -> Desacelera com Acc"]
        TRAT_PC --> CENTR["Limite de Aceleração Centrípeta<br/><b style='color:#4fc3f7'>limit_centripetal_acceleration...()</b><br/>Proteção contra tombamento em curvas"]
        CENTR --> PHI_LIM["Limite de Velocidade por Phi<br/><b style='color:#4fc3f7'>limit_current_velocity_to_max_phi()</b><br/>Envelope seguro esterçamento x vel"]
        PHI_LIM --> OA_CORE["Validação Preventiva do Plano<br/><b style='color:#4fc3f7'>obstacle_avoider()</b><br/>Simula trajetória e atenua velocidades se houver obstáculo"]
    end

    OA_CORE --> PUB["Publicação Final<br/><b style='color:#69f0ae'>obstacle_avoider_publish_base_ackerman()</b><br/>Envia comandos validados para a base"]
```

> [!NOTE]
> **Divisão de Responsabilidades no *Potential Collision*:**  
> As Etapas 1 (`set_trajectory_from_deceleration`) e 2 (`set_extension_trajectory`) geram **projeções puramente hipotéticas** para diagnosticar colisões com direção fixa e margem mínima, **sem alterar o plano real do MPP**. Já a Etapa 3 (`potential_collision_treatment`) é **a única que modifica o plano de comandos**, sendo puramente reativa ao gatilho de `global_acc_to_avoid_potential_collision` sem fazer checagem geométrica em mapas.

---

### Etapas Principais do Fluxo:

1. **[`astro_obstacle_avoider_set_trajectory_from_deceleration()`](06_desaceleracao_preventiva_phi_fixo.md)**: Etapa 1 da simulação de emergência (executada no handler do `localize`). Projeta a frenagem máxima por Torricelli mantendo o ângulo $\phi$ fixo para verificar se o veículo consegue parar sem colidir.
2. **[`set_extension_trajectory_from_deceleration()`](07_extensao_seguranca_potential_collision.md)**: Etapa 2 da simulação de emergência (executada no handler do `localize` apenas se a Etapa 1 não colidir). Avança o veículo até o limite da extensão de segurança (`extension_length`, padrão $3{,}0\text{ m}$) para garantir a margem mínima livre à frente.
3. **[`consume_motion_command_time()`](05_consumo_temporal_e_sincronizacao.md)**: Primeira etapa do `obstacle_avoider_timer_handler()`. Subtrai o tempo já transcorrido desde a última atualização, mantendo a sincronia temporal da fila de comandos e compensando a latência do MPP.
4. **[`potential_collision_treatment()`](08_atuacao_potential_collision_treatment.md)**: Etapa 3 da simulação de emergência (no `obstacle_avoider_timer_handler()`). Verifica se o alarme vindo da Etapa 1 ou 2 persistiu por 5 ciclos consecutivos (Debounce 5/5) e desacelera os comandos do plano até zerar a velocidade.
5. **[`limit_centripetal_acceleration_reducing_velocity()`](04_restricoes_dinamica_veiculo.md)**: Modula a velocidade em trechos de curva acentuada para garantir que a aceleração centrípeta não exceda $1{,}05 \cdot a_{c,\max}$.
6. **[`limit_current_velocity_according_to_max_phi()`](04_restricoes_dinamica_veiculo.md)**: Restringe a velocidade caso o ângulo de esterçamento $\phi$ exija velocidade menor.
7. **[`obstacle_avoider()`](02_planejamento_preventivo.md)**: Executa a simulação física futura do plano do MPP e atenua as velocidades caso haja obstáculos no caminho.
8. **Publicação para `base_ackerman`**: Envia o plano validado/modificado para o controle de baixo nível.
9. **[Fusão e Intersecção: Distance Map vs. Moving Objects (DATMO)](09_fusao_mapas_distance_map_e_moving_objects.md)**: Detalha a origem dos dados (LiDAR + Mapa Offline + DATMO), a distinção de objetos estáticos vs. móveis e a redundância de segurança na ordem de checagem de colisão.

---


## 5. Códigos de Erro Emitidos

O módulo publica os seguintes erros no barramento do sistema (`ASTRO_ERROR_CODE`):

* **`ASTRO_ERROR_CODE_201`**: Colisão física confirmada do veículo com obstáculo enquanto em movimento (`Robot_collided_with_obstacle`, Severidade 0 - Erro Fatal).
* **`ASTRO_ERROR_CODE_203`**: Recálculo e redução preventiva de velocidade no plano devido a colisão detectada à frente ou margem de parada física insuficiente por Torricelli (`Obstacle_avoider_recalculated_velocity`, Severidade 4). Detalhes em [`02_planejamento_preventivo.md`](02_planejamento_preventivo.md).
* **`ASTRO_ERROR_CODE_204`**: Colisão detectada na simulação de emergência (*potential collision* ou extensão de segurança com volante fixo). Detalhes em [`08_atuacao_potential_collision_treatment.md`](08_atuacao_potential_collision_treatment.md).
* **`ASTRO_ERROR_CODE_205`**: Limitação preventiva de velocidade imposta pela geometria da curva e esterçamento de referência (`OA_hold_velocity_to_reference_phi`, Severidade 4).

---
