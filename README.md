# Extensão de Segurança da Trajetória (`set_extension_trajectory_from_deceleration`)

A função [`set_extension_trajectory_from_deceleration()`](file:///home/lume/astro/src/obstacle_avoider/obstacle_avoider.cpp#L1181-L1305) constitui a **segunda etapa da verificação preventiva** executada dentro do [`localize_ackerman_globalpos_message_handler()`](file:///home/lume/astro/src/obstacle_avoider/obstacle_avoider_main.cpp#L1626-L1656).

Ela **só é executada caso a Etapa 1 ([`06_desaceleracao_preventiva_phi_fixo.md`](06_desaceleracao_preventiva_phi_fixo.md)) não tenha detectado colisão** (`fabs(global_acc_to_avoid_potential_collision) < EPSILON`). Seu propósito é verificar se essa condição livre de colisão com o ângulo $\phi$ fixo também se mantém ao longo de uma **extensão mínima de segurança** configurada para o veículo (por padrão, $3{,}0\text{ m}$).

---

## 1. Posição no Fluxo de Execução e Condição de Entrada

Dentro do escopo de [`localize_ackerman_globalpos_message_handler()`](file:///home/lume/astro/src/obstacle_avoider/obstacle_avoider_main.cpp#L1553), a Etapa 2 entra em ação imediatamente após a Etapa 1 aprovar a frenagem por Torricelli:

```mermaid
flowchart TD
    subgraph HANDLER ["localize_ackerman_globalpos_message_handler()"]
        PREV["Etapa 1: astro_obstacle_avoider_set_trajectory_from_deceleration()<br/>Simula parada por Torricelli com phi fixo"]
        PREV --> RES1{"Etapa 1 livre de colisão E<br/>effective_extension_length &gt; 0.0?"}
        
        RES1 -- "Não (Já colidiu na Etapa 1)" --> GATE{"can_execute_obstacle_avoider_handler()<br/>(dt &gt; 70 ms?)"}
        
        RES1 -- "Sim" --> GOAL_ADJ["Define desired_distance_to_obstacle<br/>(Padrão 3.0m ou ajustado perto do final_goal)"]
        GOAL_ADJ --> EXT["Etapa 2: set_extension_trajectory_from_deceleration()<br/>Estende projeção com phi fixo até desired_distance_to_obstacle"]
        
        EXT --> RES2{"Colidiu na Extensão?"}
        RES2 -- "Sim" --> SET_ACC["global_acc_to_avoid_potential_collision = -a_max<br/>Dispara ASTRO_ERROR_CODE_204"]
        RES2 -- "Não" --> FREE["global_acc_to_avoid_potential_collision = 0.0"]
        
        SET_ACC --> GATE
        FREE --> GATE
        GATE -- "Sim" --> TIMER["obstacle_avoider_timer_handler()"]
    end
```

### Definição do Limite de Extensão (`effective_extension_length`)
O tamanho da margem que o veículo deve respeitar até qualquer obstáculo vem do parâmetro `obstacle_avoider_potential_collision_extension_length` (inicializado em $3{,}0\text{ m}$) ou é forçado diretamente para **$3{,}0\text{ m}$** caso a flag `force_potential_collision_usage` esteja ativa:

```cpp
double effective_extension_length = potential_collision_extension_length;

if (force_potential_collision_usage)
{
    effective_extension_length = 3.0;
}
```
Ou seja, mesmo que o veículo esteja em baixa velocidade e consiga parar em poucos centímetros na Etapa 1, ele ainda deve cumprir um **limite mínimo de distância para o obstáculo** (tipicamente $3{,}0\text{ m}$).

### Ajuste da Distância Próximo ao Destino Final (`final_goal`)
Antes de chamar a função de extensão, o código verifica se o veículo já está chegando ao ponto de destino final (`final_destination`) e se esse destino fica naturalmente próximo de obstáculos no mapa offline (como em docas de carregamento):

```cpp
if ((fabs(global_acc_to_avoid_potential_collision) < EPSILON) && (effective_extension_length > 0.0))
{
    double desired_distance_to_obstacle = effective_extension_length;
    double default_desired_distance_to_obstacle = effective_extension_length;

    if (use_potential_collision_final_goal_implementation == 1)
    {
        if ((distance_between_pose_and_final_goal < default_desired_distance_to_obstacle) && (distance_between_final_goal_and_obstacle > 0.0)
            && (distance_between_final_goal_and_obstacle < default_desired_distance_to_obstacle))
        {
            double distance_to_compare = default_desired_distance_to_obstacle;

            if (distance_between_pose_and_final_goal < default_desired_distance_to_obstacle)
            {
                distance_to_compare = distance_between_pose_and_final_goal;
            }

            desired_distance_to_obstacle = astro_fmax(0.6 * distance_between_final_goal_and_obstacle, distance_to_compare);
        }
    }

    global_acc_to_avoid_potential_collision = set_extension_trajectory_from_deceleration(pose, astro_robot_ackerman_config,
            astro_semi_trailer_config, &occupancy_map, current_phi, motion_desired, desired_distance_to_obstacle, msg->timestamp);
}
```
* **Em condução normal na via:** `desired_distance_to_obstacle` permanece igual a `effective_extension_length` ($3{,}0\text{ m}$).
* **Próximo ao `final_goal` com obstáculo:** Reduz proporcionalmente `desired_distance_to_obstacle` para evitar falsos positivos que travariam o caminhão antes de concluir o estacionamento/atracamento.

---

## 2. Fluxo de Funcionamento da `set_extension_trajectory_from_deceleration()`

O objetivo da [`set_extension_trajectory_from_deceleration()`](file:///home/lume/astro/src/obstacle_avoider/obstacle_avoider.cpp#L1181-L1305) é **projetar o veículo mantendo seu ângulo de esterçamento (`initial_phi`) fixo por toda a distância `desired_distance_to_obstacle`** e garantir que nenhum obstáculo invada essa extensão de segurança.

Para isso, a função executa um loop espacial (`while (DIST2D(initial_pose, iterative_pose) <= desired_distance_to_obstacle)`):

### Passo 1: Avanço Geométrico da Pose ("Passinho" de $0{,}1\text{ s}$)
Partindo da `pose` atual vinda do `localize`, a função utiliza a [`astro_libcarmodel_recalc_pos_ackerman()`](file:///home/lume/astro/src/obstacle_avoider/obstacle_avoider.cpp#L1226) para calcular a coordenada do veículo (cabine e carretas) passados **$0{,}1\text{ s}$** (`full_time_interval = 0.1`, `delta_t = 0.05`) com velocidade virtual fixa de `motion_desired * 0.5` ($+0{,}5\text{ m/s}$ para frente ou $-0{,}5\text{ m/s}$ para ré) mantendo o ângulo `initial_phi` travado:

```cpp
iterative_pose = astro_libcarmodel_recalc_pos_ackerman(
    &iterative_pose, motion_desired * 0.5, initial_phi, full_time_interval,
    &distance_traveled, delta_t, &astro_robot_ackerman_config, &astro_semi_trailer_config
);
```
> *Nota:* Diferente da Etapa 1, aqui **não há redução de velocidade por Torricelli**. O cálculo avança geometricamente em pequenos passos (~$0{,}05\text{ m}$ cada) apenas para varrer o caminho no espaço até atingir a distância limite `desired_distance_to_obstacle`.

---

### Passo 2: Verificação de Colisão e Critério de Parada do Loop
A cada nova pose calculada na extensão, verifica-se imediatamente se há colisão da cabine ou das carretas contra:
1. O **mapa de distâncias / mapas especiais** (`obstacle_distance_map` / `maps_obstacle_avoider`).
2. O **mapa de ocupação** (`occupancy_map`).
3. Algum **objeto móvel** (`moving_objects`).

```cpp
while (DIST2D(initial_pose, iterative_pose) <= desired_distance_to_obstacle)
{
    // 1. Avança a pose 0.1s mantendo initial_phi fixo no sentido de motion_desired
    // 2. Verifica colisão nos 3 níveis de mapas (Mapas Especiais, Occupancy Map e Moving Objects)

    if (flag_check_collision)
    {
        set_astro_error_for_oa_mediation(ASTRO_ERROR_CODE_204, custom_string, timestamp);
        returned_acc = -astro_robot_ackerman_config.maximum_deceleration_forward;

        if (motion_desired < 0.0)
        {
            returned_acc = astro_robot_ackerman_config.maximum_deceleration_reverse;
        }
        break;
    }
}
```

* **Se houver colisão dentro da extensão:** A previsão é **interrompida imediatamente (`break`)**, registra-se o erro `ASTRO_ERROR_CODE_204` e retorna-se a desaceleração máxima de frenagem (`-maximum_deceleration_forward` ou `maximum_deceleration_reverse` em ré) para a variável `global_acc_to_avoid_potential_collision` no handler do `localize`.
* **Se não houver colisão:** O loop segue avançando até que a distância euclidiana entre a pose inicial e a pose projetada (`DIST2D(initial_pose, iterative_pose)`) ultrapasse `desired_distance_to_obstacle` (ex.: $3{,}0\text{ m}$), retornando `0.0` e confirmando que toda a margem de extensão à frente do veículo está livre.

---

## 3. Diagrama de Sequência e Fluxo Interno

```mermaid
sequenceDiagram
    autonumber
    participant OA_MAIN as Handler do Localize (Main)
    participant EXT as set_extension_trajectory_from_deceleration
    participant MODEL as astro_libcarmodel_recalc_pos_ackerman
    participant MAPS as Verificação de Colisão (3 Mapas)

    Note over OA_MAIN: Etapa 1 aprovada (acc == 0.0)<br/>Calcula desired_distance_to_obstacle (ex: 3.0m)
    OA_MAIN->>EXT: Inicia verificação da extensão
    
    loop Enquanto distancia projetada nao atingir desired_distance_to_obstacle
        EXT->>MODEL: Projeta pose 0.1s à frente (initial_phi fixo, v = motion_desired * 0.5)
        MODEL-->>EXT: Nova iterative_pose
        EXT->>MAPS: Checa iterative_pose contra Mapas Especiais, Occupancy e Moving Objects
        alt Colisão Detectada na Extensão!
            MAPS-->>EXT: flag_check_collision = true
            EXT->>OA_MAIN: Interrompe (break) e retorna frenagem máxima (-a_max)
        else Livre
            MAPS-->>EXT: Sem colisão no passo atual
        end
    end
    
    Note over EXT,OA_MAIN: Percorreu toda a margem (ex: 3.0m) sem colidir!
    EXT-->>OA_MAIN: Retorna acc = 0.00 m/s^2 (Extensão 100% livre)
```
