# Documentação Técnica: Execução do Ensaio de PID sem a flag `-tune_pid_mode`

---

## 1. Introdução e Objetivo da Task

### 1.1. Contexto Original
Historicamente, para realizar o ensaio e a sintonia fina dos controladores de velocidade e direção (PID) na plataforma ASTRO, utilizava-se o binário `./ford_escape_hybrid_tune_pid` em conjunto com o `./ford_escape_hybrid` rodando com a flag:
```bash
./ford_escape_hybrid -tune_pid_mode on
```

A flag `-tune_pid_mode on` atuava como um "atalho" de emergência no código-fonte do `vehicle_driver`. Ela instruía o módulo a ignorar toda a máquina de estados de segurança, as travas de hardware, o estado do freio de estacionamento e as permissões de navegação, atribuindo a velocidade da trajetória diretamente à variável de controle:
```cpp
if (tune_pid_mode)
{
    g_desired_velocity = v; // Atalho: burlava todas as checagens do sistema
}
```

### 1.2. O Problema
Esse atalho criava uma divergência crítica entre o **ambiente de teste de PID** e o **veículo em operação autônoma real**:
1. O controlador não era testado com as proteções reais de engate de marcha da transmissão.
2. Não se validavam as travas de liberação de atuação (`unlock`) e início de movimento (`go`).
3. O código ficava poluído com bifurcações condicionais que não representavam a operação real do carro.

### 1.3. O Objetivo da Task
O objetivo desta tarefa foi **reabilitar e padronizar o teste de PID sem o uso da flag `-tune_pid_mode on`**, garantindo que:
- O `vehicle_driver` seja executado em seu **modo normal e legítimo** de produção (`./ford_escape_hybrid -can vcan0`).
- Os controladores PID respondam através do pipeline oficial de controle e segurança.
- O ensaio possa ser executado em ambiente de bancada/simulação **sem a necessidade de rodar a pilha completa de navegação** (`route_planner`, `behavior_selector`, `task_manager`, etc.).

---

## 2. O Desafio Arquitetural: A Trava do Modo Normal

Ao desativar a flag `-tune_pid_mode`, o `vehicle_driver` cai compulsoriamente no bloco `else` da função de despacho de comandos (`vehicle_driver_main.cpp`), onde o movimento só é autorizado se **cinco condições simultâneas** forem satisfeitas:

```cpp
if (g_go_state && g_unlock_state &&
    (behavior_selector_low_level_state != Stopped) &&
    (behavior_selector_low_level_state != End_Of_Path_Reached2) &&
    (behavior_selector_going_backwards != -1))
{
    // MOVIMENTO AUTORIZADO:
    // Avalia o debounce mecânico da marcha e aplica g_desired_velocity = v;
}
else
{
    // MOVIMENTO BLOQUEADO:
    g_desired_velocity = 0.0;
    set_gear_command_to_neutral_if_needed(); // Força o câmbio para Neutro!
}
```

Se qualquer um desses cinco requisitos for falso, o sistema entra no bloco `else`: a velocidade desejada é zerada e a transmissão é forçada para **Neutro (128)**, impedindo qualquer movimentação do veículo.

---

## 3. Análise Detalhada dos 5 Requisitos de Bloqueio

Abaixo está o detalhamento de cada uma das condições, explicando o que representam no veículo real e por que elas travavam o teste quando os módulos superiores estavam desligados:

### 3.1. `g_go_state` (Autorização de Partida / Freio de Mão)
- **O que representa:** Indica se o veículo recebeu a ordem de partir. No veículo real, o estado `0` mantém o freio de estacionamento acionado e corta a aceleração.
- **Quem envia normalmente:** O módulo `navigator_ackerman` ou o botão "GO" da interface gráfica `navigator_gui2`.
- **Por que travava:** Sem o `navigator_ackerman` ativo, `g_go_state` iniciava em `0` e permanecia em `0`.

### 3.2. `g_unlock_state` (Trava de Segurança dos Atuadores)
- **O que representa:** É a trava de segurança física/lógica dos atuadores (volante, acelerador e freio). Impede que motores elétricos recebam corrente sem liberação explícita do operador.
- **Quem envia normalmente:** Módulos de auditoria/segurança ou o utilitário `util_publish_lock_unlock_message -unlock`.
- **Por que travava:** Sem uma mensagem de desbloqueio, `g_unlock_state` iniciava travado (`0`).

### 3.3. `behavior_selector_low_level_state != Stopped` (Autorização Comportamental)
- **O que representa:** O seletor de comportamentos avalia pedestres, semáforos, cruzamentos e obstáculos. Se detectar perigo ou parada programada, seu estado vai para `Stopped`.
- **Quem envia normalmente:** O módulo `behavior_selector`.
- **Por que travava:** Quando o `behavior_selector` não está rodando, a variável inicia por padrão com o valor enum `Stopped`. Além disso, dentro da física da simulação (`simulator_ackerman_simulation.c`), se o estado do BS for `Stopped`, o próprio simulador zera a tração das rodas.

### 3.4. `behavior_selector_low_level_state != End_Of_Path_Reached2` (Fim de Trajetória)
- **O que representa:** Indica se o veículo atingiu o ponto final do plano de navegação.
- **Quem envia normalmente:** O módulo `behavior_selector`.
- **Por que travava:** Sem envio ativo, era necessário assegurar que o estado não assumisse esse código de parada terminal.

### 3.5. `behavior_selector_going_backwards != -1` (Decisão de Sentido de Marcha)
- **O que representa:** Define se o veículo deve trafegar para frente (`0`) ou para trás / ré (`1`). O valor `-1` representa um estado de transição ou indecisão direcional (por exemplo, quando o veículo acabou de trocar de intenção de sentido e precisa aguardar estabilização).
- **Quem envia normalmente:** A mensagem de IPC `path_goals_and_annotations` emitida pelo `behavior_selector`.
- **Por que travava:** Se nenhuma meta for publicada ou se o sinal oscilar com intervalo inferior a 3 segundos, `going_backwards` é travado em `-1`. Consequentemente, o `vehicle_driver` comuta para Neutro.

---

## 4. A Justificativa da Solução: Auto-Habilitação no `tune_pid.c`

Para solucionar esse impasse sem precisar reativar módulos pesados que não dizem respeito ao controle PID (como SLAM, mapas globais e planejadores de missão), a rotina do `vehicle_driver_tune_pid.c` foi modificada para **emular o papel essencial desses sinais de segurança durante a inicialização**:

```c
static void
arm_vehicle_driver_for_motion(double target_velocity)
{
    // 1. Libera os atuadores
    astro_ford_escape_unlock();

    // 2. Solta o freio de estacionamento (seta g_go_state = 1)
    astro_navigator_ackerman_go();

    // 3. Emula o Behavior Selector e define sentido de marcha
    // Aguarda 3.5s para acomodação da caixa de câmbio (Neutro -> Low/Drive)
    double start_wait = astro_get_time();
    while (astro_get_time() - start_wait < 3.5)
    {
        publish_behavior_selector_state(Initializing);
        publish_behavior_selector_path_goals_and_annotations(target_velocity);
        astro_ipc_sleep(0.1);
    }
}
```

Com essa estrutura:
1. `g_unlock_state` é setado para `1`.
2. `g_go_state` é setado para `1`.
3. O estado comportamental passa a ser `Initializing` (diferente de `Stopped` e de `End_Of_Path_Reached2`).
4. `going_backwards` é definido com base no sinal de `target_velocity` (engatando `LOW` para valores positivos e `REVERSE` para negativos).

Dessa forma, o `vehicle_driver` entra de forma legítima e transparente no bloco de **MOVIMENTO AUTORIZADO**, permitindo que os testes de sintonia de PID ocorram com a fidelidade real da dinâmica veicular.

