# Arquitetura e Fluxo Geral do Simulator Ackerman CAN

No [`main()`](file:///home/lume/astro/src/simulator_ackerman/simulator_ackerman_can_main.cpp#L3010-L3077) do `simulator_ackerman_can`, o módulo inicializa a interface CAN, faz o parse do arquivo de definições de rede (DBC), cria uma thread dedicada para escuta de mensagens do barramento e inicia o loop de simulação física do veículo:

```cpp
// Em simulator_ackerman_can_main.cpp:
can_sockfd = init_can(argv[1]);
read_parameters(argc, argv, &simulator_conf);

if (dbc_path != NULL)
{
    astro_hal_can_interactions::process_dbc_file(dbc_can_process, dbc_path, argv[1]);
    astro_hal_can_interactions::register_socket_to_interface(dbc_can_process, argv[1], can_sockfd);
}

// Cria a thread concorrente de escuta da CAN
pthread_create(&can_thread, NULL, update_state_thread, NULL);

// Inicia o loop periódico de simulação e envio de telemetria
astro_ipc_pooling(one_simulation_step, shutdown_module);
```

Diferente de módulos puramente orientados a eventos de IPC, o simulador opera desacoplado em **duas frentes concorrentes**: a **escuta contínua de comandos** e a **simulação periódica com envio de telemetria**.

---

## 1. As Duas Frentes do Módulo

| Frente | Ponto de Entrada | Modelo de Execução | Papel no Módulo |
| :--- | :--- | :--- | :--- |
| **Escuta CAN (Comandos)** | [`update_state_thread()`](file:///home/lume/astro/src/simulator_ackerman/simulator_ackerman_can_main.cpp#L2992-L3007) | **Orientado a Eventos (`while(1)`)** | Bloqueia no socket CAN aguardando pacotes. Quando chegam mensagens de comando (mensagem `"write"` no DBC), extrai aceleração, freio, esterçamento e marcha, computando a dinâmica veicular. |
| **Simulação e Envio (Telemetria)** | [`one_simulation_step()`](file:///home/lume/astro/src/simulator_ackerman/simulator_ackerman_can_main.cpp#L1782-L1793) | **Orientado a Tempo (Timer Periódico)** | Disparado ciclicamente na frequência da simulação (ex: a cada 20 ms / 50 Hz). Atualiza a odometria do carro virtual e transmite frames de volta para a rede CAN (mensagem `"read"` no DBC: RPM e marcha). |

---

## 2. Diagrama Macro Integrado de Fluxo

O diagrama abaixo ilustra o fluxo completo desde a chegada de comandos pela CAN até a atualização física e o posterior envio periódico de telemetria:

```mermaid
flowchart TD
    CAN_IN["<span style='padding:0 24px'>Barramento Físico / Virtual CAN</span><br/><b style='color:#69f0ae; padding:0 24px'>Entrada de Dados (Socket CAN)</b><br/><span style='padding:0 24px'>⠀⠀Recebe pacotes enviados pelo Vehicle Driver⠀⠀</span>"]

    CAN_IN -->|"Chegada de frames CAN"| RECV

    subgraph ThreadEscuta ["1. Thread de Escuta (update_state_thread)"]
        RECV["<span style='padding:0 24px'>Recepção no Socket CAN</span><br/><b style='color:#4fc3f7; padding:0 24px'>recv_frame()</b><br/><span style='padding:0 24px'>⠀⠀Bloqueia aguardando pacotes da rede⠀⠀</span>"]
        RECV --> UPDATE["<span style='padding:0 24px'>Roteamento de Mensagens CAN</span><br/><b style='color:#4fc3f7; padding:0 24px'>update_state_from_can()</b><br/><span style='padding:0 24px'>⠀⠀Direciona frame para o modo DBC ativo⠀⠀</span>"]
        UPDATE --> PROC_DBC["<span style='padding:0 24px'>Decodificação via DBC</span><br/><b style='color:#4fc3f7; padding:0 24px'>process_frame_from_dbc_mode()</b><br/><span style='padding:0 24px'>⠀⠀Extrai Gear, Throttle, Brakes e Steering⠀⠀</span>"]
        PROC_DBC --> CALC_DYN["<span style='padding:0 24px'>Cálculo da Dinâmica Veicular</span><br/><b style='color:#4fc3f7; padding:0 24px'>compute_v_and_phi_from_efforts()</b><br/><span style='padding:0 24px'>⠀⠀Converte esforços em velocidade v e phi⠀⠀</span>"]
        CALC_DYN --> SET_TARGET["<span style='padding:0 24px'>Atualização do Comando Alvo</span><br/><b style='color:#4fc3f7; padding:0 24px'>set_target_v_and_target_phi()</b><br/><span style='padding:0 24px'>⠀⠀Grava v e phi na estrutura estática⠀⠀</span>"]
    end

    SET_TARGET -->|"Salva na memória RAM"| MEM["<span style='padding:0 24px'>Memória Compartilhada do Simulador</span><br/><b style='color:#ffb74d; padding:0 24px'>current_motion_command_vector</b><br/><span style='padding:0 24px'>⠀⠀Estrutura static compartilhada na RAM⠀⠀</span>"]

    TIMER["<span style='padding:0 24px'>Gatilho Cíclico de Tempo</span><br/><b style='color:#ffb74d; padding:0 24px'>astro_ipc_pooling (20 ms / 50 Hz)</b><br/><span style='padding:0 24px'>⠀⠀Dispara periodicamente a simulação⠀⠀</span>"] -->|"Ciclo de 20 ms"| SIM_STEP
    MEM -->|"Consome v e phi alvo"| SIM_STEP

    subgraph ThreadSimulacao ["2. Thread de Simulação e Envio (one_simulation_step)"]
        SIM_STEP["<span style='padding:0 24px'>Passo Central da Simulação</span><br/><b style='color:#4fc3f7; padding:0 24px'>simulate_car_and_publish_readings()</b><br/><span style='padding:0 24px'>⠀⠀Coordena o avanço físico do veículo⠀⠀</span>"]
        SIM_STEP --> RECALC["<span style='padding:0 24px'>Integração Cinemática Ackermann</span><br/><b style='color:#4fc3f7; padding:0 24px'>astro_simulator_ackerman_recalc_pos()</b><br/><span style='padding:0 24px'>⠀⠀Atualiza pose, velocidade e esterçamento⠀⠀</span>"]
        RECALC --> PUB_ODO["<span style='padding:0 24px'>Publicação de Odometria CAN</span><br/><b style='color:#4fc3f7; padding:0 24px'>publish_odometry()</b><br/><span style='padding:0 24px'>⠀⠀Calcula RPM simulado conforme a marcha⠀⠀</span>"]
        PUB_ODO --> BATCH["<span style='padding:0 24px'>Empacotamento de Sinais DBC</span><br/><b style='color:#4fc3f7; padding:0 24px'>build_can_frames_batch()</b><br/><span style='padding:0 24px'>⠀⠀Codifica Gear e RPM na mensagem read⠀⠀</span>"]
        BATCH --> SEND["<span style='padding:0 24px'>Transmissão no Socket CAN</span><br/><b style='color:#4fc3f7; padding:0 24px'>send_message_frame()</b><br/><span style='padding:0 24px'>⠀⠀Envia frames CAN pelo descritor can_sockfd⠀⠀</span>"]
    end

    SEND -->|"Transmite telemetria"| CAN_OUT["<span style='padding:0 24px'>Barramento Físico / Virtual CAN</span><br/><b style='color:#69f0ae; padding:0 24px'>Saída de Telemetria (Socket CAN)</b><br/><span style='padding:0 24px'>⠀⠀Disponibiliza odometria para a pilha⠀⠀</span>"]
```

---

## 3. Detalhamento dos Componentes Principais

### A. Inicialização e Parsing do DBC
No início do programa, a função [`astro_hal_can_interactions::process_dbc_file()`](file:///home/lume/astro/include/astro/astro_hal_can_interactions.hpp#L95-L150) lê o arquivo DBC passado por `--dbc_path` e constrói as tabelas hash na memória RAM:
* **`composite_to_message`**: Mapeia chaves compostas (ex: `"write::gear"`) para o ID CAN correspondente (`0x98EF00FE`).
* **`composite_to_signal`**: Mapeia chaves compostas para as propriedades físicas de decodificação (`start_bit`, `size`, `factor`, `offset`).
* **`signal_to_val_map`**: Tabela de estados discretos (`VAL_`), traduzindo códigos brutos em strings legíveis (`"neutral"`, `"drive"`, `"reverse"`).

### B. Recepção e Decodificação de Comandos
A thread [`update_state_thread()`](file:///home/lume/astro/src/simulator_ackerman/simulator_ackerman_can_main.cpp#L2992-L3007) processa cada frame que chega:
1. Compara o ID do frame com o ID da mensagem de atuadores registrada como `"write"` no DBC.
2. Extrai o estado textual da marcha via [`read_can_signal_state()`](file:///home/lume/astro/include/astro/astro_hal_can_interactions.hpp#L295-L341) e atualiza o estado interno da transmissão (`target_gear_id` e `gear_map`).
3. Extrai os esforços percentuais de aceleração, freio e esterçamento.
4. Calcula a nova velocidade desejada ($v$) e ângulo desejado ($\phi$).
5. Chama [`set_target_v_and_target_phi()`](file:///home/lume/astro/src/simulator_ackerman/simulator_ackerman_can_main.cpp#L2472-L2485), gravando os dados em uma variável declarada com **`static`**, garantindo que o comando persista válido na memória RAM para o consumo da outra thread.

### C. Simulação Cinemática e Transmissão Cíclica
No loop [`one_simulation_step()`](file:///home/lume/astro/src/simulator_ackerman/simulator_ackerman_can_main.cpp#L1782-L1793):
1. [`astro_simulator_ackerman_recalc_pos()`](file:///home/lume/astro/src/simulator_ackerman/simulator_ackerman.cpp) lê os comandos alvo da memória estática e integra o modelo cinemático Ackermann para atualizar a velocidade real `v` e o ângulo real `phi`.
2. [`publish_odometry()`](file:///home/lume/astro/src/simulator_ackerman/simulator_ackerman_can_main.cpp#L683-L795) calcula a rotação do motor (`RPM`) em função da velocidade atual e da relação de marcha.
3. As informações de telemetria são agrupadas no vetor de envio com a mensagem `"read"` do DBC.
4. O HAL executa [`build_can_frames_batch()`](file:///home/lume/astro/include/astro/astro_hal_can_interactions.hpp#L579-L627), inserindo os sinais de telemetria dentro do payload de 8 bytes e enviando para o socket com [`send_message_frame()`](file:///home/lume/astro/include/astro/astro_hal_can_interactions.hpp#L510-L548).

---

## 4. Estruturas de Memória Compartilhadas

Para permitir a comunicação entre a thread de escuta e a thread de simulação sem corrupção de dados:

* **`simulator_config`**: Ponteiro global para a configuração e estado dinâmico do simulador (`astro_simulator_ackerman_config_t`).
* **`current_motion_command_vector` (estático)**: Estrutura local com tempo de vida global (`static`), cujo ponteiro é atribuído a `simulator_config->current_motion_command_vector`.
* **`driver_outputs`**: Armazena o estado atual das saídas do motorista simulado (ex: `driver_outputs.current_gear` e `driver_outputs.rpm`).

---

## 5. Índice de Documentação Específica

Para o detalhamento aprofundado de cada funcionalidade:

1. **[`01_arquitetura_e_fluxo_geral.md`](01_arquitetura_e_fluxo_geral.md)**: Visão macro da arquitetura bi-thread, inicialização e fluxo integrado.
2. **`02_leitura_e_processamento_de_comandos.md`** *(em desenvolvimento)*: Detalhamento do parser de entrada, extração de sinais DBC e máquina de estados da transmissão.
3. **`03_escrita_e_envio_odometria_can.md`** *(em desenvolvimento)*: Detalhamento do cálculo de RPM, empacotamento em lote (`build_can_frames_batch`) e transmissão via socket CAN.
