> [!NOTE]
> Este texto terá uma barra lateral azul (Informação).

> [!TIP]
> Este terá uma barra verde (Dica).

> [!IMPORTANT]
> Este terá uma barra roxa (Importante).

> [!WARNING]
> Este terá uma barra amarela (Aviso).

> [!CAUTION]
> Este terá uma barra vermelha (Cuidado/Perigo).

A principal alteração foi realizada na lógica de identificação e leitura de arquivos de áudio, substituindo o uso de IDs numéricos por identificadores em string. Para suportar essa mudança, os arquivos de áudio foram renomeados e a formatação do caminho foi atualizada. Além disso 3 novos arquivos de áudio foram criados para abranger uma quantidade maior de estados do veículo, porém por não encontrar a mesma voz utilizada nos arquivos de áudio base, esses novos arquivos foram gerados com uma voz masculina do Antonio (https://ttsmaker.com/). OS testes realizados com o simulador abrangeram os seguintes estados: Initializing, Stopped, Free_Running, End_Of_Path_Reached e o modo_autonomo_ativado.

Detalhes:
- Atualização da formatação do caminho do arquivo (`../data/audios/%s.wav`) para utilizar o nome do estado em vez de um ID numérico.
- Renomeação dos arquivos de áudio físicos para corresponder à nova convenção de strings.
- Atualização na função `behavior_selector_state_message_handler` para enviar a string descritiva (ex: "veiculo_parado").
- Atualização na função `ford_escape_status_handler` para também adotar strings na identificação do arquivo.
- O deslocamento de pastas foi alterado também, agora considerando que o executável está sendo chamado da pasta bin.

futuras melhorias: padronizar a voz dos estados, expor o veículo a condições que abregem mais estados e inserir o restante dos estados: Free_Reverse_Running, Stopping_To_Reverse, Stopping_To_Go_Forward, Recovering_From_Error e Stopped_At_Ramp.

Mafefile:
- Remoção da compilação direta de `global_user_interface_interface.c` em `SOURCES`/`OBJECTS`.
- Inclui flag `-lglobal_user_interface_interface` e o path `-L$(ASTRO_HOME)/lib` em `LFLAGS`.
- Altera dependência em LFLAGS de `ford_escape_hybrid_interface` para `vehicle_driver_interface`.


Refatora o Makefile do módulo `audio_interface` para utilizar linkagem de bibliotecas no lugar da compilação direta de arquivos externos.

Principais alterações:
- Remoção do arquivo `../global/global_user_interface_interface.c` das etapas de compilação (`SOURCES` e `OBJECTS`), passando a utilizar a sua versão compilada via flag de linkagem (`-lglobal_user_interface_interface`).
- Adição explícita do diretório de bibliotecas `$(ASTRO_HOME)/lib` em `LFLAGS`.
- Substituição da biblioteca `ford_escape_hybrid_interface` por `vehicle_driver_interface` nas dependências.
- Limpeza do caminho inserido em `PUBLIC_INCLUDES`.
- Inclusão das variáveis de metadados `MODULE_NAME` e `MODULE_COMMENT`.

- Remove compilação direta de `global_user_interface_interface.c` em `SOURCES`/`OBJECTS`.
- Inclui flag `-lglobal_user_interface_interface` e o path `-L$(ASTRO_HOME)/lib` em `LFLAGS`.
- Altera dependência em LFLAGS de `ford_escape_hybrid_interface` para `vehicle_driver_interface`.
