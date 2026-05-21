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



Com certeza! Vamos revisar e justificar cada um dos passos que você fez, porque o seu raciocínio geral esteve muito correto, mas tem um pequeno detalhe crítico que você precisa corrigir.

### 1. Execução do `mss_process_prepare.py`
**O que você fez:** Rodou o script em cima do seu arquivo `process.ini`.
**Justificativa:** Correto! A documentação dizia que o antigo `map_server` foi substituído pelo novo `mss` e pelo `obstacle_evidence`. O papel desse script python é automatizar o trabalho chato: ele entra no seu arquivo `process.ini`, troca o executável do `map_server` pelo `mss`, adiciona as chamadas do `obstacle_evidence` e já embute a maioria dos novos parâmetros necessários.
*Obs: Note que a saída do script apontou para um mapa do `atego1730` em vez da pasta `primeiros_passos`. Isso indica que o seu arquivo de process provavelmente ainda estava com a variável `MAP_PATH` apontando para o lugar errado (o mapa do Atego).*

### 2. Modificação manual no arquivo `.ini` (CUIDADO AQUI ⚠️)
**O que você fez:** Adicionou 3 parâmetros manuais para o `mapper_laser_ldmrs` no seu arquivo `astro-astro2p-ufes-sensorbox-6.ini` com o valor `off`.
**Justificativa:** O script python injeta os novos parâmetros do `obstacle_evidence`, mas o `mapper` (que aparentemente ainda está ativo no seu process) continuou reclamando que precisava conhecer as medidas do laser. Então você fez bem em adicioná-los para parar o erro do programa.
**O Problema:** Você colocou os valores como `off`, mas esses parâmetros esperam valores **numéricos** (tamanho em metros). Se você deixa `off`, o sistema lê como `0.0`. Isso significa que o alcance máximo do seu laser será de 0 metros e ele não vai enxergar nenhum obstáculo!
**Como corrigir:** Altere os valores no seu arquivo `.ini` para números reais. Exemplo:
```ini
mapper_laser_ldmrs_range_max 130.0
mapper_laser_ldmrs_safe_height_from_ground -20.0
mapper_laser_ldmrs_unsafe_height_above_ground 10.5
```

### 3. Conversão do mapa com o `prepare_mss_maps`
**O que você fez:** Usou a ferramenta de conversão apontando para a pasta original e gerando a nova `map_files_mss`.
**Justificativa:** Exatamente o que a documentação pede. O novo formato MSS exige um arquivo `info.txt` que descreve a resolução e o tamanho dos blocos de mapa. Essa ferramenta pega o mapa velho e recria ele do zero no padrão do MSS. Foi esse passo que finalmente criou o seu arquivo `info.txt` e permitiu que o `./mss` funcionasse.
*Obs: Você usou o `--params ../src/astro-mercedes-atego.ini`. Idealmente você deveria ter usado o seu próprio arquivo do astro2p (`../data/ufes/primeiros_passos/parameters_ini/astro-astro2p-ufes-sensorbox-6.ini`), mas como as definições de resolução de grade do mapa (`mapper_map_grid_res`) costumam ser padrão entre os robôs, acabou funcionando.*

### Resumo do que você precisa fazer agora
Sua lógica de resolução de problemas foi ótima e você entendeu a nova arquitetura do sistema! Só volte no seu arquivo de parâmetros (`astro-astro2p-ufes-sensorbox-6.ini`), procure os três parâmetros do `mapper_laser_ldmrs` que você adicionou e troque o `off` pelos valores numéricos (como mostrei acima), senão o robô não vai enxergar os obstáculos. Feito isso, tudo deve rodar liso!
