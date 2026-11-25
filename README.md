# 8. O template: criando um módulo novo

Ao longo do manual construímos, passo a passo, um par de módulos que se comunicam por IPC. Este capítulo entrega o resultado já pronto, em forma de **template**: um esqueleto de módulo, **compilável e testado**, para ser usado como ponto de partida. Em vez de recomeçar do capítulo 1 a cada módulo novo, você cria o seu a partir dele e troca só o que é específico.

## 8.1 O que o template contém

O template são **dois módulos**, como no exemplo do manual: um que é **dono da mensagem** e a publica, e um que a **assina**. Ficam ao lado dos demais módulos, em `src/`:

```text
src/template_module/                    ← dono da mensagem
├── Makefile
├── template_module_messages.h          # a mensagem: struct, nome e formato
├── template_module_interface.h         # funções da mensagem: define, publish, subscribe, unsubscribe
├── template_module_interface.cpp
├── template_module_main.cpp            # main: conexão, parâmetros, publicação, Ctrl+C
├── template_module.ini                 # parâmetros do módulo
├── process-template_module.ini         # arquivo de processos (proccontrol)
└── new_module.sh                       # script que cria um módulo novo a partir deste

src/template_module_consumer/           ← assina a mensagem
├── Makefile
└── template_module_consumer_main.cpp   # main: conexão, assinatura, handler, Ctrl+C
```

A mensagem de exemplo, `astro_template_module_data`, tem um contador e um valor:

| Campo | Tipo | Significado |
|---|---|---|
| `counter` | `int` | Número sequencial da mensagem |
| `value` | `double` | Valor de exemplo |
| `timestamp`, `host` | `double`, `char *` | Obrigatórios em toda mensagem |

O `template_module` publica uma mensagem por intervalo `period`, que ele lê do `.ini`. O `template_module_consumer` assina a mensagem e imprime o que recebe. Executados juntos:

```text
template_module: publicou counter=0 value=0.00
template_module: publicou counter=1 value=0.50
template_module: publicou counter=2 value=1.00

template_module_consumer: aguardando mensagens.
template_module_consumer: counter=0 value=0.00 (de template_module@lume-M, t=1795630784.112)
template_module_consumer: counter=1 value=0.50 (de template_module@lume-M, t=1795630785.112)
template_module_consumer: counter=2 value=1.00 (de template_module@lume-M, t=1795630786.113)
```

## 8.2 Criando um módulo novo

O script `new_module.sh` copia os dois módulos do template e **troca o nome** em todos os arquivos. Execute-o dentro da pasta do `template_module`, passando o nome do seu módulo, em `snake_case`:

```bash
cd $ASTRO_HOME/src/template_module
./new_module.sh meu_sensor
```

```text
Modulos criados:
  /home/lume/astro/src/meu_sensor
  /home/lume/astro/src/meu_sensor_consumer

Proximos passos:
  1. Edite meu_sensor_messages.h (campos da mensagem) e a main de cada modulo.
  2. Compile, nesta ordem:  (cd /home/lume/astro/src/meu_sensor && make)  e depois  (cd /home/lume/astro/src/meu_sensor_consumer && make)
  3. Inicie: central, param_daemon ../src/meu_sensor/meu_sensor.ini, e os dois modulos.
```

Foram criados **dois módulos novos**, `meu_sensor` e `meu_sensor_consumer`, com tudo renomeado:

| No template | No seu módulo |
|---|---|
| `template_module_messages.h` | `meu_sensor_messages.h` |
| `astro_template_module_data_message` | `astro_meu_sensor_data_message` |
| `ASTRO_TEMPLATE_MODULE_DATA_MESSAGE_NAME` | `ASTRO_MEU_SENSOR_DATA_MESSAGE_NAME` |
| `template_module_period` (no `.ini`) | `meu_sensor_period` |
| `MODULE_NAME = TEMPLATE MODULE` | `MODULE_NAME = MEU SENSOR` |

O script **só cria arquivos**: não altera nenhum arquivo do sistema. Os artefatos de compilação do template (`.o`, bibliotecas, executáveis) não são copiados. O script recusa:

| Situação | Resposta |
|---|---|
| Nome que não está em `snake_case` (por exemplo `MeuModulo` ou `9abc`) | Erro, nada é criado |
| O próprio nome do template | Erro, nada é criado |
| Uma das pastas já existe | Erro, nada é sobrescrito |

## 8.3 O que trocar em cada arquivo

O template marca com o comentário `TROQUE` os pontos que você precisa adaptar:

| Arquivo | O que trocar |
|---|---|
| `<nome>_messages.h` | Os **campos** da struct e o `_FMT`, mantendo os dois **espelhados**. Escreva a **unidade** e o referencial de cada campo |
| `<nome>_main.cpp` | Em `publish_data()`, **preencher os dados reais**. Em `read_parameters()`, **um item por parâmetro** |
| `<nome>_consumer_main.cpp` | No handler, **tratar os dados** recebidos. Lembre-se de que eles só valem dentro do handler |
| `<nome>.ini` | Os **parâmetros** do módulo, com unidade no comentário |

O restante (a conexão, a interface, o `Makefile`, o tratamento do Ctrl+C) normalmente **não precisa mudar**.

## 8.4 Passo a passo

Depois de criar o módulo com o script:

1. **Defina a mensagem.** Em `<nome>_messages.h`, troque os campos da struct e o `_FMT`, mantendo-os espelhados (capítulo 2). O template chama a mensagem de `data`. Se quiser outro nome (por exemplo `pose`), troque `data` e `DATA` pelo novo nome nos símbolos `..._data_message`, `..._DATA_MESSAGE_...`, `publish_data` e `data_message_handler`, nos três lugares onde aparecem: a mensagem, a interface e as mains.
2. **Publique dados reais.** Em `publish_data()`, no `<nome>_main.cpp`, preencha a struct com o que o seu módulo produz (capítulos 2 e 4).
3. **Configure os parâmetros.** Acrescente cada parâmetro à lista de `read_parameters()` e ao `<nome>.ini` (capítulo 5). Leia-os **antes** de usá-los.
4. **Trate os dados no consumidor.** Em `data_message_handler`, use os campos recebidos e **copie** o que precisar guardar (capítulo 3).
5. **Compile, na ordem:** primeiro o dono da mensagem, depois o consumidor.

   ```bash
   cd $ASTRO_HOME/src/meu_sensor && make
   cd $ASTRO_HOME/src/meu_sensor_consumer && make
   ```

   O dono vem primeiro porque é ele que **publica** o header e a biblioteca da interface, de que o consumidor precisa (capítulo 3).
6. **Teste à mão**, em terminais separados, com a `central` recém-iniciada (capítulos 1 e 5):

   ```bash
   ./central
   ./param_daemon ../src/meu_sensor/meu_sensor.ini
   ./meu_sensor_consumer
   ./meu_sensor
   ```

7. **Integre.** Ajuste o `process-<nome>.ini` e suba tudo com o `proccontrol`, como no capítulo 6. Inclua os dois módulos em uma das listas `PACKAGES_TIER` do `src/Makefile`, para entrarem no build completo.
8. **Confira o checklist** do capítulo 7 (seção 7.8) antes de considerar o módulo pronto.

## 8.5 Onde está cada conceito

Se você precisa lembrar **como** ou **por quê** algo é feito no template, esta tabela aponta o arquivo e o capítulo:

| Conceito | Onde está no template | Capítulo |
|---|---|---|
| Conectar à `central`, `dispatch` | `main`: `astro_ipc_initialize` e `astro_ipc_dispatch` | 1 |
| Encerrar com Ctrl+C | `shutdown_module` | 1 |
| A mensagem: struct, nome e formato | `<nome>_messages.h` | 2 |
| `define`, `publish` (e `timestamp` e `host`) | `<nome>_interface.cpp` | 2 |
| Publicação periódica com timer | `main` do dono: `astro_ipc_addPeriodicTimer` | 2 |
| `subscribe`, handler, `LATEST` | `<nome>_interface.cpp` e a `main` do consumidor | 3 |
| Publicar a interface (`PUBLIC_*`, biblioteca) | `Makefile` do dono | 3 |
| Parâmetros: `astro_param_t`, `read_parameters` | `main` do dono e `<nome>.ini` | 5 |
| Arquivo de processos | `process-<nome>.ini` | 6 |
| Ordem dos passos do `main` | Comentários numerados na `main` | 7 |

## 8.6 Acrescentando uma segunda mensagem

Para o módulo publicar mais de uma mensagem, repita para cada uma o mesmo conjunto de peças:

1. Em `<nome>_messages.h`: uma **struct** e as macros `..._MESSAGE_NAME` e `..._MESSAGE_FMT`, dela, mantendo struct e formato espelhados.
2. Em `<nome>_interface.h/.cpp`: as funções `define`, `publish` (e, se alguém for assinar, `subscribe` e `unsubscribe`) da nova mensagem, no padrão `astro_<nome>_<ação>_<mensagem>_message`.
3. Na `main` do dono: **uma chamada à função `define`** da nova mensagem, junto com a da primeira, antes de qualquer publicação.

Nada muda no `Makefile`: o header e a biblioteca da interface já estão publicados.

## 8.7 Checkpoint

Chegamos ao fim do manual. Ao longo dos capítulos você construiu, e agora tem como ponto de partida reutilizável:

- [x] um **template** de módulo, compilável, com o dono da mensagem e um consumidor, em `src/template_module/` e `src/template_module_consumer/`;
- [x] um **script**, `new_module.sh`, que cria um módulo novo a partir dele, com tudo renomeado;
- [x] os **pontos de adaptação** marcados com `TROQUE`, e o passo a passo para chegar a um módulo seu funcionando;
- [x] o caminho de volta aos capítulos para cada conceito do template.
