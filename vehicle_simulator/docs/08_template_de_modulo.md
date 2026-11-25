# 8. O template: criando um módulo novo

Ao longo do manual construímos, passo a passo, um par de módulos que se comunicam por IPC. Este capítulo entrega o resultado em forma de **template**: um esqueleto de módulo, **compilável e testado**, para ser ponto de partida. Em vez de recomeçar do capítulo 1 a cada módulo novo, você copia o template, troca o nome e adapta só o que é específico do seu módulo.

O template **usa a mesma mensagem do exemplo** (a pose, com `x`, `y`, `theta` e `v`), então não há nada novo para entender. Ele é o exemplo do manual no estado do fim do capítulo 3, com um parâmetro do capítulo 5, e **sem** o que era só da simulação: a cinemática do capítulo 4 e o alerta de velocidade. O `vehicle_simulator` completo continua sendo a referência de um módulo real.

## 8.1 O que o template contém

São **dois módulos**, como no exemplo: um que é **dono da mensagem** e a publica, e um que a **assina**. Ficam ao lado dos demais módulos, em `src/`:

```text
src/template_module/                    ← dono da mensagem
├── Makefile
├── template_module_messages.h          # a mensagem: struct, nome e formato
├── template_module_interface.h         # funções da mensagem: define, publish, subscribe, unsubscribe
├── template_module_interface.cpp
├── template_module_main.cpp            # main: conexão, parâmetros, publicação, Ctrl+C
├── template_module.ini                 # parâmetros do módulo
└── process-template_module.ini         # arquivo de processos (proccontrol)

src/template_module_consumer/           ← assina a mensagem
├── Makefile
└── template_module_consumer_main.cpp   # main: conexão, assinatura, handler, Ctrl+C
```

O `template_module` publica a pose a cada `period` segundos, valor que ele lê do `.ini`. Os dados são fixos, como no capítulo 2. O `template_module_consumer` assina a mensagem e imprime o que recebe. Executados juntos:

```text
template_module: publicou x=0.00 y=0.00 theta=0.00 v=2.00
template_module: publicou x=0.00 y=0.00 theta=0.00 v=2.00

template_module_consumer: aguardando mensagens.
template_module_consumer: x=0.00 y=0.00 theta=0.00 v=2.00 (de template_module@lume-M, t=1795631520.559)
template_module_consumer: x=0.00 y=0.00 theta=0.00 v=2.00 (de template_module@lume-M, t=1795631521.560)
```

## 8.2 Criando um módulo novo a partir dele

O processo tem três passos: copiar as pastas, renomear os arquivos e trocar o nome **dentro** dos arquivos. Nos exemplos abaixo o módulo novo se chama `meu_sensor`.

### Passo 1: copiar as duas pastas

Se você já compilou o template, rode antes o `make clean` em cada uma, para não copiar objetos e executáveis:

```bash
cd $ASTRO_HOME/src
(cd template_module && make clean)
(cd template_module_consumer && make clean)

cp -r template_module meu_sensor
cp -r template_module_consumer meu_sensor_consumer
```

O `make clean` deixa nas pastas só os arquivos-fonte.

### Passo 2: renomear os arquivos

Os arquivos que têm `template_module` no nome passam a ter o nome do seu módulo:

| No template | No seu módulo |
|---|---|
| `template_module_messages.h` | `meu_sensor_messages.h` |
| `template_module_interface.h` | `meu_sensor_interface.h` |
| `template_module_interface.cpp` | `meu_sensor_interface.cpp` |
| `template_module_main.cpp` | `meu_sensor_main.cpp` |
| `template_module.ini` | `meu_sensor.ini` |
| `process-template_module.ini` | `process-meu_sensor.ini` |
| `template_module_consumer_main.cpp` (na pasta `meu_sensor_consumer`) | `meu_sensor_consumer_main.cpp` |

O `Makefile` mantém o nome.

### Passo 3: trocar o nome dentro dos arquivos

O nome do template aparece **dentro** dos arquivos em três formas. Substitua as três, **só nas duas pastas novas**, com a função "substituir em arquivos" do seu editor:

| Forma | Procurar | Substituir por |
|---|---|---|
| Minúsculas, com `_` | `template_module` | `meu_sensor` |
| Maiúsculas, com `_` | `TEMPLATE_MODULE` | `MEU_SENSOR` |
| Maiúsculas, com espaço | `TEMPLATE MODULE` | `MEU SENSOR` |

A primeira troca também cobre o nome do consumidor (`template_module_consumer` passa a `meu_sensor_consumer`).

Para conferir que nenhum nome antigo ficou para trás, procure por `template` nas duas pastas novas:

```bash
cd $ASTRO_HOME/src
grep -ri template meu_sensor meu_sensor_consumer
```

Não deve haver nenhuma ocorrência.

Os símbolos renomeados ficam assim:

| No template | No seu módulo |
|---|---|
| `astro_template_module_pose_message` | `astro_meu_sensor_pose_message` |
| `ASTRO_TEMPLATE_MODULE_POSE_MESSAGE_NAME` | `ASTRO_MEU_SENSOR_POSE_MESSAGE_NAME` |
| `astro_template_module_publish_pose_message` | `astro_meu_sensor_publish_pose_message` |
| `template_module_period` (no `.ini`) | `meu_sensor_period` |
| `MODULE_NAME = TEMPLATE MODULE` | `MODULE_NAME = MEU SENSOR` |

## 8.3 O que adaptar em cada arquivo

O template marca com o comentário `TROQUE` os pontos que você precisa adaptar:

| Arquivo | O que adaptar |
|---|---|
| `<nome>_messages.h` | Os **campos** da struct e o `_FMT`, mantendo os dois **espelhados**. Escreva a **unidade** e o referencial de cada campo |
| `<nome>_main.cpp` | Em `publish_pose()`, **preencher os dados reais**. Em `read_parameters()`, **um item por parâmetro** |
| `<nome>_consumer_main.cpp` | No handler, **tratar os dados** recebidos. Lembre-se de que eles só valem dentro do handler |
| `<nome>.ini` | Os **parâmetros** do módulo, com unidade no comentário |

O restante (a conexão, a interface, o `Makefile`, o tratamento do Ctrl+C) normalmente **não precisa mudar**.

## 8.4 Passo a passo

Depois de criar o módulo com os passos da seção 8.2:

1. **Defina a mensagem.** Em `<nome>_messages.h`, troque os campos da struct e o `_FMT`, mantendo-os espelhados (capítulo 2). O template chama a mensagem de `pose`. Se a sua tem outro nome (por exemplo `leitura`), troque `pose` e `POSE` pelo novo nome nos símbolos `..._pose_message`, `..._POSE_MESSAGE_...`, `publish_pose` e `pose_message_handler`, nos três lugares onde aparecem: a mensagem, a interface e as mains.
2. **Publique dados reais.** Em `publish_pose()`, no `<nome>_main.cpp`, preencha a struct com o que o seu módulo produz (capítulos 2 e 4).
3. **Configure os parâmetros.** Acrescente cada parâmetro à lista de `read_parameters()` e ao `<nome>.ini` (capítulo 5). Leia-os **antes** de usá-los.
4. **Trate os dados no consumidor.** No handler, use os campos recebidos e **copie** o que precisar guardar (capítulo 3).
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

- [x] um **template** de módulo, compilável, com o dono da mensagem e um consumidor, em `src/template_module/` e `src/template_module_consumer/`, com a mesma mensagem do exemplo;
- [x] o **passo a passo** para criar um módulo novo a partir dele: copiar, renomear arquivos e trocar o nome dentro deles;
- [x] os **pontos de adaptação** marcados com `TROQUE`, e o caminho de volta aos capítulos para cada conceito do template.
