# 3. Assinando a mensagem: o segundo módulo

No capítulo anterior o `vehicle_simulator` passou a publicar a pose do veículo, mas ninguém a recebia. Agora vamos criar um **segundo módulo**, o `vehicle_simulator_monitor`, que **assina** essa mensagem e imprime o que chega.

Esse é o desenho típico do ASTRO: um módulo produz um dado, outro o consome, e nenhum conhece o outro. Só compartilham o nome e o formato da mensagem.

```mermaid
flowchart LR
    P["vehicle_simulator<br/>(publica)"] -- "astro_vehicle_simulator_pose" --> C(("central"))
    C -- "astro_vehicle_simulator_pose" --> M["vehicle_simulator_monitor<br/>(assina)"]
```

O trabalho deste capítulo tem três etapas:

1. **Preparar a interface para ser usada por outro módulo**: acrescentar a função de assinar e publicá-la.
2. **Criar o módulo monitor**, com um *handler*: a função que o IPC chama a cada mensagem recebida.
3. **Executar os dois** e ver os dados passando de um módulo para o outro.

## 3.1 Como um módulo oferece código a outro

O monitor ficará em uma **pasta diferente** da do `vehicle_simulator`. Isso cria um problema: o código da interface (o header e o `.cpp`) está na pasta do `vehicle_simulator`, e o monitor não enxerga arquivos de outra pasta.

O ASTRO resolve isso do mesmo jeito para todos os módulos: quem **oferece** código o **publica** em dois lugares comuns, e quem **usa** o pega de lá.

| O que se publica | Onde fica | Como o `Makefile` do dono publica | Como quem usa o enxerga |
|---|---|---|---|
| Os **headers** (declarações) | `$ASTRO_HOME/include/astro/` | `PUBLIC_INCLUDES` | `#include <astro/nome.h>` |
| O **código compilado**, em uma **biblioteca** (`.a`) | `$ASTRO_HOME/lib/` | `PUBLIC_LIBRARIES` | `-lnome` no `LFLAGS` |

### O header e a biblioteca são duas metades

Para um programa usar uma função que está em outro módulo, são necessárias **duas coisas**, e elas acontecem em **momentos diferentes** da construção do programa:

| Momento | O que o programa precisa | Quem fornece |
|---|---|---|
| **Compilar** (cada `.cpp` vira um `.o`) | Saber que a função **existe** e como ela é chamada: a **declaração** | O **header** |
| **Link** (os `.o` são juntados em um executável) | O **código** da função, já compilado | A **biblioteca** (`.a`) |

O header é como o **índice** de um livro: diz que o capítulo existe e em que página, mas não traz o conteúdo. A biblioteca é o **conteúdo**. O compilador se contenta com o índice (por isso o `#include` basta para compilar), mas o link precisa do conteúdo, e é aí que entra a biblioteca.

Uma biblioteca `.a` é só um pacote de arquivos `.o` já compilados. Escrever `-lnome` no `LFLAGS` diz ao link: "junte ao meu executável o código que está em `libnome.a`", e ele a procura na pasta `lib/`.

Por isso o dono da interface publica **as duas metades**: `PUBLIC_INCLUDES` publica o índice, e `PUBLIC_LIBRARIES` publica o conteúdo.

Você já usa esse mecanismo desde o capítulo 1, sem ter prestado atenção: `#include <astro/astro.h>` pega um header publicado em `include/astro/`, e `-lglobal` e `-lipc` pegam bibliotecas publicadas em `lib/`. O `Makefile` do módulo `global` publica o `astro.h` e a `libglobal.a` usando exatamente essas mesmas variáveis. Agora o `vehicle_simulator` vai fazer o mesmo com a sua interface.

## 3.2 Acrescentando a função de assinar à interface

Na interface, só tínhamos `define` e `publish`. Falta a função que **assina** a mensagem, usada por quem quer receber. Acrescente ao `vehicle_simulator_interface.h`, antes do `#ifdef __cplusplus` que fecha o `extern "C"`:

```cpp
void
astro_vehicle_simulator_subscribe_pose_message(astro_vehicle_simulator_pose_message *message,
		astro_handler_t handler, astro_subscribe_t subscribe_how);

void
astro_vehicle_simulator_unsubscribe_pose_message(astro_handler_t handler);
```

E a implementação, ao final do `vehicle_simulator_interface.cpp`:

```cpp
void
astro_vehicle_simulator_subscribe_pose_message(astro_vehicle_simulator_pose_message *message,
		astro_handler_t handler, astro_subscribe_t subscribe_how)
{
	astro_subscribe_message(ASTRO_VEHICLE_SIMULATOR_POSE_MESSAGE_NAME, ASTRO_VEHICLE_SIMULATOR_POSE_MESSAGE_FMT,
			message, sizeof(astro_vehicle_simulator_pose_message), handler, subscribe_how);
}

void
astro_vehicle_simulator_unsubscribe_pose_message(astro_handler_t handler)
{
	astro_unsubscribe_message(ASTRO_VEHICLE_SIMULATOR_POSE_MESSAGE_NAME, handler);
}
```

As duas funções apenas **embalam** as chamadas do IPC (`astro_subscribe_message` e `astro_unsubscribe_message`), preenchendo o nome, o formato e o tamanho da struct. Quem assina só informa o *handler* e a política de entrega. Os argumentos serão explicados na seção 3.4, quando a função for usada. A `unsubscribe` faz o contrário: remove o *handler* da assinatura.

## 3.3 Publicando a interface

Agora vamos fazer o `vehicle_simulator` publicar a interface nos dois lugares comuns.

### Os `include`

Dentro do próprio módulo, os `include` da interface usavam aspas porque os arquivos estavam na mesma pasta. Como os headers passarão a ser acessados pelo caminho comum, troque-os pela forma padrão do ASTRO:

| Arquivo | Antes | Depois |
|---|---|---|
| `vehicle_simulator_interface.h` | `#include "vehicle_simulator_messages.h"` | `#include <astro/vehicle_simulator_messages.h>` |
| `vehicle_simulator_interface.cpp` | `#include "vehicle_simulator_interface.h"` | `#include <astro/vehicle_simulator_interface.h>` |
| `vehicle_simulator_main.cpp` | `#include "vehicle_simulator_interface.h"` | `#include <astro/vehicle_simulator_interface.h>` |

### O `Makefile`

Atualize o `Makefile` do `vehicle_simulator`:

```makefile
include ../Makefile.conf

MODULE_NAME = VEHICLE SIMULATOR
MODULE_COMMENT = Modulo de exemplo: simula um veiculo e publica sua pose

LFLAGS += -lvehicle_simulator_interface -lglobal -lipc -lpthread

SOURCES = vehicle_simulator_main.cpp vehicle_simulator_interface.cpp

PUBLIC_INCLUDES = vehicle_simulator_messages.h vehicle_simulator_interface.h
PUBLIC_LIBRARIES = libvehicle_simulator_interface.a
PUBLIC_BINARIES = vehicle_simulator

TARGETS = libvehicle_simulator_interface.a vehicle_simulator

libvehicle_simulator_interface.a: vehicle_simulator_interface.o

vehicle_simulator: vehicle_simulator_main.o

include ../Makefile.rules
```

O que mudou, e por quê:

| Linha | Motivo |
|---|---|
| `PUBLIC_INCLUDES` | Copia os dois headers para `include/astro/`, para outros módulos poderem fazer `#include <astro/...>` |
| `PUBLIC_LIBRARIES` | Copia a biblioteca da interface (o código compilado das funções) para `lib/`, para outros módulos poderem juntá-lo ao seu executável no link |
| `TARGETS` com `libvehicle_simulator_interface.a` | Faz o `make` construir a biblioteca |
| `libvehicle_simulator_interface.a: ...interface.o` | Diz de que arquivo objeto a biblioteca é feita |
| `vehicle_simulator: vehicle_simulator_main.o` | O executável agora só precisa do objeto do `main`: o código da interface vem da biblioteca |
| `-lvehicle_simulator_interface` em `LFLAGS` | Pede ao link que junte ao executável o código dessa biblioteca, **como qualquer outro módulo fará**: o `vehicle_simulator` é o primeiro usuário da sua própria interface |

Compile:

```bash
cd $ASTRO_HOME/src/vehicle_simulator
make
```

```text
---- Copying vehicle_simulator_messages.h to /home/lume/astro/include/astro
---- Copying vehicle_simulator_interface.h to /home/lume/astro/include/astro
---- Compiling vehicle_simulator_interface.cpp to vehicle_simulator_interface.o (g++)
---- Archiving vehicle_simulator_interface.o into libvehicle_simulator_interface.a (gcc-ar-9 cr)
---- Copying libvehicle_simulator_interface.a to /home/lume/astro/lib
---- Compiling vehicle_simulator_main.cpp to vehicle_simulator_main.o (g++)
---- Linking vehicle_simulator_main.o to vehicle_simulator (g++)
[SUCCESS] Module VEHICLE SIMULATOR built with hash: ...
```

Os arquivos publicados estão agora nos lugares comuns:

```bash
ls $ASTRO_HOME/include/astro/vehicle_simulator_*
ls $ASTRO_HOME/lib/libvehicle_simulator_interface.a
```

```text
/home/lume/astro/include/astro/vehicle_simulator_interface.h
/home/lume/astro/include/astro/vehicle_simulator_messages.h
/home/lume/astro/lib/libvehicle_simulator_interface.a
```

(FOTO AQUI DE: o `make` do `vehicle_simulator` com as linhas `Copying`, `Archiving` e o `[SUCCESS]`, seguido do `ls` dos arquivos publicados.)

> [!WARNING]
> **Os headers são copiados, não ligados.** O que fica em `include/astro/` é uma **cópia** do arquivo da pasta do módulo. Se você alterar a struct ou o formato em `vehicle_simulator_messages.h`, rode `make` no `vehicle_simulator` **de novo** para atualizar a cópia, e recompile também os módulos que usam a mensagem. Caso contrário, quem assina e quem publica passam a ter structs diferentes e os dados chegam corrompidos, sem nenhum aviso.

## 3.4 Criando o módulo monitor

Crie a pasta do módulo ao lado do `vehicle_simulator`:

```bash
mkdir $ASTRO_HOME/src/vehicle_simulator_monitor
cd $ASTRO_HOME/src/vehicle_simulator_monitor
```

Crie o `vehicle_simulator_monitor_main.cpp`. A estrutura é a que você já conhece do capítulo 1 (conectar, tratar o Ctrl+C, entrar no `dispatch`). A novidade é o trecho que **assina** a mensagem e o *handler*:

```cpp
#include <astro/astro.h>
#include <astro/vehicle_simulator_interface.h>

static void
pose_message_handler(astro_vehicle_simulator_pose_message *message)
{
	printf("vehicle_simulator_monitor: x=%.2f y=%.2f theta=%.2f v=%.2f (de %s, t=%.3f)\n",
			message->x, message->y, message->theta, message->v, message->host, message->timestamp);
}

static void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		astro_ipc_disconnect();
		printf("vehicle_simulator_monitor: desconectado.\n");
		exit(0);
	}
}

int
main(int argc, char **argv)
{
	astro_ipc_initialize(argc, argv);
	signal(SIGINT, shutdown_module);

	astro_vehicle_simulator_subscribe_pose_message(NULL, (astro_handler_t) pose_message_handler, ASTRO_SUBSCRIBE_LATEST);

	printf("vehicle_simulator_monitor: aguardando mensagens.\n");

	astro_ipc_dispatch();

	return 0;
}
```

Repare que o monitor **não escreve o nome nem o formato da mensagem em lugar nenhum**: tudo está dentro da função da interface.

### O *handler*

```cpp
static void
pose_message_handler(astro_vehicle_simulator_pose_message *message)
{
	printf(...);
}
```

O **handler** é uma função sua, que **você não chama**. Quem a chama é o IPC, **uma vez para cada mensagem que chega**. O argumento `message` é um ponteiro para a struct já preenchida com os dados recebidos. Aqui o handler só imprime os campos.

### A assinatura

```cpp
astro_vehicle_simulator_subscribe_pose_message(NULL, (astro_handler_t) pose_message_handler, ASTRO_SUBSCRIBE_LATEST);
```

| Argumento | Significado |
|---|---|
| `NULL` | Onde guardar a mensagem recebida. Com `NULL`, **o próprio IPC aloca a memória**. É o jeito mais simples e o que usamos |
| `(astro_handler_t) pose_message_handler` | O handler a ser chamado a cada mensagem. O *cast* para `astro_handler_t` é necessário porque o IPC espera uma função que recebe um `void *` |
| `ASTRO_SUBSCRIBE_LATEST` | A política de entrega, explicada a seguir |

### `ASTRO_SUBSCRIBE_LATEST` ou `ASTRO_SUBSCRIBE_ALL`

Imagine que mensagens chegam mais rápido do que o handler consegue processar. O IPC precisa decidir o que fazer com as que ficam esperando:

| Política | Comportamento |
|---|---|
| `ASTRO_SUBSCRIBE_LATEST` | Mantém **só a mais recente** na fila. Se o handler está ocupado, as mensagens antigas são descartadas |
| `ASTRO_SUBSCRIBE_ALL` | Mantém uma **fila longa** (até 1000 mensagens), sem descartar nenhuma |

Para um dado de estado, como uma pose, o que importa é o **valor mais recente**, então usamos `LATEST`. `ALL` é para quando perder uma mensagem causa um problema (um comando que não pode ser perdido, por exemplo).

### Tempo de vida dos dados no handler

> [!WARNING]
> **O que o handler recebe só vale durante a chamada do handler.** O IPC reaproveita a mesma memória a cada nova mensagem: quando a próxima chega, a anterior é liberada. Os campos numéricos você pode ler à vontade dentro do handler, mas o ponteiro `message->host` (uma `string`) **deixa de ser válido** depois dele. Se precisar guardar um valor para usar depois, **copie-o** (um `double` com `x = message->x;`, uma string com `strdup` ou `strcpy`). Guardar o ponteiro da string leva a erros difíceis de achar.

## 3.5 O `Makefile` do monitor

Crie o `Makefile` na pasta do monitor:

```makefile
include ../Makefile.conf

MODULE_NAME = VEHICLE SIMULATOR MONITOR
MODULE_COMMENT = Modulo de exemplo: assina a pose publicada pelo vehicle_simulator

LFLAGS += -lvehicle_simulator_interface -lglobal -lipc -lpthread

SOURCES = vehicle_simulator_monitor_main.cpp

PUBLIC_BINARIES = vehicle_simulator_monitor
TARGETS = vehicle_simulator_monitor

vehicle_simulator_monitor: vehicle_simulator_monitor_main.o

include ../Makefile.rules
```

É o `Makefile` do capítulo 1 com **uma biblioteca a mais** no `LFLAGS`: a `-lvehicle_simulator_interface`, que traz o **código** das funções da interface (o link precisa dele, como vimos na 3.1). O header, para compilar, vem de `include/astro/`, publicado na seção 3.3. O monitor **não tem** `PUBLIC_INCLUDES` nem `PUBLIC_LIBRARIES`, porque ele não oferece nada a ninguém: só consome.

Compile:

```bash
cd $ASTRO_HOME/src/vehicle_simulator_monitor
make
```

### Se esquecer a seção 3.3

Se você compilar o monitor sem ter publicado a interface, o erro é este:

```text
vehicle_simulator_monitor_main.cpp:2:10: fatal error: astro/vehicle_simulator_interface.h: No such file or directory
```

A solução é voltar à seção 3.3: rodar o `make` no `vehicle_simulator` com o `PUBLIC_INCLUDES` configurado.

### Se esquecer o `-lvehicle_simulator_interface`

Se o header estiver publicado, mas o `-lvehicle_simulator_interface` faltar no `LFLAGS`, o monitor **compila**, e o erro aparece só no **link**:

```text
---- Compiling vehicle_simulator_monitor_main.cpp to vehicle_simulator_monitor_main.o (g++)
---- Linking vehicle_simulator_monitor_main.o to vehicle_simulator_monitor (g++)
vehicle_simulator_monitor_main.cpp:(.text.startup.main+0x1c): undefined reference to `astro_vehicle_simulator_subscribe_pose_message'
collect2: error: ld returned 1 exit status
```

É exatamente a diferença entre as duas metades: o **header** deixou o compilador saber que a função existe, mas faltou o **código** dela no link. O `undefined reference` quer dizer isso: "sei que existe, mas não encontrei o código". A solução é acrescentar o `-lvehicle_simulator_interface` ao `LFLAGS`.

## 3.6 Executando os dois módulos

Você vai precisar de **três terminais**:

| Terminal | Comando |
|---|---|
| 1 | `cd $ASTRO_HOME/bin && ./central` |
| 2 | `cd $ASTRO_HOME/bin && ./vehicle_simulator_monitor` |
| 3 | `cd $ASTRO_HOME/bin && ./vehicle_simulator` |

Inicie nesta ordem. No terminal 2, o monitor fica esperando:

```text
ASTRO - LUME ROBOTICS SYSTEM Version 0.8.0

vehicle_simulator_monitor: aguardando mensagens.
```

Ao iniciar o `vehicle_simulator` no terminal 3, ele começa a publicar:

```text
ASTRO - LUME ROBOTICS SYSTEM Version 0.8.0

vehicle_simulator: publicou x=0.00 y=0.00 theta=0.00 v=2.00
vehicle_simulator: publicou x=0.00 y=0.00 theta=0.00 v=2.00
vehicle_simulator: publicou x=0.00 y=0.00 theta=0.00 v=2.00
```

E no terminal 2, o monitor mostra o que recebeu, um por segundo:

```text
vehicle_simulator_monitor: x=0.00 y=0.00 theta=0.00 v=2.00 (de vehicle_simulator@lume-M, t=1795546640.379)
vehicle_simulator_monitor: x=0.00 y=0.00 theta=0.00 v=2.00 (de vehicle_simulator@lume-M, t=1795546641.380)
vehicle_simulator_monitor: x=0.00 y=0.00 theta=0.00 v=2.00 (de vehicle_simulator@lume-M, t=1795546642.380)
```

(FOTO AQUI DE: os três terminais lado a lado: `central` em um, `vehicle_simulator_monitor` recebendo as linhas no outro e `vehicle_simulator` publicando no terceiro.)

Pela primeira vez a mensagem foi de um módulo a outro. Repare no que aparece na linha do monitor:

* `x`, `y`, `theta` e `v` são os valores publicados, que chegaram sem alteração.
* `host` mostra o **módulo e a máquina de origem** (`vehicle_simulator@lume-M`), preenchido dentro da função de publicação, lá no publicador.
* `t` é o `timestamp`, também preenchido pelo publicador, em segundos.

### Experimentos

Vale testar o comportamento do desacoplamento:

1. **Pare o `vehicle_simulator`** (Ctrl+C). O monitor não reclama: continua em `aguardando mensagens`, porque assinar não depende de o publicador estar vivo.
2. **Inicie-o de novo.** As mensagens voltam a chegar no monitor, sem precisar reiniciá-lo.
3. **Abra um segundo monitor** em outro terminal. Os dois recebem **as mesmas mensagens**: a `central` entrega a cada assinante.
4. **Inverta a ordem**: inicie o publicador antes do monitor. Funciona também: a mensagem só passa a chegar depois que o monitor se inscreve.

> [!NOTE]
> **Por que o monitor pode iniciar antes do publicador.** A função `astro_subscribe_message` também **define a mensagem** na `central`, "por precaução", como diz o comentário do próprio código. Então, mesmo que o publicador ainda não tenha rodado, a `central` já conhece a mensagem quando o monitor se inscreve. Isso **não** dispensa a definição no publicador: o módulo que publica deve sempre definir o que publica.

## 3.7 Checkpoint

Neste ponto você tem:

- [x] a ideia de que um módulo **oferece** código a outros publicando **headers** (`PUBLIC_INCLUDES`) e **bibliotecas** (`PUBLIC_LIBRARIES`), o mesmo mecanismo do `-lglobal` e do `-lipc`;
- [x] a interface do `vehicle_simulator` com `define`, `publish`, `subscribe` e `unsubscribe`, publicada em `include/astro/` e `lib/`;
- [x] um segundo módulo, o `vehicle_simulator_monitor`, que assina a mensagem **sem conhecer seu nome nem seu formato**;
- [x] um **handler** que o IPC chama a cada mensagem recebida;
- [x] a diferença entre `ASTRO_SUBSCRIBE_LATEST` e `ASTRO_SUBSCRIBE_ALL`;
- [x] o cuidado de que os dados do handler só valem durante a chamada;
- [x] os dois módulos conversando por meio da `central`.

**Próximo capítulo:** o veículo ainda está parado na origem. Vamos fazê-lo se mover: a pose passará a ser calculada a cada ciclo a partir da velocidade, e o monitor vai ganhar um alerta de velocidade com um parâmetro lido do `.ini`.
