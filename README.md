# 1. Estrutura e Organização de um Módulo

Um módulo vive em uma pasta própria dentro de [`src/`](file:///home/lume/astro/src). O módulo que **publica** é [`src/vehicle_simulator/`](file:///home/lume/astro/src/vehicle_simulator), e o que **assina** é [`src/vehicle_simulator_monitor/`](file:///home/lume/astro/src/vehicle_simulator_monitor):

```text
src/vehicle_simulator/
├── Makefile                        # como compilar e o que publicar em bin/, lib/ e include/
├── vehicle_simulator_messages.h          # definição da mensagem (struct, nome, formato)
├── vehicle_simulator_interface.h         # API de IPC da mensagem (define/subscribe/publish)
├── vehicle_simulator_interface.cpp       # implementação dessa API  → vira libvehicle_simulator_interface.a
└── vehicle_simulator_main.cpp            # executável publicador

src/vehicle_simulator_monitor/
├── Makefile                        # faz link com -lvehicle_simulator_interface
└── vehicle_simulator_monitor_main.cpp    # executável assinante (handler da mensagem)
```

O `vehicle_simulator_monitor` **não tem** `_messages.h` nem `_interface`: ele usa os do `vehicle_simulator`, que já foram copiados para `include/astro/` e `lib/` no build do publicador.

> [!IMPORTANT]
> **Resumo:**
> * `_main.cpp`: tem a função `main`, a inicialização, os **handlers** das mensagens recebidas e os **publicadores**. Um por executável.
> * `_messages.h`: só define a mensagem (struct, `_NAME`, `_FMT`).
> * `_interface.cpp/.h`: funções de definir, assinar e publicar a mensagem, para que **qualquer módulo** use a mensagem sem repetir código. Compilado como biblioteca.
> * `Makefile`: diz ao sistema de build o que compilar e o que copiar para as pastas públicas.

---

## 1. Responsabilidade de cada arquivo

| Arquivo | Responsabilidade | Quem o usa |
|---|---|---|
| `<modulo>_main.cpp` | `main()`, inicialização do IPC, leitura de parâmetros, handlers, publicadores, `shutdown` | Só o próprio executável |
| `<modulo>_messages.h` | Struct da mensagem e as macros `..._NAME` e `..._FMT` | Qualquer módulo que use a mensagem |
| `<modulo>_interface.h/.cpp` | `define_messages`, `subscribe_..._message`, `unsubscribe_..._message`, `publish_..._message` | Qualquer módulo que use a mensagem |
| `<modulo>.cpp` (opcional) | Lógica própria do módulo, quando o `_main.cpp` ficar grande | O próprio módulo |
| `Makefile` | Fontes, bibliotecas, alvos e arquivos públicos | Sistema de build |

> [!NOTE]
> Regra do [README do `simple_module_example`](file:///home/lume/astro/src/simple_module_example/README): se o módulo só consome mensagens de outros, **não precisa** de `_messages.h` nem de `_interface`. Bibliotecas que não geram executável são **sharedlibs**, não módulos, e ficam em `sharedlib/`.

---

## 2. Convenções de nome

| Item | Padrão | Exemplo |
|---|---|---|
| Pasta | `snake_case`, nome do módulo | `vehicle_simulator` |
| Struct da mensagem | `astro_<modulo>_<mensagem>_message` | `astro_vehicle_simulator_pose_message` |
| Nome da mensagem (IPC) | `astro_<modulo>_<mensagem>` | `"astro_vehicle_simulator_pose"` |
| Macros | `ASTRO_<MODULO>_<MENSAGEM>_MESSAGE_NAME` e `..._FMT` | `ASTRO_VEHICLE_SIMULATOR_POSE_MESSAGE_FMT` |
| Funções de interface | `astro_<modulo>_<ação>_<mensagem>_message` | `astro_vehicle_simulator_publish_pose_message` |
| Parâmetro do `.ini` | `<modulo>_<parametro>` | `vehicle_simulator_period_ms` |

Todo `_messages.h` deve ter `extern "C"` (como em [`vehicle_simulator_messages.h`](file:///home/lume/astro/src/vehicle_simulator/vehicle_simulator_messages.h)), pois a biblioteca IPC é em C. Toda mensagem **deve** ter os campos `double timestamp` e `char *host`. Escreva **unidade e referencial** de cada campo nos comentários da struct.

---

## 3. O `Makefile`

Arquivo real do [`vehicle_simulator`](file:///home/lume/astro/src/vehicle_simulator/Makefile) (o dono da mensagem):

```makefile
include ../Makefile.conf

MODULE_NAME = VEHICLE SIMULATOR
MODULE_COMMENT = Modulo de exemplo/template: simula um veiculo e publica a mensagem astro_vehicle_simulator_pose

LFLAGS += -lvehicle_simulator_interface -lparam_interface -lipc -lglobal -lpthread -lm

SOURCES = vehicle_simulator_main.cpp vehicle_simulator_interface.cpp

PUBLIC_INCLUDES = vehicle_simulator_messages.h vehicle_simulator_interface.h
PUBLIC_LIBRARIES = libvehicle_simulator_interface.a
PUBLIC_BINARIES = vehicle_simulator

TARGETS = libvehicle_simulator_interface.a vehicle_simulator

libvehicle_simulator_interface.a: vehicle_simulator_interface.o

vehicle_simulator: vehicle_simulator_main.o

include ../Makefile.rules
```

E o do [`vehicle_simulator_monitor`](file:///home/lume/astro/src/vehicle_simulator_monitor/Makefile) (apenas consumidor, sem `PUBLIC_INCLUDES` nem `PUBLIC_LIBRARIES`):

```makefile
include ../Makefile.conf

MODULE_NAME = VEHICLE SIMULATOR MONITOR
MODULE_COMMENT = Modulo de exemplo: assina a mensagem publicada pelo vehicle_simulator e alerta por velocidade

LFLAGS += -lvehicle_simulator_interface -lparam_interface -lipc -lglobal -lpthread

SOURCES = vehicle_simulator_monitor_main.cpp

PUBLIC_BINARIES = vehicle_simulator_monitor
TARGETS = vehicle_simulator_monitor

vehicle_simulator_monitor: vehicle_simulator_monitor_main.o

include ../Makefile.rules
```

| Variável | Significado |
|---|---|
| `MODULE_NAME` / `MODULE_COMMENT` | Identificação exibida no build |
| `LFLAGS` | Bibliotecas de link. O mínimo para IPC é `-lparam_interface -lipc -lglobal -lpthread`; um consumidor acrescenta a interface do módulo dono da mensagem (`-lvehicle_simulator_interface`); `-lm` só é necessário se usar funções matemáticas, como o `sin` e o `cos` do simulador |
| `SOURCES` | Todos os `.c`/`.cpp` do módulo |
| `PUBLIC_INCLUDES` | Headers copiados para `include/astro/`; por isso os módulos usam `#include <astro/vehicle_simulator_messages.h>` |
| `PUBLIC_LIBRARIES` | Bibliotecas copiadas para `lib/` |
| `PUBLIC_BINARIES` | Executáveis copiados para `bin/` |
| `TARGETS` | O que o `make` constrói |
| `alvo: arquivo.o` | Liga cada alvo ao seu objeto |

> [!WARNING]
> **`-lpthread` é obrigatório.** Sem ele o link falha com `undefined reference to pthread_mutexattr_init` vindo da `libipc.a`. Isto aconteceu na construção do exemplo.

---

## 4. Compilando

```bash
export ASTRO_HOME=/home/lume/astro
cd $ASTRO_HOME/src/vehicle_simulator && make           # primeiro o dono da mensagem
cd $ASTRO_HOME/src/vehicle_simulator_monitor && make   # depois o consumidor
```

> [!NOTE]
> A ordem importa quando se compila **um módulo isolado**: o consumidor precisa que `libvehicle_simulator_interface.a` e `vehicle_simulator_messages.h` já estejam em `lib/` e `include/astro/`. No build completo da `src/` as fases 1 (bibliotecas) e 2 (executáveis) resolvem isso sozinhas.

Saída real (resumida):

```text
---- Compiling vehicle_simulator_interface.cpp to vehicle_simulator_interface.o (g++)
---- Archiving vehicle_simulator_interface.o into libvehicle_simulator_interface.a
---- Copying libvehicle_simulator_interface.a to /home/lume/astro/lib
---- Linking vehicle_simulator_main.o to vehicle_simulator (g++)
---- Copying vehicle_simulator to /home/lume/astro/bin
[SUCCESS] Module VEHICLE SIMULATOR built with hash: ...
```

> [!NOTE]
> Os `.o`, `.a` e `include/*` não são versionados, mas os binários recém-copiados em `bin/` aparecem como não rastreados no `git status`.

**Próximo capítulo:** configurar a conexão com a `central` e a leitura de parâmetros (`02_configuracao_ipc.md`).
