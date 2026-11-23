# 1. Estrutura e Organização de um Módulo

Um módulo vive em uma pasta própria dentro de [`src/`](file:///home/lume/astro/src). O módulo de exemplo é [`src/hello_module/`](file:///home/lume/astro/src/hello_module):

```text
src/hello_module/
├── Makefile                        # como compilar e o que publicar em bin/, lib/ e include/
├── hello_module_messages.h         # definição da mensagem (struct, nome, formato)
├── hello_module_interface.h        # API de IPC da mensagem (define/subscribe/publish)
├── hello_module_interface.cpp      # implementação dessa API  → vira libhello_module_interface.a
├── hello_module_main.cpp           # executável publicador
└── hello_module_listener_main.cpp  # executável assinante
```

> [!IMPORTANT]
> **Resumo:**
> * `_main.cpp`: tem a função `main`, a inicialização, os **handlers** das mensagens recebidas e os **publicadores**. Um por executável.
> * `_messages.h`: só define a mensagem (struct, `_NAME`, `_FMT`).
> * `_interface.cpp/.h`: funções de definir, assinar e publicar a mensagem, para que **qualquer módulo** use a mensagem sem repetir código. Compilado como biblioteca.
> * `Makefile`: dizer ao sistema de build o que compilar e o que copiar para as pastas públicas.

---

## 1. Responsabilidade de cada arquivo

| Arquivo | Responsabilidade | Quem o usa |
|---|---|---|
| `<modulo>_main.cpp` | `main()`, inicialização do IPC, leitura de parâmetros, handlers, publicadores, `shutdown` | Só o próprio executável |
| `<modulo>_messages.h` | Struct da mensagem e as macros `..._NAME` e `..._FMT` | Qualquer módulo que use a mensagem |
| `<modulo>_interface.h/.cpp` | `define_messages`, `subscribe_message`, `unsubscribe_message`, `publish_message` | Qualquer módulo que use a mensagem |
| `<modulo>.cpp` (opcional) | Lógica própria do módulo, quando o `_main.cpp` ficar grande | O próprio módulo |
| `Makefile` | Fontes, bibliotecas, alvos e arquivos públicos | Sistema de build |

> [!NOTE]
> Regra do [README do `simple_module_example`](file:///home/lume/astro/src/simple_module_example/README): se o módulo só consome mensagens de outros, **não precisa** de `_messages.h` nem de `_interface`. Bibliotecas que não geram executável são **sharedlibs**, não módulos, e ficam em `sharedlib/`.

---

## 2. Convenções de nome

| Item | Padrão | Exemplo |
|---|---|---|
| Pasta | `snake_case`, nome do módulo | `hello_module` |
| Struct da mensagem | `astro_<modulo>_message` | `astro_hello_module_message` |
| Nome da mensagem (IPC) | string igual ao nome da struct | `"astro_hello_module_message"` |
| Macros | `ASTRO_<MODULO>_MESSAGE_NAME` e `..._FMT` | `ASTRO_HELLO_MODULE_MESSAGE_FMT` |
| Funções de interface | `astro_<modulo>_<ação>_message` | `astro_hello_module_publish_message` |
| Parâmetro do `.ini` | `<modulo>_<parametro>` | `hello_module_period_ms` |

Todo `_messages.h` deve ter `extern "C"` (como em [`hello_module_messages.h`](file:///home/lume/astro/src/hello_module/hello_module_messages.h)), pois a biblioteca IPC é em C. Toda mensagem **deve** ter os campos `double timestamp` e `char *host`.

---

## 3. O `Makefile`

Arquivo real do [`hello_module`](file:///home/lume/astro/src/hello_module/Makefile):

```makefile
include ../Makefile.conf

MODULE_NAME = HELLO MODULE
MODULE_COMMENT = Modulo de exemplo/template: publica e assina uma mensagem IPC

LFLAGS += -lhello_module_interface -lparam_interface -lipc -lglobal -lpthread

SOURCES = hello_module_main.cpp hello_module_listener_main.cpp hello_module_interface.cpp

PUBLIC_INCLUDES = hello_module_messages.h hello_module_interface.h
PUBLIC_LIBRARIES = libhello_module_interface.a
PUBLIC_BINARIES = hello_module hello_module_listener

TARGETS = libhello_module_interface.a hello_module hello_module_listener

libhello_module_interface.a: hello_module_interface.o

hello_module: hello_module_main.o

hello_module_listener: hello_module_listener_main.o

include ../Makefile.rules
```

| Variável | Significado |
|---|---|
| `MODULE_NAME` / `MODULE_COMMENT` | Identificação exibida no build |
| `LFLAGS` | Bibliotecas de link. O mínimo para IPC é `-lparam_interface -lipc -lglobal -lpthread` |
| `SOURCES` | Todos os `.c`/`.cpp` do módulo |
| `PUBLIC_INCLUDES` | Headers copiados para `include/astro/`; por isso os módulos usam `#include <astro/hello_module_messages.h>` |
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
cd $ASTRO_HOME/src/hello_module
make
```

Saída real (resumida):

```text
---- Compiling hello_module_interface.cpp to hello_module_interface.o (g++)
---- Archiving hello_module_interface.o into libhello_module_interface.a
---- Copying libhello_module_interface.a to /home/lume/astro/lib
---- Linking hello_module_main.o to hello_module (g++)
---- Copying hello_module to /home/lume/astro/bin
[SUCCESS] Module HELLO MODULE built with hash: ...
```

> [!NOTE]
> Os `.o`, `.a`, `include/*` e os binários em `bin/` não são versionados, mas os binários recém-copiados em `bin/` aparecem como não rastreados no `git status` (adicione-os ao `.gitignore` do módulo se for o caso).

**Próximo capítulo:** configurar a conexão com a `central` e a leitura de parâmetros (`02_configuracao_ipc.md`).
