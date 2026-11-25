# 7. Boas práticas e erros comuns

Este capítulo não constrói nada novo. Ele reúne, em um só lugar, os cuidados que apareceram ao longo do manual, para você consultar quando for criar ou depurar um módulo. Cada item aponta o capítulo em que foi explicado.

O capítulo termina com uma **tabela de erros comuns**: a mensagem que você vê, a causa provável e a solução.

## 7.1 Organização do módulo

| Arquivo | Responsabilidade |
|---|---|
| `<modulo>_messages.h` | Só **descreve** a mensagem: struct, nome e formato |
| `<modulo>_interface.h/.cpp` | As funções que **chamam o IPC** para essa mensagem: `define`, `publish`, `subscribe` |
| `<modulo>_main.cpp` | A **lógica** do módulo: inicialização, handlers, publicação e `main` |
| `Makefile` | Fontes, bibliotecas, alvos e o que publicar para outros módulos |

Quem usa a mensagem chama as funções da interface: **nunca escreve o nome nem o formato dela** em outro lugar (capítulo 2).

### A ordem do `main`

A ordem dos passos no `main` importa. Esta é a que usamos nos capítulos:

```cpp
	astro_ipc_initialize(argc, argv);                  // 1. conectar à central
	astro_param_check_version(argv[0]);                // 2. conferir o param_daemon (se o módulo lê parâmetros)
	signal(SIGINT, shutdown_module);                   // 3. tratar o Ctrl+C
	read_parameters(argc, argv);                       // 4. ler os parâmetros
	astro_vehicle_simulator_define_pose_message();     // 5. definir as mensagens que o módulo publica
	astro_ipc_addPeriodicTimer(dt, ..., NULL);         // 6. assinar mensagens e registrar timers
	astro_ipc_dispatch();                              // 7. entrar no laço de eventos
```

| Passo | Por que nessa posição |
|---|---|
| Conectar primeiro | Sem a conexão, nenhuma chamada de IPC funciona (cap. 1) |
| Parâmetros **antes** dos timers | O timer usa valores lidos do `.ini`, como o `dt` (cap. 5) |
| Definir antes de publicar | A `central` só entrega mensagens que conhece (cap. 2) |
| `astro_ipc_dispatch()` por último | Ela não retorna: é onde o módulo "vive" e responde a mensagens e timers (cap. 1) |

## 7.2 Mensagens

| Faça | Por quê | Cap. |
|---|---|---|
| Mantenha a **struct e o formato** (`_FMT`) espelhados, mesma ordem | O compilador não vê o formato. Se divergirem, os dados chegam trocados, **sem nenhum erro** | 2 |
| Termine toda struct com `double timestamp` e `char *host` | São obrigatórios em toda mensagem. A função de `publish` os preenche | 2 |
| Escreva **unidade e referencial** de cada campo nos comentários | Dois módulos podem entender `v` de formas diferentes (m/s ou km/h) sem o compilador saber | 2 |
| Use `extern "C"` no `_messages.h` e no `_interface.h` | A biblioteca de IPC é em C | 2 |
| **Defina** a mensagem no módulo que a publica | A `central` lembra mensagens já definidas: sem a definição, o erro pode ficar escondido até uma `central` nova | 2 |
| **Verifique** o resultado de toda chamada de IPC (`astro_test_ipc_exit`) | O erro aparece com o nome da mensagem, e o módulo não segue em estado inconsistente | 2 |
| Nomeie as funções da interface `astro_<modulo>_<ação>_<mensagem>_message` | Padrão que facilita encontrar e reconhecer as funções | 2 |

> [!WARNING]
> **O header publicado é uma cópia.** Depois de mudar a struct ou o formato, rode `make` no módulo **dono** da mensagem para atualizar a cópia em `include/astro/`, e **recompile todos os módulos** que a usam. Se o publicador e o assinante tiverem structs diferentes, nada avisa (cap. 3).

## 7.3 Assinando mensagens e escrevendo handlers

| Faça | Por quê | Cap. |
|---|---|---|
| Use `ASTRO_SUBSCRIBE_LATEST` para dados de **estado** (pose, velocidade) | Só o valor mais recente importa, e as mensagens antigas são descartadas se o handler estiver ocupado | 3 |
| Use `ASTRO_SUBSCRIBE_ALL` quando **perder uma mensagem** causa problema | Mantém uma fila longa (até 1000 mensagens) | 3 |
| Leia os dados **dentro do handler**; **copie** o que precisar guardar | O IPC reaproveita a memória a cada mensagem: o ponteiro `host` e outras strings deixam de valer depois do handler | 3 |
| Mantenha o handler **rápido** | Ele roda no mesmo laço do `dispatch`: um handler lento atrasa as mensagens e os timers | 3 |

## 7.4 `Makefile` e build

| Faça | Por quê | Cap. |
|---|---|---|
| Inclua no `LFLAGS`: `-lglobal -lipc -lpthread`, **nessa ordem** (`-lglobal` antes de `-lipc`) | Fora de ordem ou sem o `-lpthread`, o link falha | 1 |
| Acrescente `-lparam_interface` se o módulo lê parâmetros | O header vem com `astro.h`, mas o código está na biblioteca | 5 |
| Para **quem consome** uma interface, acrescente `-l<modulo>_interface` | O header basta para compilar, mas o link precisa do código | 3 |
| Para **quem oferece** uma interface, coloque a biblioteca no `TARGETS` e os arquivos em `PUBLIC_INCLUDES` e `PUBLIC_LIBRARIES` | O `TARGETS` constrói, e o `PUBLIC_*` só **copia** o que já foi construído | 3 |
| Mantenha no `SOURCES` todo `.cpp` do módulo, inclusive o da interface | É por estar lá que ele é compilado e entra na biblioteca | 3 |

## 7.5 Parâmetros

| Faça | Por quê | Cap. |
|---|---|---|
| Nomeie `<modulo>_<parametro>` | Padrão do `.ini`, e é como o nome é montado a partir do módulo e da variável | 5 |
| Faça o **tipo** do parâmetro combinar com o tipo da variável em C | `ASTRO_PARAM_DOUBLE` com `double`, `ASTRO_PARAM_INT` com `int`... | 5 |
| Acrescente todo parâmetro novo ao `.ini` | O módulo termina se o `param_daemon` não o conhecer | 5 |
| **Reinicie o `param_daemon` e o módulo** ao editar o `.ini` | O `param_daemon` lê o arquivo uma vez, ao iniciar | 5 |
| Escreva a **unidade** do parâmetro (no comentário do `.ini`) | Mesmo motivo das mensagens: o valor sozinho é ambíguo | 5 |

## 7.6 Executando e testando

| Faça | Por quê | Cap. |
|---|---|---|
| Exporte `ASTRO_HOME` antes de compilar | Sem ele o `make` aborta | 1 |
| Inicie a **`central` primeiro**, depois o `param_daemon` | Todo módulo precisa da `central`, e os que leem parâmetros precisam do `param_daemon` | 1, 5 |
| **Reinicie a `central`** ao testar mensagens novas | Ela lembra definições de execuções anteriores e pode esconder uma definição esquecida | 2 |
| Trate o **Ctrl+C** (`shutdown_module`) | Dá ao módulo um ponto de saída controlado para desconectar e fazer limpeza | 1 |
| Nos arquivos de processos, deixe `watch_heartbeats` em `0` | É o valor usado nos arquivos do projeto | 6 |

## 7.7 Erros comuns

Mensagens reais que apareceram no manual, o que significam e como resolver:

| O que você vê | Causa provável | Solução | Cap. |
|---|---|---|---|
| `ASTRO_HOME environment variable not set` (no `make`) | A variável não foi exportada | `export ASTRO_HOME=/caminho/do/astro` | 1 |
| `Could not connect to central.` | A `central` não está no ar | Iniciar `./central` antes do módulo | 1 |
| `unable to bind TCP/IP socket for listening` (ao iniciar a `central`) | A porta ainda está presa por uma execução anterior | Esperar alguns segundos e tentar de novo | 1 |
| `undefined reference to IPC_perror` (no link) | `-lipc` antes de `-lglobal` | Colocar `-lglobal` antes de `-lipc` | 1 |
| `undefined reference to pthread_mutexattr_init` (no link) | Falta `-lpthread` | Acrescentar `-lpthread` ao `LFLAGS` | 1 |
| `IPC_ERROR : Could not publish ... IPC message not defined` | A mensagem não foi definida (e a `central` não a conhecia) | Chamar a função `define` no `main`, antes de publicar | 2 |
| `fatal error: astro/<modulo>_interface.h: No such file or directory` | O header da interface não foi publicado | Rodar `make` no módulo dono, com `PUBLIC_INCLUDES` | 3 |
| `undefined reference to astro_<modulo>_..._message` (no link) | Falta o `-l<modulo>_interface` no `LFLAGS` de quem consome | Acrescentá-lo | 3 |
| `undefined reference to astro_param_install_params` (ou `astro_param_check_version`) | Falta o `-lparam_interface` | Acrescentá-lo ao `LFLAGS` | 5 |
| `The parameter server contains no definition for <nome>` | O parâmetro não está no `.ini` | Acrescentá-lo e reiniciar o `param_daemon` | 5 |
| `Message timed out : message astro_param_query_version` seguido de `Did you remember to start the parameter server?` | O `param_daemon` não está no ar | Iniciar o `param_daemon` | 5 |
| **Nenhuma mensagem de erro**, mas os dados chegam trocados ou sem sentido | Struct e formato dessincronizados, ou header publicado desatualizado | Conferir a struct e o `_FMT`; rodar `make` no módulo dono e recompilar quem usa a mensagem | 2, 3 |
| Um texto (`host`, por exemplo) vira lixo depois do handler | O ponteiro da string foi guardado e o IPC reaproveitou a memória | Copiar a string dentro do handler | 3 |
| Mudei o `.ini` e nada mudou | O `param_daemon` e o módulo não foram reiniciados | Reiniciar os dois | 5 |

## 7.8 Checklist para um módulo novo

Antes de considerar o módulo pronto, confira:

- [ ] A struct e o `_FMT` da mensagem são **espelhados**, e há `timestamp` e `host` no fim.
- [ ] Os campos da struct têm **unidade e referencial** nos comentários.
- [ ] O módulo dono da mensagem tem **`define`** e **`publish`**, e os chama na ordem certa.
- [ ] O `Makefile` tem o `LFLAGS` completo, o `TARGETS`, e `PUBLIC_INCLUDES` e `PUBLIC_LIBRARIES` para o que outros módulos usam.
- [ ] Quem consome a mensagem usa **só as funções da interface**.
- [ ] Os **handlers** não guardam ponteiros de strings recebidas.
- [ ] Os parâmetros estão **no `.ini`**, são lidos **antes** de serem usados, e o `.ini` foi lido por um `param_daemon` reiniciado.
- [ ] O **Ctrl+C** é tratado com um `shutdown_module`.
- [ ] O módulo foi testado com uma **`central` recém-iniciada**.
- [ ] O módulo está no **arquivo de processos** e nas **listas de pacotes** do `src/Makefile`.

**Próximo capítulo:** o **template**. Reuniremos tudo o que construímos em um esqueleto de módulo pronto para copiar, com os arquivos, o `Makefile` e o passo a passo para criar um módulo novo a partir dele.
