# Sistema Lume de Mobilidade Autônoma

O Sistema Lume de Mobilidade Autônoma possui módulos para auxiliar no controle dos veículos e sua comunicação. Ele é um framework que funciona com protocolo de troca de mensagens IPC e inclui os seguintes principais modos : 

- [*Simulator Mode (Navigate)*](../simulator_mode/simulator_mode.md)
- [*Playback*](../playback/playback.md)
- [*Record Log*](../record_log/record_log.md)
- [*Real World Use*](../real_world_use/real_world_use.md)

*******
**Tables of contents**
- [Sistema Lume de Mobilidade Autônoma](#sistema-lume-de-mobilidade-autônoma)
  - [Sistema Astro](#sistema-astro)
  - [O Arquivo Process](#o-arquivo-process)
    - [Variáveis](#variáveis)
    - [Módulos do Process](#módulos-do-process)
  - [Rodando um Process](#rodando-um-process)
    - [Corrigindo um Erro](#corrigindo-um-erro)
  - [Próximo Passo](#próximo-passo)

*******

## Sistema Astro

**Astro** é um framework e possui vários módulos que ficam na pasta *src*, e quando compilados, seus binários vão para a pasta *bin*.

<p align="center">
  <img height="250" src="./imgs/astro_dir.jpg"/>
</p>

A estrutura de organização do diretório *bin* é que cada veículo terá sua pasta contendo seu arquivo *.ini* com suas informações e seus *process* para realização de cada modo, por exemplo: 


    .
    ├── atego1730_ufes_ype             # Pasta do atego1730 que realizou trajeto na arcelor
    │   ├── astro-mercedes-atego1730-sensorbox-1.ini        # Arquivo do veículo
    │   ├── process-atego1730-navigate.ini                  # Process do modo navigate
    │   ├── process-atego1730-playback-sensorbox-1.ini      # Process do modo playback
    │   └── ...                                             # etc.
    └── ...

Caso não tenha o Astro instalado, siga o [Tutorial de instalação do Astro](../../../Installations/install_astro/README.md).

Com o Astro instalado na máquina, dar comando `git pull` dentro da pasta *astro/src* para baixar as novas atualizações do sistema. Ao dar o comando `git status`, é possível verificar quais arquivos contém alterações se comparados aos que estão arquivados localmente com os que estão no repositório do github. Caso não estiver ambientado com esses comandos, [aqui está um guia do github](github_guia_simplificado.md).

Após sincronizado com a versão do github, também dentro da pasta *astro/src*, dar o comando `make clean`, para apagar os arquivos objetos no diretório e `make` para compilar com as novas alterações.


<div id='arquivo-process'/>

## O Arquivo Process

Um process é um arquivo que termina com a nomenclatura *.ini*, é organizado em módulos e possui diversos programas que serão executados simultaneamente pelo *proccontrol*. Ele é usado para executar todos os modos do Sistema de Mobilidade Autônoma, a diferença entre cada um será os módulos e programas que estarão presentes em cada process.

O programa que é chamado para executar os módulos é o *proccontrol*, que executa todos os programas na linha de comando que são especificados pelo arquivo process que foi passado.

A seguir o exemplo de um process:

<p align="center">
  <img width="1000" src="./imgs/process.jpg"/>
</p>

No começo do arquivo são estabelecidos os valores de algumas variáveis, e em seguida, o arquivo é organizado por módulos, onde cada um possui um grupo de programas, que na coluna *requested_state* indica se ele deve ser utilizado ou não. 

### Variáveis
  - Linha 1: Caminho para o arquivo do mapa
  - Linha 2: Caminho para o arquivo do grafo
  - Linha 3: Caminho para o arquivo de anotações
  - Linha 4: Endereço do servidor
  - Linha 5: ID do veículo

Na coluna *command_line*, está o que deve ser escrito na linha de comando pelo *proccontrol*, que é o executável do programa e os argumentos, se necessário.

### Módulos do Process
* Serviços de Suporte: 
  - *./param_daemon*: Dispõe informações sobre o veículo sendo usado e a área ao seu redor para outros programas. Tem como argumento o arquivo do veículo.
  
  - *./proccontrol_gui*: Gera a interface do *proccontrol* mostrando os programas e seus estados.
  
  - *./playback*: Executa um arquivo de log e manda as mensagens armazenadas para outros módulos. Tem como argumento o arquivo de log.
  
  - *./playback_control*: Gera a barra de controle do *playback*, com botões de iniciar e pausar.
  
  - *./map_server*: Carrega a posição do veículo e o mapa offline a partir das informações do arquivo do mapa passado. Possui os seguintes argumentos:
    -  *-map_path* <file_path>: caminho para o arquivo do mapa.
    -  *-map_x* <float_x>: coordenada x global do mapa.
    -  *-map_y* <float_y>: coordenada y global do mapa.
    -  *-block_map* \<estado>: *on* ou *off*, habilita a geração do mapa com as informações dentro do diretório passado.
    -  *-lanemap_incoming_message_type*: tipo de mensagem.
* Carro:
  -  *./simulator_ackerman*: É responsável por gerar dados simulados para o veículo virtual.
  
  <br>
* Slam:
  -  *./localize_ackerman*: Usa informações de sensor do baseServer para encontrar a posição do veículo no mapa fornecido por *param_deamon*. O parâmetro *-mapping_mode* \<estado>, *on* ou *off* indica se será gerado um mapa offline a partir dos sensores.
  
  -  *./mapper*: Atualiza o mapa offline gerado pelo módulo map_server com informação em tempo real (objetos, pessoas, etc). O argumento *-map_path* <file_path> define o caminho para o arquivo do mapa.
  
  -  *./obstacle_distance_mapper*: Gera o mapa de distância para obstáculos, o mapa de custos e faz a detecção de objetos móveis. 
  
  -  *./fused_odometry*: Publica os valores instantâneos de odometria (velocidade e ângulo de volante) obtidos por fusão de sensores.  
  
  -  *./lidarodom*: Publica odometria estimada a partir das leituras de lidar, ou seja, pela comparação entre nuvens de pontos consecutivas.

* Processamento dos Dados dos Sensores:
  - *./gps_xyz*: Filtro do GPS do veículo. Converte dado bruto do GPS para dados usados no sistema, como posição xyz.
  
  - *./base_ackerman*: Aplica (bias) e republica odometria do veículo corrigida.
  
  - *./neural_object_tracker*: Utiliza redes neurais para detectar pedestres em faixa.

  - *./multiple_object_tracker*: Utiliza redes neurais para detectar objetos moveis.

  - *./aruco_position_tracker*: Utiliza biblioteca aruco para detectar posição de marcadores (QR Code) no mundo.
  
* Hierarquia de Controle:
  -  *./task_manager*: Envia ao sistema uma sequência de comandos contidos no arquivo de missão passado como parâmetro. Comandos contidos no arquivo podem incluir: definir Final Goal, definir grafo e mapa, dar início ou interromper o funcionamento autônomo, determinar tempos de espera entre um comando e outro, etc. Possui os argumentos:
     -   *-mission_from_user_app* \<estado>, *on* ou *off* que determina se a missão será definida via aplicativo.
     -   *-mission* <mission_path> que passa o arquivo de missão.
     -   *-start_delay* \<estado>, *on* ou *off* que determina se será ou não utilizado um delay padrão antes de iniciar a missão.
  
  <br/>

  -  *./route_planner*: Acessa o arquivo do grafo e a partir dele publica a sequência de posições desejadas que formam o trajeto que o veículo deverá seguir. Também publica as anotações pertinentes ao longo desse trajeto. Possui como argumento *--graph* <graph_path> <annotation_path>, que são os arquivos de grafo e anotações.
  
  -  *./offroad_planner*: Planeja caminhos não estruturados, ou seja, que começam ou terminam fora de um grafo. Responsável também por situações de manobra de ré.

  -  *./behavior_selector*: Escolhe o comportamento adequado nas diferentes situações detectadas pelos outros módulos.
  
  -  *./model_predictive_planner*: Retorna a aceleração do veículo baseado na próxima posição desejada.
  
  -  *./rrt_path_follower*: Tenta reduzir a latência no sistema de atuação.
  
  -  *./obstacle_avoider*: Controla a aceleração do veículo para evitar obstáculos.
  
* Interfaces:
  -  *./navigator_gui2*: É responsável por gerar a interface gráfica. Possui os argumentos *-map_path* <file_path> e *-annotation_path* <file_path>, que são os arquivos de mapa e anotações.
  
  -  *./viewer_3D*: Gera o gráfico 3D onde o carro em movimento e os dados dos sensores são mostrados.
  
  -  *./util_publish_initial_pose*: Indica a posição inicial do veículo. Possui como argumentos posição x, y e direção.

  - *./camera_viewer*: Gera a interface que mostra a câmera do veículo que foi gravada durante o percurso.

  - *./traffic_light* Detector de semáforos.
  
* API:
  -  *./upstream_api*: Responsável por receber as mensagens dos módulos, assinar e enviar as mensagens necessárias para o Sistema de Monitoramento WEB.
  
  -  *./downstream_api*: Responsável por receber as mensagens do Sistema de Monitoramento WEB e publicar as mensagens necessárias para o restante dos módulos.

## Rodando um Process

Para melhor visualização do funcionamento do Sistema Lume de Mobilidade Autônoma, vamos rodar um *Process Navigate*.

O primeiro passo é rodar o comando `./central` no terminal dentro do diretório *astro/bin*. O programa *central* é responsável por estabelecer conexão e permitir mensagens entre os módulos.

Caso você tente rodar o *central*, sendo que ele já está em execução em outro terminal, receberá o seguinte erro:

<p align="center">
  <img width="auto" src="./imgs/central_error.jpg"/>
</p>

 Em outro terminal, nesse mesmo diretório, devemos chamar o *proccontrol* com o *process* desejado sendo o argumento. Então, rode o comando `./proccontrol process-navigate-volta-da-ufes-sensorbox.ini`

### Corrigindo um Erro

 É possível que ocorram erros ao rodar um *process*. Um motivo comum para essa ocorrência, é quando foi determinado um arquivo com seu caminho, porém o arquivo não existe ou não foi descompactado. Por exemplo, tentando rodar o seguinte *process*: `process-navigate-atego1730_ype-sensorbox-4.ini` que está dentro da pasta do carro */atego1730_ype*, isso ocorreu:

 <p align="center">
  <img width="auto" src="./imgs/process_dying.gif"/>
</p>

A primeira coisa a ser feita é clicar nos programas em amarelo no *Proccontrol GUI* e selecionar "Stop Program", já que eles estão morrendo e sendo chamados novamente pelo *proccontrol*, e em seguida apertar Ctrl+C no terminal para encerrar. 

Para identificar o programa que estava dando problema, abra o terminal e identifique o programa que estava morrendo. Então, execute ele na linha de comando e será mostrada a mensagem de erro. No caso acima, o erro apontado foi:

`PROCCONTROL (9891): ./mapper -map_path atego1730_ype/geodata/map_atego1730_ype_20230511-sensorbox4/map_files3/ (10033) exited UNCLEANLY (code = 255).`

Ao procurar o caminho passado para o *mapper*, percebeu-se que o diretório */map_files3* não existia. Para consertar esse erro, é preciso importar o arquivo de mapa necessário para o programa e botá-lo no diretório especificado.

## Próximo Passo

 Agora que você entendeu como funciona o **Sistema Lume de Mobilidade Autônoma** e como configurar o **Sistema Astro**, o próximo passo é entender o funcionamento do Simulator Mode (Navigate), então siga [esse tutorial](../simulator_mode/simulator_mode.md).
