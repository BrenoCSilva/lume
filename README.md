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


 Resumo e guia de Uso: Plotador de Sinais do Traffic Manager

  A ferramenta plot_traffic_manager_signals foi desenvolvida para ler logs de auditoria (audit.txt), processar os sinais do semáforo enviados pelo Traffic Manager, gerar arquivos de saída formatados e
  plotar esses dados em formato gráfico de forma comparativa utilizando o Gnuplot.

  Tópicos
   1. Resumo do fluxo de operação (#resumo-do-fluxo-de-operacao)
   2. Estrutura da Linha do Log (TRAFFIC_MANAGER_SIGNAL) (#estrutura-da-linha-do-log-traffic_manager_signal)
   3. Compilação e Execução (#compilacao-e-execucao)
   4. Geração de Arquivos de Saída (#geracao-de-arquivos-de-saida)
   5. Visualização Gráfica (Gnuplot) (#visualizacao-grafica-gnuplot)

  ---

  Resumo do fluxo de operação:

  1 - Leitura e Parsing: O programa recebe um ou mais arquivos de log por argumento de linha de comando. A função read_file() percorre cada linha do arquivo identificando a palavra-chave
  TRAFFIC_MANAGER_SIGNAL. Quando encontrada, a linha é parseada através de um padrão rígido de sscanf e adicionada a um vetor de estruturas do tipo traffic_signal.

  2 - Geração de Arquivos de Saída: Para cada arquivo lido, o programa invoca a função generate_outputs_files(), que gera arquivos chamados saida1.txt, saida2.txt, etc. (correspondentes à ordem dos
  arquivos de entrada), contendo apenas as colunas signal e timestamp_velodyne.

  3 - Plotagem Comparativa no Gnuplot: O programa gerencia de forma estática o acúmulo de dados de todos os arquivos analisados. Ao final, abre uma conexão via pipe direto com o Gnuplot (popen("gnuplot
  -persist")), configurando limites de tempo baseados no primeiro e último timestamp lidos, aplicando uma escala vertical de [10:15] e desenhando um gráfico comparativo em formato de degraus (with steps)
  para facilitar a análise de transição de estados dos semáforos.

  ---

  Estrutura da Linha do Log (TRAFFIC_MANAGER_SIGNAL)

  A função read_file() espera que as linhas de log correspondentes aos sinais sigam estritamente o padrão abaixo para um parseamento correto:

   1 TRAFFIC_MANAGER_SIGNAL <num_traffic_lights> <x1> <x2> <y1> <y2> <signal> <annotation_id> <timestamp_velodyne> <descartado> <timestamp_relative>

  Detalhes das Variáveis Parseadas:
   * num_traffic_lights (int): Quantidade de semáforos identificados.
   * x1, x2, y1, y2 (int): Coordenadas de bounding box do semáforo.
   * signal (int): Estado atual/cor do sinal do semáforo (esperado entre 10 e 15).
   * annotation_id (int): ID da anotação do sinal.
   * timestamp_velodyne (double): Timestamp de referência do sensor Velodyne.
   * <descartado> (string): String intermediária ignorada pelo parser (%*s).
   * timestamp_relative (double): Timestamp relativo do sistema.

  ---

  Compilação e Execução

  Compilando o Módulo
  Navegue até a pasta do plotador e execute o make do sistema CARMEN:

   1 cd traffic_manager/plot_traffic_manager_signals
   2 make

  Executando com um Arquivo de Audit
  Para processar e plotar os dados de um único arquivo de log:

   1 ./plot_traffic_manager_signals caminho/do/seu/audit.txt

  Executando com múltiplos arquivos (Comparativo)
  O binário suporta a comparação de múltiplos arquivos simultaneamente. Ele irá sobrepor os gráficos facilitando análises comparativas de latência ou discrepância de sinal:

   1 ./plot_traffic_manager_signals audit_teste_1.txt audit_teste_2.txt

  ---

  Geração de Arquivos de Saída

  Ao rodar a execução, novos arquivos no formato saida<N>.txt serão gerados no mesmo diretório em que o comando foi invocado:

   * Se você enviou audit_teste_1.txt e audit_teste_2.txt, o programa irá criar:
     * saida1.txt (Sinais e Timestamps do arquivo 1)
     * saida2.txt (Sinais e Timestamps do arquivo 2)

  O conteúdo interno desses arquivos segue o formato simplificado ideal para importação externa:

   1 12 1718290234.123456 
   2 12 1718290234.345678 
   3 10 1718290235.112233 

  ---

  Visualização Gráfica (Gnuplot)

  Se você possuir o utilitário gnuplot instalado em seu sistema operacional, após a geração dos arquivos de texto o programa abrirá uma janela gráfica interativa.

  Configurações Automáticas do Gráfico:
   * Eixo X (Tempo): O limite horizontal do gráfico é calculado de forma dinâmica: xrange inicia no primeiro timestamp válido (> 0.0) do primeiro arquivo e encerra no último timestamp do arquivo.
   * Eixo Y (Sinal): Fixado em yrange [10:15] (faixa de operação esperada dos estados dos semáforos).
   * Interatividade: O modo mouse interativo (set mouse) e a persistência na tela estão ativos. Você pode navegar ou dar zoom na janela gerada.

  Erros Comuns na Visualização:
   * Gnuplot não abre a janela: Certifique-se de que o pacote gnuplot-x11 ou similar está instalado e que você possui encaminhamento de X11 habilitado caso esteja rodando via SSH/Docker.

   1   sudo apt update && sudo apt install gnuplot gnuplot-x11
