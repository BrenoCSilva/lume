> [!NOTE]
> Este texto terá uma barra lateral azul (Informação).

> [!TIP]
> Este terá uma barra verde (Dica).

> [!IMPORTANT]
> Este terá uma barra roxa (Importante).

> [!WARNING]
> Este terá uma barra amarela (Aviso).

> [!CAUTION]
> 
  # Resumo e guia de Testes: Plotador de Sinais do Traffic Manager
    
A ferramenta `plot_traffic_manager_signals` foi criada para processar arquivos de log de auditoria (`audit.txt`), extrair os dados de sinais gerados pelo módulo Traffic Manager, gerar arquivos de saída limpos e apresentar visualizações gráficas interativas e comparativas usando o Gnuplot.
    
## Tópicos
1. [Resumo do fluxo de operação](#resumo-do-fluxo-de-operacao)
2. [Guia de Execução e Testes](#guia-de-execucao-e-testes)
   - [Estrutura do Log e Parsing](#1-estrutura-do-log-e-parsing)
   - [Compilação do Módulo](#2-compilacao-do-modulo)
   - [Geração de Saídas Simplificadas](#3-geracao-de-saidas-simplificadas)
   - [Plotagem Gráfica Comparativa](#4-plotagem-grafica-comparativa)
   
   
## Resumo do fluxo de operação:
   
**1 - Leitura e Parsing do Log:** O programa recebe como argumentos de linha de comando os caminhos de arquivos de log de auditoria. A função `read_file()` varre linha por linha de cada arquivo em
      busca da palavra-chave `TRAFFIC_MANAGER_SIGNAL`. As linhas encontradas são desmembradas utilizando um padrão `sscanf` que mapeia e armazena os dados relevantes em um vetor de estruturas
      `traffic_signal`.
   
**2 - Geração de Arquivos de Saída:** Para cada arquivo de log passado por argumento, a função `generate_outputs_files()` gera automaticamente um arquivo formatado do tipo `saida<N>.txt`. Esse
      arquivo de saída simplificado contém apenas as colunas de `signal` e `timestamp_velodyne`.
   
**3 - Plotagem pelo Gnuplot:** A função `gnuplot()` acumula os conjuntos de dados de todos os arquivos e os envia simultaneamente para um processo do Gnuplot. O gráfico define automaticamente o intervalo horizontal (`xrange`) com base no primeiro e último timestamp lidos e desenha as curvas em formato de degraus (`with steps`) sob uma escala
      vertical fixa de `[10:15]`, que representa a faixa de estados do semáforo.
   
    
   
## Guia de Execução e Testes
   
Este guia orienta o padrão esperado dos arquivos de log, como compilar o utilitário e como realizar os testes de plotagem individual ou comparativa.
 
**Requisito do Sistema:** Para que a visualização gráfica abra com sucesso na tela, certifique-se de que o `gnuplot` e sua interface X11 estejam instalados em sua máquina Linux.
     ```bash
     sudo apt update && sudo apt install gnuplot gnuplot-x11
    ```



### 1. Estrutura do Log e Parsing

   O parser `sscanf` presente na função `read_file()` espera que as linhas de log correspondentes aos semáforos sigam estritamente o formato abaixo:
   
#### Padrão Esperado no Log: TRAFFIC_MANAGER_SIGNAL <num_traffic_lights> <x1> <x2> <y1> <y2> <signal> <annotation_id> <timestamp_velodyne> <ignorado> <timestamp_relative>
   
#### Exemplo Prático de Linha do Log: TRAFFIC_MANAGER_SIGNAL 1 100 200 150 250 12 55 1718290234.123456 WORD 1718290234.345678

  **Campos Mapeados:**
       * `num_traffic_lights` = `1`
       * `x1` = `100`, `x2` = `200`, `y1` = `150`, `y2` = `250`
       * `signal` = `12` (ID do sinal/cor do semáforo)
       * `annotation_id` = `55`
       * `timestamp_velodyne` = `1718290234.123456`
       * `timestamp_relative` = `1718290234.345678`
    

  ### 2. Compilação do Módulo
   
Navegue até a pasta do módulo e compile o arquivo utilizando o `make` do ambiente:
   
#### Comando no Terminal

```bash
  cd traffic_manager/plot_traffic_manager_signals
  make
```
O comando irá produzir o binário executável `plot_traffic_manager_signals`.


 ### 3. Geração de Saídas Simplificadas

 Sempre que o executável processar arquivos de entrada, ele gerará arquivos de saída contendo apenas o sinal e o timestamp do velodyne.
 
```bash
  ./plot_traffic_manager_signals meu_audit_1.txt
```

   
**Resultado:** Será gerado um arquivo `saida1.txt` com as colunas formatadas:

12 1718290234.123456 
12 1718290234.345678 
13 1718290235.112233 
  
#### Testando com múltiplos arquivos:
  ./plot_traffic_manager_signals meu_audit_1.txt meu_audit_2.txt


 **Resultado:** Serão criados dois arquivos no mesmo diretório:
 * `saida1.txt` (dados extraídos de `meu_audit_1.txt`)
 * `saida2.txt` (dados extraídos de `meu_audit_2.txt`)

 ---

 ### 4. Plotagem Gráfica Comparativa
  
 Para comparar e visualizar o comportamento temporal dos semáforos entre diferentes execuções ou trechos de logs, basta passar múltiplos caminhos.
 
 #### Comando no Terminal
  ./plot_traffic_manager_signals audit_execucao_A.txt audit_execucao_B.txt

#### Comportamento Esperado na Tela (Interface Gráfica):

1. Uma janela do Gnuplot se abrirá exibindo o título **"Gráfico de sinal x timestamp"**.
2. O eixo horizontal (Tempo) se ajustará automaticamente entre o menor e o maior timestamp válido.
3. O eixo vertical mostrará o sinal de semáforo fixado entre as marcas `10` e `15`.
5. O gráfico permanecerá interativo (permitindo zoom com o mouse) até que você feche a janela.
