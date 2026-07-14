# Resumo e guia de Testes: Estimador de RPM (Vehicle Driver)
    
A ferramenta de estimar RPM foi criada para processar arquivos de log de audit (`audit.txt`), extrair os dados de telemetria gerados pelo módulo Vehicle Driver, gerar arquivos de saída e apresentar visualizações gráficas comparativas usando o Gnuplot.
    
## Tópicos
1. [Resumo do fluxo de operação](#resumo-do-fluxo-de-operacao)
2. [Guia de Execução e Testes](#guia-de-execucao-e-testes)
   - [Estrutura do Log e Parsing](#1-estrutura-do-log-e-parsing)
   - [Compilação do Módulo](#2-compilacao-do-modulo)
   - [Geração de Saídas Simplificadas](#3-geracao-de-saidas-simplificadas)
   - [Plotagem Gráfica Comparativa](#4-plotagem-grafica-comparativa)
   
   
## Resumo do fluxo de operação:
   
**1 - Leitura e Parsing do Log:** O programa recebe como argumentos de linha de comando o caminho do arquivo de log de audit. A função `read_file()` varre linha por linha do arquivo em busca das palavras-chave `VD_RPM`, `VD_CURRENT_GEAR` e `ROBOTVELOCITY_ACK`. As linhas encontradas são desmembradas utilizando um padrão `sscanf` que mapeia e armazena os dados relevantes nos vetores de estruturas `rpm`, `gear` e `velocity`. Essas listas de dados são encapsulados em uma estrutura `SignalSeries`.
   
**2 - Geração de Arquivos de Saída:** Para o arquivo de log passado por argumento, a função `generate_outputs_files()` gera automaticamente arquivos formatados na pasta de saída. Esses arquivos de saída contêm os dados de telemetria (RPM, marcha e velocidade) e seus respectivos `timestamp_relative`. Também é gerado como saída um arquivo chamado `reports.txt`, no qual é apresentado, para cada marcha do veículo:
    1 - A quantidade de sinais identificados após a filtragem dos dados brutos;
    2 - A expressão matemática que retorna o RPM do veículo a partir da velocidade;
    3 - O pareamento desses sinais em uma escala percentual. Esses pareamento é realizado a partir da expressão matemárica, o percentual é obtido quando comparado com seu valor real no log de audit.
   
## Guia de Execução e Testes
   
Este guia orienta o padrão esperado dos arquivos de log, como compilar o utilitário e como realizar os testes de plotagem comparativa.
 
**Requisito do Sistema:** Para que a visualização gráfica abra com sucesso na tela, certifique-se de que o `gnuplot` esteja instalado em sua máquina.

### 1. Estrutura do Log e Parsing

   O parser `sscanf` presente na função `read_file()` espera que as linhas de log correspondentes ao veículo sigam estritamente o formato abaixo:
   
Padrão Esperado no Log: 
```bash
VD_RPM <rpm> <timestamp_velodyne> <ignorado> <timestamp_relative>
VD_RPM 1554.880000 1777900803.322888 ford_escape_hybrid@200 430.736581

ROBOTVELOCITY_ACK <v> <phi> <timestamp_velodyne> <ignorado> <timestamp_relative>
ROBOTVELOCITY_ACK 8.043000 0.062370 1777900803.320781 ford_escape_hybrid@200 430.734542

VD_CURRENT_GEAR <current_gear> <raw_gear_value> <timestamp_velodyne> <ignorado> <timestamp_relative>
VD_CURRENT_GEAR 8 81 1777900803.297890 ford_escape_hybrid@200 430.716332
```
### 2. Compilação do Módulo
   
Navegue até a pasta do módulo e compile o arquivo utilizando o `make` do ambiente:
   
#### Comando no Terminal

```bash
  cd vehicle_driver/estimar_rpm
  make
```
O comando irá produzir o binário executável `gemini`.


 ### 3. Geração de Saídas Simplificadas

 Sempre que o executável processar um arquivo de entrada, ele gerará arquivos de saída contendo apenas os dados de telemetria correspondentes e os timestamps relativos.

**Nota:** Certifique-se de que a pasta `saidas/` exista no diretório atual antes de rodar o programa, ou crie-a com o comando `mkdir saidas`
 
```bash
  ./gemini meu_audit.txt
```

   
**Resultado:** Serão gerados arquivos formatados na pasta `saidas/` (como `rpms.txt`, `gears.txt` e `velocities.txt`) com os dados organizados em colunas, nos moldes dos exemplos abaixo:

* `rpms.txt`:
```text
VD_RPM 1554.880000 430.736581 
```

* `gears.txt`:
```text
VD_GEAR 8 430.716332 
```

* `velocities.txt`:
```text
VD_VELOCITY 8.043000 430.734542 
```

* `reports.txt`:
```text
marcha 1 quantidade de sinais 10956 -  da marcha 1:    f(v) = 1056.756977 * v + 70.814914
            Distribuição da acurácia (Estimativa /RPM Real):
              < 50%       :      0 ( 0.00%)
              50%  - 60%   :      0 ( 0.00%)
              60%  - 70%   :      0 ( 0.00%)
              70%  - 80%   :      0 ( 0.00%)
              80%  - 90%   :     55 ( 0.50%)
              90%  - 100%  :   5436 (49.62%)[* Erro de 10%]
              100% - 110%  :   5378 (49.09%)[* Erro de 10%]
              110% - 120%  :     87 ( 0.79%)
              120% - 130%  :      0 ( 0.00%)
              130% - 140%  :      0 ( 0.00%)
              140% - 150%  :      0 ( 0.00%)
              > 150%      :      0 ( 0.00%)
```
 ---

 ### 4. Plotagem Gráfica 
  .
 
 #### Comando no Terminal
```bash
  ./gemini meu_audit.txt
```
#### Comportamento Esperado na Tela (Interface Gráfica):

1. Uma janela do Gnuplot se abrirá exibindo o título **"Análise de Telemetria (RPM vs Velocidade vs Marcha)"**.
2. O eixo horizontal (Tempo) se ajustará automaticamente entre o menor e o maior timestamp válido de leitura.
3. O eixo vertical esquerdo (y1) mostrará o RPM do motor calibrado entre `0` e `2500` RPM. O eixo vertical direito (y2) exibirá a escala conjunta de velocidade (m/s) e marcha ativa entre `-5` e `15`.
4. O gráfico apresentará as curvas contínuas originais brutas ao fundo (em cores claras) e superporá os pontos discretos filtrados/válidos por cima, facilitando a comparação direta visual do efeito dos filtros de mediana.
5. O gráfico permanecerá interativo (permitindo zoom com o mouse) até que você feche a janela.
