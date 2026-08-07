# Resumo e guia de Testes: Avaliador de Rotas Offroad (`offroad_route_evaluator.py`)

A ferramenta de avaliação automática de rotas offroad (`offroad_route_evaluator.py`) foi criada para automatizar a execução de baterias de testes de regressão do módulo `offroad_planner`. O script é capaz de ler pastas de casos de testes homologados (caminhos ideais), teleportar o veículo virtual no simulador do Astro para as poses iniciais, disparar o planejamento offroad de forma assíncrona até cada objetivo final, monitorar as saídas geradas e classificar os desvios de rota em tempo real usando a métrica de **Distância de Hausdorff**.

---

## Tópicos
1. [Resumo do fluxo de operação](#resumo-do-fluxo-de-operacao)
2. [Guia de Execução e Testes](#guia-de-execucao-e-testes)
   - [Filtro e Estrutura das Pastas de Teste](#1-filtro-e-estrutura-das-pastas-de-teste)
   - [Mecanismo Inteligente de Auto-Detecção (Zero-Config)](#2-mecanismo-inteligente-de-auto-detecao-zero-config)
   - [Execução da Bateria de Testes](#3-execucao-da-bateria-de-testes)
   - [Classificação de Desvios Coloridos em Tempo Real](#4-classificacao-de-desvios-coloridos-em-tempo-real)
   - [Geração do Relatório de Saída](#5-geracao-do-relatorio-de-saida)

---

## Resumo do fluxo de operação

**1 - Descoberta e Filtragem de Casos de Teste:** O script recebe como argumento de linha de comando o caminho da pasta contendo os arquivos ideais. A função `list_files()` realiza uma varredura recursiva inteligente (`os.walk`), ignorando pastas pesadas ou arquivos indesejados (como dumps, mídias ou rascunhos), e mantendo apenas subpastas oficiais de homologação que comecem com `"test_"` ou contenham `"test"` em seu nome.

**2 - Auto-Detecção do Caminho de Saída:** Para saber onde o `offroad_planner` (C++) está configurado para salvar os novos planos calculados, a função `detect_save_path_offroad()` varre a tabela de processos ativos do Linux (`ps aux`), extrai automaticamente a string passada no argumento `-save_offroad_path` do executável ativo e define o caminho de monitoramento. Caso não encontre nenhum processo rodando, o script usa os fallbacks configurados pelo argumento ou o padrão `/dados/testes/`.

**3 - Teleporte e Inicialização Fisiológica (Loop de Teste):** Para cada caso de teste encontrado:
1. O script lê as coordenadas exatas do caminhão (`robot`) e do destino (`goal`) de dentro do arquivo ideal.
2. Limpa qualquer arquivo de resultado anterior para evitar leituras de dados antigos de execuções anteriores.
3. Spawna o utilitário `/home/lume/astro/bin/util_publish_initial_pose` em background com `--wait_time 0` para teleportar o veículo no simulador.
4. Aguarda **5.0 segundos** (tempo fisiológico de segurança para garantir a convergência das partículas de localização do `localize_ackerman` e carregamento estável dos blocos do mapa de obstáculos ao redor do veículo).
5. Encerra o publicador inicial de forma limpa (`terminate()` e `wait()`).
6. Spawna o utilitário `/home/lume/astro/bin/util_publish_final_goal` em background para publicar o destino final, aguardando **5.0 segundos** para o envio da mensagem ao barramento IPC.

**4 - Monitoramento Ativo e Comparação:** O script entra em um loop de espera ativa (`while`) por até **30 segundos** (timeout), verificando a cada 200ms se o arquivo de rota gerada pelo planejador apareceu no disco e foi preenchido. Assim que o arquivo é gravado, o script chama o binário de comparação `compare_routes.py` para calcular a **Distância de Hausdorff** (pior distância espacial) entre o caminho de referência e o caminho recalculado atualmente.

**5 - Apresentação e Geração de Relatório:** O resultado de cada comparação é validado e impresso imediatamente no terminal com cores chamativas e intuitivas baseadas nos níveis de tolerância cinemática do caminhão. No final, se solicitado pelo usuário, a função `save_results()` agrupa e salva o relatório estruturado em um arquivo de texto.

---

## Guia de Execução e Testes

Este guia orienta sobre os pré-requisitos, regras de nomenclatura de pastas, uso das flags de comando e interpretação dos resultados do avaliador.

### 1. Filtro e Estrutura das Pastas de Teste

Para garantir que o script ignore "lixos" ou arquivos de rascunho de engenharia e processe apenas os testes válidos, a pasta fornecida como argumento deve seguir o padrão de nomenclatura contendo `"test_"` ou `"test"` nas subpastas.

**Exemplo de Estrutura de Pastas Esperada:**
```text
dados/casos_testes/
├── test_entrando_pista/        <-- Processado (contém 'test_')
│   ├── caso_1.txt
│   └── caso_2.txt
├── test_saindo_pista/          <-- Processado (contém 'test_')
│   └── caso_1.txt
├── need_for_speed.txt          <-- Ignorado (arquivo solto que não está em subpasta aprovada)
└── mario_galaxy/               <-- Ignorado (nome da pasta não segue o padrão de testes)
    └── screenshot.png
```

---

### 2. Mecanismo Inteligente de Auto-Detecção (Zero-Config)

A fim de evitar erros humanos e simplificar o uso do operador, o script foi projetado para descobrir de forma autônoma onde o planejador offroad está gravando os arquivos no disco.

Se você subir o seu planejador com qualquer caminho de salvamento customizado, por exemplo:
```bash
./offroad_planner -save_offroad_path /dados/testes/minha_pasta/caso_ativo.txt
```
O script Python investigará o sistema operacional e direcionará o monitoramento do teste automaticamente para o arquivo `/dados/testes/minha_pasta/caso_ativo.txt`, dispensando a necessidade de digitar esse caminho novamente no terminal do script!

---

### 3. Execução da Bateria de Testes

Para executar o script, passe o caminho da pasta contendo a estrutura de testes descrita acima.

#### Comando de Execução Padrão:
```bash
python3 offroad_route_evaluator.py /dados/casos_testes
```

#### Execução Forçando um Caminho de Saída Específico (Flag `-p` / `--path`):
Se você quiser forçar um arquivo de saída específico manualmente (ignorando o mecanismo de detecção automática por processo):
```bash
python3 offroad_route_evaluator.py /dados/casos_testes -p /dados/saida/meu_teste.txt
```

#### Execução Gravando um Relatório Final em Arquivo (Flag `-o` / `--output`):
Para rodar a bateria de testes e salvar as estatísticas consolidadas finais em um arquivo `.txt` de auditoria:
```bash
python3 offroad_route_evaluator.py /dados/casos_testes -o meu_relatorio.txt
```

---

### 4. Classificação de Desvios Coloridos em Tempo Real

Durante a execução da bateria de testes, o terminal apresentará em tempo real o andamento de cada caso de teste em cores, baseando-se nos limites de segurança e cinemática estabelecidos para veículos de grande porte (como caminhões Axor):

| Status | Cor | Faixa de Distância | Significado para a Engenharia |
| :---: | :---: | :---: | :--- |
| **OK** | **🟢 Verde** | `Até 0.50 metros` | **Comportamento Idêntico.** O veículo refez o caminho ideal de forma precisa, cobrindo apenas o ruído natural do Lidar simulado e do filtro de partículas de localização. |
| **ALERTA** | **🟡 Amarelo** | `0.51m a 1.50 metros` | **Desvio Leve de Trajetória.** O caminhão realizou o mesmo comportamento geral de manobra, mas abriu ligeiramente uma curva ou desviou de alguma borda de obstáculo. |
| **CRÍTICO** | **🔴 Vermelho** | `Maior que 1.50 metros` | **Comportamento Geométrico Diferente.** O planejador A* tomou uma decisão cinemática totalmente distinta (realizou manobras extras, deu marcha à ré desnecessária ou contornou obstáculos de forma diferente). |
| **TIMEOUT** | **🔴 Vermelho** | `Tempo limite esgotado` | **Falha de Planejamento.** O planejador offroad falhou em gerar uma rota segura no destino final dentro do limite de 30 segundos de processamento. |

#### Exemplo de Saída no Terminal (Tempo Real):
```text
[0/3] Rodando: test_entrando_pista/caso_1.txt
offroad_planner está salvando o plano em '/dados/testes/caso_3/caso_3.txt'

initial_pose: 7805794.411941 || -389283.923509 || -2.674438
final_goal: 7805772.957 || -389314.375 || -2.592353
Nova rota gerada com sucesso! Iniciando comparação...

==========================================
Resultado da Comparação [🟢 OK]:
Distância de Hausdorff: 0.0520 metros
==========================================
------------------------------------------
[1/3] Rodando: test_saindo_pista/caso_1.txt
...
==========================================
Resultado da Comparação [🔴 CRÍTICO]:
Distância de Hausdorff: 7.9492 metros
==========================================
```

---

### 5. Geração do Relatório de Saída

Se você utilizou a flag `-o` / `--output`, os resultados consolidados serão gravados de forma limpa, estruturada e categorizada por gravidade para documentação técnica e auditoria rápida:

```text
============================================================
RELATÓRIO DE REGRESSÃO OFFROAD
============================================================

🔴 TIMEOUTS / ERROS DE EXECUÇÃO:
   test_entrando_pista/caso_2.txt -> Erro: O offroad_planner atingiu o timeout de 30s e não gerou a rota.

🔴 DESVIOS CRÍTICOS (> 1.5m):
   test_saindo_pista/caso_1.txt -> 7.9492 m

🟡 ALERTAS DE DESVIO (0.5m a 1.5m):
   test_entrando_pista/caso_3.txt -> 0.8123 m

🟢 SUCESSOS (< 0.5m):
   test_entrando_pista/caso_1.txt -> 0.0520 m
```
