# Resumo e guia de Testes: Avaliador de Rotas Offroad (`offroad_route_evaluator.py`)

A ferramenta de avaliação automática de rotas offroad (`offroad_route_evaluator.py`) foi criada para automatizar a execução de testes de regressão do módulo `offroad_planner`. O script é capaz de ler pastas de casos de testes homologados (caminhos ideais), interagir com o simulador do Astro para recalcular a manobra gerada anteriormente através das poses salvas nos arquivos ideais, monitorar as saídas geradas e classificar os desvios de rota em tempo real usando a métrica de **Distância de Hausdorff**.

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

**1 - Descoberta e Filtragem de Casos de Teste:** O script recebe como argumento de linha de comando o caminho da pasta contendo os arquivos ideais. A função `list_files()` realiza uma varredura recursiva ignorando pastas ou arquivos indesejados (como .png, .pdf, tudo diferente do .txt), além de validar somentes pastas e arquivos que comecem com `"test_"` ou contenham `"test"` em seu nome, evitando verificação de arquivos/pastas indesejadas.

**2 - Auto-Detecção do Caminho de Saída:** O script opera em paralelo com a execução do sistema astro, principalmente com o módulo `offroad_planner`. O script inicialmente irá identificar após a filtragem dos arquivos de interesse a pose_intial e final_goal descrita no plano_ideal.txt. Enviar ao astro e aguardar o novo plano gerado pelo offroad. Por isso é de extrema importância que ao executar o offroad a flag `-save_offroad_path` esteja no argumento do executável ativo. Determinando onde o offroad irá armazenar o plano gerado a partir das poses que o script publicou. O salvamento é necessário, pois o programa irá validar o plano gerado comparando ele com o plano_ideal.txt.

**3 - Monitoramento Ativo e Comparação:** O script entra em um loop, verificando se o arquivo de rota gerada pelo planejador apareceu no disco e foi preenchido. Assim que o arquivo é gravado, o script chama o binário de comparação `compare_routes.py` para calcular a **Distância de Hausdorff** (pior distância espacial) entre o caminho de referência e o caminho recalculado atualmente.

**4 - Apresentação e Geração de Relatório:** O resultado de cada comparação é validado e impresso imediatamente no terminal com cores baseadas nos níveis de tolerância. No final, se solicitado pelo usuário, a função `save_results()` agrupa e salva o relatório estruturado em um arquivo de texto.

---

## Guia de Execução e Testes

Este guia orienta sobre os pré-requisitos, regras de nomenclatura de pastas, uso das flags de comando e interpretação dos resultados do script.

### 1. Filtro e Estrutura das Pastas de Teste

Para garantir que o script ignore "lixos" ou arquivos de rascunho e processe apenas os testes válidos, a pasta fornecida como argumento deve seguir o padrão de nomenclatura contendo `"test_"` ou `"test"` nas subpastas.

**Exemplo de Estrutura de Pastas Esperada:**
```text
dados/casos_testes/
├── test_entrando_pista/        <-- Processado (contém 'test_')
│   ├── caso_1.txt              <-- Ignorado (arquivo não contém 'test' no nome)
│   └── test_caso_2.txt         <-- Processado (contém 'test_')
├── test_saindo_pista/          <-- Processado (contém 'test_')
│   └── test_caso_1.txt         <-- Processado (contém 'test_')
├── need_for_speed.txt          <-- Ignorado (arquivo não contém 'test' no nome)
└── mario_galaxy/               <-- Ignorado (nome da pasta não segue o padrão de conter 'test')
    └── screenshot.png
```

---

### 2. Execução de Testes

Para executar o script, passe o caminho da pasta contendo a estrutura de testes descrita acima.

#### Comando de Execução Padrão:
```bash
python3 offroad_route_evaluator.py /dados/casos_testes
```
Como descrito anteriormente, o script precisa que o offroad salve temporariamente o plano gerado, portanto é indispensável que o offroad seja executado com a flag: 

```bash
./offroad_planner -save_offroad_path /dados/testes/minha_pasta/plan_temp.txt
```

A fim de evitar erros e simplificar o uso, o script foi projetado para descobrir de forma autônoma onde o planejador offroad está gravando temporariamente a rota do plano recalculado no disco. Porém, a primeira verificação feita é confirmar se o caminho para esse arquivo foi passado como argumento através da flag  `-p` / `--path`:

```bash
python3 offroad_route_evaluator.py /dados/casos_testes -p /dados/testes/minha_pasta/plan_temp.txt
```

No entanto caso não seja fornecido esse caminho através da flag na execução do script, o programa investigará o sistema operacional e direcionará o monitoramento do teste automaticamente para o arquivo correto caso o offroad esteja em execução com a flag correta, dispensando a necessidade de digitar esse caminho no terminal do script. O script através do terminal irá sinalizar aonde ele estará procurando esse arquivo:

```bash
python3 offroad_route_evaluator.py "/dados/testes_python" -o /home/lume/astro/src/scripts_lume/script_compare_routes/relatorio.txt
offroad_planner está salvando o plano em '/dados/testes/minha_pasta/plan_temp.txt'  <-- script irá usar esse arquivo para comparar com os arquivos_ideais
```
Porém, caso essas duas primeiras tentativas registrem falha, o programa irá procurar o plano recalculado em /dados/plan_recalculado/plan_recalculado.txt.

---
#### Execução Gravando um Relatório Final em Arquivo (Flag `-o` / `--output`):

Para rodar a bateria de testes e salvar os resultados finais em um arquivo `.txt` execute o programa com a flag -o:
```bash
python3 offroad_route_evaluator.py /dados/casos_testes -o meu_relatorio.txt
```
---

### 4. Classificação de Desvios Coloridos em Tempo Real

Durante a execução da bateria de testes, o terminal apresentará em tempo real o andamento de cada caso de teste em cores, baseando-se nos limites de segurança e cinemática estabelecidos para veículos de grande porte (como caminhões Axor):

| Status | Cor | Faixa de Distância | Significado para a Engenharia |
| :---: | :---: | :---: | :--- |
| **OK** | **🟢 Verde** | `Até 0.50 metros` | **Comportamento Idêntico.** O veículo refez o caminho ideal de forma precisa, cobrindo apenas o ruído natural. |
| **ALERTA** | **🟡 Amarelo** | `0.51m a 1.50 metros` | **Desvio Leve de Trajetória.** O caminhão realizou o mesmo comportamento geral de manobra, mas abriu ligeiramente uma curva ou desviou de alguma borda de obstáculo. |
| **CRÍTICO** | **🔴 Vermelho** | `Maior que 1.50 metros` | **Comportamento Geométrico Diferente.** O planejador tomou uma decisão totalmente distinta (realizou manobras extras, deu marcha à ré desnecessária ou contornou obstáculos de forma diferente). |
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

Se você utilizou a flag `-o` / `--output`, os resultados  serão gravados de forma limpa, estruturada e categorizada por gravidade para documentação:

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
