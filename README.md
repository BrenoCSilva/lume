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

# Resumo e Guia de Uso: Plotador de Sinais do Traffic Manager

A ferramenta `plot_traffic_manager_signals` foi desenvolvida para ler logs de auditoria (`audit.txt`), processar os sinais de semáforo enviados pelo Traffic Manager, gerar arquivos de saída formatados e plotar esses dados em formato gráfico de forma comparativa utilizando o Gnuplot.

---

## Tópicos

1. Resumo do fluxo de operação  
2. Estrutura da linha do log (TRAFFIC_MANAGER_SIGNAL)  
3. Compilação e execução  
4. Geração de arquivos de saída  
5. Visualização gráfica (Gnuplot)

---

## Resumo do fluxo de operação

### 1 - Leitura e Parsing
O programa recebe um ou mais arquivos de log por argumento de linha de comando.  
A função `read_file()` percorre cada linha do arquivo identificando a palavra-chave `TRAFFIC_MANAGER_SIGNAL`.

Quando encontrada, a linha é parseada via `sscanf` e armazenada em um vetor de estruturas do tipo `traffic_signal`.

---

### 2 - Geração de arquivos de saída
Para cada arquivo lido, o programa chama `generate_outputs_files()`, gerando arquivos como:

- `saida1.txt`
- `saida2.txt`
- etc.

Esses arquivos contêm apenas:
- `signal`
- `timestamp_velodyne`

---

### 3 - Plotagem comparativa no Gnuplot
O programa acumula os dados de forma estática para múltiplos arquivos.

Ao final, abre uma conexão com o Gnuplot via:

```bash
popen("gnuplot -persist")
