## Erro na execução do Process

O print abaixo mostra em amarelo os módulos que não foram executados:

<p align="center">
  <img width="800" src="./modulos_erros.png"/>
</p>

Para identificar o erro de execução é necessário parar o processo que deu erro clicando com o botão esquerdo do mouse em cima dele e selecionar a opção stop. Depois vá ao terminal onde está sendo executando o ./proccontrol para verificar o log do erro.

<p align="center">
  <img width="800" src="./mensagem_error.png"/>
</p>
Nesse log você consegue pegar qual foi o arquivo que não foi executado, como por exemplo o mapper:

```bash
./mapper -map_path ../data/ufes/primeiros_passos/geodata/map_primeiros_passos_20240613/map_files
Um erro que pode ocorrer ao executar seu Process através do procconctrol
```
Para encontrar qual foi o motivo do erro acima copie toda a linha e abra outro terminal e navegue até a pasta bin do astro. E o execute.
  
**Obs.:** O *./central* ainda precisa estar em execução

<p align="center">
  <img width="800" src="./mss.jpeg"/>
</p>

Como resultado verifica-se que que o módulo mss não foi encontrado. Por causa de uma atualização nos mapas do astro esse módulo se tornou obrigatório para que os mapas sejam carregados. Para corrigir, execute o comando abaixo para que ocorra 

```bash
./mss_process_prepare.py <caminho para o seu process>
```
Dica: Toda vez que um arquivo do *process* não é executado, utilize esse método para descobrir a razão. Um erro comum de acontecer é o caminho das variáveis do Process estarem errados ou não existir mais os arquivos no local.