# Módulo Simples

Dentro de `astro/src/simple_module_example` podemos encontrar um modelo básico do funcionamento de um Publisher e Subscriber. O Publisher é responsável por enviar a informação (mensagem), enquanto o Subscriber é responsável por receber esta informação.

Um subscriber normalmente tem um formato parecido com a imagem abaixo. Onde temos que passar a chave da mensagem, o formato da mensagem, um possível bloco de memória onde a mensagem será armazenada, o tamanho dessa mensagem, a função que vai funcionar como handler e subscribe_how (este parâmetro controla como a assinatura da mensagem será tratada no gerenciamento da fila de mensagens).

```
astro_subscribe_message (
    ASTRO_STRING_EXAMPLE_MESSAGE_NAME,
    ASTRO_STRING_EXAMPLE_MESSAGE_FMT,
    message,
    sizeof(astro_string_example_message),
    handler,
    subscribe_how
);
```

Já o Publisher irá variar bastante de acordo com a mensagem que deve ser enviada, ou seja, para qual módulo será a comunicação. Mas podemos ter uma noção baseada na imagem abaixo. Precisamos passar todas as informações necessárias para cada publish, principalmente contendo a mensagem em si, o host, o timestamp e a chave da mensagem (sempre analisar a tipagem da struct que irá utilizar).

```
astro_string_example_message main_message;
char *main_content;
main_content = "Mensagem enviada através do módulo publish example";

printf("Enviando mensagem: %s\n", main_content);

main_message.num_message = 1;
main_message.content_message = main_content;
main_message.timestamp = astro_get_time();
main_message.host = astro_get_host();

astro_publish_example_publish_command_message(&main_message);
```

```
void
astro_publish_example_publish_command_message(astro_string_example_message *message)
{
    IPC_RETURN_TYPE err;
    
    err = IPC_publishData((char *) ASTRO_STRING_EXAMPLE_MESSAGE_NAME, message);
    astro_test_ipc_exit(err, "Could not publish", (char *) ASTRO_STRING_EXAMPLE_MESSAGE_FMT);
}
```

Dentro de uma funcionalidade onde é necessário um Subscriber normalmente é utilizado um Handler, que é uma função responsável por realizar alguma ação após o recebimento dessa mensagem. O handler é uma função bem simples, podendo conter qualquer coisa dentro, como na imagem abaixo.

```
static void
astro_simple_handler(astro_string_example_message *current_message)
{
    printf("Mensagem recebida: %s\n", current_message->content_message);
}
```

### Anotações Bruno:

**Replicar e Simplificar**: Replique o exemplo `simple_module_example` e execute-o. Em seguida, remova qualquer parte do código que não seja necessária.

![](https://lh7-rt.googleusercontent.com/docsz/AD_4nXcH8jSDYp0-w1zapIaGi8dZOGH-NyjvEJK7Cjt7hVkRDIYmohCOAxoCrxYC_w9upRnnlvbxtACwm5ymoMkSLd8cHkAlpEyxdXVEtCT6mtDF9e2E1weKQn6oBz8GeUsvqXrivP28?key=z6YPq4aOMJRxIcAMJnYHJA)


## Criando um Módulo no Astro

Para criar um módulo no Astro precisamos começar criando uma pasta dentro de `astro/src`. Dentro desta basta criar os arquivos .c e .h necessários para o desenvolvimento do módulo.

Para compilação do módulo é necessário criar um arquivo Makefile (seguir o padrão de outros módulos). Vale ressaltar para o uso correto das flags que tem relação direta com todas as bibliotecas .h que estão sendo importadas diretamente nos arquivos que você criou. Por exemplo, se dentro do seu código está sendo utilizando a biblioteca:

```
#include <astro/robot_ackerman_interface.h>
```

será necessário incluir a flag:

```
-lrobot_ackerman_interface
```

## Executando um Módulo

Para compilar basta executar o comando make e, caso não tenha dado nenhum erro, rodar no terminal `./<nome_do_executavel>`

## Tarefa 1# - Criando um módulo

Para aplicar esse conhecimentos vamos criar nosso próprio módulo.

1. Crie um módulo usando o dentro da pasta astro/src usando como exemplo o simple_module_example.
2. Seu programa deve receber a mensagem (subscriber) astro_robot_ackerman_motion_command_message. Lembre-se que para isso será necessário criar um handler para recepção dessa mensagem.
3. Em seguida, deve-se publicar a mensagem (publisher): astro_base_ackerman_motion_command_message. Vale ressaltar que éssa mensagem é do obstacle_avoider.
4. Não é necessário fazer nenhum processamento com os dados recebido no item 2, apenas repasse direto via publisher no item 3. (Tome cuidado com os tipos das structs que estão sendo recebidas e enviadas).
5. Crie um makefile e compile o código. Se tudo tiver certo, podemos testar! :)

Observação I: É necessário modificar a função `handler`, pois ela será utilizada para implementar a lógica do Publish. No entanto, para o primeiro passo, você pode deixá-la vazia (ou printando algo), apenas para realizar testes iniciais com o Subscribe.

Observação II : Outros módulos já utilizam de métodos de publisher e subscriber dessas mensagens, então você deve reutilizar esses métodos já aplicados. Você pode utilizar o comando ./grep_all que fica dentro de astro/bin para encontrar outros códigos que utilizam as structs necessárias.

**Buscar Implementações Relevantes**: Use o comando `./grep_all` no terminal para localizar implementações relacionadas ao que você deseja encontrar. Por exemplo, para buscar códigos relacionados a `astro_robot_ackerman_motion_command_message`, digite:
```
./grep_all astro_robot_ackerman_motion_command_message
```

Identifique e documente em qual parte do código essa referência aparece.

![](https://lh7-rt.googleusercontent.com/docsz/AD_4nXdn_iGajJbQ8EoL1QApYwg3v-nrZzDCMw6k28sEZKZgwZ7mJwye4gTu-emtxzjCiuB5H2ck5Emp20Mn6pjwyfKgVdhnsYSsXgiEY3hy6T_O0h3n70_-W-YNJvS24EUgC_VvhZcwvw?key=z6YPq4aOMJRxIcAMJnYHJA)

Esta imagem ilustra o uso do comando `grep_all`. Podemos observar como ele retorna a linha e o arquivo onde o código está localizado. Assim, você pode abrir o arquivo correspondente e analisar exemplos de estruturas e o uso de cada função para auxiliar na implementação do seu código.

> [!NOTE]
> ALTERAÇÃO REALIZADA AQUI!

Dica: Examine diversos códigos para identificar um "padrão" de implementação. Para facilitar sua compreensão, considere a struct `astro_robot_ackerman_motion_command_message` como a __interface primária de entrada__ de dados do `obstacle_avoider`. O `obstacle_avoider` é o módulo de segurança do sistema de mobilidade autônoma. Ele atua como um ouvinte ativo para receber e processar os comandos de movimento. Tais comandos são orquestrados e publicados por módulos de alto nível, como o `navigator_ackerman`, o `motion_planner` ou até mesmo instâncias customizadas como a do seu novo módulo.


**Testar o Subscribe**: Após localizar as funções necessárias para implementar o Subscribe, incorpore-as no código. Em seguida, execute o programa e verifique se ele funciona corretamente.

**Implementar o Publish**: Repita os passos 2 e 3 para implementar o Publish. Sempre que possível, mantenha todas as funcionalidades no mesmo arquivo .c, para garantir simplicidade e alinhamento com o padrão dos códigos existentes.

**Seguir a Tarefa 2**: Após concluir os passos acima, continue com a execução da tarefa 2.

## Tarefa 2# - Testando o módulo

Para testar este módulo criado você deverá seguir os passos para execução de um [Process](https://github.com/LumeRobotics/docs/blob/main/Tutorials/ASTRO/sistema_lume_mobilidade_autonoma/sistema_lume_mobilidade_autonoma_pt-BR.md) e [Simulator Mode](https://github.com/LumeRobotics/docs/blob/main/Tutorials/ASTRO/simulator_mode/simulator_mode.md). Com todo o sistema rodando perfeitamente, você deve:

1. Inserir o veículo no mapa, clicando em Place Robot
2. Marcar o destino com Place Final Goal
3. Calcular a rota
4. Executar clicando em Go

Se tudo tiver de acordo o veículo deve trafegar sem nenhum problema até o destino final marcado. Isso se dá por todas as variáveis externas ativas e funcionando adequadamente. O ProControl GUI deve estar com essa aparência:

![](https://lh7-rt.googleusercontent.com/docsz/AD_4nXd_C_Wy4JJu3HUXKBnqtTjAQMyC6oGoPj5mqMk478UNC-K6tTP5ReP_QbmjbMXB43LPB3Qy0hK1INdUO08P_cJs7h3i-YK8OPD6LazurB3afNCRSYQqPSZYjSE96r-kmNnc3aKHMDUAhIZmWLkwM12hofY?key=z6YPq4aOMJRxIcAMJnYHJA)

> [!NOTE]
> ALTERAÇÃO REALIZADA AQUI!

De maneira geral, o sistema de mobilidade autônoma opera através da execução contínua de vários módulos em paralelo. À medida que o sistema é executado, seus diversos módulos trocam mensagens continuamente. Qualquer módulo que deseje ler essas informações só precisa se inscrever (*subscribe*) para "escutá-las". Portanto, você não precisa publicar mensagens do tipo `astro_robot_ackerman_motion_command_message` manualmente para ver os valores mudando, desde que o próprio sistema esteja rodando para alimentar esse fluxo de dados.

É exatamente isso que ocorre com o `obstacle_avoider` configurado no arquivo `process-navigate_primeiros_passos_sensorbox-6.ini`. Enquanto o simulador de navegação estiver ativo, ele publicará essas mensagens de comando de forma constante. Você poderá comprovar isso apenas observando o seu módulo *subscriber* sendo executado: a publicação manual que você fez na etapa anterior não será mais necessária.

Agora, vamos expandir as funcionalidades do módulo criado na __Tarefa #1__. Para compreender a justificativa prática dessa nova implementação, precisaremos forçar a parada do `obstacle_avoider`. Para fazer isso, vá até o __Proccontrol GUI__, selecione o processo `obstacle_avoider` e clique em __Stop__. O processo deverá ficar vermelho na interface, indicando a interrupção, conforme mostra a imagem abaixo.


![](https://lh7-rt.googleusercontent.com/docsz/AD_4nXeYfgm8T6wlQOiuQjk4js-Gtk_8sBX3sZe7K6LSjhWrwKjEgRJUNdjuJr2axojwtpFcc2dWzk07IY38KiDh_o8Ih-pDINTPCIpL5UaVrF7gIUTyN24up1nDsyHSV_NJAfLfQjIfLHDnjcoYBSM9PHy7850U?key=z6YPq4aOMJRxIcAMJnYHJA)

Ao fazer isso, e tentar trafegar com o veículo novamente, você irá observar que o mesmo não irá andar. 


> [!NOTE]
> ALTERAÇÃO REALIZADA AQUI!

### Por que o veículo para quando o `obstacle_avoider` é interrompido?

Isso ocorre devido a uma rigorosa medida de segurança do sistema. O fluxo de controle do veículo funciona em cascata: o módulo de planejamento gera as rotas e as envia para o `obstacle_avoider`, que, por sua vez, as valida e as repassa continuamente para a camada de baixo nível ou simulador (através da mensagem `ASTRO_BASE_ACKERMAN_MOTION_COMMAND_NAME`).

Para entender esse processo, vale lembrar que a mensagem de planejamento possui o atributo `motion_command` (do tipo `astro_robot_and_trailers_motion_command_t`), que descreve exatamente a trajetória __futura__ e __desejada__ para o robô. Assim que o `obstacle_avoider` intercepta essa estrutura, ele aplica seus algoritmos de segurança. Se detectar que a rota causará uma colisão ou exige uma aceleração perigosa, ele __altera os comandos originais__ na mesma hora (recalculando a velocidade *v* para acionar os freios, por exemplo).

Após essa filtragem, o `obstacle_avoider` empacota os dados já validados em uma __nova__ mensagem (`astro_base_ackerman_motion_command_message`) e a publica. É apenas essa nova mensagem que tem autoridade final sobre o que as rodas do veículo devem de fato executar.

Na outra ponta dessa comunicação, o módulo `base_ackerman` possui um mecanismo que monitora o tempo decorrido desde o recebimento do último comando (controlado pelo parâmetro `motion_timeout`). Quando você força a parada do processo `obstacle_avoider` via Proccontrol, o envio dessas mensagens de controle é interrompido. Ao atingir o tempo limite sem instruções, o sistema assume que houve uma falha crítica de comunicação. Imediatamente, ele limpa a fila de execução e zera a velocidade do carro. O veículo jamais tem permissão para continuar andando "às cegas".

A situação atual é a seguinte: sem o `obstacle_avoider` publicando as informações, o mecanismo de proteção atuou e travou o carro. __Sua tarefa agora__ é suprir essa falta de comunicação. Você deverá atualizar, compilar e executar o código da __Tarefa #1__ para forçar o envio dessas mensagens, assumindo temporariamente o papel de publicação do `obstacle_avoider` e garantindo que o veículo volte a andar.

Dica: use o ./grep_all

Se tudo der certo, ao rodar seu arquivo executável, seu veículo começará a andar novamente.

## Próximo Tutorial

Para continuar seu aprendizado, siga para o próximo tutorial: [Implementando Parâmetros em seu Módulo](./implementing_parameters_in_modules.md)


// Observação: Atente-se para o fato de que <variable> é o nome do parametro nomeado em {"<module_name>", "<param_name>", ASTRO_PARAM_ONOFF, &var, 1, NULL}.

