# Implementando Parâmetros em seu Módulo

Tópicos
- Introdução
- Criando Arquivos de Inicialização
- Interfaces e Implementação de Parâmetros
    - Definições
    - Parâmetros Mandatórios e Opcionais
    - Parâmetros Dinâmicos
- Complete sua implementação
- Testando Parâmetros
    - Atualizando Parâmetros Durante Teste

## Introdução
Parâmetros são importantes para configuração e atualização de valores entre processos.

Para mais detalhes consulte [Como Modificar Dinamicamente Parâmetros Astro](https://github.com/LumeRobotics/astro/blob/main/bin/README_COMO_MODIFICAR_DINAMICAMENTE_PARAMETROS_ASTRO_INI.md) (Também disponível no seu código fonte em: `~/astro/bin/README_COMO_MODIFICAR_DINAMICAMENTE_PARAMETROS_ASTRO_INI.md`)

Para começar certifique-se que seu código está na branch main e devidamente atualizado, utilize git stash para evitar perda de códigos.

## Criando Arquivos de Inicialização
Valores iniciais de parâmetros podem ser definidos de duas formas:
- Arquivos de Processos
- Arquivos de Parâmetros

Para este tutorial você irá implementar um parâmetro opcional e outro mandatório, primeiro faça uma cópia de um arquivo navigate para usarmos como base.
```
cd data/ufes/primeiros_passos/process/
cp process-navigate_primeiros_passos_sensorbox-6.ini \
   process-navigate_primeiros_passos_parameters.ini
```

Escolha o nome dos seus parâmetros e insira as linhas seguindo o padrão da última linha abaixo:

```ini
#### Carro
 simulator 	    support 	1		0		./simulator_ackerman 
 $seu_modulo    support 	1		0		./$executavel -$nome_do_parametro $valor 
```

## Interfaces e Implementação de Parâmetros

### Definições
Estruturas e interfaces relacionadas a parâmetros podem ser encontradas no arquivo `src/param_daemon/param_interface.h`

```c
typedef struct {
  char *module;                      /**<The module name of this parameter. */
  char *variable;                    /**<The variable name to be loaded. */
  astro_param_type_t type;          /**<Type should match user_variable:
					 e.g., ASTRO_PARAM_INT if the local
					 variable storage is an integer. */
  void *user_variable;               /**<A pointer to the local variable
					storage. */
  int subscribe;                     /**<If the param_daemon publishes a
					change to this variable value (because
					someone has changed the variable
					value, should the local value be
					updated? 1 for yes, 0 for no. */
  astro_param_change_handler_t handler; /**<Declare a handler if the
					    param_daemon publishes a change to
					    this variable's value. */
} astro_param_t, *astro_param_p;
```

### Tipos de Parâmetros
As seguintes constantes podem ser usadas para definir o tipo do parâmetro.

| Nome | Descrição | Exemplo |
| -- | -- | -- |
| ASTRO_PARAM_INT     | Identificadores, opção entre items enumerados, números que não podem ser aproximados, onde não é ideal uso de ponto flutuante | -val 42 |
| ASTRO_PARAM_DOUBLE  | Usar para frações, como coordenadas, ponto flutuante | -pos 1.25 |
| ASTRO_PARAM_ONOFF   | Usar para toggles, valores booleanos | -enabled on |
| ASTRO_PARAM_STRING  | Texto, valores alfanuméricos | -text "Hello, World" |
| ASTRO_PARAM_FILE    | Caminho de Arquivo | -file ./data.txt |
| ASTRO_PARAM_DIR     | Caminho de Diretório | -dir ./data |

### Implementando Parâmetros Mandatórios e Opcionais

Você poderá usar o seguinte snippet em seu código, edite as strings de acordo com o nome do seu módulo e o nome dos seus parâmetros.
```c
bool var1_name = FALSE;
bool var2_name = FALSE;

static void 
read_parameters(int argc, char *argv[])
{
    // Required parameters
    astro_param_t required_params[] = {
        {"<module_name>", "<param_name>", ASTRO_PARAM_ONOFF, &var1_name, 1, NULL},
    };
    astro_param_allow_unfound_variables(0); // Use 0 for required params
    astro_param_install_params(argc, argv, required_params, sizeof(required_params)/sizeof(required_params[0]));

    // Optional parameters
    astro_param_t optional_params[] = 
	{
        {"<module_name>", "<param_name>", ASTRO_PARAM_ONOFF, &var2_name, 1, NULL},
    };
    astro_param_allow_unfound_variables(1); // Use 1 for optional params
    astro_param_install_params(argc, argv, optional_params, sizeof(optional_params)/sizeof(optional_params[0]));
}
```

### Implementando Parâmetros Dinâmicos

Parâmetros dinâmicos atualizam variáveis durante a execução. De acordo com a definição da estrutura, devemos alterar o valor do campo subscribe para `1` se quisermos utilizar esse recurso.

```c
astro_param_t optional_params[] =  {
    {"<module_name>", "<param_name>", ASTRO_PARAM_ONOFF, &var, /*->*/ 1 /*<-*/, NULL},
};
```

Caso seu parâmetro tenha que ser estático utilize `0`.

### Complete sua implementação
Utilizando **seu código do tutorial anterior** complete a sua implementação de uma forma que deverá fazer o seu veículo parar e andar de acordo com o valor do parâmetro.

### Atualizando parâmetros durante teste
Parâmetros podem ser atualizados durante a execução com o comando
```
./param_edit -once <module> <variable> <value>
```

> [!NOTE]
> ALTERAÇÃO REALIZADA AQUI!

Observação: Atente-se para o fato de que <variable> é o nome do parametro nomeado em {"<module_name>", "<param_name>", ASTRO_PARAM_ONOFF, &var, 1, NULL}.


Exemplo:

```
./param_edit -once robot state_lock 1
```

## Simulação
1. Coloque robot para dar uma volta completa
2. Desative outros módulos que possam interferir com o teste como o `obstacle_avoider`
3. Enquanto o veículo anda pelo caminho edite os parâmetros usando `param_edit` em outro terminal, mudando os valores para `on` e `off`
4. Após alguns segundos o veículo deverá parar

Se o veículo responder corretamente seu módulo está pronto!

## Próximo Tutorial

Para continuar seu aprendizado, siga para o próximo tutorial: [Executando um audit](./tutorial_como_executar_audit.md)
