# Guia de Testes: Validação do Filtro VD (RemoteKit)

Este guia orienta o padrão esperado da string JSON para utilizar o filtro da rede can. A string por padrão irá vir do **atributo command_signals_string** da struct **astro_vehicle_driver_command_signals_message**. Abaixo é demonstrado essa mensagem sendo enviada via terminal.

> **Envio pelo Terminal:** Sempre abra a string com aspas simples (`'`) por fora e utilize aspas duplas (`"`) para as chaves e valores do JSON por dentro. Isso evita erros de interpretação do Bash.

---

## 1. Estrutura Completa da Mensagem

Abaixo está um exemplo contendo todos os campos disponíveis:

### Comando no Terminal

```bash
./test_publish_command_signals '{"name": "RPM", "level": "Alto", "remove": "yes", "config": {"function": "pairing", "id": "0x8000023a", "position": 0, "byte": "0xBD"}}'
```

### Observações

Nem todos os campos são necessários para todas as operações. Entretanto, sempre que os campos abaixo forem enviados, eles devem obrigatoriamente estar dentro da chave `config`:

* `function`
* `id`
* `byte`
* `position`

---

Existem campos que se enviados de maneira isolada nada fazem, são ignoradas pois não existe tratamento. O programa atual só trata mensagens em que o campo remove == yes e/ou function == pairing.

```bash
./test_publish_command_signals '{"name": "RPM"}'
```
astro_error correspondente: **4 - 110 - Rejected_filter_configuration (no treatment for this command.)**


```bash
./test_publish_command_signals '{"config": {"id": "0x8000023a"}}'
```
astro_error correspondente: **4 - 110 - Rejected_filter_configuration (no treatment for this command.)**

Envio de campos malformados:
```bash
./test_publish_command_signals '{"config": {"id": 0x201}}'
```

```bash
./test_publish_command_signals '{"name": "RPM", "config": {"function": "pairing", "id":}}'
```

astro_error correspondente: **4 - 110 - Rejected_filter_configuration ( JSON [command_signal] with invalid syntax.)**

Campo remove não acompanhado do campo config: 
```bash
./test_publish_command_signals '{"name": "RPM", "remove": "yes"}'
```
```bash
./test_publish_command_signals '{"name": "RPM", "remove": "yes", "config": {}}'
```
astro_error correspondente: **4 - 110 - Rejected_filter_configuration ( It is mandatory to send the 'functionality' and at least the 'id' or 'name'.)**

```bash
./test_publish_command_signals '{"name": "RPM", "remove": "yes", "config": {"function": "pairing"}}'
```
astro_error correspondente: **4 - 110 - Rejected_filter_configuration (Exclusion ignored: No filter exists with this ID.)**


## 2. Cadastrar um Novo Filtro

Utilizado para ativar a interceptação de um sinal do RemoteKit em um ID específico da rede CAN.

### Regras

* O campo `"function"` deve ser obrigatoriamente `"pairing"`.
* Os campos `id`, `position` e `byte` são obrigatórios para a operação de cadastro do pareamento.
* O campo `"remove"` pode ser omitido ou possuir qualquer valor diferente de `"yes"`.
* Caso a função informada não seja `"pairing"`, a regra será ignorada, pois atualmente não existe tratamento para outras funcionalidades.

### Comando no Terminal

```bash
./test_publish_command_signals '{"config": {"function": "pairing", "id": "0x8000023a", "position": 0, "byte": "0xBD"}}'
```

### Resposta Esperada no VD

O sistema realiza o parse com sucesso, e registra a regra para o filtro na tabela hash. Caso não obtenha erros, admita que o processo foi realizado com o sucesso.


### Cenário: Parâmetros Obrigatórios Ausentes

Tentar executar a funcionalidade de pareamento sem os campos do config:

```bash
./test_publish_command_signals '{"name": "RPM", "config": {"function": "pairing"}}'
```

astro_error correspondente: **4 - 110 - Rejected_filter_configuration (Required keys [config] are missing for the 'pairing' function in the JSON.)**

---

## 3. Bloqueio de Filtro Duplicado

Impede que a mesma regra seja inserida múltiplas vezes na tabela hash.

Para testar, execute exatamente o mesmo comando de cadastro anterior uma segunda vez.

### Comando no Terminal

```bash
./test_publish_command_signals '{"name": "RPM", "level": "Alto", "config": {"function": "pairing", "id": "0x8000023a", "position": 0, "byte": "0xBD"}}'
```

**Observação:** Apesar de `name` e `level` serem enviados, a validação de duplicidade utiliza apenas os campos armazenados na estrutura do filtro, ou seja, os valores presentes em `config`.

### Resposta Esperada no VD

A função de segurança detecta a colisão de ID, byte e posição e rejeita a inserção.

astro_error correspondente: **4 - 110 - Rejected_filter_configuration ( Filter creation ignored: The requested rule already exists for this ID.)**

---

## 4. Remover um Filtro Ativo (Delete)

Aciona a lógica de remoção de filtros previamente cadastrados.

É obrigatório enviar a chave `"remove": "yes"` no primeiro nível do JSON.

### Remoção por Nome

Para remover utilizando apenas o nome do filtro:

#### Comando no Terminal

```bash
./test_publish_command_signals '{"name": "RPM", "remove": "yes", "config": {"function": "pairing"}}'
```

### Remoção por ID

Para remover utilizando o ID do filtro:

#### Comando no Terminal

```bash
./test_publish_command_signals '{"remove": "yes", "config": {"function": "pairing", "id": "0x8000023a"}}'
```

### Resposta Esperada no VD

O ID é localizado na hash, a regra de pareamento associada é removida do vetor. Caso não obtenha erros, admita que o processo foi realizado com o sucesso.

### Possíveis Erros no Remove

Campo remove não acompanhado do campo config: 
```bash
./test_publish_command_signals '{"name": "RPM", "remove": "yes"}'
```
```bash
./test_publish_command_signals '{"name": "RPM", "remove": "yes", "config": {}}'
```
astro_error correspondente: **4 - 110 - Rejected_filter_configuration ( It is mandatory to send the 'functionality' and at least the 'id' or 'name'.)**

Tentando remover um filtro não existente: 

```bash
./test_publish_command_signals '{"name": "RPM", "remove": "yes", "config": {"function": "pairing"}}'
```
astro_error correspondente: **4 - 110 - Rejected_filter_configuration (Exclusion ignored: No filter exists with this ID.)**

Sinal passado no campo "name" não existe ID correspondente no arquivo dbc:


```bash
./test_publish_command_signals '{"name": "Sinal", "remove": "yes", "config": {"function": "pairing"}}'
```

astro_error correspondente: **4 - 110 - Rejected_filter_configuration (This signal does not exist in the dbc file.)**

