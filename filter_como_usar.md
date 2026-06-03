# Guia de Testes: Validação do Filtro VD (RemoteKit)

Este guia orienta o padrão esperado para a string JSON coletada para utilizar o filtro da rede can. A string por padrão irá vir do **atributo command_signals_string** da struct **astro_vehicle_driver_command_signals_message**. Abaixo é demonstrado essa mensagem sendo enviada via terminal.

> **Envio pelo Terminal:** Sempre abra a string com aspas simples (`'`) por fora e utilize aspas duplas (`"`) para as chaves e valores do JSON por dentro. Isso evita erros de interpretação do Bash.

---

## 0. Estrutura Completa da Mensagem

Abaixo está um exemplo contendo todos os campos disponíveis:

### Comando no Terminal

```bash
./publish '{"name": "SinalA", "level": "Alto", "remove": "yes", "config": {"function": "pairing", "id": "0x201", "position": 0, "byte": "0xBD"}}'
```

### Observações

Nem todos os campos são necessários para todas as operações. Entretanto, sempre que os campos abaixo forem enviados, eles devem obrigatoriamente estar dentro do objeto `config`:

* `function`
* `id`
* `byte`
* `position`

---

Porém, existe campos que enviados de maneira isolada nada fazem, por não existir tratamento são ignorados:


```bash
./publish '{"name": "SinalA"}'
```

## 1. Cadastrar um Novo Filtro

Utilizado para ativar a interceptação de um sinal do RemoteKit em um ID específico da rede CAN.

### Regras

* O campo `"function"` deve ser obrigatoriamente `"pairing"`.
* Os campos `id`, `position` e `byte` são obrigatórios para a operação de cadastro.
* O campo `"remove"` pode ser omitido ou possuir qualquer valor diferente de `"yes"`.
* Caso a função informada não seja `"pairing"`, a regra será ignorada, pois atualmente não existe tratamento para outras funcionalidades.

### Comando no Terminal

```bash
./publish '{"config": {"function": "pairing", "id": "0x201", "position": 0, "byte": "0xBD"}}'
```

### Resposta Esperada no VD

O sistema realiza o parse com sucesso, converte as strings hexadecimais para inteiros nativos e registra o filtro na tabela hash.

```text
[ADD] Register pairing!
ID Completo: 201
funcionalidade: pairing
Dados:     0xBD
```

### Cenário: Parâmetros Obrigatórios Ausentes

#### Comando

```bash
./publish '{"name": "SinalA", "config": {"function": "pairing"}}'
```

#### Resposta Esperada no VD

```text
[ADD IGNORE] Parametros do config incompletos para o pairing
```

---

## 2. Bloqueio de Filtro Duplicado

Impede que a mesma regra seja inserida múltiplas vezes na tabela hash.

Para testar, execute exatamente o mesmo comando de cadastro anterior uma segunda vez.

### Comando no Terminal

```bash
./publish '{"name": "SinalA", "level": "Alto", "config": {"function": "pairing", "id": "0x201", "position": 0, "byte": "0xBD"}}'
```

### Observação

Apesar de `name` e `level` serem enviados, a validação de duplicidade utiliza apenas os campos armazenados na estrutura do filtro, ou seja, os valores presentes em `config`.

### Resposta Esperada no VD

A função de segurança detecta a colisão de ID, byte e posição e rejeita a inserção.

```text
[IGNORAR ADD] Tentando adicionar um filter já existente!
```

---

## 3. Remover um Filtro Ativo (Delete)

Aciona a lógica de remoção de filtros previamente cadastrados.

É obrigatório enviar a chave `"remove": "yes"` no primeiro nível do JSON.

### Remoção por Nome

Para remover utilizando apenas o nome do filtro:

#### Comando no Terminal

```bash
./publish '{"name": "SinalA", "remove": "yes", "config": {"function": "pairing"}}'
```

### Remoção por ID

Para remover utilizando o ID do filtro:

#### Comando no Terminal

```bash
./publish '{"remove": "yes", "config": {"function": "pairing", "id": "0x201"}}'
```

### Observação

Caso `name` e `id` sejam enviados simultaneamente, o `id` possui precedência durante a busca do filtro.

### Resposta Esperada no VD

O ID é localizado na hash, a regra de pareamento associada é removida do vetor e o log confirma a exclusão.

```text
[DELETE]
ID Completo: 201
funcionalidade: pairing
Dados:     0xBD
```

---

## 4. Teste de Defesas e Tratamento de Erros

Cenários criados para validar o comportamento do parser diante da string JSON inválidos ou malformados.

### Cenário A: Erro de Sintaxe no Objeto `config`

#### Valor hexadecimal sem aspas

```bash
./publish '{"name": "SinalA", "config": {"function": "pairing", "id": 0x201}}'
```

#### Chave sem valor

```bash
./publish '{"name": "SinalA", "config": {"function": "pairing", "id":}}'
```

### Resposta Esperada no VD

Todos os cenários acima resultam na mesma falha de validação:

```text
[ERROR PARSE] mandou chave sem valor 
```

---

### Cenário B: Tentativa de Remoção sem Parâmetros Obrigatórios

Envia a flag de remoção ativa, mas envia o objeto `config` vazio, impossibilitando a identificação do filtro a ser removido.

#### Comando

```bash
./publish '{"name": "SinalA", "remove": "yes", "config": {}}'
```

#### Resposta Esperada no VD

```text
[IGNORAR DELETE] Tentando remover um filter sem o campo functionality ou sem name/id
```
