# Guia de Testes: Validação do Filtro VD (RemoteKit)

Este guia orienta como realizar o envio de payloads JSON via terminal utilizando o utilitário `./publish` para validar as diferentes ramificações de lógica (**Cadastro**, **Duplicados**, **Remoção** e **Tratamento de Erros**) do gerenciador de filtros.

> **Regra de Ouro do Terminal:** Sempre abra a string com aspas simples (`'`) por fora e utilize aspas duplas (`"`) para as chaves e valores do JSON por dentro. Isso evita erros de interpretação do Bash.

---

## 1. Cadastrar um Novo Filtro (Pairing)

Utilizado para ativar a interceptação de um sinal do RemoteKit em um ID específico da rede CAN. O campo `"remove"` deve ser omitido e a `"function"` deve ser obrigatoriamente `"pairing"`.

### Comando no Terminal

```bash
./publish '{"name": "SinalA", "level": "Alto", "config": {"function": "pairing", "id": "0x201", "position": 0, "byte": "0xBD"}}'
```

### Resposta Esperada no VD

O sistema realiza o parse com sucesso, converte as strings hexadecimais para inteiros nativos e registra o filtro na tabela hash.

```text
[ADD] Register pairing!
ID Completo: 201
funcionalidade: pairing
Dados:     0xBD
```

---

## 2. Bloqueio de Filtro Duplicado

Garante a robustez da memória impedindo que a mesma regra seja injetada múltiplas vezes na tabela hash.

Para testar, execute exatamente o mesmo comando de cadastro anterior uma segunda vez.

### Comando no Terminal (Repetido)

```bash
./publish '{"name": "SinalA", "level": "Alto", "config": {"function": "pairing", "id": "0x201", "position": 0, "byte": "0xBD"}}'
```

### Resposta Esperada no VD

A função de segurança detecta a colisão de ID, byte e posição e rejeita a inserção.

```text
[IGNORAR ADD] Tentando adicionar um filter já existente!
```

---

## 3. Remover um Filtro Ativo (Delete)

Aciona a lógica de remoção sequencial de dentro do vetor de regras do ID informado.

É obrigatório passar a chave `"remove": "yes"` no primeiro nível do JSON e preencher a função alvo e o ID dentro de `config`.

### Comando no Terminal

```bash
./publish '{"name": "SinalA", "level": "Alto", "remove": "yes", "config": {"function": "pairing", "id": "0x201"}}'
```

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

Cenários criados para validar que payloads corrompidos ou malformados não causem falhas críticas (*Segmentation Fault*) ou vazamentos de memória no robô.

### Cenário A: Erro de Sintaxe no Sub-JSON (`config`)

Força a falha do `json_tokener_parse` interno enviando um valor sem aspas (sintaxe inválida para o JSON-C).

#### Comando

```bash
./publish '{"name": "SinalA", "config": {"function": "pairing", "id": 0x201}}'
```

#### Retorno Esperado no VD

```text
[ERROR PARSE] mandou chave sem valor no config
```

---

### Cenário B: Tentativa de Remoção sem Parâmetros Obrigatórios

Envia a flag de remoção ativa, mas envia o objeto `config` completamente vazio, quebrando as regras de validação.

#### Comando

```bash
./publish '{"name": "SinalA", "remove": "yes", "config": {}}'
```

#### Retorno Esperado no VD

```text
[IGNORAR DELETE] Tentando remover um filter sem o campo functionality ou sem name/id
```
