
Chaves que passam pelo parse. 

```bash
./publish '{"name": "SinalA", "level": "Alto", "remove": "yes", "config": {"function": "pairing", "id": "0x201", "position": 0, "byte": "0xBD"}}'
```

Qualquer outra vai ser ignorada.

```bash
./publish '{"ano": "2026"}'

```

Existe campos que enviados de maneira isolada nada fazem, são ignoradas pois não existe tratamento.
O programa atual só trata remove == yes e function == pairing.


```bash
./publish '{"config": {"id": "0x201"}}'

```
```bash
./publish '{"name": "SinalA"}'

```

mandar chave sem valor ou vazio mensagem vai ser ignorada: 

```bash
./publish '{"config": {"id": 0x201}}'

```

O remove sem config o programa ignora mensagem 

```bash
./publish '{"name": "SinalA", "remove": "yes"}'

```

e um function == pairing sem id/position/byte tambem ignora: 

```bash
./publish '{"name": "SinalA", "config": {"function": "pairing"}}'

```


------------------------------------------------------------------------
ADICIONAR UM REGRA NO FILTRO


```bash
./publish '{"config": {"function": "pairing", "id": "0x201", "position": 0, "byte": "0xBD"}}'

```

Atualização feita, não permito duplicatas (mesmo func, id, pos, byte).

id igual mas byte/pos/func diferente, tudo bem: 


```bash
./publish '{"config": {"function": "pairing", "id": "0x201", "position": 1, "byte": "0xBD"}}'

```

```bash
./publish '{"config": {"function": "pairing", "id": "0x201", "position": 0, "byte": "0xCC"}}'

```

func tudo bem, mas não há tratamento para func != pairing então nao vai para a hash só ignora:


```bash
./publish '{"config": {"function": "block", "id": "0x201", "position": 0, "byte": "0xCC"}}'

```
REMOVE 

Remoção por ID, só manda remove, function, id : 


```bash
./publish '{"remove": "yes", "config": {"function": "pairing", "id": "0x201"}}'

```

tentando remover id que não existe uma regra: 

```bash
./publish '{"remove": "yes", "config": {"function": "pairing", "id": "0x501"}}'

```

Remoção por NAME : 

```bash
./publish '{"name": "SinalA", "remove": "yes", "config": {"function": "pairing"}}'
```

id tem prececendia por name: 


```bash
./publish '{"name": "SinalA", "remove": "yes", "config": {"function": "pairing", "id": "0x501"}}'
```