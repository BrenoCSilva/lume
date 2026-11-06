# HAL CAN & DBC: Arquitetura de Hashes e Mapeamento de Sinais

Este documento explica como o módulo **Astro HAL CAN** interpreta arquivos `.dbc`, como organiza essas informações em tabelas Hash na memória RAM e como os bytes de um frame CAN são fatiados e decodificados em tempo real.

---

## 1. O Pacote de Controle: `0x98EF00FE write`

Considere a seguinte declaração no arquivo DBC ([`ojAstro_fullDBC.dbc`](dbc_file_example/ojAstro_fullDBC.dbc)):

```dbc
BO_ 0x98EF00FE write: 8 DRIVER
 SG_ BrakesEffort : 0|16@1+ (0.1,0) [0|100] "" DRIVER
 SG_ ThrottleEffort : 16|16@1+ (0.1,0) [0|100] "" DRIVER
 SG_ SteeringEffort : 32|16@1- (0.1,0) [-100|100] "" DRIVER
 SG_ SignalsByte : 48|8@1+ (1,0) [0|100] "" DRIVER
 SG_ Gear : 56|8@1+ (1,0) [0|255] "" DRIVER
VAL_ 0x98EF00FE Gear 16 "drive" 32 "reverse" 8 "neutral" ;
```

### O que isso significa no Barramento Físico?
Quando a mensagem com **CAN ID `0x98EF00FE`** trafega na rede, ela carrega um payload de **8 bytes (64 bits)**. 

O DBC nos ensina exatamente como fatiar esses 8 bytes:

```text
Payload de 8 Bytes (64 bits):
+---------------+---------------+---------------+---------------+---------------+---------------+---------------+---------------+
|    Byte 0     |    Byte 1     |    Byte 2     |    Byte 3     |    Byte 4     |    Byte 5     |    Byte 6     |    Byte 7     |
| (Bits 0 a 7)  | (Bits 8 a 15) | (Bits 16 a 23)| (Bits 24 a 31)| (Bits 32 a 39)| (Bits 40 a 47)| (Bits 48 a 55)| (Bits 56 a 63)|
+---------------+---------------+---------------+---------------+---------------+---------------+---------------+---------------+
|<------- BrakesEffort -------->|<------ ThrottleEffort ------->|<------ SteeringEffort ------->|  SignalsByte  |     Gear      |
|           (16 bits)           |           (16 bits)           |           (16 bits)           |   (8 bits)    |   (8 bits)    |
+-------------------------------+-------------------------------+-------------------------------+---------------+---------------+
```

* **`BrakesEffort` (Bits 0 a 15):** Esforço de frenagem (0 a 100%).
* **`ThrottleEffort` (Bits 16 a 31):** Esforço de aceleração (0 a 100%).
* **`SteeringEffort` (Bits 32 a 47):** Esforço do volante com sinal negativo/positivo (-100 a +100%).
* **`SignalsByte` (Bits 48 a 55):** Byte de flags de sinalização (setas, buzina, farol, retarder).
* **`Gear` (Bits 56 a 63):** Comando de marcha (`8` = neutral, `16` = drive, `32` = reverse).

---

## 2. A Estrutura das Tabelas Hash na Memória

Para buscar e decodificar qualquer sinal em tempo constante ($O(1)$) sem percorrer listas a cada frame recebido, o HAL utiliza **duas tabelas hash principais** indexadas pela mesma **Chave Composta** (`<nome_da_msg>::<nome_do_sinal>`):

```text
Chave Composta: "write::brakeseffort"
   │
   ├──> composite_to_message["write::brakeseffort"] -> Aponta para Message: ID 0x98EF00FE, DLC: 8
   └──> composite_to_signal["write::brakeseffort"]  -> Aponta para Signal: start_bit: 0, size: 16, factor: 0.1
```

### Tabela 1: `composite_to_message` (Validação de Endereço)
* **Chave:** `std::string` (`"write::brakeseffort"`, `"write::gear"`, etc.)
* **Valor:** Ponteiro `const Libdbc::Message*`
* **Função:** Guarda o **endereço CAN (ID)** da mensagem onde o sinal reside.
  * Quando o frame chega da CAN, o simulador compara: `frame->can_id == msg->id()`.
  * Se bater, o simulador sabe que aquele frame contém o sinal desejado.

### Tabela 2: `composite_to_signal` (A Receita de Interpretação)
* **Chave:** Mesma string composta (`"write::brakeseffort"`, `"write::gear"`, etc.)
* **Valor:** Ponteiro `const Libdbc::Signal*`
* **Função:** Guarda a receita matemática de decodificação extraída da linha `SG_`:
  ```cpp
  struct Signal {
      uint32_t start_bit; // Onde começa o sinal (ex: bit 0)
      uint32_t size;      // Quantos bits ocupa (ex: 16 bits)
      bool is_signed;     // Se tem sinal (+ ou -)
      double factor;      // Fator multiplicador (ex: 0.1)
      double offset;      // Deslocamento (ex: 0.0)
      ...
  };
  ```

### Tabela 3: `signal_to_val_map` (Tabela de Estados `VAL_`)
* **Chave Primária:** Chave Composta (`"write::gear"`)
* **Chave Secundária:** Valor numérico bruto (`uint64_t`)
* **Valor:** Nome textual (`std::string`)
  * `signal_to_val_map["write::gear"][8]  = "neutral"`
  * `signal_to_val_map["write::gear"][16] = "drive"`
  * `signal_to_val_map["write::gear"][32] = "reverse"`

---

## 3. Fluxo de Decodificação na Prática: `read_signal()`

Quando o código do simulador executa:
```cpp
bool is_match = false;
double valor = astro_hal_can_interactions::read_signal(dbc, "write", "BrakesEffort", &frame, is_match);
```

O processamento ocorre em 3 passos:

1. **Gera a chave:** Converte para minúsculas: `"write::brakeseffort"`.
2. **Checa o ID:** 
   * Busca em `composite_to_message["write::brakeseffort"]` $\rightarrow$ ID esperado é `0x98EF00FE`.
   * Compara com `frame->can_id`. Se forem iguais, `is_match = true`.
3. **Decodifica os bits:**
   * Busca em `composite_to_signal["write::brakeseffort"]` $\rightarrow$ `start_bit = 0`, `size = 16`, `factor = 0.1`.
   * Isola os 16 bits iniciais de `frame->data`.
   * Aplica a fórmula:
     $$\text{valor} = (\text{bits brutos}) \times 0.1 + 0.0$$

---

## 4. Resumo das Responsabilidades

| Componente | Papel no Sistema |
| :--- | :--- |
| **Linha `BO_`** | Define o endereço (ID CAN), o tamanho total e quem transmite a mensagem. |
| **Linha `SG_`** | Fatia o payload da mensagem, definindo a posição de início, tamanho e conversão física. |
| **Linha `VAL_`** | Mapeia números discretos para palavras textuais (ex: marcha, modos de operação). |
| **`composite_to_message`** | Hash que responde: *"Qual o ID CAN que devo esperar para ler este sinal?"* |
| **`composite_to_signal`** | Hash que responde: *"Como recorto os bits e converto para o valor físico real?"* |
| **`signal_to_val_map`** | Hash que responde: *"Qual o nome textual associado a este número de estado?"* |
