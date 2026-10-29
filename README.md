# 02 - Planejamento Preventivo e Validação de Trajetória (`obstacle_avoider`)

O trabalho principal do [`obstacle_avoider()`](file:///home/lume/astro/src/obstacle_avoider/obstacle_avoider.cpp#L1510) é pegar os comandos de movimento que o MPP enviou, desenhar por onde o caminhão vai passar no mapa e verificar com antecedência se ele vai bater em alguma coisa.

---

## 1. O que o MPP envia vs. O que o OA calcula

O MPP **não** diz as coordenadas no mapa $(X, Y)$ de onde o caminhão vai estar. Ele só manda uma receita de volante e acelerador: *"ande a tantos m/s, vire as rodas tantos graus e faça isso por tanto tempo"*:

```c
typedef struct {
    double x;                               // ~~Não vem do MPP~~ -> Calculado pelo OA
    double y;                               // ~~Não vem do MPP~~ -> Calculado pelo OA
    double theta;                           // ~~Não vem do MPP~~ -> Calculado pelo OA
    int num_trailers;                       // ~~Não vem do MPP~~ -> Preenchido pelo OA
    double trailer_theta[MAX_NUM_TRAILERS]; // ~~Não vem do MPP~~ -> Calculado pelo OA
    double v;                               // Preenchido pelo MPP (velocidade linear)
    double phi;                             // Preenchido pelo MPP (ângulo das rodas dianteiras)
    double time;                            // Preenchido pelo MPP (tempo de duração do comando)
} astro_robot_and_trailers_motion_command_t;
```

> [!NOTE]
> **Uma diferença importante entre as duas checagens do OA:**  
> - **Potential Collision (Emergência):** Faz uma simulação de pânico supondo que o caminhão vai frear com o volante travado reto na direção atual.  
> - **obstacle_avoider() (Preventivo):** Simula o caminho real, considerando que o caminhão vai de fato esterçar as rodas e fazer as curvas que o MPP planejou.

---

## 2. Como o `obstacle_avoider()` funciona por dentro

A rotina roda em 5 passos práticos dentro de um ciclo de repetição:

1. **Projetar o caminho passo a passo ([`build_predicted_trajectory`](file:///home/lume/astro/src/obstacle_avoider/obstacle_avoider.cpp#L1471)):**  
   A projeção começa na posição atual onde o caminhão está agora (vinda do `localize`). Aplicando o primeiro comando `motion[0]` do MPP, o modelo do veículo calcula onde a cabine e os semi-reboques vão parar e grava essas coordenadas no próprio `motion[0]`. Em seguida, essa nova posição é usada como ponto de partida para calcular a posição seguinte `motion[1]`, e assim por diante até desenhar todo o caminho no mapa.

2. **Checar se já está batendo agora:**  
   Verifica se a posição atual do caminhão já está colidindo com algo. Se estiver colidindo e o caminhão estiver em movimento ($v > 0{,}001\text{ m/s}$), ele zera a velocidade de todos os comandos na hora e avisa o sistema emitindo o **`ASTRO_ERROR_CODE_201`**.

3. **Checar os pontos futuros do caminho:**  
   Varre os pontos calculados lá na frente contra o mapa de obstáculos e os objetos em movimento (DATMO). Se encontrar alguma colisão à frente:  
   $\rightarrow$ Reduz a velocidade do plano em 5% a cada comando ([`velocity_recalculate`](file:///home/lume/astro/src/obstacle_avoider/obstacle_avoider.cpp#L1435)).  
   $\rightarrow$ Emite o aviso **`ASTRO_ERROR_CODE_203`**.

4. **Verificar se dá tempo de frear com segurança (Torricelli):**  
   Calcula quantos metros o caminhão precisa para frear até parar com a velocidade atual ($\Delta S = \frac{v^2}{2a}$). Se o caminho restante que sobrou no plano for menor do que essa distância (por exemplo, se o MPP travar ou parar de mandar novos planos):  
   $\rightarrow$ Também reduz as velocidades em 5% para forçar a parada antes de acabar a pista.  
   $\rightarrow$ Emite o aviso **`ASTRO_ERROR_CODE_203`**.

5. **Repetir o ajuste até o caminho ficar livre:**  
   Se houve colisão no passo 3 ou faltou espaço de frenagem no passo 4, o OA volta para o passo 1 e recalcula todas as posições com essas velocidades mais baixas. A lógica é simples: se antes o caminhão batia lá na frente (por exemplo, na pose `[37]`), com o plano mais lento ele vai percorrer menos espaço no tempo e parar antes do obstáculo. Esse ciclo se repete até o plano ficar 100% livre de colisão ou o caminhão parar de vez.

> [!IMPORTANT]
> **Avisando a base para frear:**  
> Sempre que o OA precisa intervir no plano (seja reduzindo a velocidade ou parando o veículo), ele ativa a flag `global_new_motion_command_need_to_be_published = 1`. Isso garante que o plano corrigido seja enviado imediatamente para a base ackerman executar a frenagem.

---

## 3. Entendendo a Matemática Sem Complicação

### 3.1. Como o caminho é desenhado (`build_predicted_trajectory`)
A função [`astro_libcarmodel_recalc_pos_ackerman()`](file:///home/lume/astro/sharedlib/libcarmodel/ackerman_model.c) recebe a pose atual do caminhão, a velocidade $v$, o esterçamento $\phi$ e o tempo $dt$. Ela devolve a nova pose da cabine e o ângulo de cada semi-reboque atrelado:

$$\mathbf{P}_{\text{novo}} = f_{\text{Ackerman}}(\mathbf{P}_{\text{anterior}}, v, \phi, dt)$$

Com isso, o OA sabe exatamente onde cada círculo de colisão (marker) da cabine e das carretas estará no chão a cada fração de segundo.

### 3.2. A Frenagem de Torricelli e a Morte do MPP
Para não depender de tempo de relógio, o OA usa a física clássica de Torricelli para saber a distância mínima de parada:

$$\Delta S_{\text{parada}} = \frac{v^2}{2 \cdot a_{\max}}$$

Onde $a_{\max}$ (`maximum_deceleration_forward`) é a capacidade máxima de frenagem da cabine configurada no arquivo `.ini`.

Ao mesmo tempo, o OA soma a distância real que o plano do MPP ainda tem pela frente:
$$D_{\text{plano}} = \text{distância somada de todos os comandos que ainda restam}$$

- **Quando tudo está normal:** O MPP manda planos de $\approx 18\text{ m}$ a cada $100\text{ ms}$, enquanto a $10\text{ km/h}$ o caminhão precisa de apenas $\approx 3{,}8\text{ m}$ para parar. Margem de sobra!
- **Quando o MPP morre:** Nenhum plano novo chega. O caminhão vai andando e o [`consume_motion_command_time()`](file:///home/lume/astro/src/obstacle_avoider/obstacle_avoider_main.cpp#L421) vai descartando os comandos já executados. O plano vai encolhendo: $140 \to 100 \to 50 \to 25$ comandos.
- **O momento do resgate:** Quando o plano encolhe tanto que fica menor que a distância de parada ($D_{\text{plano}} < \Delta S_{\text{parada}}$), o OA assume o controle, aciona o recálculo e freia o caminhão suavemente até parar no limite da pista autorizada.

### 3.3. Como a velocidade é reduzida em 5% (`velocity_recalculate`)
Para frear sem dar tranco no caminhão e sem causar efeito canivete com os semi-reboques, a velocidade decai 5% a cada comando:

$$v_0 = v_0 \cdot 0{,}95, \quad v_1 = v_1 \cdot (0{,}95)^2, \quad v_2 = v_2 \cdot (0{,}95)^3 \dots$$

E tem um detalhe genial:
```cpp
motion_commands_vector[i].v *= reduction_factor;
motion_commands_vector[i].time *= 1.0 / reduction_factor;
```
Quando a velocidade é multiplicada por $0{,}95$, o tempo $dt$ é dividido por $0{,}95$.  
Como $\text{distância} = v \cdot dt$, o resultado da multiplicação dá exatamente o mesmo valor!
> **Significado prático:** A geometria do caminho não muda um milímetro sequer. O caminhão vai fazer a mesma curva e passar exatamente pelo mesmo lugar, apenas andando mais devagar.

Quando a velocidade cai abaixo de $0{,}05\text{ m/s}$ ($5\text{ cm/s}$), o OA zera tudo (`v = 0.0`) e o caminhão atinge a parada total com segurança.

---

## 4. O que Acontece na Prática (Logs Reais)

Abaixo vemos o terminal de frenagem (`/tmp/oa_braking_distance.log`) no momento em que o MPP foi desligado durante um teste a $10\text{ km/h}$:

```text
>>> NOVO PLANO DO MPP ENVIADO!!! 140 comandos <<<
[OA Frenagem - 140 comandos] Vel: 2.77 m/s (10.0 km/h) | Parada Nec.: 3.82 m | Restante Plano: 19.20 m | Margem: +15.38 m -> [OK - SEGURO]
[OA Frenagem - 118 comandos] Vel: 2.77 m/s (10.0 km/h) | Parada Nec.: 3.82 m | Restante Plano: 17.24 m | Margem: +13.41 m -> [OK - SEGURO]
[OA Frenagem -  87 comandos] Vel: 2.77 m/s (10.0 km/h) | Parada Nec.: 3.82 m | Restante Plano: 13.82 m | Margem:  +9.99 m -> [OK - SEGURO]
[OA Frenagem -  48 comandos] Vel: 2.77 m/s (10.0 km/h) | Parada Nec.: 3.82 m | Restante Plano:  7.85 m | Margem:  +4.03 m -> [OK - SEGURO]
[OA Frenagem -  22 comandos] Vel: 2.75 m/s ( 9.9 km/h) | Parada Nec.: 3.78 m | Restante Plano:  3.68 m | Margem:  -0.10 m -> [INSUFICIENTE - FRENAGEM FORÇADA]
[OA Frenagem -  21 comandos] Vel: 2.53 m/s ( 9.1 km/h) | Parada Nec.: 3.20 m | Restante Plano:  1.91 m | Margem:  -1.29 m -> [INSUFICIENTE - FRENAGEM FORÇADA]
[OA Frenagem -  21 comandos] Vel: 1.84 m/s ( 6.6 km/h) | Parada Nec.: 1.69 m | Restante Plano:  0.78 m | Margem:  -0.91 m -> [INSUFICIENTE - FRENAGEM FORÇADA]
[OA Frenagem -  21 comandos] Vel: 0.89 m/s ( 3.2 km/h) | Parada Nec.: 0.40 m | Restante Plano:  0.02 m | Margem:  -0.38 m -> [INSUFICIENTE - FRENAGEM FORÇADA]
[OA Frenagem -  21 comandos] Vel: 0.26 m/s ( 0.9 km/h) | Parada Nec.: 0.03 m | Restante Plano:  0.04 m | Margem:  +0.00 m -> [OK - SEGURO]
[OA Frenagem -  21 comandos] Vel: -0.00 m/s (-0.0 km/h) | Parada Nec.: 0.00 m | Restante Plano:  0.08 m | Margem:  +0.08 m -> [OK - SEGURO]
```

### O que esses números mostram:
1. **Comandos caindo:** De 140 até 22 comandos, o caminhão continuou andando e consumindo o buffer do último plano recebido.
2. **O disparo no limite:** No comando 22, a distância restante ($3{,}68\text{ m}$) ficou menor que a distância necessária para parar ($3{,}78\text{ m}$). A margem ficou negativa ($-0{,}10\text{ m}$).
3. **Frenagem suave:** O OA entrou em ação na hora, reduzindo gradativamente a velocidade ($2{,}75 \to 2{,}53 \to 1{,}84 \to 0{,}89 \to 0{,}26 \to 0{,}00\text{ m/s}$), parando o caminhão com folga sem encostar no final do trajeto.

---

## 5. Resumo Rápido

| O que acontece | O que o `obstacle_avoider()` faz | Aviso emitido |
| :--- | :--- | :--- |
| **Caminho livre e plano longo** | Não mexe em nada, envia os comandos originais para o caminhão. | *Nenhum (Tudo OK)* |
| **Obstáculo detectado à frente** | Reduz as velocidades em 5% por comando até o caminho ficar seguro ou parar. | **`ASTRO_ERROR_CODE_203`** |
| **MPP parou de mandar planos** | Deixa o caminhão andar até a margem mínima e freia com segurança antes de acabar o plano. | **`ASTRO_ERROR_CODE_203`** |
| **Caminhão bateu enquanto andava** | Zera todos os comandos imediatamente para travar o veículo. | **`ASTRO_ERROR_CODE_201`** (Erro Fatal) |
| **Perda total de comunicação** | Timeout estourou sem receber sensores ou comandos. | Parada de emergência na base ackerman. |
