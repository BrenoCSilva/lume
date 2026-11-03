# 02 - Planejamento futuro e Validação de Trajetória (`obstacle_avoider`)

A principal função do [`obstacle_avoider()`](file:///home/lume/astro/src/obstacle_avoider/obstacle_avoider.cpp#L1510) é projetar no espaço cartesiano o plano de movimento enviado pelo MPP e avaliar preventivamente o risco de colisão dessa projeção futura.

---

## 1. O que o MPP envia vs O que o OA calcula

O MPP **não** envia as coordenadas cartesianas globais $(X, Y, \theta)$ do plano:

<pre>
typedef struct {
    <s>double x;</s><                              <span style="color: #6a9955;">// Calculado pelo OA (não preenchido pelo MPP)</span>
    <u><s>double y;</s></u>                               <span style="color: #6a9955;">// Calculado pelo OA (não preenchido pelo MPP)</span>
    <u><s>double theta;</s></u>                           <span style="color: #6a9955;">// Calculado pelo OA (não preenchido pelo MPP)</span>
    <u><s>int num_trailers;</s></u>                       <span style="color: #6a9955;">// Calculado pelo OA (não preenchido pelo MPP)</span>
    <u><s>double trailer_theta[MAX_NUM_TRAILERS];</s></u> <span style="color: #6a9955;">// Calculado pelo OA (não preenchido pelo MPP)</span>
    double v;                               <span style="color: #6a9955;">// Enviado pelo MPP</span>
    double phi;                             <span style="color: #6a9955;">// Enviado pelo MPP</span>
    double time;                            <span style="color: #6a9955;">// Enviado pelo MPP</span>
} astro_robot_and_trailers_motion_command_t;
</pre>

*(As variáveis **<u>sublinhadas em negrito</u>** não são preenchidas pelo MPP, cabendo ao `obstacle_avoider()` calculá-las).*

> [!NOTE]
> **Diferença de escopo entre as checagens do OA:**  
> - **Potential Collision (Emergência):** Simula uma frenagem de emergência supondo que o veículo freará mantendo o ângulo $\phi$ fixo (rodas mantidas na direção atual).  
> - **obstacle_avoider() (Preventivo):** O objetivo é fazer a simulação supondo que o veículo execute os movimentos planejados pelo MPP.

---

## 2. Núcleo de Operação do `obstacle_avoider()`

O fluxo de operação se baseia em 5 passos principais:

1. **Projeção espacial da trajetória ([`função: build_predicted_trajectory`](file:///home/lume/astro/src/obstacle_avoider/obstacle_avoider.cpp#L1471)):**  
   A predição espacial do veículo é realizada a partir da `pose_atual` vinda do `localize`. Aplicando o movimento `motion[0]` do MPP, uma nova pose é gerada e atribuída à própria `pose[0]` do plano, e de forma sequencial essa nova pose é usada como origem para a pose seguinte, calculando as coordenadas da cabine e dos reboques ao longo de todo o plano.
 
$$\mathbf{P}_{\text{nova}} = f_{\text{recalc-pos-ackerman}}(\mathbf{P}_{\text{anterior}}, v, \phi, dt)$$


2. **Avaliar colisão na `pose_atual`:**  
   Verifica se o veículo já se encontra em colisão na posição presente. Caso haja colisão e o veículo esteja em movimento ($v > 0{,}001\text{ m/s}$), as velocidades de todos os comandos do plano são setadas para zero e o sistema emite o **`ASTRO_ERROR_CODE_201`**.

3. **Avaliar colisão nas poses futuras:**  
   Varre os pontos calculados à frente contra o mapa de obstáculos e os objetos em movimento (DATMO). Caso haja colisão prevista:  
   $\rightarrow$ As velocidades do plano são reduzidas em 5% exponencialmente ([`velocity_recalculate`](file:///home/lume/astro/src/obstacle_avoider/obstacle_avoider.cpp#L1435)).  
   $\rightarrow$ Emite o erro **`ASTRO_ERROR_CODE_203`**.

4. **Verificar se o tamanho do plano tem distância suficiente para parar:**  
   Calcula a distância necessária para o veículo parar com a velocidade atual pela equação de Torricelli ($\Delta S = \frac{v^2}{2a}$). Caso o plano restante seja menor que essa distância (por exemplo, quando o MPP para de enviar novos planos):  
   $\rightarrow$ As velocidades do plano também são reduzidas em 5% exponencialmente para forçar a parada antes do término da trajetória autorizada.  
   $\rightarrow$ Emite o erro **`ASTRO_ERROR_CODE_203`**.

5. **Ajuste em ciclo até o caminho ficar seguro:**  
   Ocorrendo colisão no passo 3 ou margem insuficiente no passo 4, o passo 1 é chamado novamente com as velocidades atenuadas. A expectativa é que, se a colisão ocorreu na pose `[37]`, com as reduções espera-se chegar a um plano em que o veículo pare antes de colidir. Esse ciclo se repete até o plano ficar livre de colisão ou o veículo parar de vez.

> [!IMPORTANT]
> **Publicação do plano modificado:**  
> Sempre que o OA age sobre o plano por emergência (reduzindo a velocidade ou forçando a parada), ele verifica se a global `global_new_motion_command_need_to_be_published` não foi setada e a aciona para que seu plano recalculado seja publicado imediatamente.

---

## 3. Detalhes do fluxo

### 3.1. Como o caminho é projetado (`build_predicted_trajectory`)
A função [`astro_libcarmodel_recalc_pos_ackerman()`](file:///home/lume/astro/sharedlib/libcarmodel/ackerman_model.c) recebe a pose atual do caminhão, a velocidade $v$, o esterçamento $\phi$ e o tempo $dt$. Ela devolve a nova pose da cabine e o ângulo de cada semi-reboque atrelado. Sendo i diferente de 0:

$$\mathbf{P}_{\text{i}} = f_{\text{recalc-pos-ackerman}}(\mathbf{P}_{\text{anterior}}, v, \phi, dt)$$

Sendo a primeira iteração teremos:

$$\mathbf{P}_{\text{i=0}} = f_{\text{recalc-pos-ackerman}}(\mathbf{P}_{\text{localize}}, v, \phi, dt)$$

Com isso, o OA sabe exatamente onde cada círculo de colisão (marker) da cabine e das carretas estará localizado.

### 3.2. A Frenagem de Torricelli e a Morte do MPP
Para não depender de tempo de relógio, o OA usa a física clássica de Torricelli para saber a distância mínima de parada:

$$\Delta S_{\text{parada}} = \frac{v^2}{2 \cdot a_{\max}}$$

Onde $a_{\max}$ (`maximum_deceleration_forward`) é a capacidade máxima de frenagem da cabine configurada no arquivo `.ini`.

Ao mesmo tempo, o OA soma a distância real que o plano do MPP ainda tem pela frente:
$$D_{\text{plano}} = \text{distância somada de todos os comandos que ainda restam}$$

- **Quando o fluxo não tem conflitos:** O MPP manda planos de $\approx 18\text{ m}$ a cada $100\text{ ms}$, enquanto a $10\text{ km/h}$ o caminhão precisa de apenas $\approx 3{,}8\text{ m}$ para parar. Margem de sobra!
- **Quando o MPP morre:** Nenhum plano novo chega. O caminhão vai andando e o [`consume_motion_command_time()`](file:///home/lume/astro/src/obstacle_avoider/obstacle_avoider_main.cpp#L421) vai descartando os comandos já executados. O plano vai encolhendo: $140 \to 100 \to 50 \to 25$ comandos.
- **O momento do resgate:** Quando o plano encolhe tanto que fica menor que a distância de parada ($D_{\text{plano}} < \Delta S_{\text{parada}}$), o OA assume o controle, aciona o recálculo e freia o caminhão suavemente até parar no limite da pista autorizada.

### 3.3. Como a velocidade é reduzida em 5% (`velocity_recalculate`)

Para frear sem dar tranco no caminhão e sem causar efeito canivete com os semirreboques, a velocidade decai progressivamente a cada comando:

```cpp
void velocity_recalculate(astro_robot_ackerman_motion_command_t *motion_commands_vector, int num_motion_commands)
{
	double reduction_factor = 0.95;
	for (int i = 0; i < num_motion_commands; i++)
	{
		motion_commands_vector[i].v *= reduction_factor;
		motion_commands_vector[i].time *= 1.0 / reduction_factor;
		reduction_factor *= 0.95;

		if (motion_commands_vector[i].v < 0.05)
			motion_commands_vector[i].v = 0.0;
	}
}
```
O objetivo dessa função é fazer o veículo **desacelerar sem mudar o trajeto físico** planejado pelo MPP.

 Como o veículo se desloca em uma velocidade 5% menor(v diminui), para que percorra aquele mesmo pedaço do trajeto ele obrigatoriamente precisa de **mais tempo** ($\Delta t$ aumenta).
* **A proporção exata:** Se a velocidade cai para $95\%$ ($v \cdot 0{,}95$), o tempo de duração daquele comando aumenta na exata mesma proporção ($\text{time} \div 0{,}95$).

> [!NOTE]
> **Significado prático:** O caminho percorrido pelo veículo não muda, a mesma curva será feita, apenas em uma velocidade 5% menor do que a original.

---

## 4. Exemplo prático (Logs Reais)

Logo abaixo é mostrado o terminal de frenagem no momento em que o MPP foi desligado durante um teste a $10\text{ km/h}$:

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
3. **Frenagem :** O módulo OA como descrito em na seção 2 (4 - Verificar se o tamanho do plano tem distância suficiente para parar:), ocorre a redução gradativamente da velocidade ($2{,}75 \to 2{,}53 \to 1{,}84 \to 0{,}89 \to 0{,}26 \to 0{,}00\text{ m/s}$), parando o veículo em poucos ciclos.

---

## 5. Resumo Rápido

| O que acontece | O que o `obstacle_avoider()` faz | Aviso emitido |
| :--- | :--- | :--- |
| **Caminho livre e plano longo** | Não mexe em nada, envia os comandos originais para o veículo. | *Nenhum (Tudo OK)* |
| **Obstáculo detectado à frente** | Reduz as velocidades em 5% por comando até o caminho ficar seguro ou parar. | **`ASTRO_ERROR_CODE_203`** |
| **MPP parou de mandar planos** | Deixa o veículo se deslocar até a margem mínima e freia com segurança antes de acabar o plano. | **`ASTRO_ERROR_CODE_203`** |
| **Caminhão bateu enquanto andava** | Zera todos os comandos imediatamente para travar o veículo. | **`ASTRO_ERROR_CODE_201`** (Erro Fatal) |

---
