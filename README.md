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

### 3.2. Como a velocidade é reduzida em 5% (`velocity_recalculate`)

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

## 4. Exemplo Prático: Validação do Tamanho do Plano na Ausência do MPP (Etapa 4 da Seção 2)

Este teste em ambiente real evidencia exatamente o funcionamento da **Etapa 4 da Seção 2** (*Verificar se o tamanho do plano tem distância suficiente para parar*): o que acontece quando o MPP é desligado ou interrompe o envio de trajetórias enquanto o veículo trafega a $10\text{ km/h}$ ($2{,}77\text{ m/s}$).

![Validação do Tamanho do Plano com MPP Desligado](img/validacao_tamanho_plano_mpp_desligado.jpg)

Abaixo está o registro extraído do terminal de frenagem correspondente à imagem acima:

```text
>>> NOVO PLANO DO MPP ENVIADO!!! 150 comandos <<<
[OA Frenagem - 113 comandos] Vel:  2.77 m/s (10.0 km/h) | Parada Nec.:  3.82 m | Restante Plano: 15.06 m | Margem: +11.23 m -> [OK - SEGURO]
[OA Frenagem -  96 comandos] Vel:  2.77 m/s (10.0 km/h) | Parada Nec.:  3.82 m | Restante Plano: 14.12 m | Margem: +10.29 m -> [OK - SEGURO]
[OA Frenagem -  80 comandos] Vel:  2.78 m/s (10.0 km/h) | Parada Nec.:  3.85 m | Restante Plano: 13.24 m | Margem:  +9.39 m -> [OK - SEGURO]
[OA Frenagem -  73 comandos] Vel:  2.78 m/s (10.0 km/h) | Parada Nec.:  3.85 m | Restante Plano: 11.85 m | Margem:  +7.99 m -> [OK - SEGURO]
[OA Frenagem -  68 comandos] Vel:  2.77 m/s (10.0 km/h) | Parada Nec.:  3.82 m | Restante Plano: 11.02 m | Margem:  +7.19 m -> [OK - SEGURO]
[OA Frenagem -  62 comandos] Vel:  2.77 m/s (10.0 km/h) | Parada Nec.:  3.82 m | Restante Plano: 10.02 m | Margem:  +6.20 m -> [OK - SEGURO]
[OA Frenagem -  57 comandos] Vel:  2.77 m/s (10.0 km/h) | Parada Nec.:  3.82 m | Restante Plano:  9.18 m | Margem:  +5.36 m -> [OK - SEGURO]
[OA Frenagem -  51 comandos] Vel:  2.77 m/s (10.0 km/h) | Parada Nec.:  3.82 m | Restante Plano:  8.21 m | Margem:  +4.38 m -> [OK - SEGURO]
[OA Frenagem -  46 comandos] Vel:  2.77 m/s (10.0 km/h) | Parada Nec.:  3.82 m | Restante Plano:  7.34 m | Margem:  +3.52 m -> [OK - SEGURO]
[OA Frenagem -  40 comandos] Vel:  2.77 m/s (10.0 km/h) | Parada Nec.:  3.82 m | Restante Plano:  6.35 m | Margem:  +2.52 m -> [OK - SEGURO]
[OA Frenagem -  35 comandos] Vel:  2.78 m/s (10.0 km/h) | Parada Nec.:  3.85 m | Restante Plano:  5.53 m | Margem:  +1.68 m -> [OK - SEGURO]
[OA Frenagem -  29 comandos] Vel:  2.77 m/s (10.0 km/h) | Parada Nec.:  3.82 m | Restante Plano:  4.54 m | Margem:  +0.72 m -> [OK - SEGURO]
[OA Frenagem -  24 comandos] Vel:  2.77 m/s (10.0 km/h) | Parada Nec.:  3.82 m | Restante Plano:  3.69 m | Margem:  -0.13 m -> [INSUFICIENTE - FRENAGEM FORÇADA]
[OA Frenagem -  21 comandos] Vel:  2.63 m/s ( 9.5 km/h) | Parada Nec.:  3.45 m | Restante Plano:  1.80 m | Margem:  -1.65 m -> [INSUFICIENTE - FRENAGEM FORÇADA]
[OA Frenagem -  21 comandos] Vel:  2.31 m/s ( 8.3 km/h) | Parada Nec.:  2.67 m | Restante Plano:  1.21 m | Margem:  -1.46 m -> [INSUFICIENTE - FRENAGEM FORÇADA]
[OA Frenagem -  21 comandos] Vel:  1.99 m/s ( 7.2 km/h) | Parada Nec.:  1.97 m | Restante Plano:  0.64 m | Margem:  -1.34 m -> [INSUFICIENTE - FRENAGEM FORÇADA]
[OA Frenagem -  21 comandos] Vel:  1.71 m/s ( 6.1 km/h) | Parada Nec.:  1.46 m | Restante Plano:  0.26 m | Margem:  -1.20 m -> [INSUFICIENTE - FRENAGEM FORÇADA]
[OA Frenagem -  21 comandos] Vel:  1.32 m/s ( 4.8 km/h) | Parada Nec.:  0.87 m | Restante Plano:  0.11 m | Margem:  -0.77 m -> [INSUFICIENTE - FRENAGEM FORÇADA]
[OA Frenagem -  21 comandos] Vel:  0.98 m/s ( 3.5 km/h) | Parada Nec.:  0.48 m | Restante Plano:  0.32 m | Margem:  -0.16 m -> [INSUFICIENTE - FRENAGEM FORÇADA]
[OA Frenagem -  21 comandos] Vel:  0.66 m/s ( 2.4 km/h) | Parada Nec.:  0.22 m | Restante Plano:  0.38 m | Margem:  +0.16 m -> [OK - SEGURO]
[OA Frenagem -  21 comandos] Vel:  0.35 m/s ( 1.3 km/h) | Parada Nec.:  0.06 m | Restante Plano:  0.32 m | Margem:  +0.26 m -> [OK - SEGURO]
[OA Frenagem -  21 comandos] Vel:  0.05 m/s ( 0.2 km/h) | Parada Nec.:  0.00 m | Restante Plano:  0.28 m | Margem:  +0.28 m -> [OK - SEGURO]
[OA Frenagem -  20 comandos] Vel: -0.00 m/s (-0.0 km/h) | Parada Nec.:  0.00 m | Restante Plano:  0.01 m | Margem:  +0.01 m -> [OK - SEGURO]
```

### O que esse comportamento evidencia:

1. **Comandos caindo:** De 113 até 24 comandos, o veículo continuou se deslocando e consumindo o buffer do último plano recebido. Isso ocorreu porque o MPP foi desligado: o que antes tinha uma taxa de atualização frequente (um novo plano com ~150 comandos chegava e renovava o buffer a cada ~2 ciclos), dessa vez se estendeu indefinidamente, fazendo com que o plano fosse progressivamente encurtado pelo consumo de tempo.
2. **O disparo no limite (Gatilho da Etapa 4):** No comando 24, a distância restante do plano ($3{,}69\text{ m}$) ficou menor que a distância necessária para parar o veículo ($3{,}82\text{ m}$). A margem de segurança ficou negativa ($-0{,}13\text{ m}$), acionando o estado `[INSUFICIENTE - FRENAGEM FORÇADA]`.
3. **Frenagem progressiva e suave (`velocity_recalculate`):** A partir do comando 24, o módulo OA entra imediatamente em ação recalculando as velocidades em 5% por comando. A velocidade decai suavemente ciclo a ciclo ($2{,}77 \to 2{,}63 \to 2{,}31 \to 1{,}99 \to 1{,}71 \to 1{,}32 \to 0{,}98 \to 0{,}66 \to 0{,}35 \to 0{,}05 \to 0{,}00\text{ m/s}$), imobilizando o veículo com segurança e mantendo rigorosamente a geometria da trajetória.

---

## 5. Resumo Rápido

| O que acontece | O que o `obstacle_avoider()` faz | Aviso emitido |
| :--- | :--- | :--- |
| **Caminho livre e plano longo** | Não mexe em nada, envia os comandos originais para o veículo. | *Nenhum (Tudo OK)* |
| **Obstáculo detectado à frente** | Reduz as velocidades em 5% por comando até o caminho ficar seguro ou parar. | **`ASTRO_ERROR_CODE_203`** |
| **MPP parou de mandar planos** | Deixa o veículo se deslocar até a margem mínima e freia com segurança antes de acabar o plano. | **`ASTRO_ERROR_CODE_203`** |
| **Caminhão bateu enquanto andava** | Zera todos os comandos imediatamente para travar o veículo. | **`ASTRO_ERROR_CODE_201`** (Erro Fatal) |

---
