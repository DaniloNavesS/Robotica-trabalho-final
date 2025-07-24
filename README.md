# Requisitos e Instalação

Antes de rodar os scripts Python deste projeto, é necessário instalar as seguintes dependências:

- [pyzmq](https://pypi.org/project/pyzmq/): Biblioteca para comunicação via ZeroMQ.
- [cbor2](https://pypi.org/project/cbor2/): Biblioteca para serialização de dados em CBOR.

Você pode instalar ambas de forma simples usando o pip. Execute no terminal:

```bash
pip install pyzmq cbor2
```

Se estiver usando um ambiente virtual (recomendado), ative-o antes de instalar as dependências.

---

## Funcionamento do Projeto ur3Quadradov2.py

O projeto `ur3Quadradov2.py` demonstra o controle de um robô UR3 no CoppeliaSim para desenhar um quadrado no espaço, utilizando o sistema de cinemática inversa (IK) interno do simulador. O objetivo é mostrar, de forma didática e visual, como a cinemática inversa pode ser usada para controlar manipuladores robóticos de múltiplos graus de liberdade.

### Visão Geral do Fluxo
- O script Python se conecta ao CoppeliaSim e, para cada ponto do quadrado, move o dummy alvo (target) para a posição desejada.
- O CoppeliaSim, por meio do seu solver de IK, calcula automaticamente os ângulos das juntas necessários para que o tip (extremidade do robô) alcance o target.
- O robô executa o movimento, desenhando o quadrado no ar.
- O processo é repetido para todos os pontos, formando a trajetória desejada.

### Como o IK do CoppeliaSim Funciona (Explicação Matemática)
O IK (Inverse Kinematics) é o processo de encontrar os valores das variáveis articulares (ângulos das juntas) que levam o end-effector (tip) do robô a uma posição e orientação desejadas no espaço cartesiano.

#### 1. Modelagem Cinemática
- O robô é modelado como uma cadeia de elos e juntas, cada uma com parâmetros Denavit-Hartenberg (DH) ou equivalentes.
- A pose do tip é obtida pela multiplicação sucessiva das matrizes de transformação homogênea de cada elo:
  
  \[
  T = T_1(\theta_1) \cdot T_2(\theta_2) \cdot ... \cdot T_n(\theta_n)
  \]
  onde cada \(T_i(\theta_i)\) depende do ângulo da junta \(\theta_i\).

#### 2. Problema de IK
- Dado um target (posição e orientação desejadas), o problema é encontrar o vetor de juntas \(\theta = [\theta_1, \theta_2, ..., \theta_n]\) tal que:
  
  \[
  f(\theta) = x_{desired}
  \]
  onde \(f(\theta)\) é a função cinemática direta do robô.

#### 3. Solução Numérica (Jacobiano)
- O CoppeliaSim resolve o IK numericamente, usando métodos como Pseudo-Inversa ou Damped Least Squares (DLS).
- O erro entre a pose atual e a desejada é calculado:
  
  \[
  e = x_{desired} - f(\theta)
  \]
- O Jacobiano \(J\) relaciona variações nas juntas com variações na pose do tip:
  
  \[
  \dot{x} = J(\theta) \dot{\theta}
  \]
- A atualização das juntas é feita por:
  
  - **Pseudo-inversa:**
    \[
    \Delta\theta = J^+ e
    \]
    onde \(J^+\) é a pseudo-inversa de Moore-Penrose do Jacobiano.
  - **Damped Least Squares (DLS):**
    \[
    \Delta\theta = J^T (J J^T + \lambda^2 I)^{-1} e
    \]
    onde \(\lambda\) é o fator de amortecimento, que melhora a estabilidade perto de singularidades.

#### 4. Restrições e Singularidades
- O solver do CoppeliaSim pode lidar com restrições de posição, orientação, ou ambas (pose).
- O método DLS é especialmente útil para evitar movimentos bruscos ou instabilidades quando o robô está próximo de uma configuração singular (onde o Jacobiano perde posto).
- O solver também pode considerar limites de junta, offsets, e frames intermediários automaticamente, pois opera sobre a estrutura real do modelo carregado na cena.

### Resumo Conceitual
- O projeto demonstra, na prática, como a cinemática inversa permite controlar o robô em coordenadas cartesianas, abstraindo a complexidade dos ângulos das juntas.
- O CoppeliaSim resolve o IK de forma robusta, usando métodos matemáticos clássicos de robótica, e move o robô para seguir o target de forma suave e realista.
- O usuário pode visualizar o resultado diretamente na simulação, vendo o tip do UR3 desenhar o quadrado no espaço.