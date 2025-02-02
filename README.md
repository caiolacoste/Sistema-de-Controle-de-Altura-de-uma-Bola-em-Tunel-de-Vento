# Sistema de Controle de Posicao da Bola em Túnel de Vento

## 1. Introdução

O objetivo do projeto é a montagem de um sistema de levitação, usando o fluxo de ar para suspender um objeto. Através de um sensor de profundidade, é possível deixar o objeto a uma altura desejada, utilizando um controlador PID.

#### Componentes Necessários

1. Bola de isopor;
2. Sensor Ultrassônico;
3. Potenciômetro;
4. Ventoinha;
5. Ponte H L298N;
6. Arduino Mega;
7. Tubo de acetato;
8. Papel Paraná (ou qualquer outro material para fazer a base para fixar o tubo);
9. Kit de jumpers;
10. Protoboard;
11. Fonte de tensão 12V

## 2. Metodologia

### 2.1 Montagem do sistema

O circuito montado ficou assim:

![Lab_controle2_ckt](https://github.com/user-attachments/assets/cf7edc82-1fbe-4488-84e8-0dcb5745bbd4)

A ventoinha na parte de baixo foi retirada de um cooler de computador. Esses coolers são motores BLDC, que já possuem um ESC embutido neles, de forma que sua pinagem seja (1) GND, (2) 12V, (3) PWM e (4) TACH. NO nosso caso, usamos a ponte H como interface entre o Arduino e o motor, fazendo com que apenas as portas GND e 12V sejam utilizadas, enquanto que a fonte de 12V é conectada na ponte H. A alimentação da parte de controle do circuito é feita através da porta USB do computador. O sensor ultrassônico é o único sensor do sistema, com função de medir a distância entre o topo do tubo e a bolinha de isopor, enquanto que o potenciômetro é utilizado para definidir o referencial do sistema. Além disso, foi colocado um simples botão para ligar e desligar o controle.

O sistema completo ficou assim:

![image](https://github.com/user-attachments/assets/17d2a855-327e-48a3-9fd6-ca91f05753f5)


### 2.2 Diagrama de blocos

Através do uso de um potenciômetro, foi possível deixar a bola em diferentes alturas, variando a tensão de entrada do motor, como se fosse em malha aberta. Com isso, foi gerado o gráfico mostrado na figura abaixo, em que o eixo X representa a distância captada pelo sensor ultrassônico e o eixo Y é o valor de tensão enviada para o motor.

![chart](https://github.com/user-attachments/assets/f0d21a77-2c11-4a64-b39b-e5989353e95c)

Esse gráfico é bem diferente do controle de velocidade de um motor de corrente contínua, porque não existe uma relação linear entre a altura da bola de isopor e a tensão de entrada do motor.

A justificativa para essa **não linearidade** é o fato de que, na montagem, o tubo de acetato foi fixado com cola quente em sua base e possui diversas camadas. Dessa maneira, nenhum ar é perdido ao longo do tubo, portanto a tensão para manter a bola equilibrada em qualquer altura é a mesma. O controle da altura se dá de forma a perturbar para mais ou menos em torno dessa tensão de equilíbrio, e depois de chegar na altura desejada, a tensão volta para o valor original.

Em resumo, a altura da bola pode ser controlada variando a tensão de entrada em torno de um valor de equilíbrio, e depois que a bola chegar na altura desejada, o motor volta para esse valor novamente. Visto isso, é possível desenhar o diagrama de blocos do sistema, mostrado abaixo, em que o **offset** foi calculado por meio do gráfico obtido anteriormente, valendo 98 de PWM.

![Diagrama de blocos](https://github.com/user-attachments/assets/1de0c9ae-4b7f-430d-9cf5-d2944583e991)

### 2.3 Bloco de Controle PID

Tendo o valor de Offset para o equilíbrio da bola, a próxima coisa a se fazer é **converter o erro das alturas em centímetros para um valor PWM**. Como o tamanho do tubo é de aproximadamente $57cm$, descartando o diâmetro da bola, o erro máximo que pode ocorrer vale 57, situação em que a bola está na base do tubo. Já para o valor PWM, 98 é o equilíbrio, e se for desejado chegar à base, seria utilizado o valor 0, portanto a variação de PWM é 98 e a variação de altura é 57. Com isso se chega na relação:

$$
\begin{equation}
    \frac{PWM}{altura} = \frac{98}{57} = 1.719
\end{equation}
$$

Com o erro já em PWM, o PID pôde ser implementado, segundo a fórmula:

$$
\begin{equation}
    u(t) = K_p (e(t) + \frac{1}{T_i} \int_{0}^{t} e(\tau) d\tau + T_d \frac{d}{dt} e(t))
\end{equation}
$$

A sintonia do controlador foi feita manualmente. 

O primeiro a ser ajustado foi o $K_p$. Deixando $K_i$ e $K_d$ zerados, tendo só a influência da **parcela proporcional**, o valor é aumentado desde 1, até chegar em um ponto em que o sistema esteja instável. Nesse momento diminuímos um pouco e passamos para a próxima parcela.

O próximo passo é o $K_i$. A **parcela integral** é responsável por zerar o erro em regime permanente, de forma que quanto mais tempo passa, mais efeito tem a parcela sobre o sistema. O tempo de integração $T_i$ representa o tempo que demora para a ação integral chegar no valor da ação proporcional. É usado seu inverso, porque quanto menor, mais rápido a parcela cresce. Na prática, o tempo $T_i$ começa muito grande ($K_i$ = 0) e seu valor é diminuído até chegar em um ponto em que o tempo de resposta está aceitável. 

Como a ação integrativa zera o erro, mas sacrifica um pouco o tempo de resposta, é utilizada a **ação derivativa** junto. Porém, a parcela derivativa não é recomendada em sistemas que necessitam de uma resposta rápida, podendo sacrificar a estabilidade do sistema. São mais usadas em sistemas como ajuste de temperatura, em que demora para chegar no **setpoint**. Por isso, nesse caso a parcela derivativa $K_d$ foi zerada.

No final do ajuste fino, foram utilizados os seguintes valores para as constantes:

$$
\begin{align*}
    K_p = 3.0 \\
    K_i = 0.2 \\
    K_d = 0.0 \\
\end{align*}
$$

## 3. Implementação no código

Primeiramente as variáveis são inicializadas, os pinos referenciados e as variáveis necessárias criadas. Para o controle da ventoinha, foi utilizada a biblioteca _DMPH.h_, dada sua facilidade de controle do sinal enviado para a ponte H (Ela precisa ser baixada [nesse](https://github.com/rodriguesfas/DMPH_L298N) repositório).

```cpp
#include <DMPH.h>

#define trigPin 11
#define echoPin 9
#define buttonPin A2

bool botaoAnterior = false;
bool run = false;

// Variáveis relacionadas ao PID
double dt, last_time;
double integral, previous, output = 0;
double kp = 3.0;
double ki = 0.2;
double kd = 0.0;

// Valor de altura esperada (cm)
double setpoint = 40.00;
int offsetPWM = 98;
// Conversão entre altura [cm] para tensão do motor [PWM]
double Kconversion = 1.719;

// Valores relacionados ao sensor ultrassônico
long duration;
int distance;

// Instância um objeto chamado motor, passando por parâmetros os pinos de conexão onde esta ligado o motor (pin_motor, pin_motor, pin_motor_velocidade).
DMPH motor(6,5,7);
```

A seguir os pinos são inicializados, especificando as entradas e saídas do microcontrolador. Os pinos referentes ao motor não precisam porque isso já é feito na criação do objeto da bibliteca _DMPH.h_.

```cpp
void setup() {
  // Configura os pinos de trigger e echo
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(buttonPin, INPUT_PULLUP);

  last_time = 0;
  
  // Inicia a comunicação serial para enviar os dados ao monitor serial
  Serial.begin(9600);
}
```

Abaixo é apresentada a função _pid_, que recebe o erro de entrada. Como o código não é contínua no tempo, mas funciona por meio de iterações da função _loop()_, é necessária a discretização da função anteriormente exibida. Essa discretização é feita através da variável **dt**, que calcula o tempo passado em cada iteração e multiplica pelo valor do erro, permitindo calcular a integral como a soma dos valores do erro. Depois de calculadas todas as parcelas elas são somadas e dão o retorno da função.

```cpp
double pid(double error)
{
  double proportional = error;
  integral += error * dt;
  double derivative = (error - previous) / dt;
  previous = error;
  double output = (kp * proportional) + (ki * integral) + (kd * derivative);
  return output;
}
```

Agora a função é utilizada no _loop()_. A primeira coisa a ser feita é calcular o tempo, que será usado no PID. O valor é dividido por 1000.00 para passar de milissegundos para segundos.

```cpp
void loop() {
  // Calcula o tempo entre loops para usar no PID
  double now = millis();
  dt = (now - last_time)/1000.00;
  last_time = now;
```

A seguir o valor da distância é calculado, através dos sensor ultrassônico. Isso é feito gerando um pulso no pino de _trigger_ do sensor e depois vendo quanto tempo demora para esse mesmo pulso ser captado pelo pino de _echo_. Como o sensor está no topo do tubo, no final é subtraído o valor máximo do sensor para poder ter o valor a partir da base.

```cpp
  // ---------- Aquisição do sensor ultrassônico --------------
  // Limpa o pino de trigger
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // Emite um pulso de 10 microssegundos no pino de trigger
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Lê a duração do pulso no pino de echo
  duration = pulseIn(echoPin, HIGH);
  
  // Calcula a distância em cm
  distance = duration * 0.034 / 2;
  // Sabemos que a distância até a base é 57, então usamos para retornar a altura a partir da base
  distance = 57-distance;
```

Com a distância em mãos, e o _setpoint_ declarado no começo do código, é possível calcular o erro e chamar a função de PID, convertendo o valor de centímetros para PWM antes:

```cpp
  // ----------------- Cálculo da tensão de saída ---------------------
  // Calculamos o erro em cm
  double atual = distance;
  double error = setpoint - atual;
  // Transformamos o erro em PWM
  double error_PWM = error*Kconversion;
  // Calculamos a saída do sistema
  output = pid(error_PWM);
  // Adicionamos o offset do equilíbrio
  output += offsetPWM;
```

Por fim, o botão é utilizado para facilitar reiniciar o sistema, de forma a só mandar o valor correto para o motor caso a variável _run_ esteja ligada. Depois disso são feitos alguns prints para controle do sistema.

```cpp
  // Lógica Liga/Desliga do botão
  bool botaoAtual = !digitalRead(buttonPin);
  if (botaoAtual && (botaoAtual != botaoAnterior)){
    run = !run;
  }
  botaoAnterior = botaoAtual;

  if (run){
    // Envia a potência para a ventoinha
    motor.move(output);
    // Printa os valores no monitor serial
    Serial.print("Distancia: ");
    Serial.print(distance);
    Serial.print(" PWM: ");
    Serial.println(output);
  }
  else{
    motor.move(0);
  }
}
```

## 4. Conclusão

O sistema rodando em tempo real é mostrado em vídeo, clicando [aqui](https://youtube.com/shorts/0C3vNNsVyzI?feature=share).

Como possível melhoria do sistema, para resolver o problema da não linearidade da relação altura X tensão, a solução podem ser furos, permitindo que ar vaze ao longo do tubo. Dessa maneira, quanto mais longe da ventoinha, mais ar será necessário para levantá-la, fazendo com que cada altura tenha uma velocidade correspondente. Dessa forma é possível encontrar uma função de transferência para o sistema e tornar o processo de sintonia do PID mais eficiente, utilizando a resposta ao degrau em Matlab.

# Referências

[1] [Arduino PID Controller - From Scratch](https://youtu.be/RZW1PsfgVEI?si=x77hKxyhFkN3pazl);

[2] [Levitador Neumático - Control Mecatrónico II - UCSM](https://youtu.be/LMHCa8fpCmo?si=lBqsoEQ4rMSBtc_3);

[3] [Controlador PID para sistema de levitación con arduino](https://youtu.be/t3eOe9rOqTg?si=HPt72X0eNJOwVO0x);

[4] [Arduino Project - Ball Levitation using PID controller](https://youtu.be/k0yTh2D-ypQ?si=_VlBqsFumVZsH6iZ);

[5] [PID Controller Implementation in Software - Phil's Lab #6](https://youtu.be/zOByx3Izf5U?si=XtUsFN4n_1-_-7RP)



