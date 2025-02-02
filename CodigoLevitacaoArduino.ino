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

void setup() {
  // Configura os pinos de trigger e echo
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(buttonPin, INPUT_PULLUP);

  last_time = 0;
  
  // Inicia a comunicação serial para enviar os dados ao monitor serial
  Serial.begin(9600);
}

void loop() {
  // Calcula o tempo entre loops para usar no PID
  double now = millis();
  dt = (now - last_time)/1000.00;
  last_time = now;

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

double pid(double error)
{
  double proportional = error;
  integral += error * dt;
  double derivative = (error - previous) / dt;
  previous = error;
  double output = (kp * proportional) + (ki * integral) + (kd * derivative);
  return output;
}
