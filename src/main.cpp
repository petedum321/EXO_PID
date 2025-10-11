/****************************************************************************************
 * *
 * PROGRAMA INTERATIVO DE CONTROLE DE MOTOR DC COM DRIVER BTS7960 E ESP32       *
 * *
 * Funcionalidades:                                                                    *
 * - Aguarda um valor de velocidade (-255 a 255) ser enviado pelo Monitor Serial.      *
 * - Controla a velocidade e direção do motor com base no valor recebido.              *
 * - Mede e imprime continuamente a corrente consumida pelo motor.                     *
 * *
 * Conexões de Hardware:                                                               *
 * - RPWM (Driver Pino 1) -> GPIO 26 (ESP32)                                           *
 * - LPWM (Driver Pino 2) -> GPIO 25 (ESP32)                                           *
 * - R_EN, L_EN, VCC (Driver Pinos 3, 4, 7) -> 3V3 (ESP32)                              *
 * - GND (Driver Pino 8) -> GND (ESP32) -> GND (Fonte Externa) - TERRA COMUM            *
 * - R_IS (Driver Pino 5) -> GPIO 34 (ESP32) E Resistor de 560Ω para o GND              *
 * - L_IS (Driver Pino 6) -> GPIO 35 (ESP32) E Resistor de 560Ω para o GND              *
 * *
 ****************************************************************************************/

// --- CONSTANTES E DEFINIÇÕES ---
#include <Arduino.h>

// Pinos de controle PWM do motor
const int RPWM_PIN = 26;
const int LPWM_PIN = 25;

// Pinos de entrada analógica para sensoriamento de corrente
const int R_IS_PIN = 34;
const int L_IS_PIN = 35;

// Parâmetros Físicos e do Hardware
const float RESISTOR_IS = 560.0;
const float K_ILIS = 8500.0;
const float VREF = 3.3;
const float ADC_RESOLUTION = 4095.0;

// Configurações do PWM (LEDC - ESP32)
const int PWM_FREQ = 1000;
const int PWM_CHANNEL_R = 0;
const int PWM_CHANNEL_L = 1;
const int PWM_RESOLUTION = 8;

// Protótipos das funções
void setMotorSpeed(int speed);
float getMotorCurrent();

void setup() {
  // Inicializa a comunicação serial
  Serial.begin(115200);
  Serial.println("Inicializando o Controle de Motor BTS7960...");

  // Configuração do PWM (LEDC)
  ledcSetup(PWM_CHANNEL_R, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(RPWM_PIN, PWM_CHANNEL_R);
  ledcSetup(PWM_CHANNEL_L, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(LPWM_PIN, PWM_CHANNEL_L);
  
  // Para o motor no início
  setMotorSpeed(0);

  // Instruções para o usuário
  Serial.println("\nSistema pronto.");
  Serial.println("Digite um valor de velocidade de -255 (ré máxima) a 255 (frente máxima) e pressione Enter.");
}

void loop() {
  // --- LÓGICA DE CONTROLE VIA SERIAL ---

  // Verifica se há algum dado disponível no buffer da serial
  if (Serial.available() > 0) {
    // Lê a string inteira até encontrar uma nova linha
    String command = Serial.readStringUntil('\n');
    
    // Converte a string recebida para um número inteiro
    int speed = command.toInt();

    // Garante que o valor esteja dentro dos limites (-255 a 255)
    speed = constrain(speed, -255, 255);
    
    Serial.print("Comando recebido. Ajustando velocidade para: ");
    Serial.println(speed);
    
    // Ajusta a velocidade do motor
    setMotorSpeed(speed);
  }

  // --- MONITORAMENTO CONTÍNUO DA CORRENTE ---
  
  // Mede e imprime a corrente atual
  float current = getMotorCurrent();
  Serial.print("Corrente Atual: ");
  Serial.print(current, 3); // Imprime com 3 casas decimais
  Serial.println(" A");

  // Pequeno atraso para não sobrecarregar o Monitor Serial
  delay(250);
}

/**
 * @brief Controla a velocidade e direção do motor.
 * @param speed Um valor de -255 a 255.
 */
void setMotorSpeed(int speed) {
  if (speed > 0) {
    ledcWrite(PWM_CHANNEL_L, 0);
    ledcWrite(PWM_CHANNEL_R, speed);
    Serial.print("PWM_CHANNEL_R: "); Serial.println(speed);
    Serial.print("PWM_CHANNEL_L: "); Serial.println(0);
  } else if (speed < 0) {
    ledcWrite(PWM_CHANNEL_R, 0);
    ledcWrite(PWM_CHANNEL_L, -speed);
    Serial.print("PWM_CHANNEL_R: "); Serial.println(0);
    Serial.print("PWM_CHANNEL_L: "); Serial.println(-speed);
  } else {
    ledcWrite(PWM_CHANNEL_R, 0);
    ledcWrite(PWM_CHANNEL_L, 0);
    Serial.println("Motor parado");
  }
}

/**
 * @brief Lê os pinos de sensoriamento e calcula a corrente real do motor.
 * @return A corrente do motor em Amperes (A).
 */
float getMotorCurrent() {
  // Média de 10 leituras para cada pino
  int soma_R = 0, soma_L = 0;
  for (int i = 0; i < 10; i++) {
    soma_R += analogRead(R_IS_PIN);
    soma_L += analogRead(L_IS_PIN);
    delay(2);
  }
  int raw_R_IS = soma_R / 10;
  int raw_L_IS = soma_L / 10;
  Serial.print("raw_R_IS: "); Serial.println(raw_R_IS);
  Serial.print("raw_L_IS: "); Serial.println(raw_L_IS);
  float v_is_R = raw_R_IS * (VREF / ADC_RESOLUTION);
  float v_is_L = raw_L_IS * (VREF / ADC_RESOLUTION);
  Serial.print("V_R_IS: "); Serial.println(v_is_R, 4);
  Serial.print("V_L_IS: "); Serial.println(v_is_L, 4);
  int active_raw_value = (raw_R_IS > raw_L_IS) ? raw_R_IS : raw_L_IS;
  float v_is = active_raw_value * (VREF / ADC_RESOLUTION);
  float i_is = v_is / RESISTOR_IS;
  float motor_current = i_is * K_ILIS;
  return motor_current;
}