
#include <Arduino.h>
#include <math.h>
#include <WiFi.h>
#include <WebServer.h>

// CONFIGURAÇÃO GERAL
const int NUM_MOTORS = 4;
const int PPR = 360;

// Variáveis de controle de sequência de marcha (precisam ser declaradas antes do WebServer)
int passoAtual = 0;
int qtde_passos = 0;
unsigned long tempoInicioMovimento = 0;
bool aguardandoPosicao = false;
const float TOLERANCIA_GRAUS = 15.0;
const float TOLERANCIA_GRAUS_PARADO = 5.0;
const float tol_deg = 15.0f;

// --- WiFi/WebServer ---
const char* ssid = "ESP32-AP";
const char* password = "12345678";
WebServer server(80);

// Variáveis de controle de marcha via WiFi
bool marcha_ativa = false;
String perfil = "nenhum";

// CORS
void addCorsHeaders() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    server.sendHeader("Access-Control-Allow-Headers", "Content-Type, Origin, X-Requested-With, Accept");
}
void handleOptions() {
    addCorsHeaders();
    server.send(204);
}

void startComPerfil(String nomePerfil) {
    perfil = nomePerfil;
    marcha_ativa = true;
    passoAtual = 0;
    tempoInicioMovimento = 0;
    aguardandoPosicao = false;
    addCorsHeaders();
    server.send(200, "text/plain", "Marcha iniciada: " + perfil);
}
void handleStop() {
    marcha_ativa = false;

    // --- ADICIONADO ---
    // Reseta o estado da máquina de estados da marcha para um início limpo na próxima vez.
    passoAtual = 0;
    tempoInicioMovimento = 0;
    aguardandoPosicao = false;
    // ------------------

    addCorsHeaders();
    server.send(200, "text/plain", "Marcha parada");
}
void handleStatus() {
    String statusJson = "{";
    statusJson += "\"perfil\": \"" + perfil + "\",";
    statusJson += "\"marcha_ativa\": " + String(marcha_ativa ? "true" : "false") + ",";
    statusJson += "\"passoAtual\": " + String(qtde_passos);
    statusJson += "}";
    addCorsHeaders();
    server.send(200, "application/json", statusJson);
}
// Motor 0 (Quadril Direito)
#define RPWM0_PIN 33
#define LPWM0_PIN 32
#define REN0_PIN  0
#define LEN0_PIN  0
#define ENC0_A_PIN 25
#define ENC0_B_PIN 26

// Motor 1 (Joelho Direito)
#define RPWM1_PIN 13
#define LPWM1_PIN 12
#define REN1_PIN  0
#define LEN1_PIN  0
#define ENC1_A_PIN 27
#define ENC1_B_PIN 14
#define IS1_PIN 0

// Motor 2 (Quadril Esquerdo)
#define RPWM2_PIN 19
#define LPWM2_PIN 21
#define REN2_PIN  0
#define LEN2_PIN  0
#define ENC2_A_PIN 22
#define ENC2_B_PIN 18

// Motor 3 (Joelho Esquerdo)
#define RPWM3_PIN 4
#define LPWM3_PIN 16
#define REN3_PIN  0
#define LEN3_PIN  0
#define ENC3_A_PIN 5
#define ENC3_B_PIN 17

#define PID_DESCIDA 0
#define PID_SUBIDA  1

struct Motor {
    // Pinos (Hardware)
    int rpwm_pin, lpwm_pin, ren_pin, len_pin;
    int enc_a_pin, enc_b_pin;
    int id; 
    
    int current_sense_pin; // Pino para leitura da corrente

    volatile long encCount = 0;
    volatile long encOffset = 0;
 
    float target_deg = 0.0f;
    float setpoint_deg = 0.0f;
    float current_speed_dps = 0.0f;
    //Estados PID
    float Kp_up = 1.5f, Ki_up = 0.5f, Kd_up = 1.5f;
    float Kp_down = 0.2f, Ki_down = 0.1f, Kd_down = 2.0f;
    float Kb = 1.0f;
    float iTerm = 0.0f;
    float pv_prev = 0.0f; 

    uint32_t lastUpdate = 0;

    int last_duty_sign = 0;            
    bool is_in_dead_time = false;       
    unsigned long dead_time_start = 0; 
    int modoPidAtual = PID_DESCIDA;
};

struct Passo {
    float motor0_target_deg;
    float motor1_target_deg;
    float motor2_target_deg;
    float motor3_target_deg;
    unsigned long duracao_ms;
    int ganhoMotor0;
    int ganhoMotor1;
    int ganhoMotor2;
    int ganhoMotor3;
};

//Marcha
Passo sequenciaDePassos[] = {
//m0_Alvo, M1_Alvo, M2_Alvo, M3_Alvo. Duranção, PID M0, PID M1, PID M2, PID M3
    {0.0, 40.0, 0.0, 0.0, 2000, PID_DESCIDA, PID_DESCIDA, PID_DESCIDA, PID_DESCIDA},            // Movimento 1
    {40.0, 40.0, 0.0, 0.0, 1200, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA}, //1° Passo    // Movimento 2
    {40.0, 40.0, 30.0, -30.0, 1200, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA, PID_DESCIDA} ,          // Movimento 3
    {40.0, 40.0, 60.0, -30.0, 1200, PID_SUBIDA, PID_SUBIDA, PID_DESCIDA, PID_SUBIDA},           // Movimento 4
    {0.0, 0.0, 60.0, -30.0, 800, PID_SUBIDA, PID_SUBIDA, PID_DESCIDA, PID_SUBIDA},              // Movimento 5
    {0.0, 0.0, 60.0, 0.0, 1200, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA},                // Movimento 6
    {0.0, 0.0, 0.0, 0.0, 2000, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA },                // Movimento 7
    {0.0, 0.0, 0.0, 40.0, 1200, PID_SUBIDA, PID_SUBIDA, PID_DESCIDA, PID_DESCIDA},              // Movimento 8
    {0.0, 0.0, 40.0, 40.0, 1200, PID_SUBIDA, PID_SUBIDA, PID_DESCIDA, PID_DESCIDA}, //2° Passo  // Movimento 9
    {0.0, -40.0, 40.0, 40.0,  1200, PID_DESCIDA, PID_SUBIDA, PID_DESCIDA, PID_SUBIDA},          // Movimento 10
    {30.0, -40.0, 40.0, 40.0,  1000, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA},           // Movimento 11
    {40.0, -40.0, 0.0, 0.0, 1200, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA},              // Movimento 12
    {0.0, 0.0, 0.0, 0.0, 1000, PID_DESCIDA, PID_SUBIDA, PID_DESCIDA, PID_SUBIDA }               // Movimento 13
};

//Quadril Esquerdo
Passo sequenciaDePassos_quadril_esquerdo[] = {
//m0_Alvo, M1_Alvo, M2_Alvo, M3_Alvo. Duranção, PID M0, PID M1, PID M2, PID M3
    {0.0, 40.0, 0.0, 0.0, 2000, PID_DESCIDA, PID_DESCIDA, PID_DESCIDA, PID_DESCIDA},            // Movimento 1
    {0.0, 40.0, 0.0, 0.0, 100, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA},                // Movimento 2
    {0.0, 40.0, 0.0, 0.0, 100, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA, PID_DESCIDA} ,              // Movimento 3
    {0.0, 40.0, 0.0, 0.0, 100, PID_SUBIDA, PID_SUBIDA, PID_DESCIDA, PID_SUBIDA},              // Movimento 4
    {0.0, 0.0, 0.0, 0.0, 1500, PID_SUBIDA, PID_SUBIDA, PID_DESCIDA, PID_SUBIDA},                // Movimento 5
    {0.0, 0.0, 0.0, 0.0, 100, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA},                 // Movimento 6
    {0.0, 0.0, 0.0, 0.0, 100, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA },                // Movimento 7
    {0.0, 0.0, 0.0, 0.0, 100, PID_SUBIDA, PID_SUBIDA, PID_DESCIDA, PID_DESCIDA},               // Movimento 8
    {0.0, 0.0, 0.0, 0.0, 100, PID_SUBIDA, PID_SUBIDA, PID_DESCIDA, PID_DESCIDA},               // Movimento 9
    {0.0, -40.0, 0.0, 0.0,  1500, PID_DESCIDA, PID_SUBIDA, PID_DESCIDA, PID_SUBIDA},           // Movimento 10
    {0.0, -40.0, 0.0, 0.0,  100, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA},             // Movimento 11
    {0.0, -40.0, 0.0, 0.0, 100, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA},              // Movimento 12
    {0.0, 0.0, 0.0, 0.0, 1500, PID_DESCIDA, PID_SUBIDA, PID_DESCIDA, PID_SUBIDA }              // Movimento 13
};

//Quadril Direito
Passo sequenciaDePassos_quadril_direito[] = {
//m0_Alvo, M1_Alvo, M2_Alvo, M3_Alvo. Duranção, PID M0, PID M1, PID M2, PID M3
    {0.0, 0.0, 0.0, 0.0, 100, PID_DESCIDA, PID_DESCIDA, PID_DESCIDA, PID_DESCIDA},            // Movimento 1
    {0.0, 0.0, 0.0, 0.0, 100, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA},                // Movimento 2
    {0.0, 0.0, 0.0, -30.0, 1500, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA, PID_DESCIDA} ,           // Movimento 3
    {0.0, 0.0, 0.0, -30.0, 100, PID_SUBIDA, PID_SUBIDA, PID_DESCIDA, PID_SUBIDA},            // Movimento 4
    {0.0, 0.0, 0.0, -30.0, 100, PID_SUBIDA, PID_SUBIDA, PID_DESCIDA, PID_SUBIDA},              // Movimento 5
    {0.0, 0.0, 0.0, 0.0, 1500, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA},                 // Movimento 6
    {0.0, 0.0, 0.0, 0.0, 100, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA },                // Movimento 7
    {0.0, 0.0, 0.0, 40.0, 100, PID_SUBIDA, PID_SUBIDA, PID_DESCIDA, PID_DESCIDA},              // Movimento 8
    {0.0, 0.0, 0.0, 40.0, 100, PID_SUBIDA, PID_SUBIDA, PID_DESCIDA, PID_DESCIDA},             // Movimento 9
    {0.0, 0.0, 0.0, 40.0,  100, PID_DESCIDA, PID_SUBIDA, PID_DESCIDA, PID_SUBIDA},             // Movimento 10
    {0.0, 0.0, 0.0, 40.0,  100, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA},              // Movimento 11
    {0.0, 0.0, 0.0, 0.0, 1500, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA},                // Movimento 12
    {0.0, 0.0, 0.0, 0.0, 100, PID_DESCIDA, PID_SUBIDA, PID_DESCIDA, PID_SUBIDA }               // Movimento 13
};

//Joelho Esquerdo
Passo sequenciaDePassos_joelho_esquerdo[] = {
//m0_Alvo, M1_Alvo, M2_Alvo, M3_Alvo. Duranção, PID M0, PID M1, PID M2, PID M3
    {0.0, 0.0, 0.0, 0.0, 100, PID_DESCIDA, PID_DESCIDA, PID_DESCIDA, PID_DESCIDA},            // Movimento 1
    {40.0, 0.0, 0.0, 0.0, 1500, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA}, //1° Passo    // Movimento 2
    {40.0, 0.0, 0.0, 0.0, 100, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA, PID_DESCIDA} ,          // Movimento 3
    {40.0, 0.0, 0.0, 0.0, 100, PID_SUBIDA, PID_SUBIDA, PID_DESCIDA, PID_SUBIDA},           // Movimento 4
    {0.0, 0.0, 0.0, 0.0, 1500, PID_SUBIDA, PID_SUBIDA, PID_DESCIDA, PID_SUBIDA},              // Movimento 5
    {0.0, 0.0, 0.0, 0.0, 100, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA},                // Movimento 6
    {0.0, 0.0, 0.0, 0.0, 100, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA },                // Movimento 7
    {0.0, 0.0, 0.0, 0.0, 100, PID_SUBIDA, PID_SUBIDA, PID_DESCIDA, PID_DESCIDA},              // Movimento 8
    {0.0, 0.0, 0.0, 0.0, 100, PID_SUBIDA, PID_SUBIDA, PID_DESCIDA, PID_DESCIDA}, //2° Passo  // Movimento 9
    {0.0, 0.0, 0.0, 0.0, 100, PID_DESCIDA, PID_SUBIDA, PID_DESCIDA, PID_SUBIDA},          // Movimento 10
    {30.0, 0.0, 0.0, 0.0, 1500, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA},           // Movimento 11
    {40.0, 0.0, 0.0, 0.0, 1500, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA},              // Movimento 12
    {0.0, 0.0, 0.0, 0.0, 1500, PID_DESCIDA, PID_SUBIDA, PID_DESCIDA, PID_SUBIDA }               // Movimento 13
};

//Joelho Direito
Passo sequenciaDePassos_joelho_direito[] = {
//m0_Alvo, M1_Alvo, M2_Alvo, M3_Alvo. Duranção, PID M0, PID M1, PID M2, PID M3
    {0.0, 0.0, 0.0, 0.0, 100, PID_DESCIDA, PID_DESCIDA, PID_DESCIDA, PID_DESCIDA},            // Movimento 1
    {0.0, 0.0, 40.0, 0.0, 1500, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA}, //1° Passo    // Movimento 2
    {0.0, 0.0, 40.0, 0.0, 100, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA, PID_DESCIDA} ,          // Movimento 3
    {0.0, 0.0, 40.0, 0.0, 100, PID_SUBIDA, PID_SUBIDA, PID_DESCIDA, PID_SUBIDA},           // Movimento 4
    {0.0, 0.0, 0.0, 0.0, 1500, PID_SUBIDA, PID_SUBIDA, PID_DESCIDA, PID_SUBIDA},              // Movimento 5
    {0.0, 0.0, 0.0, 0.0, 100, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA},                // Movimento 6
    {0.0, 0.0, 0.0, 0.0, 100, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA },                // Movimento 7
    {0.0, 0.0, 0.0, 0.0, 100, PID_SUBIDA, PID_SUBIDA, PID_DESCIDA, PID_DESCIDA},              // Movimento 8
    {0.0, 0.0, 0.0, 0.0, 100, PID_SUBIDA, PID_SUBIDA, PID_DESCIDA, PID_DESCIDA}, //2° Passo  // Movimento 9
    {0.0, 0.0, 0.0, 0.0, 100, PID_DESCIDA, PID_SUBIDA, PID_DESCIDA, PID_SUBIDA},          // Movimento 10
    {0.0, 0.0, 30.0, 0.0, 1500, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA},           // Movimento 11
    {0.0, 0.0, 40.0, 0.0, 1500, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA, PID_SUBIDA},              // Movimento 12
    {0.0, 0.0, 0.0, 0.0, 1500, PID_DESCIDA, PID_SUBIDA, PID_DESCIDA, PID_SUBIDA }               // Movimento 13
};


const int NUM_PASSOS = sizeof(sequenciaDePassos) / sizeof(Passo);
// Variáveis de controle de sequência de marcha são declaradas no início do arquivo

// Variáveis para guardar a posição inicial de cada motor no início da transição
float motor_pos_inicial[NUM_MOTORS]={0.0, 0.0, 0.0, 0.0};

Motor all_motors[NUM_MOTORS];

void motorSet(Motor &motor, int duty);
float readEncoderDeg(Motor &motor);
float angleError(float sp_deg, float pv_deg);
void pidStep(Motor &motor);
void motionProfilerStep(Motor &motor);

void IRAM_ATTR encoder_isr_motor0() { all_motors[0].encCount += (digitalRead(all_motors[0].enc_a_pin) == digitalRead(all_motors[0].enc_b_pin)) ? 1 : -1; }
void IRAM_ATTR encoder_isr_motor1() { all_motors[1].encCount += (digitalRead(all_motors[1].enc_a_pin) == digitalRead(all_motors[1].enc_b_pin)) ? 1 : -1; }
void IRAM_ATTR encoder_isr_motor2() { all_motors[2].encCount += (digitalRead(all_motors[2].enc_a_pin) == digitalRead(all_motors[2].enc_b_pin)) ? 1 : -1; }
void IRAM_ATTR encoder_isr_motor3() { all_motors[3].encCount += (digitalRead(all_motors[3].enc_a_pin) == digitalRead(all_motors[3].enc_b_pin)) ? 1 : -1; }

const float RESISTOR_SENSE_OHMS = 10000.0; // Valor em Ohms do seu resistor (Rsense)
const float PROPORCAO_CORRENTE = 8500.0; // Proporção I_motor / I_is do chip

const float R1_DIVISOR = 20000.0; // Resistor de 20kΩ
const float R2_DIVISOR = 10000.0; // Resistor de 10kΩ

void motorControlTask(void *pvParameters) {
    int motor_id = *(int*)pvParameters; 
    Motor &motor = all_motors[motor_id];
    const uint32_t Ts_ms = 10;
    TickType_t xLastWakeTime = xTaskGetTickCount(); 
    Serial.printf("Task para Motor %d iniciada no Nucleo %d\n", motor_id, xPortGetCoreID());
    const float max_speed_dps = 360.0f;
    for (;;) {
        
        const float max_step = max_speed_dps * (Ts_ms / 1000.0f);
        float error_to_target = angleError(motor.target_deg, motor.setpoint_deg);
        float step = constrain(error_to_target, -max_step, max_step);
        motor.setpoint_deg += step;

        pidStep(motor);
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Ts_ms));
    }
}

//Movimento Quadril Esquerdo

void movimento_suave_quadril_esquerdo() {
    if (aguardandoPosicao) {
        Passo target = sequenciaDePassos_quadril_esquerdo[passoAtual];

        float posAtualM0 = readEncoderDeg(all_motors[0]);
        float posAtualM1 = readEncoderDeg(all_motors[1]);
        float posAtualM2 = readEncoderDeg(all_motors[2]);
        float posAtualM3 = readEncoderDeg(all_motors[3]);
        bool motor0_chegou = fabsf(angleError(target.motor0_target_deg, posAtualM0)) <= TOLERANCIA_GRAUS;
        bool motor1_chegou = fabsf(angleError(target.motor1_target_deg, posAtualM1)) <= TOLERANCIA_GRAUS;
        bool motor2_chegou = fabsf(angleError(target.motor2_target_deg, posAtualM2)) <= TOLERANCIA_GRAUS;
        bool motor3_chegou = fabsf(angleError(target.motor3_target_deg, posAtualM3)) <= TOLERANCIA_GRAUS;

        all_motors[0].modoPidAtual = target.ganhoMotor0;
        all_motors[1].modoPidAtual = target.ganhoMotor1;
        all_motors[2].modoPidAtual = target.ganhoMotor2;
        all_motors[3].modoPidAtual = target.ganhoMotor3;
        
        if (motor0_chegou && motor1_chegou && motor2_chegou && motor3_chegou) {
            Serial.printf(">>> POSICAO CONFIRMADA! M0=%.1f, M1=%.1f, M2=%.1f, M3=%.1f <<<\n", posAtualM0, posAtualM1, posAtualM2, posAtualM3);
            
            aguardandoPosicao = false; 

            motor_pos_inicial[0] = readEncoderDeg(all_motors[0]);
            motor_pos_inicial[1] = readEncoderDeg(all_motors[1]);
            motor_pos_inicial[2] = readEncoderDeg(all_motors[2]);
            motor_pos_inicial[3] = readEncoderDeg(all_motors[3]);

            passoAtual++;
            if (passoAtual >= NUM_PASSOS) {
                passoAtual = 0;
            }
            
            tempoInicioMovimento = millis();
            
            Serial.printf("--- INICIANDO PROXIMO PASSO --- \nPasso: %d, Ponto de Partida: M0=%.1f, M1=%.1f, M2=%.1f, M3=%.1f\n", passoAtual, motor_pos_inicial[0], motor_pos_inicial[1], motor_pos_inicial[2], motor_pos_inicial[3]);
        }

        return; 
    }


    if (tempoInicioMovimento == 0) {
        motor_pos_inicial[0] = readEncoderDeg(all_motors[0]);
        motor_pos_inicial[1] = readEncoderDeg(all_motors[1]);
        motor_pos_inicial[2] = readEncoderDeg(all_motors[2]);
        motor_pos_inicial[3] = readEncoderDeg(all_motors[3]);
        tempoInicioMovimento = millis();
        Serial.printf("\n--- INICIANDO SEQUENCIA --- \nPasso: %d, Ponto de Partida: M0=%.1f, M1=%.1f, M2=%.1f, M3=%.1f\n", passoAtual, motor_pos_inicial[0], motor_pos_inicial[1], motor_pos_inicial[2], motor_pos_inicial[3]);
    }

    Passo target = sequenciaDePassos_quadril_esquerdo[passoAtual];
    unsigned long tempoDecorrido = millis() - tempoInicioMovimento;

    if (tempoDecorrido < target.duracao_ms) {
        // Interpola suavemente enquanto o tempo não acaba
        float progresso = (float)tempoDecorrido / (float)target.duracao_ms;
        float fator = (1.0 - cos(progresso * PI)) / 2.0;
        all_motors[0].target_deg = motor_pos_inicial[0] + (target.motor0_target_deg - motor_pos_inicial[0]) * fator;
        all_motors[1].target_deg = motor_pos_inicial[1] + (target.motor1_target_deg - motor_pos_inicial[1]) * fator;
        all_motors[2].target_deg = motor_pos_inicial[2] + (target.motor2_target_deg - motor_pos_inicial[2]) * fator;
        all_motors[3].target_deg = motor_pos_inicial[3] + (target.motor3_target_deg - motor_pos_inicial[3]) * fator;
    } else {
        // O tempo acabou! Hora de entrar no modo de "espera".
        Serial.printf("!!! Passo %d TEMPO CONCLUIDO. Alvo Final: M0=%.1f, M1=%.1f. Aguardando posicao...\n", passoAtual, target.motor0_target_deg, target.motor1_target_deg);
        
        // Garante que o alvo final seja cravado para o PID ter uma referência fixa
        all_motors[0].target_deg = target.motor0_target_deg;
        all_motors[1].target_deg = target.motor1_target_deg;
        all_motors[2].target_deg = target.motor2_target_deg;
        all_motors[3].target_deg = target.motor3_target_deg;
        
        aguardandoPosicao = true; // Ativa a flag de espera
    }
}


//Movimento Quadril Direito

void movimento_suave_quadril_direito() {
    if (aguardandoPosicao) {
        Passo target = sequenciaDePassos_quadril_direito[passoAtual];

        float posAtualM0 = readEncoderDeg(all_motors[0]);
        float posAtualM1 = readEncoderDeg(all_motors[1]);
        float posAtualM2 = readEncoderDeg(all_motors[2]);
        float posAtualM3 = readEncoderDeg(all_motors[3]);
        bool motor0_chegou = fabsf(angleError(target.motor0_target_deg, posAtualM0)) <= TOLERANCIA_GRAUS;
        bool motor1_chegou = fabsf(angleError(target.motor1_target_deg, posAtualM1)) <= TOLERANCIA_GRAUS;
        bool motor2_chegou = fabsf(angleError(target.motor2_target_deg, posAtualM2)) <= TOLERANCIA_GRAUS;
        bool motor3_chegou = fabsf(angleError(target.motor3_target_deg, posAtualM3)) <= TOLERANCIA_GRAUS;

        all_motors[0].modoPidAtual = target.ganhoMotor0;
        all_motors[1].modoPidAtual = target.ganhoMotor1;
        all_motors[2].modoPidAtual = target.ganhoMotor2;
        all_motors[3].modoPidAtual = target.ganhoMotor3;
        
        if (motor0_chegou && motor1_chegou && motor2_chegou && motor3_chegou) {
            Serial.printf(">>> POSICAO CONFIRMADA! M0=%.1f, M1=%.1f, M2=%.1f, M3=%.1f <<<\n", posAtualM0, posAtualM1, posAtualM2, posAtualM3);
            
            aguardandoPosicao = false; 

            motor_pos_inicial[0] = readEncoderDeg(all_motors[0]);
            motor_pos_inicial[1] = readEncoderDeg(all_motors[1]);
            motor_pos_inicial[2] = readEncoderDeg(all_motors[2]);
            motor_pos_inicial[3] = readEncoderDeg(all_motors[3]);

            passoAtual++;
            if (passoAtual >= NUM_PASSOS) {
                passoAtual = 0;
            }
            
            tempoInicioMovimento = millis();
            
            Serial.printf("--- INICIANDO PROXIMO PASSO --- \nPasso: %d, Ponto de Partida: M0=%.1f, M1=%.1f, M2=%.1f, M3=%.1f\n", passoAtual, motor_pos_inicial[0], motor_pos_inicial[1], motor_pos_inicial[2], motor_pos_inicial[3]);
        }

        return; 
    }


    if (tempoInicioMovimento == 0) {
        motor_pos_inicial[0] = readEncoderDeg(all_motors[0]);
        motor_pos_inicial[1] = readEncoderDeg(all_motors[1]);
        motor_pos_inicial[2] = readEncoderDeg(all_motors[2]);
        motor_pos_inicial[3] = readEncoderDeg(all_motors[3]);
        tempoInicioMovimento = millis();
        Serial.printf("\n--- INICIANDO SEQUENCIA --- \nPasso: %d, Ponto de Partida: M0=%.1f, M1=%.1f, M2=%.1f, M3=%.1f\n", passoAtual, motor_pos_inicial[0], motor_pos_inicial[1], motor_pos_inicial[2], motor_pos_inicial[3]);
    }

    Passo target = sequenciaDePassos_quadril_direito[passoAtual];
    unsigned long tempoDecorrido = millis() - tempoInicioMovimento;

    if (tempoDecorrido < target.duracao_ms) {
        // Interpola suavemente enquanto o tempo não acaba
        float progresso = (float)tempoDecorrido / (float)target.duracao_ms;
        float fator = (1.0 - cos(progresso * PI)) / 2.0;
        all_motors[0].target_deg = motor_pos_inicial[0] + (target.motor0_target_deg - motor_pos_inicial[0]) * fator;
        all_motors[1].target_deg = motor_pos_inicial[1] + (target.motor1_target_deg - motor_pos_inicial[1]) * fator;
        all_motors[2].target_deg = motor_pos_inicial[2] + (target.motor2_target_deg - motor_pos_inicial[2]) * fator;
        all_motors[3].target_deg = motor_pos_inicial[3] + (target.motor3_target_deg - motor_pos_inicial[3]) * fator;
    } else {
        // O tempo acabou! Hora de entrar no modo de "espera".
        Serial.printf("!!! Passo %d TEMPO CONCLUIDO. Alvo Final: M0=%.1f, M1=%.1f. Aguardando posicao...\n", passoAtual, target.motor0_target_deg, target.motor1_target_deg);
        
        // Garante que o alvo final seja cravado para o PID ter uma referência fixa
        all_motors[0].target_deg = target.motor0_target_deg;
        all_motors[1].target_deg = target.motor1_target_deg;
        all_motors[2].target_deg = target.motor2_target_deg;
        all_motors[3].target_deg = target.motor3_target_deg;
        
        aguardandoPosicao = true; // Ativa a flag de espera
    }
}


//Movimento Joelho Esquerdo

void movimento_suave_joelho_esquerdo() {
    if (aguardandoPosicao) {
        Passo target = sequenciaDePassos_joelho_esquerdo[passoAtual];

        float posAtualM0 = readEncoderDeg(all_motors[0]);
        float posAtualM1 = readEncoderDeg(all_motors[1]);
        float posAtualM2 = readEncoderDeg(all_motors[2]);
        float posAtualM3 = readEncoderDeg(all_motors[3]);
        bool motor0_chegou = fabsf(angleError(target.motor0_target_deg, posAtualM0)) <= TOLERANCIA_GRAUS;
        bool motor1_chegou = fabsf(angleError(target.motor1_target_deg, posAtualM1)) <= TOLERANCIA_GRAUS;
        bool motor2_chegou = fabsf(angleError(target.motor2_target_deg, posAtualM2)) <= TOLERANCIA_GRAUS;
        bool motor3_chegou = fabsf(angleError(target.motor3_target_deg, posAtualM3)) <= TOLERANCIA_GRAUS;

        all_motors[0].modoPidAtual = target.ganhoMotor0;
        all_motors[1].modoPidAtual = target.ganhoMotor1;
        all_motors[2].modoPidAtual = target.ganhoMotor2;
        all_motors[3].modoPidAtual = target.ganhoMotor3;
        
        if (motor0_chegou && motor1_chegou && motor2_chegou && motor3_chegou) {
            Serial.printf(">>> POSICAO CONFIRMADA! M0=%.1f, M1=%.1f, M2=%.1f, M3=%.1f <<<\n", posAtualM0, posAtualM1, posAtualM2, posAtualM3);
            
            aguardandoPosicao = false; 

            motor_pos_inicial[0] = readEncoderDeg(all_motors[0]);
            motor_pos_inicial[1] = readEncoderDeg(all_motors[1]);
            motor_pos_inicial[2] = readEncoderDeg(all_motors[2]);
            motor_pos_inicial[3] = readEncoderDeg(all_motors[3]);

            passoAtual++;
            if (passoAtual >= NUM_PASSOS) {
                passoAtual = 0;
            }
            
            tempoInicioMovimento = millis();
            
            Serial.printf("--- INICIANDO PROXIMO PASSO --- \nPasso: %d, Ponto de Partida: M0=%.1f, M1=%.1f, M2=%.1f, M3=%.1f\n", passoAtual, motor_pos_inicial[0], motor_pos_inicial[1], motor_pos_inicial[2], motor_pos_inicial[3]);
        }

        return; 
    }


    if (tempoInicioMovimento == 0) {
        motor_pos_inicial[0] = readEncoderDeg(all_motors[0]);
        motor_pos_inicial[1] = readEncoderDeg(all_motors[1]);
        motor_pos_inicial[2] = readEncoderDeg(all_motors[2]);
        motor_pos_inicial[3] = readEncoderDeg(all_motors[3]);
        tempoInicioMovimento = millis();
        Serial.printf("\n--- INICIANDO SEQUENCIA --- \nPasso: %d, Ponto de Partida: M0=%.1f, M1=%.1f, M2=%.1f, M3=%.1f\n", passoAtual, motor_pos_inicial[0], motor_pos_inicial[1], motor_pos_inicial[2], motor_pos_inicial[3]);
    }

    Passo target = sequenciaDePassos_joelho_esquerdo[passoAtual];
    unsigned long tempoDecorrido = millis() - tempoInicioMovimento;

    if (tempoDecorrido < target.duracao_ms) {
        // Interpola suavemente enquanto o tempo não acaba
        float progresso = (float)tempoDecorrido / (float)target.duracao_ms;
        float fator = (1.0 - cos(progresso * PI)) / 2.0;
        all_motors[0].target_deg = motor_pos_inicial[0] + (target.motor0_target_deg - motor_pos_inicial[0]) * fator;
        all_motors[1].target_deg = motor_pos_inicial[1] + (target.motor1_target_deg - motor_pos_inicial[1]) * fator;
        all_motors[2].target_deg = motor_pos_inicial[2] + (target.motor2_target_deg - motor_pos_inicial[2]) * fator;
        all_motors[3].target_deg = motor_pos_inicial[3] + (target.motor3_target_deg - motor_pos_inicial[3]) * fator;
    } else {
        // O tempo acabou! Hora de entrar no modo de "espera".
        Serial.printf("!!! Passo %d TEMPO CONCLUIDO. Alvo Final: M0=%.1f, M1=%.1f. Aguardando posicao...\n", passoAtual, target.motor0_target_deg, target.motor1_target_deg);
        
        // Garante que o alvo final seja cravado para o PID ter uma referência fixa
        all_motors[0].target_deg = target.motor0_target_deg;
        all_motors[1].target_deg = target.motor1_target_deg;
        all_motors[2].target_deg = target.motor2_target_deg;
        all_motors[3].target_deg = target.motor3_target_deg;
        
        aguardandoPosicao = true; // Ativa a flag de espera
    }
}


//Movimento Joelho Direito

void movimento_suave_joelho_direito() {
    if (aguardandoPosicao) {
        Passo target = sequenciaDePassos_joelho_direito[passoAtual];

        float posAtualM0 = readEncoderDeg(all_motors[0]);
        float posAtualM1 = readEncoderDeg(all_motors[1]);
        float posAtualM2 = readEncoderDeg(all_motors[2]);
        float posAtualM3 = readEncoderDeg(all_motors[3]);
        bool motor0_chegou = fabsf(angleError(target.motor0_target_deg, posAtualM0)) <= TOLERANCIA_GRAUS;
        bool motor1_chegou = fabsf(angleError(target.motor1_target_deg, posAtualM1)) <= TOLERANCIA_GRAUS;
        bool motor2_chegou = fabsf(angleError(target.motor2_target_deg, posAtualM2)) <= TOLERANCIA_GRAUS;
        bool motor3_chegou = fabsf(angleError(target.motor3_target_deg, posAtualM3)) <= TOLERANCIA_GRAUS;

        all_motors[0].modoPidAtual = target.ganhoMotor0;
        all_motors[1].modoPidAtual = target.ganhoMotor1;
        all_motors[2].modoPidAtual = target.ganhoMotor2;
        all_motors[3].modoPidAtual = target.ganhoMotor3;
        
        if (motor0_chegou && motor1_chegou && motor2_chegou && motor3_chegou) {
            Serial.printf(">>> POSICAO CONFIRMADA! M0=%.1f, M1=%.1f, M2=%.1f, M3=%.1f <<<\n", posAtualM0, posAtualM1, posAtualM2, posAtualM3);
            
            aguardandoPosicao = false; 

            motor_pos_inicial[0] = readEncoderDeg(all_motors[0]);
            motor_pos_inicial[1] = readEncoderDeg(all_motors[1]);
            motor_pos_inicial[2] = readEncoderDeg(all_motors[2]);
            motor_pos_inicial[3] = readEncoderDeg(all_motors[3]);

            passoAtual++;
            if (passoAtual >= NUM_PASSOS) {
                passoAtual = 0;
            }
            
            tempoInicioMovimento = millis();
            
            Serial.printf("--- INICIANDO PROXIMO PASSO --- \nPasso: %d, Ponto de Partida: M0=%.1f, M1=%.1f, M2=%.1f, M3=%.1f\n", passoAtual, motor_pos_inicial[0], motor_pos_inicial[1], motor_pos_inicial[2], motor_pos_inicial[3]);
        }

        return; 
    }


    if (tempoInicioMovimento == 0) {
        motor_pos_inicial[0] = readEncoderDeg(all_motors[0]);
        motor_pos_inicial[1] = readEncoderDeg(all_motors[1]);
        motor_pos_inicial[2] = readEncoderDeg(all_motors[2]);
        motor_pos_inicial[3] = readEncoderDeg(all_motors[3]);
        tempoInicioMovimento = millis();
        Serial.printf("\n--- INICIANDO SEQUENCIA --- \nPasso: %d, Ponto de Partida: M0=%.1f, M1=%.1f, M2=%.1f, M3=%.1f\n", passoAtual, motor_pos_inicial[0], motor_pos_inicial[1], motor_pos_inicial[2], motor_pos_inicial[3]);
    }

    Passo target = sequenciaDePassos_joelho_direito[passoAtual];
    unsigned long tempoDecorrido = millis() - tempoInicioMovimento;

    if (tempoDecorrido < target.duracao_ms) {
        // Interpola suavemente enquanto o tempo não acaba
        float progresso = (float)tempoDecorrido / (float)target.duracao_ms;
        float fator = (1.0 - cos(progresso * PI)) / 2.0;
        all_motors[0].target_deg = motor_pos_inicial[0] + (target.motor0_target_deg - motor_pos_inicial[0]) * fator;
        all_motors[1].target_deg = motor_pos_inicial[1] + (target.motor1_target_deg - motor_pos_inicial[1]) * fator;
        all_motors[2].target_deg = motor_pos_inicial[2] + (target.motor2_target_deg - motor_pos_inicial[2]) * fator;
        all_motors[3].target_deg = motor_pos_inicial[3] + (target.motor3_target_deg - motor_pos_inicial[3]) * fator;
    } else {
        // O tempo acabou! Hora de entrar no modo de "espera".
        Serial.printf("!!! Passo %d TEMPO CONCLUIDO. Alvo Final: M0=%.1f, M1=%.1f. Aguardando posicao...\n", passoAtual, target.motor0_target_deg, target.motor1_target_deg);
        
        // Garante que o alvo final seja cravado para o PID ter uma referência fixa
        all_motors[0].target_deg = target.motor0_target_deg;
        all_motors[1].target_deg = target.motor1_target_deg;
        all_motors[2].target_deg = target.motor2_target_deg;
        all_motors[3].target_deg = target.motor3_target_deg;
        
        aguardandoPosicao = true; // Ativa a flag de espera
    }
}


void movimento_suave() {
    if (aguardandoPosicao) {
        Passo target = sequenciaDePassos[passoAtual];

        float posAtualM0 = readEncoderDeg(all_motors[0]);
        float posAtualM1 = readEncoderDeg(all_motors[1]);
        float posAtualM2 = readEncoderDeg(all_motors[2]);
        float posAtualM3 = readEncoderDeg(all_motors[3]);
        bool motor0_chegou = fabsf(angleError(target.motor0_target_deg, posAtualM0)) <= TOLERANCIA_GRAUS;
        bool motor1_chegou = fabsf(angleError(target.motor1_target_deg, posAtualM1)) <= TOLERANCIA_GRAUS;
        bool motor2_chegou = fabsf(angleError(target.motor2_target_deg, posAtualM2)) <= TOLERANCIA_GRAUS;
        bool motor3_chegou = fabsf(angleError(target.motor3_target_deg, posAtualM3)) <= TOLERANCIA_GRAUS;

        all_motors[0].modoPidAtual = target.ganhoMotor0;
        all_motors[1].modoPidAtual = target.ganhoMotor1;
        all_motors[2].modoPidAtual = target.ganhoMotor2;
        all_motors[3].modoPidAtual = target.ganhoMotor3;

        //Serial.println("target.motor0_target_deg - posAtualM0 =");
        //Serial.print(target.motor0_target_deg - posAtualM0);
        //Serial.println("posAtualM0 = ");
        //Serial.print(posAtualM0);
        //Serial.println(" ");

        //Serial.println("target.motor1_target_deg - posAtualM1 =");
        //Serial.print(target.motor1_target_deg - posAtualM1);
        //Serial.println("posAtualM1 = ");
        //Serial.print(posAtualM1);
        //Serial.println(" ");
        
        if (motor0_chegou && motor1_chegou && motor2_chegou && motor3_chegou) {
            Serial.printf(">>> POSICAO CONFIRMADA! M0=%.1f, M1=%.1f, M2=%.1f, M3=%.1f <<<\n", posAtualM0, posAtualM1, posAtualM2, posAtualM3);
            
            aguardandoPosicao = false; 

            //motor0_pos_inicial = target.motor0_target_deg;
            //motor1_pos_inicial = target.motor1_target_deg;

            motor_pos_inicial[0] = readEncoderDeg(all_motors[0]);
            motor_pos_inicial[1] = readEncoderDeg(all_motors[1]);
            motor_pos_inicial[2] = readEncoderDeg(all_motors[2]);
            motor_pos_inicial[3] = readEncoderDeg(all_motors[3]);

            passoAtual++;
            if (passoAtual >= NUM_PASSOS) {
                passoAtual = 0;
            }

            if (passoAtual == 2 || passoAtual == 9) {
                qtde_passos++;
                Serial.printf("!!! INCREMENTO CONCLUÍDO. qtde_passos: %d\n", qtde_passos);
            }
            
            tempoInicioMovimento = millis();
            
            Serial.printf("--- INICIANDO PROXIMO PASSO --- \nPasso: %d, Ponto de Partida: M0=%.1f, M1=%.1f, M2=%.1f, M3=%.1f\n", passoAtual, motor_pos_inicial[0], motor_pos_inicial[1], motor_pos_inicial[2], motor_pos_inicial[3]);
        }

        return; 
    }


    if (tempoInicioMovimento == 0) {
        motor_pos_inicial[0] = readEncoderDeg(all_motors[0]);
        motor_pos_inicial[1] = readEncoderDeg(all_motors[1]);
        motor_pos_inicial[2] = readEncoderDeg(all_motors[2]);
        motor_pos_inicial[3] = readEncoderDeg(all_motors[3]);
        tempoInicioMovimento = millis();
        Serial.printf("\n--- INICIANDO SEQUENCIA --- \nPasso: %d, Ponto de Partida: M0=%.1f, M1=%.1f, M2=%.1f, M3=%.1f\n", passoAtual, motor_pos_inicial[0], motor_pos_inicial[1], motor_pos_inicial[2], motor_pos_inicial[3]);
    }

    Passo target = sequenciaDePassos[passoAtual];
    unsigned long tempoDecorrido = millis() - tempoInicioMovimento;

    if (tempoDecorrido < target.duracao_ms) {
        // Interpola suavemente enquanto o tempo não acaba
        float progresso = (float)tempoDecorrido / (float)target.duracao_ms;
        float fator = (1.0 - cos(progresso * PI)) / 2.0;
        all_motors[0].target_deg = motor_pos_inicial[0] + (target.motor0_target_deg - motor_pos_inicial[0]) * fator;
        all_motors[1].target_deg = motor_pos_inicial[1] + (target.motor1_target_deg - motor_pos_inicial[1]) * fator;
        all_motors[2].target_deg = motor_pos_inicial[2] + (target.motor2_target_deg - motor_pos_inicial[2]) * fator;
        all_motors[3].target_deg = motor_pos_inicial[3] + (target.motor3_target_deg - motor_pos_inicial[3]) * fator;
    } else {
        // O tempo acabou! Hora de entrar no modo de "espera".
        Serial.printf("!!! Passo %d TEMPO CONCLUIDO. Alvo Final: M0=%.1f, M1=%.1f. Aguardando posicao...\n", passoAtual, target.motor0_target_deg, target.motor1_target_deg);
        
        // Garante que o alvo final seja cravado para o PID ter uma referência fixa
        all_motors[0].target_deg = target.motor0_target_deg;
        all_motors[1].target_deg = target.motor1_target_deg;
        all_motors[2].target_deg = target.motor2_target_deg;
        all_motors[3].target_deg = target.motor3_target_deg;
        
        aguardandoPosicao = true; // Ativa a flag de espera
    }
}

float lerCorrente(int pino_sensor) {
    // Para maior estabilidade, fazemos uma média de algumas leituras
    int somaLeiturasADC = 0;
    somaLeiturasADC += analogRead(pino_sensor);
    float tensao_no_ESP32 = (somaLeiturasADC / 4095.0) * 3.3;
    float tensao_original_IS = tensao_no_ESP32 * (R1_DIVISOR + R2_DIVISOR) / R2_DIVISOR;
    float corrente_IS = tensao_original_IS / RESISTOR_SENSE_OHMS;
    float correnteMotor = corrente_IS * PROPORCAO_CORRENTE;

    return correnteMotor;
}

void motionProfilerStep(Motor &motor) {
    const float max_speed_dps = 360.0f;
    const float acceleration_dps2 = 400.0f;
    const float Ts = 10.0f / 1000.0f;
    
    float dist_to_go = angleError(motor.target_deg, motor.setpoint_deg);

    if (fabsf(dist_to_go) < 0.1f && fabsf(motor.current_speed_dps) < 0.1f) {
        motor.current_speed_dps = 0.0f; motor.setpoint_deg = motor.target_deg; return;
    }
    float braking_dist = (motor.current_speed_dps * motor.current_speed_dps) / (2.0f * acceleration_dps2);
    if (fabsf(dist_to_go) > braking_dist) {
        motor.current_speed_dps += acceleration_dps2 * Ts;
        if (motor.current_speed_dps > max_speed_dps) motor.current_speed_dps = max_speed_dps;
    } else {
        motor.current_speed_dps -= acceleration_dps2 * Ts;
    }
    motor.current_speed_dps = copysignf(fabsf(motor.current_speed_dps), dist_to_go);
    float step = motor.current_speed_dps * Ts;
    if (fabsf(step) > fabsf(dist_to_go)) {
        step = dist_to_go; motor.current_speed_dps = 0.0f;
    }
    motor.setpoint_deg += step;
    if (motor.setpoint_deg >= 360.0f) motor.setpoint_deg -= 360.0f;
    if (motor.setpoint_deg < 0.0f) motor.setpoint_deg += 360.0f;
}

void pidStep(Motor &motor) {
    // --- CORREÇÃO DE ESCOPO ---
    float tol_deg_atual; // Variável declarada fora dos blocos
    if (marcha_ativa) {
        tol_deg_atual = TOLERANCIA_GRAUS; // Usa a tolerância de 15.0 para a marcha
    } else {
        tol_deg_atual = 3.0f; // Tolerância bem baixa (.0 grau) para quando estiver parado
    }
    // -------------------------

    const int UPPER_PWM = 255;
    const int LOWER_PWM = -255;
    const float iTermLimit = 255.0f;
    const float Ts = 10.0f / 1000.0f;

    float pv = readEncoderDeg(motor);
    float e = angleError(motor.setpoint_deg, pv);

    bool movimentoTerminou = fabsf(angleError(motor.target_deg, motor.setpoint_deg)) < 0.1f;
    // --- USA A VARIÁVEL CORRETA ---
    bool dentroDaTolerancia = fabsf(angleError(motor.target_deg, pv)) <= tol_deg_atual;

    if (movimentoTerminou && dentroDaTolerancia) {
        motorSet(motor, 0);
        motor.iTerm = 0.0f;
        motor.pv_prev = pv;
        return;
    }

    // --- O RESTO DA FUNÇÃO CONTINUA IGUAL ---
    float Kp_atual, Ki_atual, Kd_atual;

    if (motor.modoPidAtual == PID_DESCIDA) {
        Kp_atual = motor.Kp_up;
        Ki_atual = motor.Ki_up;
        Kd_atual = motor.Kd_up;
    } else {
        Kp_atual = motor.Kp_down;
        Ki_atual = motor.Ki_down;
        Kd_atual = motor.Kd_down;
    }

    float pTerm = Kp_atual * e;
    float dTerm = -Kd_atual * (pv - motor.pv_prev) / Ts;
    motor.pv_prev = pv;
    float u = pTerm + motor.iTerm + dTerm;

    int duty = lroundf(u);

    float saturationError = u - duty;
    motor.iTerm += (Ki_atual * e - motor.Kb * saturationError) * Ts;
    motor.iTerm = constrain(motor.iTerm, -iTermLimit, iTermLimit);

    motorSet(motor, duty);
}

void motorSet(Motor &motor, int duty) {
    const int UPPER_PWM = 255; 
    const int LOWER_PWM = -255;
    const int DEAD_TIME_MS = 100; // Delay de 10 ms para inverter as pontes

    duty = constrain(duty, LOWER_PWM, UPPER_PWM);

    if (motor.is_in_dead_time) {
  
        if (millis() - motor.dead_time_start >= DEAD_TIME_MS) {
         
            motor.is_in_dead_time = false; 
        } else {
        
            analogWrite(motor.rpwm_pin, 0);
            analogWrite(motor.lpwm_pin, 0);
            return; 
        }
    }

   
    int current_duty_sign = 0;
    if (duty > 0) current_duty_sign = 1;
    if (duty < 0) current_duty_sign = -1;

   
    if (current_duty_sign != 0 && current_duty_sign != motor.last_duty_sign) {
      
        motor.is_in_dead_time = true;    
        motor.dead_time_start = millis();   
        
 
        analogWrite(motor.rpwm_pin, 0);
        analogWrite(motor.lpwm_pin, 0);

        motor.last_duty_sign = current_duty_sign;
        return; 
    }


    if (duty > 0) { 
        analogWrite(motor.rpwm_pin, duty); 
        analogWrite(motor.lpwm_pin, 0); 
    }
    else if (duty < 0) { 
        analogWrite(motor.rpwm_pin, 0); 
        analogWrite(motor.lpwm_pin, -duty); 
    }
    else { 
        analogWrite(motor.rpwm_pin, 0); 
        analogWrite(motor.lpwm_pin, 0); 
    }

  
    motor.last_duty_sign = current_duty_sign;
}

float readEncoderDeg(Motor &motor) {
    noInterrupts(); long c = motor.encCount - motor.encOffset; interrupts();
    //long pos = ((c % PPR) + PPR) % PPR;
    return c * 360.0f / PPR;
}

float angleError(float sp, float pv) {
    float e = sp - pv;
    while (e > 180.0f) e -= 360.0f;
    while (e < -180.0f) e += 360.0f;
    return e;
}

void serialTask() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        Serial.println("--------------------");
        Serial.print("Recebido: '"); Serial.print(input); Serial.println("'");

        if (input.startsWith("m")) {
            input.remove(0, 1); // Remove "m"
            
            int motor_id = input.toInt();
            Serial.print("ID do Motor Lido: "); Serial.println(motor_id);

            if (motor_id >= 0 && motor_id < NUM_MOTORS) {
                int space_index = input.indexOf(' ');
                if (space_index == -1) {
                    Serial.println(">> ERRO: Nao encontrei espaco no comando. Formato: 'm<ID> <valor>'");
                    return;
                }

                String cmd_str = input.substring(space_index + 1);
                cmd_str.trim();
                Serial.print("Comando Lido: '"); Serial.print(cmd_str); Serial.println("'");

                if (cmd_str == "z") {
                    Serial.println(">> Acao: Zerando encoder.");
                    noInterrupts();
                    all_motors[motor_id].encOffset = all_motors[motor_id].encCount;
                    interrupts();
                    all_motors[motor_id].target_deg = 0;
                    all_motors[motor_id].setpoint_deg = 0;
                } else {
                    float new_target = cmd_str.toFloat();
                    Serial.print(">> Acao: Novo Alvo Numerico: "); Serial.println(new_target);
                    all_motors[motor_id].target_deg = new_target;
                }
            } else {
                Serial.println(">> ERRO: ID de motor invalido.");
            }
        }
         Serial.println("--------------------");
    }
}

void printTask() {
    static uint32_t t0 = 0;
    if (millis() - t0 >= 200) {
        t0 = millis();
        // Para testes apenas os 2 motores
        for (int i = 0; i < 4; i++) {
            Serial.print("M"); Serial.print(i);
            Serial.print(" TGT="); Serial.print(all_motors[i].target_deg, 1);
            Serial.print(" ANG="); Serial.print(readEncoderDeg(all_motors[i]), 1);
            //float corrente = lerCorrente(all_motors[1].current_sense_pin);
            //float adc = analogRead(all_motors[1].current_sense_pin);
            //Serial.printf("I="); Serial.print(corrente, 2); Serial.print("A ");
            //Serial.printf("V="); Serial.print(adc, 2);
            //Serial.printf("Vaj="); Serial.print(adc, 2)*0.000806;
            if (i < 3) Serial.print(" || ");
        }
        Serial.println();
        
    }
}

void setup() {

    Serial.begin(115200);
    delay(500);

    // --- WiFi/WebServer ---
    WiFi.softAP(ssid, password);
    IPAddress myIP = WiFi.softAPIP();
    Serial.println("\nAccess Point iniciado!");
    Serial.print("IP: ");
    Serial.println(myIP);

    // Endpoints principais
    server.on("/start-marcha", HTTP_GET, [](){ startComPerfil("marcha"); });
    server.on("/start-quadril_esquerdo", HTTP_GET, [](){ startComPerfil("quadril_esquerdo"); });
    server.on("/start-quadril_direito", HTTP_GET, [](){ startComPerfil("quadril_direito"); });
    server.on("/start-joelho_esquerdo", HTTP_GET, [](){ startComPerfil("joelho_esquerdo"); });
    server.on("/start-joelho_direito", HTTP_GET, [](){ startComPerfil("joelho_direito"); });

    server.on("/stop", HTTP_GET, handleStop);
    server.on("/status", HTTP_GET, handleStatus);
    // CORS preflight
    server.on("/start-marcha", HTTP_OPTIONS, handleOptions);
    server.on("/stop", HTTP_OPTIONS, handleOptions);
    server.on("/status", HTTP_OPTIONS, handleOptions);
    server.begin();
    Serial.println("Servidor HTTP iniciado");

    // --- Configurando Motor 0 ---
    all_motors[0].rpwm_pin = RPWM0_PIN;
    all_motors[0].lpwm_pin = LPWM0_PIN;
    all_motors[0].ren_pin  = REN0_PIN;
    all_motors[0].len_pin  = LEN0_PIN;
    all_motors[0].enc_a_pin= ENC0_A_PIN;
    all_motors[0].enc_b_pin= ENC0_B_PIN;
    all_motors[0].id       = 0;
    all_motors[0].Kp_up = 1.2f; //subir   
    all_motors[0].Ki_up = 0.8f; //subir
    all_motors[0].Kd_up = 0.0f; //subir
    all_motors[0].Kb = 1.0f;
    all_motors[0].Kp_down = 0.1f; //descer
    all_motors[0].Ki_down = 0.8f; //descer
    all_motors[0].Kd_down = 0.05f; //descern

    // --- Configurando Motor 1 ---
    all_motors[1].rpwm_pin = RPWM1_PIN;
    all_motors[1].lpwm_pin = LPWM1_PIN;
    all_motors[1].ren_pin  = REN1_PIN;
    all_motors[1].len_pin  = LEN1_PIN;
    all_motors[1].enc_a_pin= ENC1_A_PIN;
    all_motors[1].enc_b_pin= ENC1_B_PIN;
    all_motors[1].current_sense_pin = IS1_PIN;
    all_motors[1].id       = 1;
    all_motors[1].Kp_up = 1.0f; //subir
    all_motors[1].Ki_up = 0.8f; //subir
    all_motors[1].Kd_up = 0.0f; //subir
    all_motors[1].Kb = 1.0f;
    all_motors[1].Kp_down = 0.2f; // descer
    all_motors[1].Ki_down = 0.3f; // descer
    all_motors[1].Kd_down = 0.05f; //descer

    // --- Configurando Motor 2 ---
    all_motors[2].rpwm_pin = RPWM2_PIN;
    all_motors[2].lpwm_pin = LPWM2_PIN;
    all_motors[2].ren_pin  = REN2_PIN;
    all_motors[2].len_pin  = LEN2_PIN;
    all_motors[2].enc_a_pin= ENC2_A_PIN;
    all_motors[2].enc_b_pin= ENC2_B_PIN;
    all_motors[2].id       = 2;
    all_motors[2].Kp_up = 1.2f; //subir   
    all_motors[2].Ki_up = 0.8f; //subir
    all_motors[2].Kd_up = 0.0f; //subir
    all_motors[2].Kb = 1.0f;
    all_motors[2].Kp_down = 0.4f; //descer
    all_motors[2].Ki_down = 0.8f; //descer
    all_motors[2].Kd_down = 0.05f; //descern
    
    // --- Configurando Motor 3 ---
    all_motors[3].rpwm_pin = RPWM3_PIN;
    all_motors[3].lpwm_pin = LPWM3_PIN;
    all_motors[3].ren_pin  = REN3_PIN;
    all_motors[3].len_pin  = LEN3_PIN;
    all_motors[3].enc_a_pin= ENC3_A_PIN;
    all_motors[3].enc_b_pin= ENC3_B_PIN;
    all_motors[3].id       = 3;
    all_motors[3].Kp_up = 0.6f; //subir
    all_motors[3].Ki_up = 0.4f; //subir
    all_motors[3].Kd_up = 0.0f; //subir
    all_motors[3].Kb = 1.0f;
    all_motors[3].Kp_down = 0.6f; // descer
    all_motors[3].Ki_down = 0.2f; // descer
    all_motors[3].Kd_down = 0.05f; //descer

    for (int i = 0; i < NUM_MOTORS; i++) {
        pinMode(all_motors[i].ren_pin, OUTPUT); digitalWrite(all_motors[i].ren_pin, HIGH);
        pinMode(all_motors[i].len_pin, OUTPUT); digitalWrite(all_motors[i].len_pin, HIGH);
        pinMode(all_motors[i].rpwm_pin, OUTPUT);
        pinMode(all_motors[i].lpwm_pin, OUTPUT);
        pinMode(all_motors[i].enc_a_pin, INPUT_PULLUP);
        pinMode(all_motors[i].enc_b_pin, INPUT_PULLUP);
        all_motors[i].setpoint_deg = readEncoderDeg(all_motors[i]); 
        all_motors[i].target_deg = all_motors[i].setpoint_deg;
    }

    // Configura as interrupções
    attachInterrupt(digitalPinToInterrupt(all_motors[0].enc_a_pin), encoder_isr_motor0, CHANGE);
    attachInterrupt(digitalPinToInterrupt(all_motors[1].enc_a_pin), encoder_isr_motor1, CHANGE);
 
    // DESCOMENTE AS LINHAS ABAIXO QUANDO FOR USAR OS MOTORES 3 E 4
    attachInterrupt(digitalPinToInterrupt(all_motors[2].enc_a_pin), encoder_isr_motor2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(all_motors[3].enc_a_pin), encoder_isr_motor3, CHANGE);
    static int motor_ids[NUM_MOTORS];
    for (int i = 0; i < 4; i++) {
        motor_ids[i] = i;
        xTaskCreatePinnedToCore(
            motorControlTask,           // Função da tarefa
            "PID",// Nome da tarefa (para debug)
            4096,                       // Tamanho da pilha (stack size)
            &motor_ids[i],              // Parâmetro da tarefa (o ID do motor)
            5,                          // Prioridade da tarefa (alta)
            NULL,                       // Handle da tarefa (não precisamos)
            0                           // Núcleo do processador (0 para controle, 1 para o resto)
        );
    }
    
    Serial.println("Controle de Exoesqueleto INICIADO (Versao Simples).");
    Serial.println("Comandos: 'm0 90', 'm1 45', 'm0 z', etc.");
}

void loop() {
    const float max_speed_dps = 360.0f;
    const uint32_t Ts_ms = 10;
/*
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (i < 2) {
            if (millis() - all_motors[i].lastUpdate >= Ts_ms) {
                all_motors[i].lastUpdate += Ts_ms;
            
                // ISTO AQUI É UMA FORMA DE "MOTION PROFILER" SIMPLES
                const float max_step = max_speed_dps * (Ts_ms / 1000.0f);
                float error_to_target = angleError(all_motors[i].target_deg, all_motors[i].setpoint_deg);
                float step = constrain(error_to_target, -max_step, max_step);
                all_motors[i].setpoint_deg += step;

                // A CHAMADA DO PID ACONTECIA AQUI
                pidStep(all_motors[i]);
            }
        }
    }
 */       
    serialTask();
    printTask();
    server.handleClient();
   if (marcha_ativa) {
        // Se a marcha está ativa, executa a sequência de passos suave.
        if(perfil=="marcha"){
            movimento_suave();
        }
        if(perfil=="quadril_esquerdo"){
            movimento_suave_quadril_esquerdo();
        }
        if(perfil=="quadril_direito"){
            movimento_suave_quadril_direito();
        }
        if(perfil=="joelho_esquerdo"){
            movimento_suave_joelho_esquerdo();
        }
        if(perfil=="joelho_direito"){
            movimento_suave_joelho_direito();
        }

    } else {
        // Se a marcha está parada, o alvo é sempre zero para todos os motores.
        // O controle PID (que roda em suas próprias tasks) se encarregará de
        // levar os motores para essa posição com a tolerância de 1.0 grau
        // que definimos em pidStep().
        all_motors[0].target_deg = 0.0f;
        all_motors[1].target_deg = 0.0f;
        all_motors[2].target_deg = 0.0f;
        all_motors[3].target_deg = 0.0f;
    }
}