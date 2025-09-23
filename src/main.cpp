#include <Arduino.h>
#include <math.h>
// --- Adiciona WiFi/WebServer ---
#include <WiFi.h>
#include <WebServer.h>

const char* ssid = "ESP32-AP";
const char* password = "12345678";
WebServer server(80);

// Variáveis de estado do WebServer
bool contando = false;
unsigned long passos = 0;
String perfil = "nenhum";

// --- CORS ---
void addCorsHeaders() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    server.sendHeader("Access-Control-Allow-Headers", "Content-Type, Origin, X-Requested-With, Accept");
}
void handleOptions() {
    addCorsHeaders();
    server.send(204);
}

void atualizarPassos() {
    if (contando) {
        passos = 2;
    }
}

void startComPerfil(String nomePerfil) {
    perfil = nomePerfil;
    contando = true;
    passos = 0;
    addCorsHeaders();
    server.send(200, "text/plain", "Contagem iniciada com perfil: " + perfil);
}
void handleStop() {
    contando = false;
    addCorsHeaders();
    server.send(200, "text/plain", "Contagem parada");
}
void handleStatus() {
    String statusJson = "{";
    statusJson += "\"perfil\": \"" + perfil + "\",";
    statusJson += "\"passos\": " + String(passos);
    statusJson += "}";
    addCorsHeaders();
    server.send(200, "application/json", statusJson);
}

// CONFIGURAÇÃO GERAL
const int NUM_MOTORS = 4; 
const int PPR = 360;      

// Motor 0 (Quadril Direito)
#define RPWM0_PIN 26
#define LPWM0_PIN 27
#define REN0_PIN  16
#define LEN0_PIN  17
#define ENC0_A_PIN 32
#define ENC0_B_PIN 33

// Motor 1 (Joelho Direito)
#define RPWM1_PIN 14
#define LPWM1_PIN 12
#define REN1_PIN  23
#define LEN1_PIN  22
#define ENC1_A_PIN 15
#define ENC1_B_PIN 18
#define IS1_PIN 34

// Motor 2 (Quadril Esquerdo) - Precisa definir
#define RPWM2_PIN 0
#define LPWM2_PIN 0
#define REN2_PIN  0
#define LEN2_PIN  0
#define ENC2_A_PIN 0
#define ENC2_B_PIN 0

// Motor 3 (Joelho Esquerdo) - Precisa definir
#define RPWM3_PIN 0
#define LPWM3_PIN 0
#define REN3_PIN  0
#define LEN3_PIN  0
#define ENC3_A_PIN 0
#define ENC3_B_PIN 0

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
};


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
    const float tol_deg = 5.0f;
    const int UPPER_PWM = 255;
    const int LOWER_PWM = -255;
    const float iTermLimit = 255.0f;
    const float Ts = 10.0f / 1000.0f;

    float pv = readEncoderDeg(motor);
    float e = angleError(motor.setpoint_deg, pv);
    
    bool movimentoTerminou = fabsf(angleError(motor.target_deg, motor.setpoint_deg)) < 0.1f;

    bool dentroDaTolerancia = fabsf(angleError(motor.target_deg, pv)) <= tol_deg;

    if (movimentoTerminou && dentroDaTolerancia) {
        motorSet(motor, 0); 
        motor.iTerm = 0.0f; 
        motor.pv_prev = pv; 
        return;
    }
    
    // --- O RESTO DA FUNÇÃO CONTINUA IGUAL ---
    float Kp_atual, Ki_atual, Kd_atual;

    if (e > 0) {
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

/*
void motorSet(Motor &motor, int duty) {
    //if (duty != 0) {   // dedug
     // Serial.print("motorSet -> ID: ");
    //Serial.print(motor.id);
     // Serial.print(", Duty: ");
     // Serial.println(duty);
    //}
    const int UPPER_PWM = 255; const int LOWER_PWM = -255;
    duty = constrain(duty, LOWER_PWM, UPPER_PWM);
    if (duty > 0) { analogWrite(motor.rpwm_pin, duty); analogWrite(motor.lpwm_pin, 0); }
    else if (duty < 0) { analogWrite(motor.rpwm_pin, 0); analogWrite(motor.lpwm_pin, -duty); }
    else { analogWrite(motor.rpwm_pin, 0); analogWrite(motor.lpwm_pin, 0); }
}
*/
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

bool time(unsigned long duracao_ms){
    static unsigned long tempoInicial = 0;
    if (tempoInicial == 0){
        tempoInicial = millis();
    }
    if(millis()-tempoInicial >= duracao_ms){
        tempoInicial = 0;
        return true;
    }
    return false;
}

void serialTask() {
    if (Serial.available()) {
        // Só aceita comandos se perfil == "idoso" e contando == true
        if (perfil == "idoso" && contando) {
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
        } else {
            // Ignora comandos se não estiver autorizado
            String input = Serial.readStringUntil('\n');
            input.trim();
            Serial.println("Comando ignorado. Use /start-idoso para habilitar comandos na serial.");
        }
    }
}

void printTask() {
    static uint32_t t0 = 0;
    if (millis() - t0 >= 200) {
        t0 = millis();
        // Para testes apenas os 2 motores
        for (int i = 0; i < 2; i++) {
            Serial.print("M"); Serial.print(i);
            Serial.print(" TGT="); Serial.print(all_motors[i].target_deg, 1);
            Serial.print(" ANG="); Serial.print(readEncoderDeg(all_motors[i]), 1);
            float corrente = lerCorrente(all_motors[1].current_sense_pin);
            float adc = analogRead(all_motors[1].current_sense_pin);
            Serial.printf("I="); Serial.print(corrente, 2); Serial.print("A ");
            Serial.printf("V="); Serial.print(adc, 2);
            Serial.printf("Vaj="); Serial.print(adc, 2)*0.000806;
            if (i < 1) Serial.print(" || ");
        }
        Serial.println();
        
    }
}

int passoDoMovimento = 1;

void movimento() {
    switch (passoDoMovimento) {

        case 1:
            //all_motors[0].target_deg = 20.0;
            all_motors[1].target_deg = 40.0;

            if (time(2000)) {
                passoDoMovimento = 2;
            }
            break;

        case 2:
            //all_motors[0].target_deg = 0.0;
            all_motors[1].target_deg = 20.0;
            
            if (time(2000)) {
                passoDoMovimento = 3;
            }
            break;
 
        case 3:
            //all_motors[0].target_deg = 0.0;
            all_motors[1].target_deg = 0.0;
            
            if (time(2000)) {
                passoDoMovimento = 4;
            }
            break;

        case 4:
            //all_motors[0].target_deg = 0.0;
            all_motors[1].target_deg = -20.0;
            
            if (time(2000)) {
                passoDoMovimento = 5;
            }
            break;
                        
        case 5:
            break;
    }
}

void setup() {
    Serial.begin(115200);
    delay(500);

    // --- Configuração dos motores (igual ao original) ---
    all_motors[0].rpwm_pin = RPWM0_PIN;
    all_motors[0].lpwm_pin = LPWM0_PIN;
    all_motors[0].ren_pin  = REN0_PIN;
    all_motors[0].len_pin  = LEN0_PIN;
    all_motors[0].enc_a_pin= ENC0_A_PIN;
    all_motors[0].enc_b_pin= ENC0_B_PIN;
    all_motors[0].id       = 0;
    all_motors[0].Kp_up = 1.2f; //subir   
    all_motors[0].Ki_up = 0.4f; //subir
    all_motors[0].Kd_up = 0.3f; //subir
    all_motors[0].Kb = 1.0f;
    all_motors[0].Kp_down = 1.2f; //descer
    all_motors[0].Ki_down = 0.4f; //descer
    all_motors[0].Kd_down = 0.3f; //descer

    all_motors[1].rpwm_pin = RPWM1_PIN;
    all_motors[1].lpwm_pin = LPWM1_PIN;
    all_motors[1].ren_pin  = REN1_PIN;
    all_motors[1].len_pin  = LEN1_PIN;
    all_motors[1].enc_a_pin= ENC1_A_PIN;
    all_motors[1].enc_b_pin= ENC1_B_PIN;
    all_motors[1].current_sense_pin = IS1_PIN;
    all_motors[1].id       = 1;
    all_motors[1].Kp_up = 0.8f; //subir
    all_motors[1].Ki_up = 0.9f; //subir
    all_motors[1].Kd_up = 0.0f; //subir
    all_motors[1].Kb = 1.0f;
    all_motors[1].Kp_down = 0.1f; // descer
    all_motors[1].Ki_down = 0.4f; // descer
    all_motors[1].Kd_down = 0.0f; //descer

    all_motors[2].rpwm_pin = RPWM2_PIN;
    all_motors[2].lpwm_pin = LPWM2_PIN;
    all_motors[2].ren_pin  = REN2_PIN;
    all_motors[2].len_pin  = LEN2_PIN;
    all_motors[2].enc_a_pin= ENC2_A_PIN;
    all_motors[2].enc_b_pin= ENC2_B_PIN;
    all_motors[2].id       = 2;

    all_motors[3].rpwm_pin = RPWM3_PIN;
    all_motors[3].lpwm_pin = LPWM3_PIN;
    all_motors[3].ren_pin  = REN3_PIN;
    all_motors[3].len_pin  = LEN3_PIN;
    all_motors[3].enc_a_pin= ENC3_A_PIN;
    all_motors[3].enc_b_pin= ENC3_B_PIN;
    all_motors[3].id       = 3;

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

    attachInterrupt(digitalPinToInterrupt(all_motors[0].enc_a_pin), encoder_isr_motor0, CHANGE);
    attachInterrupt(digitalPinToInterrupt(all_motors[1].enc_a_pin), encoder_isr_motor1, CHANGE);
    // attachInterrupt(digitalPinToInterrupt(all_motors[2].enc_a_pin), encoder_isr_motor2, CHANGE);
    // attachInterrupt(digitalPinToInterrupt(all_motors[3].enc_a_pin), encoder_isr_motor3, CHANGE);

    // --- Inicializa WiFi/WebServer ---
    WiFi.softAP(ssid, password);
    IPAddress myIP = WiFi.softAPIP();
    Serial.println("\nAccess Point iniciado!");
    Serial.print("IP: ");
    Serial.println(myIP);

    // Rotas principais
    server.on("/start-jovem", HTTP_GET, [](){ startComPerfil("jovem"); });
    server.on("/start-adulto", HTTP_GET, [](){ startComPerfil("adulto"); });
    server.on("/start-idoso", HTTP_GET, [](){ startComPerfil("idoso"); });
    server.on("/stop", HTTP_GET, handleStop);
    server.on("/status", HTTP_GET, handleStatus);
    // Rotas OPTIONS para CORS
    server.on("/start-jovem", HTTP_OPTIONS, handleOptions);
    server.on("/start-adulto", HTTP_OPTIONS, handleOptions);
    server.on("/start-idoso", HTTP_OPTIONS, handleOptions);
    server.on("/stop", HTTP_OPTIONS, handleOptions);
    server.on("/status", HTTP_OPTIONS, handleOptions);

    server.begin();
    Serial.println("Servidor iniciado");

    Serial.println("Controle de Exoesqueleto INICIADO (Versao Simples).");
    Serial.println("Comandos: 'm0 90', 'm1 45', 'm0 z', etc.");
}

void loop() {
    // WebServer
    server.handleClient();
    atualizarPassos();

    // Motores
    const float max_speed_dps = 360.0f;
    const uint32_t Ts_ms = 10;
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (i < 2) {
            if (millis() - all_motors[i].lastUpdate >= Ts_ms) {
                all_motors[i].lastUpdate += Ts_ms;
                const float max_step = max_speed_dps * (Ts_ms / 1000.0f);
                float error_to_target = angleError(all_motors[i].target_deg, all_motors[i].setpoint_deg);
                float step = constrain(error_to_target, -max_step, max_step);
                all_motors[i].setpoint_deg += step;
                pidStep(all_motors[i]);
            }
        }
    }
    serialTask();
    printTask();
    movimento();


    if(perfil == "idoso" && contando == true){
        movimento();
    }
    
}