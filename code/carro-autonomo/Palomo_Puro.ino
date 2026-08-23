#include <Arduino.h>
#include <cmath>
#include <WiFi.h>
#include <ArduinoJson.h>

#define IN1_B 33  // Dirección Motor B
#define IN2_B 27  // Dirección Motor B
#define IN1_A 16  // Dirección Motor A
#define IN2_A 18  // Dirección Motor A

#define ENC_A 21  // Encoder Motor A
#define ENC_B 35  // Encoder Motor B

#define PWM_CHANNEL_A 0
#define PWM_CHANNEL_B 1
#define PWM_FREQ 5000
#define PWM_RES 8  // 8 bits

// WiFi y servidor
const char* wifiSSID = "Roy";
const char* wifiPass = "shullmusic";
WiFiServer wifiServer(1234);

// Estado del robot
struct RobotState {
  double x;
  double y;
  double theta;
};

RobotState rbtState = {0.0, 0.0, 0.0};
double simDt = 0.05; // Actualización de la simulación

// Coordenadas globales
double xd = 0, yd = 0;    // Posición deseada
double xo = 0, yo = 0;    // Posición obstáculo

const double r = 0.033;   // Radio de la rueda
const double s = 0.13;    // Separación entre ruedas
const double Kr = 0.15;   // Radio para atracción
const double vMax = 0.35; // Velocidad máxima tangencial
const double omax = 13.0; // Velocidad angular máxima
const double Rrep = 0.65; // Radio repulsión
const double distThresh = 0.05; // Umbral para destino

bool coordsFlag = false;

float angRPM_A = 0, angRPM_B = 0;
volatile int pulseCountA = 0, pulseCountB = 0;
float pulsesRev = 40;
float rpmTargetA = 0, rpmTargetB = 0; 
float sampleTimeSec = 0.01;  // Tiempo de muestreo
float Kp = 0.15, Ki = 8.0;
float errSumA = 0.0, errSumB = 0.0;

float errGlobalA = 0;
float errGlobalB = 0;

unsigned long lastTickA = 0;
unsigned long lastTickB = 0;
unsigned long debounceMicros = 5000;  // 5 ms

int pidOutputA = 0;
int pidOutputB = 0;

double calcX(double theta, double wL, double wR) {
  return (r / 2.0) * (wL + wR) * cos(theta);
}

double calcY(double theta, double wL, double wR) {
  return (r / 2.0) * (wL + wR) * sin(theta);
}

double calcTheta(double wL, double wR) {
  return (r / s) * (wR - wL);
}

// Runge-Kutta de 4º orden
RobotState updateState(RobotState state, double wL, double wR, double deltaT) {
  RobotState rk1, rk2, rk3, rk4;
  
  rk1.x = calcX(state.theta, wL, wR);
  rk1.y = calcY(state.theta, wL, wR);
  rk1.theta = calcTheta(wL, wR);
  
  rk2.x = calcX(state.theta + 0.5 * deltaT * rk1.theta, wL, wR);
  rk2.y = calcY(state.theta + 0.5 * deltaT * rk1.theta, wL, wR);
  rk2.theta = calcTheta(wL, wR);
  
  rk3.x = calcX(state.theta + 0.5 * deltaT * rk2.theta, wL, wR);
  rk3.y = calcY(state.theta + 0.5 * deltaT * rk2.theta, wL, wR);
  rk3.theta = calcTheta(wL, wR);
  
  rk4.x = calcX(state.theta + deltaT * rk3.theta, wL, wR);
  rk4.y = calcY(state.theta + deltaT * rk3.theta, wL, wR);
  rk4.theta = calcTheta(wL, wR);
  
  state.x += (deltaT / 6.0) * (rk1.x + 2*rk2.x + 2*rk3.x + rk4.x);
  state.y += (deltaT / 6.0) * (rk1.y + 2*rk2.y + 2*rk3.y + rk4.y);
  state.theta += (deltaT / 6.0) * (rk1.theta + 2*rk2.theta + 2*rk3.theta + rk4.theta);
  
  return state;
}

void Control(double x, double y, double xd, double yd,
             double xo, double yo, double th,
             double &Vr, double &velang)
{
  // Distancia y ángulo hacia el objetivo
  double ex = xd - x;
  double ey = yd - y;
  double dg = sqrt(ex * ex + ey * ey);

  if (dg < distThresh) {
    Vr = 0;
    velang = 0;
    Kp = 0;
    Ki = 0;
    errGlobalA = 0;
    errGlobalB = 0;
    rpmTargetA = 0;
    rpmTargetB = 0;
    motorsStop();
    return;
  }

  double theta_g = atan2(ey, ex);

  // Vector de atracción
  double mag_a = (dg >= Kr) ? vMax : (vMax * dg / Kr);
  double Va_x = mag_a * cos(theta_g);
  double Va_y = mag_a * sin(theta_g);

  // Vector de repulsión
  double dxo = x - xo;
  double dyo = y - yo;
  double do_ = sqrt(dxo * dxo + dyo * dyo);

  double Vr_x_rep = 0.0;
  double Vr_y_rep = 0.0;
  if (do_ < Rrep) {
    double repMag = vMax * (Rrep - do_) / Rrep;
    double theta_rep = atan2(y - yo, x - xo);
    Vr_x_rep = repMag * cos(theta_rep);
    Vr_y_rep = repMag * sin(theta_rep);
  }

  double Vtot_x = Va_x + Vr_x_rep;
  double Vtot_y = Va_y + Vr_y_rep;

  double Vtot_mag = sqrt(Vtot_x * Vtot_x + Vtot_y * Vtot_y);
  double angleDesired = atan2(Vtot_y, Vtot_x);
  if (Vtot_mag > vMax) {
    Vtot_mag = vMax;
  }

  Vr = Vtot_mag;
  double angleError = angleDesired - th;
  velang = omax * sin(angleError);
}

// Velocidades de las ruedas
void V_Ll(double Vr, double velang, double &phi_L, double &phi_R) {
  phi_L = (Vr / r) - (s / (2 * r)) * velang;
  phi_R = (Vr / r) + (s / (2 * r)) * velang;
}

void IRAM_ATTR encHandlerA() {
  unsigned long curTime = micros();
  if (curTime - lastTickA > debounceMicros) {
    pulseCountA++;
    lastTickA = curTime;
  }
}

void IRAM_ATTR encHandlerB() {
  unsigned long curTime = micros();
  if (curTime - lastTickB > debounceMicros) {
    pulseCountB++;
    lastTickB = curTime;
  }
}

void motorASet(int pwm) {
  analogWrite(IN1_A, (pwm > 0) ? pwm : 0);
  analogWrite(IN2_A, (pwm < 0) ? -pwm : 0);
}

void motorBSet(int pwm) {
  analogWrite(IN1_B, (pwm > 0) ? pwm : 0);
  analogWrite(IN2_B, (pwm < 0) ? -pwm : 0);
}

void motorsStop() {
  analogWrite(IN1_A, 0);
  analogWrite(IN2_A, 0);
  analogWrite(IN1_B, 0);
  analogWrite(IN2_B, 0);
}

void rpmUpdate() {
  angRPM_A = (pulseCountA / pulsesRev) * (60.0 / sampleTimeSec);
  angRPM_B = (pulseCountB / pulsesRev) * (60.0 / sampleTimeSec);
  pulseCountA = 0;
  pulseCountB = 0;
}

void PID() {
  rpmUpdate();
  
  float errLocalA = rpmTargetA - angRPM_A;
  float errLocalB = rpmTargetB - angRPM_B;
  
  errSumA += errLocalA * sampleTimeSec;
  errSumB += errLocalB * sampleTimeSec;
  
  pidOutputA = Kp * errLocalA + Ki * errSumA;
  pidOutputB = Kp * errLocalB + Ki * errSumB;
  
  pidOutputA = constrain(pidOutputA, -255, 255);
  pidOutputB = constrain(pidOutputB, -255, 255);
  
  motorASet(pidOutputA);
  motorBSet(pidOutputB);
}


void setup() {
  Serial.begin(115200);

  pinMode(IN1_A, OUTPUT);
  pinMode(IN2_A, OUTPUT);
  pinMode(IN1_B, OUTPUT);
  pinMode(IN2_B, OUTPUT);
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_A), encHandlerA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), encHandlerB, CHANGE);

  // Inicializar WiFi
  Serial.println();
  Serial.println("Conectando a WiFi...");
  WiFi.begin(wifiSSID, wifiPass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi conectado!");
  Serial.print("Dirección IP: ");
  Serial.println(WiFi.localIP());

  // Iniciar el servidor
  wifiServer.begin();
  Serial.println("Servidor iniciado en el puerto 1234");
}


void loop() {
  static unsigned long lastPIDTime = 0;
  unsigned long curMillis = millis();

  // Recepción de datos vía WiFi (JSON)
  WiFiClient clientConn = wifiServer.available();
  if (clientConn) {
    Serial.println("\nCliente conectado");
    String jsonMsg = "";
    unsigned long msgStartTime = millis();
  
    while (clientConn.connected() && (millis() - msgStartTime < 5000)) {
      while (clientConn.available()) {
        char c = clientConn.read();
        
        if (c == '\n') {
          break;
        }
        jsonMsg += c;
      }
      if (jsonMsg.length() > 0 && jsonMsg.endsWith("\n")) {
        break;
      }
    }
    
    if (jsonMsg.length() > 0) {
      Serial.println("JSON recibido: " + jsonMsg);
      // Definir el tamaño del buffer para el JSON (4 pares clave-valor + margen)
      const size_t capacity = JSON_OBJECT_SIZE(4) + 60;
      DynamicJsonDocument doc(capacity);
      
      DeserializationError error = deserializeJson(doc, jsonMsg);
      if (!error) {
        float objX = doc["x_des"];
        float objY = doc["y_des"];
        float obsX = doc["x_object"];
        float obsY = doc["y_object"];
        
        xd = objX;
        yd = objY;
        xo = obsX;
        yo = obsY;
        
        coordsFlag = true;
        
        Serial.print("Objective: (");
        Serial.print(xd);
        Serial.print(", ");
        Serial.print(yd);
        Serial.print(") | Obstacle: (");
        Serial.print(xo);
        Serial.print(", ");
        Serial.print(yo);
        Serial.println(")");
        
        clientConn.println("Datos recibidos");
      } else {
        Serial.print("Error al parsear JSON: ");
        Serial.println(error.c_str());
        clientConn.println("Error en JSON");
      }
    }
    delay(10);
    clientConn.stop();
    Serial.println("Cliente desconectado");
  }
  
  if (curMillis - lastPIDTime >= (unsigned long)(sampleTimeSec * 1000)) {
    if (coordsFlag) {
      PID();
    } else {
      rpmTargetA = 0;
      rpmTargetB = 0;
      motorsStop();
    }
    lastPIDTime = curMillis;
  }
  
  static unsigned long lastSimTick = 0;
  if (millis() - lastSimTick >= (unsigned long)(simDt * 1000)) {
    double Vr = 0;
    double vAng = 0;
    
    if (coordsFlag) {
      Control(rbtState.x, rbtState.y, xd, yd, xo, yo, rbtState.theta, Vr, vAng);
    } else {
      Vr = 0;
      vAng = 0;
    }
    
    double wLeft, wRight;
    V_Ll(Vr, vAng, wLeft, wRight);
    
    rbtState = updateState(rbtState, wLeft, wRight, simDt);
    
    // Conversión de rad/s a RPM
    rpmTargetA = wLeft * (60.0 / (2.0 * PI)) * 0.985; //Valor de 0.985 constante de calibración propuesto
    rpmTargetB = wRight * (60.0 / (2.0 * PI));
    
    Serial.print("Sim -> x: "); Serial.print(rbtState.x);
    Serial.print(", y: "); Serial.print(rbtState.y);
    Serial.print(", theta: "); Serial.println(rbtState.theta);
    Serial.print("Target RPM A: "); Serial.print(rpmTargetA);
    Serial.print(", Target RPM B: "); Serial.println(rpmTargetB);
    
    lastSimTick = millis();
  }
  
  delay(1);
}