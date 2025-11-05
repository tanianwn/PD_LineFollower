const int MOTOR_A_IN1 = 18;
const int MOTOR_A_IN2 = 19;
const int MOTOR_B_IN1 = 20;
const int MOTOR_B_IN2 = 21;

const int ENCA1 = 2;
const int ENCA2 = 3;
const int ENCB1 = 23;
const int ENCB2 = 22;

const int MUX_S0 = 4;
const int MUX_S1 = 5;
const int MUX_S2 = 0;
const int MUX_Y  = 1;

const int START_BUTTON = 14;
bool robotActivo = false;
bool lastButtonState = HIGH;

const int NUM_SENSORS = 8;
int sensorValues[NUM_SENSORS];

const float Kp = 9.9;
const float Ki = 0.00000115;
const float Kd = 1.5;

const float SENSOR_WEIGHTS[NUM_SENSORS] = {-4, -4, -3, -2.8, 0, 3, 4, 6};

const int BASE_SPEED = 255;
const int MAX_PWM = 255;

float prev_error = 0.0;
float integral = 0.0;
float filtered_deriv = 0.0;
unsigned long last_time = 0;

unsigned long tiempoNegro = 0;

float left_speed = BASE_SPEED;
float right_speed = BASE_SPEED;

bool lineLost = false;
unsigned long lineLostTime = 0;
float lastValidError = 0.0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Seguidor de Líneas Iniciado");

  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);

  pinMode(ENCA1, INPUT);
  pinMode(ENCA2, INPUT);
  pinMode(ENCB1, INPUT);
  pinMode(ENCB2, INPUT);

  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_Y, INPUT);

  pinMode(START_BUTTON, INPUT_PULLUP);

  Serial.println("Sistema listo. Presiona el botón para iniciar 🚀");
  last_time = millis();
}

void loop() {
  int buttonState = digitalRead(START_BUTTON);

  if (buttonState == LOW && lastButtonState == HIGH) {
    robotActivo = !robotActivo;
    if (robotActivo) Serial.println("🚗 Robot activado");
    else {
      Serial.println("🛑 Robot detenido");
      detenerMotores();
    }
    delay(200);
  }

  lastButtonState = buttonState;
  if (!robotActivo) return;

  readIRSensors();

  if (detectarNegro0_3s()) {
    Serial.println("🎯 Negro detectado 0.3s. Deteniendo...");
    detenerMotores();
    robotActivo = false;
    return;
  }

  float error = calculateError();
  unsigned long now = millis();
  float dt = (now - last_time) / 1000.0;
  last_time = now;

  bool lineaDetectada = false;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorValues[i] == 1) {
      lineaDetectada = true;
      break;
    }
  }

  float control = pidControl(error, dt);
  float factor = 1.0;
  if (fabs(error) > 0.6) factor = 0.7;
  if (fabs(error) > 0.9) factor = 0.5;

  if (sensoresMediosActivos()) {
    left_speed  = BASE_SPEED;
    right_speed = BASE_SPEED;
  } 
  else if (sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 0) {
    left_speed  = 30;
    right_speed = 255;
  } 
  else {
    float giroFactor = 1.3;
    left_speed  = (BASE_SPEED - control * giroFactor) * factor;
    right_speed = (BASE_SPEED + control * giroFactor) * factor;
  }

  left_speed  = constrain(left_speed, 0, MAX_PWM);
  right_speed = constrain(right_speed, 0, MAX_PWM);

  setMotorSpeeds(left_speed, right_speed);
  delay(20);
}

void readIRSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    digitalWrite(MUX_S0, i & 0x01);
    digitalWrite(MUX_S1, (i >> 1) & 0x01);
    digitalWrite(MUX_S2, (i >> 2) & 0x01);
    delayMicroseconds(150);
    sensorValues[i] = digitalRead(MUX_Y);
  }
}

bool detectarNegro0_3s() {
  bool todosNegros = true;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorValues[i] == 1) {
      todosNegros = false;
      break;
    }
  }

  if (todosNegros) {
    if (tiempoNegro == 0) tiempoNegro = millis();
    else if (millis() - tiempoNegro >= 250) return true; 
  } else {
    tiempoNegro = 0;
  }
  return false;
}

float calculateError() {
  float weighted_sum = 0.0;
  int active_count = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorValues[i] == 1) {
      weighted_sum += SENSOR_WEIGHTS[i];
      active_count++;
    }
  }

  if (active_count > 0) {
    float error = constrain(weighted_sum / active_count, -5.0, 5.0) / 5.0;
    lastValidError = error;
    lineLost = false;
    return error;
  } else {
    if (!lineLost) {
      lineLost = true;
      lineLostTime = millis();
    }

    if (millis() - lineLostTime < 100) return lastValidError;
    return 0.0;
  }
}

float pidControl(float error, float dt) {
  float Kp_eff = Kp;
  float Kd_eff = Kd;

  if (lineLost) {
    Kd_eff = Kd * 0.4;
  }

  if (sensorValues[0] == 1 || sensorValues[1] == 1 || sensorValues[6] == 1 || sensorValues[7] == 1) {
    Kp_eff = Kp * 2.0;
  } 
  else if (sensoresMediosActivos()) {
    Kp_eff = Kp;
  }
  else {
    if (fabs(error) > 0.6) Kp_eff = Kp * 1.8;
    if (fabs(error) > 0.9) Kp_eff = Kp * 0.8;
  }

  integral += error * dt;
  float derivative = (error - prev_error) / dt;
  filtered_deriv = 0.2 * derivative + 0.8 * filtered_deriv;
  prev_error = error;

  return (Kp_eff * error + Ki * integral + Kd_eff * filtered_deriv) * 70;
}

void setMotorSpeeds(int left_speed_val, int right_speed_val) {
  if (left_speed_val >= 0) {
    digitalWrite(MOTOR_A_IN2, LOW);
    analogWrite(MOTOR_A_IN1, left_speed_val);
  } else {
    digitalWrite(MOTOR_A_IN2, HIGH);
    analogWrite(MOTOR_A_IN1, 255 + left_speed_val);
  }

  if (right_speed_val >= 0) {
    digitalWrite(MOTOR_B_IN2, HIGH);
    analogWrite(MOTOR_B_IN1, 255 - right_speed_val);
  } else {
    digitalWrite(MOTOR_B_IN2, LOW);
    analogWrite(MOTOR_B_IN1, -right_speed_val);
  }
}

void detenerMotores() {
  analogWrite(MOTOR_A_IN1, 0);
  analogWrite(MOTOR_B_IN1, 0);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN2, LOW);
}

bool sensoresMediosActivos() {
  bool sensor3 = (sensorValues[3] == 1);
  bool sensor4 = (sensorValues[4] == 1);

  int otrosActivos = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (i != 3 && i != 4 && sensorValues[i] == 1) {
      otrosActivos++;
    }
  }

  if ((sensor3 || sensor4) && otrosActivos == 0) {
    return true;
  }

  return false;
}
