#include <EEPROM.h>
#include "esp32-hal-cpu.h" // مكتبة لتغيير تردد المعالج

// ===== إعدادات الأجهزة =====
const int MOTORS[2][2] = {
  {18, 4},   // المحرك الأول (RPWM, LPWM)
  {21, 19}   // المحرك الثاني (RPWM, LPWM)
};

// الحساسات (Front, Right, Left) → JS40F Digital
const int SENSORS[3] = {32, 33, 25};

// مستشعرات الخط (Analog)
const int LINE[] = {36, 39, 34, 35};

// ===== إعدادات الروبوت =====
const float LEARNING_RATE = 0.01;
const float EPSILON_MIN = 0.05;
const float EDGE_THRESHOLD = 0.8;
const unsigned long AVOID_TIMINGS[] = {50, 200, 300}; // بالمللي ثانية
const unsigned long START_DELAY = 5000; // 5 ثواني قبل بدء الحركة

// ===== متغيرات الذكاء الاصطناعي =====
float weights[24];        // 7 حساسات + زاوية العدو
float sensorsValues[8];   // 7 حساسات + زاوية العدو
float epsilon = 0.2;
int wins = 0, battles = 0;

// ===== حالة البداية =====
unsigned long startTime;
bool started = false;

// ===== حالة تجنب الحافة =====
struct EdgeAvoid {
  bool active = false;
  int step = 0;
  unsigned long start = 0;
} edgeAvoid;

// ===================== الوظائف =====================

// ضبط المحركات
void setMotors(int left, int right) {
  digitalWrite(MOTORS[0][1], left < 0);
  digitalWrite(MOTORS[0][0], left > 0);
  analogWrite(MOTORS[0][0], abs(left));

  digitalWrite(MOTORS[1][1], right < 0);
  digitalWrite(MOTORS[1][0], right > 0);
  analogWrite(MOTORS[1][0], abs(right));
}

void stopMotors() { setMotors(0, 0); }

// قراءة الحساسات
void readSensors() {
  // 🟢 الحساسات الأمامية (JS40F Digital → 0 أو 1)
  for (int i = 0; i < 3; i++) {
    sensorsValues[i] = digitalRead(SENSORS[i]);
  }

  // ⚪ حساسات الخط (Analog → 0 إلى 1)
  for (int i = 0; i < 4; i++) {
    sensorsValues[3+i] = analogRead(LINE[i]) / 4095.0;
  }

  // 🔵 زاوية العدو (يمين أو يسار حسب الحساسات)
  sensorsValues[7] = (sensorsValues[1] > sensorsValues[2]) ? 0.25 : 0.75;
}

// اتخاذ القرار باستخدام التعلم المعزز
int aiDecision() {
  if (random(1000) < epsilon * 1000) return random(0, 6);

  float scores[6] = {0};
  for (int action = 0; action < 6; action++) {
    for (int i = 0; i < 8; i++) {
      scores[action] += sensorsValues[i] * weights[i*3 + (action%3)];
    }
  }

  int best = 0;
  for (int i = 1; i < 6; i++) if (scores[i] > scores[best]) best = i;
  return best;
}

// تنفيذ الحركة
void executeAction(int action) {
  switch(action) {
    case 0: stopMotors(); break;
    case 1: setMotors(255, 255); break;
    case 2: setMotors(150, 255); break;
    case 3: setMotors(255, 150); break;
    case 4: setMotors(-100, 100); break;
    case 5: setMotors(-200, -200); break;
  }
}

// حساب المكافأة
float getReward(int action) {
  float reward = sensorsValues[0] * 10; // الأمامي
  if ((action == 2 || action == 3) && sensorsValues[0] > 0.5) reward += 20;

  float minEdge = min(min(sensorsValues[3], sensorsValues[4]),
                      min(sensorsValues[5], sensorsValues[6]));
  if (minEdge > EDGE_THRESHOLD) reward -= 30;

  return reward;
}

// التعلم
void learn(int action, float reward) {
  for (int i = 0; i < 8; i++) {
    weights[i*3 + (action%3)] += LEARNING_RATE * reward * sensorsValues[i];
    weights[i*3 + (action%3)] = constrain(weights[i*3 + (action%3)], -2.0, 2.0);
  }

  epsilon *= 0.9999;
  if (epsilon < EPSILON_MIN) epsilon = EPSILON_MIN;

  static int saveCounter = 0;
  if (++saveCounter > 100) { saveAI(); saveCounter = 0; }
}

// ===== تجنب الحافة بدون delay =====
bool checkEdge() {
  for (int i = 3; i < 7; i++) if (sensorsValues[i] > EDGE_THRESHOLD) return true;
  return false;
}

void startAvoidEdge() {
  edgeAvoid.active = true;
  edgeAvoid.step = 0;
  edgeAvoid.start = millis();
  stopMotors();
}

void handleAvoidEdge() {
  unsigned long now = millis();
  switch(edgeAvoid.step) {
    case 0:
      if (now - edgeAvoid.start >= AVOID_TIMINGS[0]) { setMotors(-255,-255); edgeAvoid.step=1; edgeAvoid.start=now; }
      break;
    case 1:
      if (now - edgeAvoid.start >= AVOID_TIMINGS[1]) { setMotors(-150,150); edgeAvoid.step=2; edgeAvoid.start=now; }
      break;
    case 2:
      if (now - edgeAvoid.start >= AVOID_TIMINGS[2]) { stopMotors(); edgeAvoid.active=false; }
      break;
  }
}

// ===== حفظ واسترجاع الذكاء =====
void saveAI() {
  for (int i = 0; i < 24; i++) EEPROM.put(i*4, weights[i]);
  EEPROM.put(100, epsilon);
  EEPROM.put(104, wins);
  EEPROM.put(108, battles);
  EEPROM.commit();
}

void loadAI() {
  for (int i = 0; i < 24; i++) {
    EEPROM.get(i*4, weights[i]);
    if (isnan(weights[i])) weights[i] = random(-100,100)/100.0;
  }
  EEPROM.get(100, epsilon);
  EEPROM.get(104, wins);
  EEPROM.get(108, battles);
  if (isnan(epsilon) || epsilon > 1.0) epsilon = 0.2;
}

// ===== setup و loop =====
void setup() {
  // ضبط تردد الـ CPU على أقصى سرعة
  setCpuFrequencyMhz(240);

  EEPROM.begin(512);

  // إعداد المحركات
  for (int i = 0; i < 2; i++) {
    pinMode(MOTORS[i][0], OUTPUT);
    pinMode(MOTORS[i][1], OUTPUT);
  }
  // إعداد الحساسات (JS40F Digital)
  for (int i = 0; i < 3; i++) pinMode(SENSORS[i], INPUT);
  // إعداد حساسات الخط
  for (int i = 0; i < 4; i++) pinMode(LINE[i], INPUT);

  loadAI();
  startTime = millis();
}

void loop() {
  readSensors();
  int action = aiDecision();

  if (!started && millis() - startTime < START_DELAY) {
    stopMotors();
    return;
  }
  started = true;

  if (edgeAvoid.active) { handleAvoidEdge(); return; }
  if (checkEdge()) { startAvoidEdge(); return; }

  executeAction(action);
  float reward = getReward(action);
  learn(action, reward);
}
