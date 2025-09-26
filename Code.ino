#include <EEPROM.h>

// ===== إعدادات الأجهزة =====
const int TRIG[] = {2, 4, 16}, ECHO[] = {15, 17, 5};        // 3 مستشعرات مسافة
const int LINE[] = {36, 39, 34, 35};                        // 4 مستشعرات خط
const int ML1=25, ML2=26, MR1=32, MR2=33, MPWM1=27, MPWM2=14; // محركات

// ===== متغيرات الذكاء الاصطناعي =====
float weights[24];           
float sensors[8];   // 7 حساسات + قيمة مشتقة (زاوية العدو)         
float epsilon = 0.2;         
int wins = 0, battles = 0;   

// ===== متغيرات الانتظار =====
unsigned long startTime;
bool started = false;

// ===== متغيرات تجنب الحافة (بدون delay) =====
bool avoiding = false;
int avoidStep = 0;
unsigned long avoidStart = 0;

void setup() {
  // Serial.begin(115200); // غير مفعّل
  EEPROM.begin(512);
  
  // إعداد المحركات
  pinMode(ML1, OUTPUT); pinMode(ML2, OUTPUT); pinMode(MPWM1, OUTPUT);
  pinMode(MR1, OUTPUT); pinMode(MR2, OUTPUT); pinMode(MPWM2, OUTPUT);
  
  // إعداد مستشعرات المسافة
  for(int i=0; i<3; i++) {
    pinMode(TRIG[i], OUTPUT); 
    pinMode(ECHO[i], INPUT);
  }
  
  loadAI();  
  // Serial.println("🤖 Smart Sumo Robot Ready! Waiting 5s...");
  
  startTime = millis(); 
}

void loop() {
  // يقرأ الحساسات ويقرر أثناء الانتظار
  readSensors();
  int action = aiDecision();

  // انتظار 5 ثواني قبل البدء
  if(!started && millis() - startTime < 5000) {
    motor(0, 0); 
    return;
  }
  started = true;

  // إذا كان في حالة تجنب الحافة -> يكمل الروتين
  if(avoiding) {
    handleAvoidEdge();
    return;
  }

  if(checkEdge()) { startAvoidEdge(); return; }  
  
  executeAction(action);            
  float reward = getReward(action); 
  learn(action, reward);            
}

// ===== قراءة المستشعرات =====
void readSensors() {
  for(int i=0; i<3; i++) {
    digitalWrite(TRIG[i], HIGH); delayMicroseconds(10); digitalWrite(TRIG[i], LOW);
    float dist = pulseIn(ECHO[i], HIGH, 30000) * 0.034 / 2;
    sensors[i] = constrain(1.0 - dist/300.0, 0, 1); 
  }
  
  for(int i=0; i<4; i++) {
    sensors[3+i] = analogRead(LINE[i]) / 4095.0;
  }
  
  sensors[7] = (sensors[1] > sensors[2]) ? 0.25 : 0.75; 
}

// ===== اتخاذ القرار الذكي =====
int aiDecision() {
  if(random(1000) < epsilon * 1000) {
    return random(0, 6); 
  }
  
  float scores[6] = {0};
  for(int action=0; action<6; action++) {
    for(int i=0; i<8; i++) {
      scores[action] += sensors[i] * weights[i*3 + (action%3)];
    }
  }
  
  int bestAction = 0;
  for(int i=1; i<6; i++) {
    if(scores[i] > scores[bestAction]) bestAction = i;
  }
  
  return bestAction;
}

// ===== تنفيذ الأفعال =====
void executeAction(int action) {
  switch(action) {
    case 0: motor(0, 0);      break; 
    case 1: motor(255, 255);  break; 
    case 2: motor(150, 255);  break; 
    case 3: motor(255, 150);  break; 
    case 4: motor(-100, 100); break; 
    case 5: motor(-200, -200);break; 
  }
}

void motor(int left, int right) {
  digitalWrite(ML1, left > 0); digitalWrite(ML2, left < 0);
  analogWrite(MPWM1, abs(left));
  
  digitalWrite(MR1, right > 0); digitalWrite(MR2, right < 0);
  analogWrite(MPWM2, abs(right));
}

// ===== حساب المكافأة =====
float getReward(int action) {
  float reward = 0;
  reward += sensors[0] * 10;
  
  if(action == 2 || action == 3) {
    if(sensors[0] > 0.5) reward += 20; 
  }
  
  float minEdge = min(min(sensors[3], sensors[4]), min(sensors[5], sensors[6]));
  if(minEdge > 0.8) reward -= 30; 
  
  return reward;
}

// ===== التعلم =====
void learn(int action, float reward) {
  float learningRate = 0.01;
  for(int i=0; i<8; i++) {
    weights[i*3 + (action%3)] += learningRate * reward * sensors[i];
    weights[i*3 + (action%3)] = constrain(weights[i*3 + (action%3)], -2.0, 2.0);
  }
  
  epsilon *= 0.9999;
  if(epsilon < 0.05) epsilon = 0.05;
  
  static int saveCounter = 0;
  if(++saveCounter > 100) {
    saveAI();
    saveCounter = 0;
  }
}

// ===== تجنب الحافة بدون delay =====
bool checkEdge() {
  for(int i=3; i<7; i++) {
    if(sensors[i] > 0.8) return true; 
  }
  return false;
}

void startAvoidEdge() {
  avoiding = true;
  avoidStep = 0;
  avoidStart = millis();
  motor(0, 0); // أول شيء يوقف
}

void handleAvoidEdge() {
  unsigned long now = millis();
  switch(avoidStep) {
    case 0: 
      if(now - avoidStart >= 50) {
        motor(-255, -255); 
        avoidStep = 1; 
        avoidStart = now;
      }
      break;
    case 1: 
      if(now - avoidStart >= 200) {
        motor(-150, 150); 
        avoidStep = 2; 
        avoidStart = now;
      }
      break;
    case 2: 
      if(now - avoidStart >= 300) {
        motor(0, 0); 
        avoiding = false; // خلص
      }
      break;
  }
}

// ===== حفظ وتحميل الذكاء =====
void saveAI() {
  for(int i=0; i<24; i++) {
    EEPROM.put(i*4, weights[i]);
  }
  EEPROM.put(100, epsilon);
  EEPROM.put(104, wins);
  EEPROM.put(108, battles);
  EEPROM.commit();
}

void loadAI() {
  for(int i=0; i<24; i++) {
    EEPROM.get(i*4, weights[i]);
    if(isnan(weights[i])) weights[i] = random(-100, 100) / 100.0; 
  }
  EEPROM.get(100, epsilon);
  EEPROM.get(104, wins);
  EEPROM.get(108, battles);
  
  if(isnan(epsilon) || epsilon > 1.0) epsilon = 0.2;
  // if(battles > 0) {
  //   Serial.printf("📊 Loaded: %d battles, %d wins (%.1f%%)\n", 
  //                 battles, wins, (float)wins/battles*100);
  // }
}
