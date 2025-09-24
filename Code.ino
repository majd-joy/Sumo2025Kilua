#include <EEPROM.h>

// ===== إعدادات الأجهزة =====
const int TRIG[] = {2, 4, 16}, ECHO[] = {15, 17, 5};        // 3 مستشعرات مسافة
const int LINE[] = {36, 39, 34, 35};                        // 4 مستشعرات خط
const int ML1=25, ML2=26, MR1=32, MR2=33, MPWM1=27, MPWM2=14; // محركات

// ===== متغيرات الذكاء الاصطناعي =====
float weights[24];           // أوزان بسيطة للشبكة (8 inputs × 3 hidden)
float sensors[8];            // حالة المستشعرات
float epsilon = 0.2;         // معامل الاستطلاع
int wins = 0, battles = 0;   // إحصائيات

void setup() {
  Serial.begin(115200);
  EEPROM.begin(512);
  
  // إعداد المحركات
  pinMode(ML1, OUTPUT); pinMode(ML2, OUTPUT); pinMode(MPWM1, OUTPUT);
  pinMode(MR1, OUTPUT); pinMode(MR2, OUTPUT); pinMode(MPWM2, OUTPUT);
  
  // إعداد مستشعرات المسافة
  for(int i=0; i<3; i++) {
    pinMode(TRIG[i], OUTPUT); 
    pinMode(ECHO[i], INPUT);
  }
  
  loadAI();  // تحميل الذكاء المحفوظ
  Serial.println("🤖 Smart Sumo Robot Ready!");
}

void loop() {
  readSensors();                    // قراءة المستشعرات
  if(checkEdge()) { avoidEdge(); return; }  // تجنب الحافة
  
  int action = aiDecision();        // قرار ذكي
  executeAction(action);            // تنفيذ الفعل
  
  float reward = getReward(action); // حساب المكافأة
  learn(action, reward);            // التعلم
  
  delay(100);
}

// ===== قراءة المستشعرات =====
void readSensors() {
  // مستشعرات المسافة (0-2)
  for(int i=0; i<3; i++) {
    digitalWrite(TRIG[i], HIGH); delayMicroseconds(10); digitalWrite(TRIG[i], LOW);
    float dist = pulseIn(ECHO[i], HIGH, 30000) * 0.034 / 2;
    sensors[i] = constrain(1.0 - dist/300.0, 0, 1); // قريب = 1, بعيد = 0
  }
  
  // مستشعرات الخط (3-6)
  for(int i=0; i<4; i++) {
    sensors[3+i] = analogRead(LINE[i]) / 4095.0;
  }
  
  // زاوية العدو (7)
  sensors[7] = (sensors[1] > sensors[2]) ? 0.25 : 0.75; // يسار أم يمين
}

// ===== اتخاذ القرار الذكي =====
int aiDecision() {
  // استطلاع عشوائي أم استغلال الذكاء؟
  if(random(1000) < epsilon * 1000) {
    return random(0, 6); // فعل عشوائي
  }
  
  // حساب درجات الأفعال باستخدام الشبكة العصبية
  float scores[6] = {0};
  for(int action=0; action<6; action++) {
    for(int i=0; i<8; i++) {
      scores[action] += sensors[i] * weights[i*3 + (action%3)];
    }
  }
  
  // اختيار أفضل فعل
  int bestAction = 0;
  for(int i=1; i<6; i++) {
    if(scores[i] > scores[bestAction]) bestAction = i;
  }
  
  return bestAction;
}

// ===== تنفيذ الأفعال =====
void executeAction(int action) {
  switch(action) {
    case 0: motor(0, 0);      break; // توقف
    case 1: motor(255, 255);  break; // أمام سريع
    case 2: motor(150, 255);  break; // قطري يسار
    case 3: motor(255, 150);  break; // قطري يمين
    case 4: motor(-100, 100); break; // دوران
    case 5: motor(-200, -200);break; // تراجع
  }
}

void motor(int left, int right) {
  // المحرك الأيسر
  digitalWrite(ML1, left > 0); digitalWrite(ML2, left < 0);
  analogWrite(MPWM1, abs(left));
  
  // المحرك الأيمن
  digitalWrite(MR1, right > 0); digitalWrite(MR2, right < 0);
  analogWrite(MPWM2, abs(right));
}

// ===== حساب المكافأة =====
float getReward(int action) {
  float reward = 0;
  
  // مكافأة للاقتراب من العدو
  reward += sensors[0] * 10;
  
  // مكافأة خاصة للهجمات القطرية
  if(action == 2 || action == 3) {
    if(sensors[0] > 0.5) reward += 20; // عدو قريب
  }
  
  // عقوبة للحافة
  float minEdge = min(min(sensors[3], sensors[4]), min(sensors[5], sensors[6]));
  if(minEdge > 0.8) reward -= 30; // قريب من الحافة
  
  return reward;
}

// ===== التعلم البسيط =====
void learn(int action, float reward) {
  // تحديث بسيط للأوزان
  float learningRate = 0.01;
  for(int i=0; i<8; i++) {
    weights[i*3 + (action%3)] += learningRate * reward * sensors[i];
    
    // تحديد حدود الأوزان
    weights[i*3 + (action%3)] = constrain(weights[i*3 + (action%3)], -2.0, 2.0);
  }
  
  // تقليل الاستطلاع تدريجياً
  epsilon *= 0.9999;
  if(epsilon < 0.05) epsilon = 0.05;
  
  // حفظ كل فترة
  static int saveCounter = 0;
  if(++saveCounter > 100) {
    saveAI();
    saveCounter = 0;
  }
}

// ===== تجنب الحافة =====
bool checkEdge() {
  for(int i=3; i<7; i++) {
    if(sensors[i] > 0.8) return true; // حافة مكتشفة
  }
  return false;
}

void avoidEdge() {
  motor(0, 0); delay(50);           // توقف
  motor(-255, -255); delay(200);    // تراجع
  motor(-150, 150); delay(300);     // دوران
  motor(0, 0); delay(100);          // توقف
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
    if(isnan(weights[i])) weights[i] = random(-100, 100) / 100.0; // قيمة افتراضية
  }
  EEPROM.get(100, epsilon);
  EEPROM.get(104, wins);
  EEPROM.get(108, battles);
  
  if(isnan(epsilon) || epsilon > 1.0) epsilon = 0.2;
  if(battles > 0) {
    Serial.printf("📊 Loaded: %d battles, %d wins (%.1f%%)\n", 
                  battles, wins, (float)wins/battles*100);
  }
}