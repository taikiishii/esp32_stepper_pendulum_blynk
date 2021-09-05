#include <Wire.h>
#include <BlynkSimpleEsp32_BLE.h>
#include <BLEDevice.h>
#include <BLEServer.h>

#define BLYNK_USE_DIRECT_CONNECT

// Blynkからメールで受け取った認証トークンに書き換える
char auth[] = "lobYh1dF_GldbL1c8f6fBZufJU6Ge2zV";

// k1:傾き　k2:倒れる速度　k3:位置　k4:移動速度
volatile float k1 = 300, k2 = 25, k3 = 35, k4 = 20; //マイクロステップなし

#define DIR_L   32
#define STEP_L  25
#define DIR_R   27
#define STEP_R  33
#define DIR_C   26
#define STEP_C  14
#define EN      13
#define SDA     21
#define SCL     22
#define BATTERY_MONITOR 4

#define PULSE_PERIOD   20   // パルス生成用のtimer1の周期 マイクロ秒単位
#define CONTROL_PERIOD 4000  // 制御ループ用のtimer2の周期 マイクロ秒単位 最小4000で、それ以下だと立たない
#define LIMIT 450            // ステッピングモータの最大スピード
#define STEPS 120            // 1回転のSTEPS
#define MICRO_STEP 1         // ステッピングモータのマイクロステップの逆数 (1/4 -> 4)

// 加速度・ジャイロセンサーの制御定数
#define MPU6050_ADDR         0x68 // MPU-6050 device address
#define MPU6050_SMPLRT_DIV   0x19 // MPU-6050 register address
#define MPU6050_CONFIG       0x1a
#define MPU6050_GYRO_CONFIG  0x1b
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_ACCEL_XOUT_H 0x3b
#define MPU6050_ACCEL_XOUT_L 0x3c
#define MPU6050_ACCEL_YOUT_H 0x3d
#define MPU6050_ACCEL_YOUT_L 0x3e
#define MPU6050_ACCEL_ZOUT_H 0x3f
#define MPU6050_ACCEL_ZOUT_L 0x40
#define MPU6050_GYRO_XOUT_H  0x43
#define MPU6050_GYRO_XOUT_L  0x44
#define MPU6050_GYRO_YOUT_H  0x45
#define MPU6050_GYRO_YOUT_L  0x46
#define MPU6050_GYRO_ZOUT_H  0x47
#define MPU6050_GYRO_ZOUT_L  0x48
#define MPU6050_PWR_MGMT_1   0x6b
#define MPU6050_WHO_AM_I     0x75

// timer1はパルス生成用、timer2は制御ループ用
hw_timer_t * timer1 = NULL;
hw_timer_t * timer2 = NULL;
volatile SemaphoreHandle_t timerSemaphore1;
volatile SemaphoreHandle_t timerSemaphore2;
portMUX_TYPE timerMux1 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMux2 = portMUX_INITIALIZER_UNLOCKED;

volatile bool outL = false, outR = false;
volatile int16_t countL, countR;
volatile int16_t controlSpeed;
volatile int16_t moveSpeed, turnSpeed;
volatile int16_t motorL, motorR;
volatile int16_t rawAccX, rawAccZ;
volatile int16_t rawGyroY;
volatile int16_t temperature;

volatile int32_t lastUptime, startTime, currentTime;
volatile int32_t nextBlynkRunTime;

volatile float dt;
volatile float caribAccAngleY;
volatile float caribGyroY;
volatile float accX, accZ;
volatile float accAngleY;
volatile float gyroY;
volatile float dpsY, degY, pos;
volatile float batteryMonitor;

int headStand = 0;

void startStand() {
  degY = 0;
  pos = 0;
  controlSpeed = 0;
  motorL = 0; motorR = 0;
  countL = 0; countR = 0;
  moveSpeed = 0;
  turnSpeed = 0;

  nextBlynkRunTime = micros() + 100000; //100ms

  // 制御パラメータをスマホのBlynkアプリに送信
  Blynk.virtualWrite(V0, k1);
  Blynk.virtualWrite(V1, k2);
  Blynk.virtualWrite(V2, k3);
  Blynk.virtualWrite(V3, k4);
  Blynk.virtualWrite(V4, 0);
  Blynk.virtualWrite(V5, 0);

  //ステッピングモータを有効化し倒立制御開始
  digitalWrite(EN, LOW);
  headStand = 1;
  Serial.println("GO!");

  // dt計測用
  lastUptime = micros();

  //　倒立時間計測用
  startTime = micros();
}

void IRAM_ATTR onTimer1() {
  countL += motorL * MICRO_STEP * STEPS / 480;
  if (countL > 5000) {
    digitalWrite(STEP_L, outL);
    outL = !outL;
    countL -= 5000;
  }
  else if (countL < -5000) {
    digitalWrite(STEP_L, outL);
    outL = !outL;
    countL += 5000;
  }

  countR += motorR * MICRO_STEP * STEPS / 480;
  if (countR > 5000) {
    digitalWrite(STEP_R, outR);
    outR = !outR;
    countR -= 5000;
  }
  else   if (countR < -5000) {
    digitalWrite(STEP_R, outR);
    outR = !outR;
    countR += 5000;
  }

  //xSemaphoreGiveFromISR(timerSemaphore1, NULL);
}

void IRAM_ATTR onTimer2() {
  xSemaphoreGiveFromISR(timerSemaphore2, NULL);
}

// センサーへのコマンド送信
void writeMPU6050(byte reg, byte data) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

// センサーからのデータ読み込み
byte readMPU6050(byte reg) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 1, true);
  while (! Wire.available()) {
    //Serial.println("Waiting for Wire.available（）");
  }
  byte data =  Wire.read();
  Wire.endTransmission(true);
  return data;
}

void setup() {
  Blynk.setDeviceName("ESP32");
  Blynk.begin(auth);

  Serial.begin(115200);
  Serial.println("*******************RESTARTED********************");
  Serial.println("*****************stepper_pendulum***************");
  Serial.print(k1); Serial.print(","); Serial.print(k2); Serial.print(","); Serial.print(k3); Serial.print(","); Serial.print(k4); Serial.println();

  // モータードライバ制御用ピンの初期化
  pinMode(DIR_L, OUTPUT);
  pinMode(STEP_L, OUTPUT);
  pinMode(DIR_R, OUTPUT);
  pinMode(STEP_R, OUTPUT);
  pinMode(DIR_C, OUTPUT);
  pinMode(STEP_C, OUTPUT);
  pinMode(EN, OUTPUT);

  // バッテリ電圧の監視
  pinMode(BATTERY_MONITOR, ANALOG);
  analogSetAttenuation(ADC_11db);

  if (analogReadMilliVolts(BATTERY_MONITOR) < 800) {
    while (true) {
      Serial.print("LOW BATTERY.");
      Serial.println(analogReadMilliVolts(BATTERY_MONITOR));
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
  }

  // センサーの初期化
  Wire.setClock(400000);
  Wire.begin(SDA, SCL);

  if (readMPU6050(MPU6050_WHO_AM_I) != 0x68) {
    while (true) {
      Serial.println("WHO_AM_I error.");
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
  }
  else {
    Serial.println("WHO_AM_I OK.");
  }

  writeMPU6050(MPU6050_SMPLRT_DIV, 0x07);   // sample rate: 8kHz/(7+1) = 1kHz
  writeMPU6050(MPU6050_CONFIG, 0x00);       // disable DLPF, gyro output rate = 8kHz
  writeMPU6050(MPU6050_GYRO_CONFIG, 0x00);  // gyro range: 0x00⇒±250dps 131LSB、0x08⇒±500dps 65.5LSB、0x10⇒±1000dps 32.8LSB、0x18⇒±2000dps 16.4LSB
  writeMPU6050(MPU6050_ACCEL_CONFIG, 0x00); // accel range: 0x00⇒±2g 16384LSB/g、0x01⇒±4g 8192LSB/g、0x02⇒±8g 4096LSB/g、0x03⇒±16g 2048LSB/g、
  writeMPU6050(MPU6050_PWR_MGMT_1, 0x01);   // disable sleep mode, PLL with X gyro
  Serial.println("MPU6050 Setup OK.");
  delay(2000);

  //ジャイロのゼロ点調整のために静止時の出力を1000回計測して平均を算出
  caribAccAngleY = 0;
  caribGyroY = 0;
  for (int i = 0; i < 1000  ; i++)  {
    rawAccX = (readMPU6050(MPU6050_ACCEL_XOUT_H) << 8) | readMPU6050(MPU6050_ACCEL_XOUT_L);
    rawAccZ = (readMPU6050(MPU6050_ACCEL_ZOUT_H) << 8) | readMPU6050(MPU6050_ACCEL_ZOUT_L);
    rawGyroY = (readMPU6050(MPU6050_GYRO_YOUT_H) << 8) | readMPU6050(MPU6050_GYRO_YOUT_L);

    accX = (float)rawAccX / 16384.0;
    accZ = (float)rawAccZ / 16384.0;
    accAngleY = atan2(accX, accZ) * 360 / -2.0 / PI;
    caribAccAngleY += accAngleY;
    caribGyroY += (float)rawGyroY;
  }
  caribAccAngleY /= 1000;
  caribGyroY /= 1000;
  Serial.print("Carib OK. caribGyroY:");
  Serial.println(caribGyroY);

  //割込みタイマの設定
  timerSemaphore1 = xSemaphoreCreateBinary();
  timer1 = timerBegin(0, getApbFrequency() / 1000000, true); // timer=1us
  timerAttachInterrupt(timer1, &onTimer1, true);
  timerAlarmWrite(timer1, PULSE_PERIOD, true);
  timerAlarmEnable(timer1);

  timerSemaphore2 = xSemaphoreCreateBinary();
  timer2 = timerBegin(1, getApbFrequency() / 1000000, true); // timer=1us
  timerAttachInterrupt(timer2, &onTimer2, true);
  timerAlarmWrite(timer2, CONTROL_PERIOD, true);
  timerAlarmEnable(timer2);

  //倒立開始
  startStand();
}


void loop() {
  currentTime = micros();
  if (headStand == 1) {
    //if (xSemaphoreTake(timerSemaphore1, 0) == pdTRUE) {
    //}

    if (xSemaphoreTake(timerSemaphore2, 0) == pdTRUE) {
      // dt計測
      dt = (currentTime - lastUptime) * 0.000001;
      lastUptime = currentTime;

      // 加速度、角速度を取得
      //rawAccX = (readMPU6050(MPU6050_ACCEL_XOUT_H) << 8) | readMPU6050(MPU6050_ACCEL_XOUT_L);
      //rawAccZ = (readMPU6050(MPU6050_ACCEL_ZOUT_H) << 8) | readMPU6050(MPU6050_ACCEL_ZOUT_L);
      //rawGyroY = (readMPU6050(MPU6050_GYRO_YOUT_H) << 8) | readMPU6050(MPU6050_GYRO_YOUT_L);

      Wire.beginTransmission(MPU6050_ADDR);
      Wire.write(MPU6050_ACCEL_XOUT_H);
      Wire.endTransmission(false);
      Wire.requestFrom(MPU6050_ADDR, 14, true);
      while (Wire.available() < 14);

      int16_t dummy;

      rawAccX = Wire.read() << 8 | Wire.read();
      dummy = Wire.read() << 8 | Wire.read();
      rawAccZ = Wire.read() << 8 | Wire.read();
      dummy = Wire.read() << 8 | Wire.read();
      dummy = Wire.read() << 8 | Wire.read();
      rawGyroY = Wire.read() << 8 | Wire.read();
      dummy = Wire.read() << 8 | Wire.read();

      accX = (float)rawAccX / 16384.0;
      accZ = (float)rawAccZ / 16384.0;
      accAngleY = atan2(accX, accZ) * 360 / -2.0 / PI - caribAccAngleY;

      gyroY = (float)rawGyroY - caribGyroY;
      dpsY =  gyroY / 131.0;

      // 相補フィルターで角度を求める
      degY = 0.999 * (degY + dpsY * dt) + 0.001 * accAngleY;

      // 位置制御
      pos += controlSpeed * dt;

      // 制御量の計算
      controlSpeed += (k1 * degY + k2 * dpsY + k3 * pos + k4 * controlSpeed) * dt;

      // 倒立制御 + 転回
      motorL = controlSpeed + turnSpeed;
      motorR = controlSpeed - turnSpeed;

      motorL = constrain(motorL, 0 - LIMIT * MICRO_STEP, LIMIT * MICRO_STEP);
      motorR = constrain(motorR, 0 - LIMIT * MICRO_STEP, LIMIT * MICRO_STEP);

      // ステッピングモータの回転方向を指定
      digitalWrite(DIR_L, (motorL < 0));
      digitalWrite(DIR_R, (motorR < 0));
    }

    // 倒れたらモーター停止
    if (45 < abs(degY)) {
      headStand = 0;
      Serial.println("*********************STOP***********************");
      Serial.println((lastUptime - startTime) / 1000000);
    }
  }
  else {
    motorL = 0;
    motorR = 0;
    digitalWrite(EN, HIGH);
  }

  //100msに1回だけ実行
  if (currentTime > nextBlynkRunTime) {
    // Blynkアプリから値取得
    Blynk.run();
    nextBlynkRunTime = micros() + 100000;

    // posにmoveSpeedを加算し、位置制御で前進・後退させる
    pos += moveSpeed;

    // バッテリ電圧の監視 1/4に分圧しているので3.0Vが閾値となる
    batteryMonitor = analogReadMilliVolts(BATTERY_MONITOR);
    if ( batteryMonitor < 750) {
      Serial.printf("LOW BATTERY. %d mV\n", batteryMonitor * 4);
      headStand = 0;
      while (true) {
        Serial.printf("LOW BATTERY. Min:%f mV, Now:%f mV\n", batteryMonitor * 4, analogReadMilliVolts(BATTERY_MONITOR) * 4);
        vTaskDelay(1000 / portTICK_RATE_MS);
      }
    }
  }
}


BLYNK_WRITE(V0) {
  k1 = (float) param[0].asInt();
}

BLYNK_WRITE(V1) {
  k2 = (float) param[0].asInt();
}

BLYNK_WRITE(V2) {
  k3 = (float) param[0].asInt();
}

BLYNK_WRITE(V3) {
  k4 = (float) param[0].asInt();
}
BLYNK_WRITE(V4) {
  // 転回スピード -30～30
  turnSpeed = param[0].asInt();
}

BLYNK_WRITE(V5) {
  // 前進・後退スピード -25～25
  moveSpeed = param[0].asInt();
}

BLYNK_WRITE(V6) {
  vTaskDelay(1000 / portTICK_RATE_MS);
  startStand();
}
