#include <PID_v1.h>
#include <PinChangeInterrupt.h>
#include<Wire.h>

// モータ制御ピン 
const int motorPWM = 6;    // モータA PWM
const int motorDIR = 7;    // モータA 方向
const int motorPWM_2 = 9;  // モータB PWM
const int motorDIR_2 = 8;  // モータB 方向

// エンコーダピン 
const int encoderPinA = 2;  // Hall sensor A
const int encoderPinB = 3;  // Hall seynsor B
const int encoderPinC = 4;  // Hall sensor C
const int encoderPinD = 5;  // Hall sensor D

// エンコーダと角度制御関連変数
long encoderTicks[2] = { 0, 0 };
double motorSpeed;           // モータの速度
double setPoint[2];          // 目標位置
double input[2], output[2];  // 入力と出力をモータごとに分ける
int Motor_loop_breaker=0; //1となると、モータが強制的に停止

//　サーボの関連変数
const int servo_Pin = 10;
int current_servo=0;
const int tire_length = 72 * M_PI;

//シリアル受信用LED 
const int LED_Pin=11;

//緊急用非常停止ボタン
const int E_Button=12;

//ジャイロ関連変数
const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
unsigned long previousTime = 0;
float angleZ = 0;
int a=0;
float angle_calibration =0;
int count=0;

//シリアル通信関連変数
int num;
int num_2;
String textPart = "Nan";
String numPart = "Nan";
String numPart_2 = "Nan";

//=========================================================
//ロボットの旋回
void rotation_robot(double rotation_Angle) {
  double arch_Length = 201 * M_PI * (double)rotation_Angle / 360*87/90;
  double num_rotation = arch_Length / tire_length;
  int target_angle = num_rotation*360;
  //if(rotation_Angle<=90){
  //  rotation_Angle=map(rotation_Angle,-90,90,-84,84);
  //}
  rotation_tire_ver4(target_angle,target_angle,rotation_Angle);
  //rotation_tire_ver2(target_angle,target_angle,rotation_Angle);
  //rotation_tire_ver3(target_angle,target_angle,rotation_Angle);
  //rotation_tire(target_angle,target_angle);
  //Serial.print("result: ");
  //Serial.print(angleZ);
  //Serial.println("degree rotation");
  Serial.println("stopped");
}
//ロボットの前進
void go_straight(int distance){
  double target_angle = ((double)distance / tire_length)*360;
  rotation_tire(-target_angle,target_angle,1);
}
//ロボットの後進
void go_back(int distance){
  double target_angle = ((double)distance / tire_length)*360;
  rotation_tire(target_angle,-target_angle,0);
}
//ロボットのタイヤ制御関数(エンコーダのみ)(ジャイロ測定はあり)
void rotation_tire(int targetAngle_A, int targetAngle_B,int Straight) {
  calibrateGyro();
  //PID制御==========================================================================
  // PIDパラメータ
  //double Kp = 0.08, Ki = 0.00001, Kd = 0.0001;
  double Kp = 0.8, Ki = 0.000125, Kd = 0.01;
  PID myPID(&input[0], &output[0], &setPoint[0], Kp, Ki, Kd, DIRECT);
  PID myPID_2(&input[1], &output[1], &setPoint[1], Kp, Ki, Kd, DIRECT);

  // PIDコントローラの初期化
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-150, 150);
  myPID_2.SetMode(AUTOMATIC);
  myPID_2.SetOutputLimits(-200, 200);
  // モータの停止とエンコーダカウントのリセット
  analogWrite(motorPWM, 0);
  analogWrite(motorPWM_2, 0);
  encoderTicks[0] = 0;
  encoderTicks[1] = 0;

  // 角度変換: 1回転あたりのエンコーダカウント数
  const long encoderCountsPerRevolution = 2794;
  const long encoderCountsPerRevolution_2 = 2794;

  // 目標位置（カウント）
  setPoint[0] = abs((targetAngle_A / 360.0) * encoderCountsPerRevolution);
  setPoint[1] = abs((targetAngle_B / 360.0) * encoderCountsPerRevolution_2);

  int motor_A_state=0;
  int motor_B_state=0;
  int old_motor_A_state=0;
  int old_motor_B_state=0;
  int count_change=0;
  int count=0;
  Motor_loop_breaker=0;
  count=0;
  angleZ=0;
  
  // 速度と距離
  float velocityX = 0.0;
  float distanceX = 0.0;
  float velocityY = 0.0;
  float distanceY = 0.0;
  const float accelScaleFactor = 16384.0; // 例: MPU6050の場合±2gで設定

  while (Motor_loop_breaker==0) {
    //ジャイロスコープによる角度測定==================================================
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);
    AcX=Wire.read()<<8|Wire.read();
    AcY=Wire.read()<<8|Wire.read();
    AcZ=Wire.read()<<8|Wire.read();
    Tmp=Wire.read()<<8|Wire.read();
    GyX=Wire.read()<<8|Wire.read();
    GyY=Wire.read()<<8|Wire.read();
    GyZ=Wire.read()<<8|Wire.read();

    unsigned long currentTime = millis();
    if(count==0){
      previousTime=currentTime;
    }

    float deltaTime = (currentTime - previousTime) / 1000.0; // Δt in seconds
    previousTime = currentTime;
    float gyroZ = GyZ / 131.0;
    angleZ += gyroZ * deltaTime-angle_calibration; // Integrate to get the angle change
    count++;
    // 加速度データの取得と処理
    float accelX = AcX/accelScaleFactor; // 単位をm/s²に変換
    float accelY = AcY/accelScaleFactor;

    // 速度の計算（積分）
    velocityX += accelX * deltaTime;
    velocityY += accelY * deltaTime;

    // 距離の計算（積分）
    distanceX += velocityX * deltaTime*100;
    distanceY += velocityY * deltaTime*100;

    int Motor_loop_breaker_2 = 0;
    for (int i = 0; i < 2; i++) {
      noInterrupts();              // 割り込みを無効化
      input[i] = abs(encoderTicks[i]);  // 値を安全にコピー
      interrupts();                // 割り込みを再度有効化
      // PID制御の更新
      if (i == 0) {
        myPID.Compute();
      } else {
        myPID_2.Compute();
      }

      // 最小出力制限
      if (abs(output[i]) < 50||count<10) {
        output[i] = output[i] > 0 ? 80 : -80;
      }
      if (abs(abs(setPoint[i])-abs(input[i]))<100) {
        Motor_loop_breaker_2 ++;
        output[i] = 0;
        analogWrite(i == 0 ? motorPWM : motorPWM_2, 0);
      }

      if (Motor_loop_breaker_2  >= 1) {
        Serial.println("stop");
        break;
      }

    }

    if (Motor_loop_breaker_2  >= 1) {
      analogWrite(motorPWM_2, 0);
      analogWrite(motorPWM, 0);
      break;
    }
    Motor_loop_breaker_2 = 0;

    for (int i = 0; i < 2; i++) {
      if (output[i] > 0) {
        if (i == 0) {
          digitalWrite(motorDIR,Straight==1?HIGH:LOW);

          analogWrite(motorPWM, abs(output[0]));
          motor_A_state=0;
        } else {
          digitalWrite(motorDIR_2,Straight==1?LOW:HIGH);
          analogWrite(motorPWM_2, abs(output[0]));
          motor_B_state=0;
        }
      } else {
        if (i == 0) {
          digitalWrite(motorDIR, Straight==1?LOW:HIGH);
          analogWrite(motorPWM, abs(output[0]));
          motor_A_state=1;
        } else {
          digitalWrite(motorDIR_2, Straight==1?HIGH:LOW);
          analogWrite(motorPWM_2, abs(output[0]));
          motor_B_state=1;
        }
      }
      if(motor_A_state!=old_motor_A_state||motor_B_state!=old_motor_B_state){
        count_change++;
      }
      if(count_change>30){
        Serial.println("stopped by state change count");
        analogWrite(motorPWM_2, 0);
        analogWrite(motorPWM, 0);
        Motor_loop_breaker_2 =2;
        break;
      }
      old_motor_A_state=motor_A_state;
      old_motor_B_state=motor_B_state;
    }

    if (Motor_loop_breaker_2 >=2) {
      analogWrite(motorPWM_2, 0);
      analogWrite(motorPWM, 0);
      break;
    }
    count++;
  }
  analogWrite(motorPWM_2, 0);
  analogWrite(motorPWM, 0);
  Serial.println("Stopped");
  Serial.print("angle:");
  Serial.println(angleZ);
  if(abs(angleZ)>0.5){
    rotation_robot(-angleZ);
  }
}

//ロボットのタイヤ制御関数(ジャイロのみ)
void rotation_tire_ver2(int targetAngle_A, int targetAngle_B,int rotation) {
    int count=0;
    Motor_loop_breaker=0;
    count=0;
    angleZ=0;

  while (Motor_loop_breaker==0) {
    //ジャイロスコープによる角度測定==================================================
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);
    AcX=Wire.read()<<8|Wire.read();
    AcY=Wire.read()<<8|Wire.read();
    AcZ=Wire.read()<<8|Wire.read();
    Tmp=Wire.read()<<8|Wire.read();
    GyX=Wire.read()<<8|Wire.read();
    GyY=Wire.read()<<8|Wire.read();
    GyZ=Wire.read()<<8|Wire.read();

    unsigned long currentTime = millis();
    if(count==0){
      previousTime=currentTime;
    }

    float deltaTime = (currentTime - previousTime) / 1000.0; // Δt in seconds
    previousTime = currentTime;

    float gyroZ = GyZ / 131.0;

    angleZ += gyroZ * deltaTime-angle_calibration; // Integrate to get the angle change

    //Serial.print("Angle Z: ");
    //Serial.println(angleZ);
    count++;

    if(abs(angleZ)>abs(rotation)){
      break;
    }
    if(rotation>0){
      digitalWrite(motorDIR, LOW);
      analogWrite(motorPWM, 80);
      digitalWrite(motorDIR_2, LOW);
      analogWrite(motorPWM_2, 80);
    }
    if(rotation<0){
      digitalWrite(motorDIR,HIGH);
      analogWrite(motorPWM, 80);
      digitalWrite(motorDIR_2, HIGH);
      analogWrite(motorPWM_2, 80);
    }
    if(Serial.available()>0){
      get_serial_value();
      if(textPart=="Emergency"){
        Emergency();
      }
    }
  }
  analogWrite(motorPWM_2, 0);
  analogWrite(motorPWM, 0);
  Serial.println("Stopped");
}

//ロボットのタイヤ制御関数(組合せ)
void rotation_tire_ver3(int targetAngle_A, int targetAngle_B,int rotation) {

  //PID制御==========================================================================
  // PIDパラメータ
  //double Kp = 0.08, Ki = 0.001, Kd = 0.01;
  double Kp = 0.2, Ki = 0.000125, Kd = 0.01;
  PID myPID(&input[0], &output[0], &setPoint[0], Kp, Ki, Kd, DIRECT);
  PID myPID_2(&input[1], &output[1], &setPoint[1], Kp, Ki, Kd, DIRECT);

  // PIDコントローラの初期化
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-150, 150);
  myPID_2.SetMode(AUTOMATIC);
  myPID_2.SetOutputLimits(-150, 150);
  // モータの停止とエンコーダカウントのリセット
  analogWrite(motorPWM, 0);
  analogWrite(motorPWM_2, 0);
  encoderTicks[0] = 0;
  encoderTicks[1] = 0;

  // 角度変換: 1回転あたりのエンコーダカウント数
  const long encoderCountsPerRevolution = 2794;
  const long encoderCountsPerRevolution_2 = 2794;

  // 目標位置（カウント）
  setPoint[0] = (targetAngle_A / 360.0) * encoderCountsPerRevolution;
  setPoint[1] = (targetAngle_B / 360.0) * encoderCountsPerRevolution_2;

  int motor_A_state=0;
  int motor_B_state=0;
  int old_motor_A_state=0;
  int old_motor_B_state=0;
  int count_change=0;
  int count=0;
  Motor_loop_breaker=0;
  count=0;
  angleZ=0;
  while (Motor_loop_breaker==0) {
    //ジャイロスコープによる角度測定==================================================
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);
    AcX=Wire.read()<<8|Wire.read();
    AcY=Wire.read()<<8|Wire.read();
    AcZ=Wire.read()<<8|Wire.read();
    Tmp=Wire.read()<<8|Wire.read();
    GyX=Wire.read()<<8|Wire.read();
    GyY=Wire.read()<<8|Wire.read();
    GyZ=Wire.read()<<8|Wire.read();

    unsigned long currentTime = millis();
    if(count==0){
      previousTime=currentTime;
    }

    float deltaTime = (currentTime - previousTime) / 1000.0; // Δt in seconds
    previousTime = currentTime;

    float gyroZ = GyZ / 131.0;

    angleZ += gyroZ * deltaTime-angle_calibration; // Integrate to get the angle change

    //Serial.print("Angle Z: ");
    //Serial.println(angleZ);
    count++;

    int Motor_loop_breaker_2 = 0;
    //目標旋回角度の70%まではPIDにより旋回する
    if(abs(angleZ)<abs(rotation)*0.6){
      //PID出力値の計算
      for (int i = 0; i < 2; i++) {
        noInterrupts();              // 割り込みを無効化
        input[i] = encoderTicks[i];  // 値を安全にコピー
        interrupts();                // 割り込みを再度有効化
        if (abs(targetAngle_A) > 180) {
          myPID.SetOutputLimits(-100, 100);
        } else {
          myPID.SetOutputLimits(-150, 150);
        }
        if (abs(targetAngle_B) > 180) {
          myPID_2.SetOutputLimits(-100, 100);
        } else {
          myPID_2.SetOutputLimits(-150, 150);
        }
        
        // PID制御の計算
        if (i == 0) {
          myPID.Compute();
        } else {
          myPID_2.Compute();
        }

        //最小出力
        if (abs(output[i]) < 70) {
          output[i] = output[i] > 0 ? 70 : -70;
        }
        //目標との差が50未満であれば、ループから抜ける
        if (abs(setPoint[i]-input[i])<50) {
          Motor_loop_breaker_2 ++;
          output[i] = 0;
          analogWrite(i == 0 ? motorPWM : motorPWM_2, 0);
        }
      }   
      //ループ抜け
      if (Motor_loop_breaker_2  >= 2) {
        analogWrite(motorPWM_2, 0);
        analogWrite(motorPWM, 0);
        break;
      }
      //PIDの出力
      Motor_loop_breaker_2 = 0;
      for (int i = 0; i < 2; i++) {
        if (output[i] > 0) {
          if (i == 0) {
            digitalWrite(motorDIR, LOW);
            analogWrite(motorPWM, output[i]);
            motor_A_state=0;
          } else {
            digitalWrite(motorDIR_2, LOW);
            analogWrite(motorPWM_2, output[i]);
            motor_B_state=0;
          }
        } else {
          if (i == 0) {
            digitalWrite(motorDIR, HIGH);
            analogWrite(motorPWM, -output[i]);
            motor_A_state=1;
          } else {
            digitalWrite(motorDIR_2, HIGH);
            analogWrite(motorPWM_2, -output[i]);
            motor_B_state=1;
          }
        }
        if(motor_A_state!=old_motor_A_state||motor_B_state!=old_motor_B_state){
          count_change++;
          //Serial.println("state changed");
        }
        if(count_change>30){
          //Serial.println("stopped by state change count");
          analogWrite(motorPWM_2, 0);
          analogWrite(motorPWM, 0);
          Motor_loop_breaker_2 =2;
          break;
        }
        old_motor_A_state=motor_A_state;
        old_motor_B_state=motor_B_state;
      }
      //ループ抜け
      if (Motor_loop_breaker_2 >=2) {
        analogWrite(motorPWM_2, 0);
        analogWrite(motorPWM, 0);
        break;
      }
    }

    //目標旋回角度の残り30%はジャイロを用いて制御を行う
    else{
      if(abs(angleZ)>abs(rotation)){
        break;
      }
      if(rotation>0){
        digitalWrite(motorDIR, LOW);
        analogWrite(motorPWM, 70);
        digitalWrite(motorDIR_2, LOW);
        analogWrite(motorPWM_2, 70);
      }
      if(rotation<0){
        digitalWrite(motorDIR,HIGH);
        analogWrite(motorPWM, 70);
        digitalWrite(motorDIR_2, HIGH);
        analogWrite(motorPWM_2, 70);
      }
    }

    //緊急停止
    if(Serial.available()>0){
      get_serial_value();
      if(textPart=="Emergency"){
        Emergency();
      }
    }
    count++;
  }

  analogWrite(motorPWM_2, 0);
  analogWrite(motorPWM, 0);
  Serial.println("Stopped");
}

//ロボットのタイヤ制御関数(ジャイロ-PID)
void rotation_tire_ver4(int targetAngle_A, int targetAngle_B,double rotation) {
    calibrateGyro();
    double Kp = 3, Ki = 0.01, Kd = 0.1;
    PID myPID(&input[0], &output[0], &setPoint[0], Kp, Ki, Kd, DIRECT);

    // PIDコントローラの初期化
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(-150, 150);
    analogWrite(motorPWM, 0);
    analogWrite(motorPWM_2, 0);

    // 目標位置（カウント）
    setPoint[0] = abs(rotation);

    int count=0;
    Motor_loop_breaker=0;
    count=0;
    angleZ=0;
    Serial.print("rotation");
    Serial.println(rotation);
  while (Motor_loop_breaker==0) {
    //ジャイロスコープによる角度測定==================================================
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);
    AcX=Wire.read()<<8|Wire.read();
    AcY=Wire.read()<<8|Wire.read();
    AcZ=Wire.read()<<8|Wire.read();
    Tmp=Wire.read()<<8|Wire.read();
    GyX=Wire.read()<<8|Wire.read();
    GyY=Wire.read()<<8|Wire.read();
    GyZ=Wire.read()<<8|Wire.read();

    unsigned long currentTime = millis();
    if(count==0){
      previousTime=currentTime;
    }

    float deltaTime = (currentTime - previousTime) / 1000.0; // Δt in seconds
    previousTime = currentTime;
    float gyroZ = GyZ / 131.0;
    angleZ += gyroZ * deltaTime-angle_calibration; // Integrate to get the angle change
    count++;

    input[0] = abs(angleZ);  // 値を安全にコピー
    // PID制御の更新
    //Serial.print("in: ");
    //Serial.println(input[0]);
    //Serial.print("set: ");
    //Serial.println(setPoint[0]);
    myPID.Compute();
    //Serial.print("output[0]:");
    //Serial.println(output[0]);

    if(abs(angleZ-rotation)<0.6){
      break;
    }
    if(abs(output[0])<80){
      if(output[0]>0){
        output[0]=70;
      }else{
        output[0]=-70;
      }
    }
    if(rotation>0){
      if(output[0]<0){
        digitalWrite(motorDIR, LOW);
        analogWrite(motorPWM, abs(output[0]));
        digitalWrite(motorDIR_2, LOW);
        analogWrite(motorPWM_2, abs(output[0]));
      }
      else{
        digitalWrite(motorDIR,HIGH);
        analogWrite(motorPWM, output[0]);
        digitalWrite(motorDIR_2, HIGH);
        analogWrite(motorPWM_2,output[0]);
      }
    }
    else{
      if(output[0]>0){
        digitalWrite(motorDIR, LOW);
        analogWrite(motorPWM, abs(output[0]));
        digitalWrite(motorDIR_2, LOW);
        analogWrite(motorPWM_2, abs(output[0]));
      }
      else{
        digitalWrite(motorDIR,HIGH);
        analogWrite(motorPWM, output[0]);
        digitalWrite(motorDIR_2, HIGH);
        analogWrite(motorPWM_2,output[0]);
      }
    }
  }
  analogWrite(motorPWM_2, 0);
  analogWrite(motorPWM, 0);
  Serial.println("Stopped");
}

//=========================================================
//サーボモータ角度の（ゴールへの）自動位置合わせ関数
void launcher_angle(int i) {
  Serial.print(i);
  Serial.println("(degree) launcher rotation!");
  moveServo(current_servo, i);
  current_servo=i;
}
//サーボモータの制御関数(角度・入力)
void moveServo(int startAngle, int endAngle) {
  int startTime = millis();
  for(int i=0;i<100;i++){
    func(endAngle);
  } 
  Serial.println("finished");
}
//サーボモータの制御関数
void func(int x) {
  x=x+30;
  int angle = x * 9.35 + 500;
  digitalWrite(servo_Pin, HIGH);
  delayMicroseconds(angle);
  digitalWrite(servo_Pin, LOW);
  delayMicroseconds(500000);
}

//=========================================================
//エンコーダ付DCモータの制御
void encoder_read_AB() {
  static uint8_t lastState_AB = 0;

  uint8_t currentState_AB = (digitalRead(encoderPinA) << 1) | digitalRead(encoderPinB);
  if (currentState_AB != lastState_AB) {
    if (((lastState_AB == 0b00) && (currentState_AB == 0b10)) || ((lastState_AB == 0b10) && (currentState_AB == 0b11)) || ((lastState_AB == 0b11) && (currentState_AB == 0b01)) || ((lastState_AB == 0b01) && (currentState_AB == 0b00))) {
      encoderTicks[0]++;
    } else {
      encoderTicks[0]--;
    }
    lastState_AB = currentState_AB;
  }
}
void encoder_read_CD() {
  static uint8_t lastState_CD = 0;

  uint8_t currentState_CD = (digitalRead(encoderPinC) << 1) | digitalRead(encoderPinD);
  if (currentState_CD != lastState_CD) {
    if (((lastState_CD == 0b00) && (currentState_CD == 0b10)) || ((lastState_CD == 0b10) && (currentState_CD == 0b11)) || ((lastState_CD == 0b11) && (currentState_CD == 0b01)) || ((lastState_CD == 0b01) && (currentState_CD == 0b00))) {
      encoderTicks[1]++;
    } else {
      encoderTicks[1]--;
    }
    lastState_CD = currentState_CD;
  }
}

//=========================================================
//緊急非常停止ボタン
void Emergency(){
  Motor_loop_breaker=1;
  digitalWrite(LED_Pin, LOW);
}

//=========================================================
//シリアル通信判断関数
bool isNumeric(String str) {
  int decimalCount = 0;  // 小数点の数をカウントする変数
  for (int i = 0; i < str.length(); i++) {
    if (!isDigit(str[i])) {
      if (str[i] == '.' && decimalCount == 0) {  // 小数点が1つだけ許容される
        decimalCount++;
      } else if (i == 0 && str[i] == '-') {  // 最初の文字がマイナス記号の場合は許容
      } else {
        return false;  // 数字でも小数点でもマイナス記号でもない場合はfalseを返す
      }
    }
  }
  return true;
}
void LED_ON(){
  digitalWrite(LED_Pin, HIGH);
  delay(5000);
}

int comma_count(String data){
  int commaCount = 0;
  for (int i = 0; i < data.length(); i++) {
    if (data[i] == ',') {
      commaCount++;
    }
  }
  return commaCount;
}

void get_serial_value(){
  String data = Serial.readStringUntil('\n');
  data.trim();  // 余分なスペースや改行を削除
  int count_comma = comma_count(data);
  //データを,区切りで分析
  int commaIndex = data.indexOf(',');
  if (commaIndex != -1) {
    textPart = data.substring(0, commaIndex);  // カンマより前
    numPart = data.substring(commaIndex + 1);  // カンマより後ろ
    // カンマの後ろの部分を数値に変換
    if (isNumeric(numPart)) {  // 数値かどうかのチェック
      num = numPart.toInt();   // double型の数値に変換
    }
  }
  if(commaIndex != -1&&count_comma==2){
    int commaIndex_2 = data.indexOf(',',commaIndex+1);
    textPart = data.substring(0, commaIndex);  // カンマより前
    numPart = data.substring(commaIndex + 1,commaIndex_2);  // カンマより後ろ
    numPart_2 = data.substring(commaIndex_2+1);
    // カンマの後ろの部分を数値に変換
    if (isNumeric(numPart)) {  // 数値かどうかのチェック
      num = numPart.toInt();   // double型の数値に変換
      num_2 = numPart_2.toInt();
    }
  }
}

//=========================================================
void rotation_calibrateGyro(){
  angleZ=0;
  int i=0;
  while(1){
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);
    AcX=Wire.read()<<8|Wire.read();
    AcY=Wire.read()<<8|Wire.read();
    AcZ=Wire.read()<<8|Wire.read();
    Tmp=Wire.read()<<8|Wire.read();
    GyX=Wire.read()<<8|Wire.read();
    GyY=Wire.read()<<8|Wire.read();
    GyZ=Wire.read()<<8|Wire.read();
    unsigned long currentTime = millis();

    if(i==0){
      previousTime=currentTime;
    }
    float deltaTime = (currentTime - previousTime) / 1000.0; // Δt in seconds
    previousTime = currentTime;

    float gyroZ = GyZ / 131.0;

    angleZ += gyroZ * deltaTime; 
    Serial.print("Angle_Z: ");
    Serial.println(angleZ);
    delay(10);
  }
  Serial.print("Angle_Z_offset: ");
  Serial.println(angleZ/100);
  angle_calibration=angleZ/100;
  angleZ=0;
}
void calibrateGyro(){
  angleZ=0;
  float AcX;
  for(int i=0;i<100;i++){
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);
    AcX=Wire.read()<<8|Wire.read();
    AcY=Wire.read()<<8|Wire.read();
    AcZ=Wire.read()<<8|Wire.read();
    Tmp=Wire.read()<<8|Wire.read();
    GyX=Wire.read()<<8|Wire.read();
    GyY=Wire.read()<<8|Wire.read();
    GyZ=Wire.read()<<8|Wire.read();

    unsigned long currentTime = millis();

    if(i==0){
      previousTime=currentTime;
    }
    float deltaTime = (currentTime - previousTime) / 1000.0; // Δt in seconds
    previousTime = currentTime;

    float gyroZ = GyZ / 131.0;

    angleZ += gyroZ * deltaTime; 
    //Serial.print("Angle Z: ");
    //Serial.println(angleZ);
    
  }
  Serial.print("Angle_Z_offset: ");
  Serial.println(angleZ/100);
  angle_calibration=angleZ/100;
  angleZ=0;
}

//=========================================================
void setup() {
  Serial.begin(9600);  // シリアル通信の初期化
  Serial.setTimeout(2000);

  pinMode(motorPWM, OUTPUT);
  pinMode(motorDIR, OUTPUT);
  pinMode(motorPWM_2, OUTPUT);
  pinMode(motorDIR_2, OUTPUT);
  pinMode(LED_Pin, OUTPUT);

  pinMode(encoderPinA, INPUT_PULLUP);  // ピンAを入力として設定
  pinMode(encoderPinB, INPUT_PULLUP);  // ピンBを入力として設定
  pinMode(encoderPinC, INPUT_PULLUP);  // ピンCを入力として設定
  pinMode(encoderPinD, INPUT_PULLUP);  // ピンDを入力として設定

  attachPCINT(digitalPinToPCINT(encoderPinA), encoder_read_AB, CHANGE);
  attachPCINT(digitalPinToPCINT(encoderPinB), encoder_read_AB, CHANGE);

  attachPCINT(digitalPinToPCINT(encoderPinC), encoder_read_CD, CHANGE);
  attachPCINT(digitalPinToPCINT(encoderPinD), encoder_read_CD, CHANGE);
  pinMode(servo_Pin, OUTPUT);
  launcher_angle(0);
  
  pinMode(E_Button, INPUT_PULLUP); // 内部プルアップ抵抗を使用
  attachPCINT(digitalPinToPCINT(E_Button), Emergency, FALLING);

  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

void loop() {
  if (Serial.available() > 0) {
    get_serial_value();

    //自動位置合わせ
    if(textPart == "automation"){
      rotation_robot(num);
      launcher_angle(num_2);
    }

    //旋回
    if (textPart == "rotation") {
      if (abs(num) <= 360) {
        Serial.print(num);
        Serial.println("rotation");
        rotation_robot(num);
      }
    }

    if (textPart == "launcher_angle") {
      launcher_angle(num); 
    }

    //前進・後進
    if(textPart == "GoStraight"){
      go_straight(num*10);
    }
    if(textPart == "GoBack"){
      go_back(num*10);
    }

    if(textPart== "a"){
      Serial.println("LED");
     LED_ON();
    }

    if(textPart=="calib"){
      calibrateGyro();
    }

    if(textPart=="Emergency"){
      Emergency();
    }

    digitalWrite(LED_Pin, LOW);
  }
}
