#include <PinChangeInterrupt.h>

const int PWM_A=11;
const int PWM_B=5;
const int PWM_C=3;

//投射機用
const int LED_pin=10;
const int Motor_A_1=14;
const int Motor_A_2=15;
const int Motor_B_1=16;
const int Motor_B_2=17;

//回収機能用
const int Motor_C_1=18;
const int Motor_C_2=19;

//サーボ用
const int Servo_pin=9;
unsigned int current_time_launcher=0;
unsigned int previous_time_launcher=0;
const int interval=1000;//1秒毎
int current_servo=0;

//ボール押し出しサーボ
const int servo_for_ball=8;

//非常停止ボタン
const int emer=2;
int loop_breaker=0;

int launcher=0;
int collector=0;

String textPart;
String numPart;
int num;
//========================================================
void emergency_dayo(){
  Serial.println("Emergency");
  loop_breaker=1;
}

//=====================================================
void setup() {
  Serial.begin(9600);
  pinMode(PWM_A,OUTPUT);
  pinMode(PWM_B,OUTPUT);
  pinMode(PWM_C,OUTPUT);
  pinMode(Motor_A_1,OUTPUT);
  pinMode(Motor_A_2,OUTPUT);
  pinMode(Motor_B_1,OUTPUT);
  pinMode(Motor_B_2,OUTPUT);
  pinMode(Motor_C_1,OUTPUT);
  pinMode(Motor_C_2,OUTPUT);
  pinMode(LED_pin,OUTPUT);
  pinMode(Servo_pin,OUTPUT);
  pinMode(servo_for_ball,OUTPUT);

  pinMode(emer, INPUT_PULLUP); // 内部プルアップ抵抗を使用
  attachPCINT(digitalPinToPCINT(emer),emergency_dayo , FALLING);

  analogWrite(PWM_A,0);
  analogWrite(PWM_B,0);
  analogWrite(PWM_C,0);
  digitalWrite(Motor_A_1,HIGH);
  digitalWrite(Motor_A_2,HIGH);
  digitalWrite(Motor_B_1,HIGH);
  digitalWrite(Motor_B_2,HIGH);
  digitalWrite(Motor_C_1,HIGH);
  digitalWrite(Motor_C_2,HIGH);
  digitalWrite(LED_pin,LOW);
  delay(1000);


  servo(90,Servo_pin);
  servo(0,servo_for_ball);
}
void loop() {
  if (Serial.available() > 0) {
    get_serial_value();
    if(textPart=="ON"){ 
      digitalWrite(LED_pin,HIGH);
      digitalWrite(Motor_A_1,LOW);
      digitalWrite(Motor_A_2,HIGH);
      digitalWrite(Motor_B_1,LOW);
      digitalWrite(Motor_B_2,HIGH);
      analogWrite(PWM_A,255);
      analogWrite(PWM_B,255);
      delay(1000);
      analogWrite(PWM_A,num);
      analogWrite(PWM_B,num);
      launcher=1;
      previous_time_launcher=millis();
    }
    if(textPart=="OFF"){
      digitalWrite(LED_pin,LOW);
      digitalWrite(Motor_A_1,LOW);
      digitalWrite(Motor_A_2,LOW);
      digitalWrite(Motor_B_1,LOW);
      digitalWrite(Motor_B_2,LOW);
      Serial.println("stop");
      analogWrite(PWM_A,0);
      analogWrite(PWM_B,0);
      launcher=0;
    }
    if(textPart=="cON"){ 
      digitalWrite(Motor_C_1,LOW);
      digitalWrite(Motor_C_2,HIGH);
      analogWrite(PWM_C,num);
      collector=1;
    }
    if(textPart=="cOFF"){
      Serial.println("stop");
      digitalWrite(Motor_C_1,LOW);
      digitalWrite(Motor_C_2,LOW);
      analogWrite(PWM_C,0);
      collector=0;
    } 
    if(textPart== "Test"){
      digitalWrite(LED_pin,HIGH);
      digitalWrite(Motor_A_1,LOW);
      digitalWrite(Motor_A_2,HIGH);
      digitalWrite(Motor_B_1,LOW);
      digitalWrite(Motor_B_2,HIGH);
      analogWrite(PWM_A,num);
      analogWrite(PWM_B,num);
      delay(5000);  
      launcher=0;
    }
    if(textPart=="open"){
      servo(20,servo_for_ball);
      servo(0,servo_for_ball);
    }
    if(textPart=="push"){
      Serial.println("push");
      servo(0,Servo_pin);
      delay(1000);
      servo(90,Servo_pin);
    }
  }
  
  if(loop_breaker==1){  
    Serial.println(loop_breaker);
    Serial.println("stop");
    analogWrite(PWM_A,0);
    analogWrite(PWM_B,0);
    analogWrite(PWM_C,0);
    loop_breaker=0;
  }
}

//========================================================
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


void get_serial_value(){
  String data = Serial.readStringUntil('\n');
  data.trim();  // 余分なスペースや改行を削除
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
}

//サーボモータ角度の（ゴールへの）自動位置合わせ関数
void servo(int i,int Pin) {
  Serial.print(i);
  Serial.println("(degree) launcher rotation!");
  moveServo(current_servo, i, Pin);
  current_servo=i;
}
//サーボモータの制御関数(角度・入力)
void moveServo(int startAngle, int endAngle, int Pin) {
  int startTime = millis();
  for(int i=0;i<100;i++){
    func(endAngle,Pin);
  } 
  Serial.println("finished");
}
//サーボモータの制御関数
void func(int x,int Pin) {
  int angle = x * 9.35 + 500;
  digitalWrite(Pin, HIGH);
  delayMicroseconds(angle);
  digitalWrite(Pin, LOW);
  delayMicroseconds(500000);
}