#include <U8glib.h>

const int sensorPins[6] = {A5, A4, A3,A2,A1,A0};

const int ENA = 6;     
const int IN1 = 7;
const int IN2 = 8;
const int ENB = 5;     
const int IN3 = 9;
const int IN4 = 10;
const int trigPin = 12;
const int echoPin = 11;


float Kp = 30.5;
float Ki = 0.0;
float Kd = 80.0;

float threshold = 400.0;
float lastError = 0, integral = 0;
float error = 0.00;
int sensorValues[6];
int s[6];


void setup() {
  Serial.begin(115200);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
      pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

    ADCSRA = (ADCSRA & ~0x07) | 0x06;

}


void avoidObstacle() {
  stopMotor();
  delay(100);

  //Xoay phải 45 
  moveMotor(150, -150);  
  delay(250);            
  stopMotor();
  delay(100);

  // Chạy thẳng 
  moveMotor(150, 150);
  delay(600);           
  stopMotor();
  delay(100);

  //  Xoay trái 45 
  moveMotor(-150, 150);
  delay(400);
  stopMotor();
  delay(100);

  //  bắt lại line 
  moveMotor(150, 150);
  delay(500);  
  stopMotor();
  delay(100);

  //  tìm lại line 
  while (!hasLineAhead()) {
    moveMotor(120, 120);
  }
  stopMotor();
  delay(80);
}

void loop() {
    long distance = getDistance();
  Serial.print(" | Distance: ");
  Serial.println(distance);

  //  Kiểm tra vật cản 
  if (distance > 0 && distance < 10) {  
    avoidObstacle();  // gọi hàm mới
    return;           // tránh vật cản xong thì quay lại PID
  }


  float error = getError();  // lấy sai số từ cảm biến

if (isTurnLeft90()) {
  stopMotor();
  delay(70);
  Serial.println("Re trai 90 do");

  // Xoay tại chỗ cho tới khi 1 trong 2 cảm biến giữa thấy lại line
  while (!hasLineAhead()) {
    moveMotor(-130, 130);  // quay trái
  }
  moveMotor(-130, 130);  // xoay thêm chút cho chắc
    delay(50);
  stopMotor();
  delay(100);
}

else if (isTurnRight90()) {
  stopMotor();
  delay(70);
  Serial.println("Re phai 90 do");

  while (!hasLineAhead()) {
    moveMotor(130, -130);  // quay phải
  }
   moveMotor(130, -130);  // xoay thêm chút
    delay(50);
  stopMotor();
  delay(100);
}
  else {

  integral += error;
  float output = Kp * error +  Kd * (error - lastError);
  lastError = error;

  int baseSpeed ;
  if (error < -1.0 || error > 1.0){baseSpeed = 115;}
  else baseSpeed = 150;
  int leftSpeed = baseSpeed -output;
  int rightSpeed = baseSpeed +output;
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);
    moveMotor(leftSpeed, rightSpeed);
  
  }


  Serial.print("Error: "); Serial.print(error);
  // // Serial.print("\tLeft: "); Serial.print(leftSpeed);
  // // Serial.print("\tRight: "); Serial.println(rightSpeed); 
  Serial.print("\t"); for(int i =0; i<6; i++){     
    sensorValues[i] = analogRead(sensorPins[i]);
    s[i] = (sensorValues[i] > threshold) ? 1 : 0;
    Serial.print(s[i]);Serial.print(" ");}
  Serial.println();
   delay(30);
}

// calculate
float getError() {
  int s[6];
  for (int i = 0; i < 6; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
    s[i] = (sensorValues[i] < threshold) ? 1 : 0;
  }
  if      ( s[0] && !s[1] && !s[2] && !s[3] && !s[4] && !s[5])            error = -3.0;    // 100000
  else if ( s[0] &&  s[1] && !s[2] && !s[3] && !s[4] && !s[5])             error = -2.25;   // 110000
  else if (!s[0] &&  s[1] && !s[2] && !s[3] && !s[4] && !s[5])             error = -1.7;  // 010000
  else if (!s[0] &&  s[1] &&  s[2] && !s[3] && !s[4] && !s[5])              error = -1.27; // 011000
  else if (!s[0] && !s[1] &&  s[2] && !s[3] && !s[4] && !s[5])               error = -0.6;// 001000
  else if (!s[0] && !s[1] &&  s[2] &&  s[3] && !s[4] && !s[5])                 error = 0.0;// 001100
  else if (!s[0] && !s[1] && !s[2] &&  s[3] && !s[4] && !s[5])                error = 0.6;// 000100
  else if (!s[0] && !s[1] && !s[2] &&  s[3] &&  s[4] && !s[5])               error = 1.27; // 000110
  else if (!s[0] && !s[1] && !s[2] && !s[3] &&  s[4] && !s[5])               error = 1.7; // 000010
  else if (!s[0] && !s[1] && !s[2] && !s[3] &&  s[4] &&  s[5])              error = 2.25;   // 000011
  else if (!s[0] && !s[1] && !s[2] && !s[3] && !s[4] &&  s[5])             error = 3.0;  // 000001
  else if ( s[0] &&  s[1] &&  s[2] && !s[3] && !s[4] &&  !s[5])             error = -2.45;  // 111000
  else if (!s[0] &&  s[1] &&  s[2] &&  s[3] && !s[4] && !s[5])             error = -155;  // 011100
  else if (!s[0] && !s[1] &&  s[2] &&  s[3] &&  s[4] && !s[5])             error = 155;  // 001110
  else if (!s[0] && !s[1] && !s[2] &&  s[3] &&  s[4] &&  s[5])             error = 2.45;  // 000111

  else error = 0.0; // nếu không xác định được line
  return error;
}

bool isTurnLeft90() {
  int s[6];
  for (int i = 0; i < 6; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
    s[i] = (sensorValues[i] < threshold) ? 1 : 0;
  }
  return ( (s[2] && s[3] && s[4] && s[5] && !s[0] && !s[1]) || (( !s[2] && s[3] && s[4] && s[5] && !s[0] && !s[1] ))); // 3-4 cảm biến trái
}

// PHÁT HIỆN RẼ PHẢI 90° 
bool isTurnRight90() {
  int s[6];
  for (int i = 0; i < 6; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
    s[i] = (sensorValues[i] < threshold) ? 1 : 0;
  }
  return ( (s[0] && s[1] && s[2] && s[3] && !s[4] && !s[5])|| ( s[0] && s[1] && s[2] && !s[3] && !s[4] && !s[5])); // 3-4 cảm biến phải
}


bool hasLineAhead() {
  // cảm biến giữa: 2,3
  int midLeft = analogRead(sensorPins[2]);
  int midRight = analogRead(sensorPins[3]);
  return (midLeft < threshold || midRight < threshold);
}


void moveMotor(int leftSpeed, int rightSpeed) {
  int leftPWM = abs(leftSpeed);
  int rightPWM = abs(rightSpeed);

  // Giới hạn an toàn
  // leftPWM = constrain(leftPWM, 0, 255);
  // rightPWM = constrain(rightPWM, 0, 255);

  // Động cơ trái
  if (leftSpeed >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  analogWrite(ENA, leftPWM);

  // Động cơ phải
  if (rightSpeed >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  analogWrite(ENB, rightPWM);
}

void stopMotor() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

long getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 25000); // timeout 25ms = 4m
  if (duration == 0) return -1; // không phản hồi
  return duration * 0.034 / 2;  // đổi ra cm
}
