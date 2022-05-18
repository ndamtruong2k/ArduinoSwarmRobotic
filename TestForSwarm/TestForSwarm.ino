// Khai báo chân của động cơ
#define ENCA1 9   // Chân thứ 3 của Motor1
#define ENCB1 10  // Chân thứ 4 của Motor1
#define ENCA2 11  // Chân thứ 3 của Motor2
#define ENCB2 12  // Chân thứ 4 của Motor2
#define IN1 7
#define IN2 6 // Chân PWM
#define IN3 5 // Chân PWM
#define IN4 4
#define MAX_SPEED 255
#define MIN_SPEED 0

// Khai báo chân của cảm biến siêu âm
#define trig_1 15 // Cảm biến siêu âm trước thẳng
#define echo_1 14

#define trig_2 17 // Cảm biến siêu âm trước trái
#define echo_2 16

#define trig_3 19 // Cảm biến siêu âm trước phải
#define echo_3 18

#define trig_4 21 // Cảm biến siêu âm sau phải
#define echo_4 20

#define trig_5 23 // Cảm biến siêu âm sau trái
#define echo_5 22

// Khai báo khoảng cách
#define DIS_MIN 10
#define DIS_MAX 100

volatile int posi[] = {0,0};
long prevT = 0;
float eprev = 0;
float eintegral = 0;

void setup()
{
  Serial.begin(9600);
  
  // Cấp loại tín hiệu cho chân của động cơ
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Chân cấp tín hiệu cho Encoder
  pinMode(ENCA1,INPUT);
  pinMode(ENCB1,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA1),readEncoder,RISING);

  pinMode(ENCA2,INPUT);
  pinMode(ENCB2,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA2),readEncoder,RISING);

  // Cấp loại tín hiệu cho chân của cảm biến siêu âm  
  pinMode(trig_1,OUTPUT);
  pinMode(echo_1,INPUT);

  pinMode(trig_2,OUTPUT);
  pinMode(echo_2,INPUT);

  pinMode(trig_3,OUTPUT);
  pinMode(echo_3,INPUT);

  pinMode(trig_4,OUTPUT);
  pinMode(echo_4,INPUT);

  pinMode(trig_5,OUTPUT);
  pinMode(echo_5,INPUT);

  Serial.println("target pos");
}

// Hàm điều khiển động cơ
void setMotor_1(int dir,int speed, int in1, int in2){
  if(dir == -1){
    speed = constrain(speed, MIN_SPEED, MAX_SPEED);
    digitalWrite(in1, HIGH);// chân này không có PWM
    analogWrite(in2, 255 - speed);
  }
  else if(dir == 1){
    speed = constrain(speed, MIN_SPEED, MAX_SPEED);
    digitalWrite(in1, LOW);// chân này không có PWM
    analogWrite(in2, speed);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  } 
}
void setMotor_2(int dir,int speed, int in3, int in4){
  if(dir == 1){
    speed = constrain(speed, MIN_SPEED, MAX_SPEED);
    analogWrite(in3, speed);
    digitalWrite(in4, LOW);// chân này không có PWM
  }
  else if(dir == -1){
    speed = constrain(speed, MIN_SPEED, MAX_SPEED);
    analogWrite(in3, 255 - speed);
    digitalWrite(in4, HIGH);// chân này không có PWM
  }
  else{
    digitalWrite(in3,LOW);
    digitalWrite(in4,LOW);
  } 
}

// Hàm đo khoảng cách
int getDistance(int trig, int echo){
  int distanceCm;
  unsigned long duration;
  digitalWrite(trig,LOW);
  delayMicroseconds(200);
  digitalWrite(trig,HIGH);
  delayMicroseconds(500);
  digitalWrite(trig,LOW);
  
  duration = pulseIn(echo,HIGH);
  distanceCm = int(duration/2/29.412);
  Serial.println(distanceCm);
  delay(100);
  return distanceCm;
}

// Hàm đọc Encoder
void readEncoder(){
  int b = digitalRead(ENCB1);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}
// Hàm điều chỉnh vận tốc bằng PID
void PIDcontrol(int motor, int dir, int encoder){
  // set target position
  int target = encoder ;

  // PID constants
  float kp = 1;
  float kd = 0.025;
  float ki = 0.0;

  // time difference
  long currT = micros();
  float deltaT = ((float)(currT - prevT))/(1.0e6);
  prevT= currT;

  // Read the position
  int pos = 0;
  noInterrupts(); //vô hiệu hóa tạm thời gián đoạn trong khi đọc
  pos = posi;
  interrupts(); //bật lại ngắt

  // Lỗi - error
  int e = pos - target;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwm = fabs(u);
  if( pwm > 255 ){
    pwm = 255;
  }

  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }

  if(motor == 1){
    // signal the motor1
    setMotor_1(dir,pwm,IN1,IN2);
  }
  else if(motor == 2){
    // signal the motor2
    setMotor_2(dir,pwm,IN3,IN4);
  }
  
  // store previous error
  eprev = e;
}

void separation(){
  if (distance(trig_1,echo_1)<=DIS_MIN || distance(trig_2,echo_2)<=DIS_MIN || distance(trig_3,echo_3)<DIS_MIN){
    setMotor_1(0,255,IN1,IN2);
    setMotor_2(0,255,IN3,IN4);
    delay(1000);
    // Khi bên phải có vật cản robot di chuyển sang trái 
    while(distance(trig_1,echo_1)<= DIS_MIN){
      setMotor_1(0,255,IN1,IN2);
      setMotor_2(1,255,IN3,IN4);
    }
    // Khi đằng trước có vật cản, robot lùi về sau
    while(distance(trig_2,echo_2)<= DIS_MIN){
      setMotor_1(-1,255,IN1,IN2);
      setMotor_2(-1,255,IN3,IN4);
    }
    // Khi bên trái có vật cản di chuyển sang phải
    while(distance(trig_3,echo_3)<= DIS_MIN){
      setMotor_1(1,255,IN1,IN2);
      setMotor_2(0,255,IN3,IN4);
    }
  }
  
}
void tien(){
    setMotor_1(1,200,IN1,IN2);
    setMotor_2(1,200,IN3,IN4);
}
void loop() {
    tien();
    delay(1000);
    setMotor_1(0,255,IN1,IN2);
    setMotor_2(0,255,IN3,IN4);
    delay(1000000000);
//    setMotor_1(1,100,IN1,IN2);
//    setMotor_2(1,100,IN3,IN4);
//  int distance_1 = getDistance(trig_1,echo_1); // Khoảng cách cảm biến trước thẳng
//  int distance_2 = getDistance(trig_2,echo_2); // Khoảng cách cảm biến trước trái
//  int distance_3 = getDistance(trig_3,echo_3); // Khoảng cách cảm biến trước phải
//
//  if (distance_1 <20 )
//  {
//    setMotor_1(0,180,IN1,IN2);
//    setMotor_2(0,180,IN3,IN4);
//    delay(500);
//    setMotor_1(-1,180,IN1,IN2);
//    setMotor_2(-1,180,IN3,IN4);
//    delay(1000);
//  }
//  else if (distance_2 <20)
//  {
//    setMotor_1(0,180,IN1,IN2);
//    setMotor_2(0,180,IN3,IN4);
//    delay(500);
//    setMotor_1(0,180,IN1,IN2);
//    setMotor_2(1,180,IN3,IN4);
//    delay(1000);
//  
//  }
//  else if (distance_3 <20)
//  {
//    setMotor_1(0,180,IN1,IN2);
//    setMotor_2(0,180,IN3,IN4);
//    delay(500);
//    setMotor_1(1,180,IN1,IN2);
//    setMotor_2(0,180,IN3,IN4);
//    delay(1000);
//  }
//  else
//  {
//    setMotor_1(1,180,IN1,IN2);
//    setMotor_2(1,180,IN3,IN4);
//    delay(500);
//  }



//PID Control
   // set target position
  int target = 1600 ;

  // PID constants
  float kp = 1;
  float kd = 0.025;
  float ki = 0.0;

  // time difference
  long currT = micros();
  float deltaT = ((float)(currT - prevT))/(1.0e6);
  prevT= currT;

  // Read the position
  int pos = 0;
  noInterrupts(); //vô hiệu hóa tạm thời gián đoạn trong khi đọc
  pos = posi;
  interrupts(); //bật lại ngắt

  // Lỗi - error
  int e = pos - target;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwm = fabs(u);
  if( pwm > 255 ){
    pwm = 255;
  }

  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }

//  if(motor == 1){
//    // signal the motor1
//    setMotor_1(dir,pwm,IN1,IN2);
//  }
//  else if(motor == 2){
//    // signal the motor2
    setMotor_2(dir,pwm,IN3,IN4);
//  }
  
  // store previous error
  eprev = e;
  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.println();
//    int pos = 0;
//  noInterrupts(); //vô hiệu hóa tạm thời gián đoạn trong khi đọc
//  pos = posi;
//  interrupts(); //bật lại ngắt
//  Serial.println(pos);
//  setMotor_2(1,255,IN3,IN4);
  
}
