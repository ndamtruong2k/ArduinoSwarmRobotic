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

volatile int posi = 0;
void setup()
{
  Serial.begin (9600);
  
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
}

// Hàm điều khiển động cơ
void setMotor_1(int dir,int speed, int in1, int in2){
  if(dir == 1){
    speed = constrain(speed, MIN_SPEED, MAX_SPEED);
    digitalWrite(in1, HIGH);// chân này không có PWM
    analogWrite(in2, 255 - speed);
  }
  else if(dir == -1){
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
void loop()
{
  Serial.println(posi);
  setMotor_1(1,50,IN1,IN2);
  setMotor_2(1,50,IN3,IN4); 
  delay(2000);//tiến 2 s
  setMotor_1(0,50,IN1,IN2);
  setMotor_2(0,50,IN3,IN4); 
  delay(1000000000000);//dừng 10s
}
