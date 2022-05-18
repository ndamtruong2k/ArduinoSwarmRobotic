// Ghi Chú: 
/*Khi đo ở thời gian 1s và xung là 255
+ Vận tốc của xe khi ở 255 xung là 74 ~ 76 cm/s.
+ Bánh thứ 2 có vận tốc lớn hơn bánh thứ 1 tầm 1 cm/s.
*/

/*Khi đo ở thời gian 1s và xung là 200
+ Vận tốc của xe đạt được là 60~62 cm/s.
+ Chưa thấy sự chêch lệch về góc của xe.
*/

// Khai báo chân của động cơ
#include <util/atomic.h>
#include "Arduino.h"
#include <SPI.h>
#include <RF24.h>
#include <RF24Network.h>

class SimplePI{
  private:
    float kp, ki, kd, umax;
    float eprev, eintegral;
  
  public:
  //Constructor
  SimplePI() : kp(1),kd(0) , ki(0), umax(255),eprev(0.0), eintegral(0.0){}

  void setParams(float kpIn, float kiIn, float kdIn, float umaxIn){
    kp = kpIn; ki = kiIn; kd= kdIn ;umax = umaxIn;
  }

  void evalu(int value, int target, float deltaT, int &pwr, int &dir){
    //error
    int e = target - value;

    //derivative
    float dedt = (e-eprev)/deltaT;  
     
    //integral
    eintegral = eintegral + e*deltaT;

    // control signal
    float u = kp*e + ki*eintegral + kd*dedt;

    pwr = (int) fabs(u);
    if(pwr > umax){
      pwr = umax;
    }

    // motor direction
    dir = 1;
    if(u<0){
      dir = -1;
    }
    // store previous error
    eprev = e;   
  }
};

//Khai báo số lượng robot
#define Robot 2

// Motor
#define NMOTORS 2
#define MIN_SPEED 0
#define MAX_SPEED 255
#define DIS_MIN 10
#define Dimension 3
#define Pi 3.1416

//Khai báo biến thời gian
unsigned long time;


//Robot parameter: Distance between two wheels
const float distanceWheel = 18;

//Píns
const int enca[] = {18,19}; // Bánh Trái - Bánh Phải
const int encb[] = {42,43}; //    
const int in1 = 7;
const int in2 = 6;
const int in3 = 5;
const int in4 = 4;

float v1Filt[] ={0,0};
float v1Prev[] ={0,0};
float v2Filt[] ={0,0};
float v2Prev[] ={0,0};

//Globals
long prevT[] = {0,0};
int posPrev[] = {0,0};

// Use the "volatile" directive for variables
// used in an interrupt
volatile int pos_i[] = {0,0};
volatile float velocity_i[] = {0,0};
volatile long prevT_i[] = {0,0};

SimplePI pi[NMOTORS];

// Khai báo chân của cảm biến siêu âm
#define trig_1 45 // Cảm biến siêu âm trước thẳng
#define echo_1 44

#define trig_2 25// Cảm biến siêu âm trước trái
#define echo_2 24

#define trig_3 27 // Cảm biến siêu âm trước phải
#define echo_3 26


/*
  Khai báo các chân truyền thông trong robot
*/
// This is just the way the RF24 library works:
RF24 radio(9, 53); // CE, CSN
RF24Network network(radio); //Include the radio in the network
  
byte addresses[Robot][6] = {"1Node", "2Node"};
/*
  Khai báo các điểm bắt đầu và kết thúc
*/
float intal_Pos[Dimension] = {-1.00,1.00,Pi/4};
float target_Pos[Dimension]= {150.00,150.00,Pi/2};
float update_Pos[Dimension]= {-1.00,1.00,Pi/4};

// Polar variables
float rho;
float phi;
float alpha;

// v and w variables for control laws
float v;
float w;
float vantoc_trai = 0;
float vantoc_phai = 0;

//Control parameters
const float gamma = 3;
const float lamda = 6;
const float h = 1;

void setup()
{
  
  Serial.begin(9600);
  
  // Cấp tín hiệu cho chân của encoder
  for(int k = 0; k < NMOTORS; k++){
    pinMode(enca[k],INPUT);
    pinMode(encb[k],INPUT);
    // Cài đặt hệ số PID
    pi[k].setParams(2.5,10,0,255);
  }

  // Khởi tạo giá trị thời gian là giá trị hiện tại
  time = millis();

  // Cấp tín hiệu cho chân nguồn động cơ
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);
  pinMode(in3,OUTPUT);
  pinMode(in4,OUTPUT);

  attachInterrupt(digitalPinToInterrupt(enca[0]),readEncoder<0>,RISING);
  attachInterrupt(digitalPinToInterrupt(enca[1]),readEncoder<1>,RISING);
  
  // Cấp loại tín hiệu cho chân của cảm biến siêu âm  
  pinMode(trig_1,OUTPUT);
  pinMode(echo_1,INPUT);

  pinMode(trig_2,OUTPUT);
  pinMode(echo_2,INPUT);

  pinMode(trig_3,OUTPUT);
  pinMode(echo_3,INPUT);


  /* 
    Setup quá trình truyền nhận dữ liệu
  */
  SPI.begin();
  // Initiate the radio object
  radio.begin();
  // chanel, node address
  network.begin(90,addresses[0]);
  // Set the speed of the transmission to the quickest available
  radio.setDataRate(RF24_2MBPS);
}

void loop() {
  /* 
    Luật điều khiển được sử dụng để dẫn đường đi
    Point to point
  */
  float time_update = 0.01;
  update_Pos[0] = update_Pos[0] + (0.5)*(vantoc_phai*time_update + vantoc_trai*time_update)*cos(update_Pos[2]);
  update_Pos[1] = update_Pos[1] + (0.5)*(vantoc_phai*time_update + vantoc_trai*time_update)*sin(update_Pos[2]);
  update_Pos[2] = update_Pos[2] + (vantoc_phai*time_update - vantoc_trai*time_update)/distanceWheel;

  time = millis();
  
//  Serial.print("Time step: ");
//  Serial.println(time_update);
//  Serial.print(update_Pos[0]);
//  Serial.print("  ");
//  Serial.print(update_Pos[1]);
//  Serial.print("  ");
//  Serial.println(update_Pos[2]);  
//  Serial.print("VT: ");
//  Serial.print(vantoc_trai);
//  Serial.print("VP: ");
//  Serial.println(vantoc_phai);
  float delta_Pos[Dimension];
  for(int i = 0; i < Dimension;i++ ){
    delta_Pos[i]= target_Pos[i] - update_Pos[i];
  }

  // Calculate polar variables
  rho = sqrt(delta_Pos[0]*delta_Pos[0] +  delta_Pos[1]*delta_Pos[1]);
  //  Serial.println(rho);
  phi = atan2(target_Pos[1]-update_Pos[1],target_Pos[0]-update_Pos[0]) - target_Pos[2];
  //  Serial.println(phi);
  alpha = phi + target_Pos[2] - update_Pos[2];   
  //  Serial.println(alpha);
  
  // Calculate control lawsf
  v = gamma*cos(alpha)*rho;
  //  Serial.println(v);
  w = lamda*alpha + gamma*cos(alpha)*sin(alpha)/alpha*(alpha+h*phi);
  //  Serial.println(w);
  
  //Calculate Vl and Vr from v and w. Vận tốc tính bằng Cm/s
  vantoc_phai = constrain((2*v + distanceWheel*w)/2,-70,70);
  vantoc_trai = constrain((2*v - distanceWheel*w)/2,-70,70);

  
//  // Đo khoảng cách cảm biến 1
//  int distanceCm_1;
//  unsigned long duration_1;
//  digitalWrite(trig_1,LOW);
//  delayMicroseconds(2);
//  digitalWrite(trig_1,HIGH);
//  delayMicroseconds(5);
//  digitalWrite(trig_1,LOW);
//  
//  duration_1 = pulseIn(echo_1,HIGH);
//  distanceCm_1 = int(duration_1/2/29.412); 
//
//  int distanceCm_2;
//  unsigned long duration_2;
//  digitalWrite(trig_2,LOW);
//  delayMicroseconds(2);
//  digitalWrite(trig_2,HIGH);
//  delayMicroseconds(5);
//  digitalWrite(trig_2,LOW);
//  
//  duration_2 = pulseIn(echo_2,HIGH);
//  distanceCm_2 = int(duration_2/2/29.412); 
//
//  // Đo khoảng cách cảm biến 3
//  int distanceCm_3;
//  unsigned long duration_3;
//  digitalWrite(trig_3,LOW);
//  delayMicroseconds(2);
//  digitalWrite(trig_3,HIGH);
//  delayMicroseconds(5);
//  digitalWrite(trig_3,LOW);
//
//  duration_3 = pulseIn(echo_3,HIGH);
//  distanceCm_3 = int(duration_3/2/29.412);
//
//  if(distanceCm_1  < DIS_MIN){
//    vantoc_trai += 150/(-distanceCm_1);
//    vantoc_phai += 150/(-distanceCm_1);
//  }
//
//  if(distanceCm_2 < DIS_MIN){
//    vantoc_trai += 100/(-distanceCm_2);
//    vantoc_phai += 100/(-2*distanceCm_2);
//  }
//
//  if(distanceCm_3 < DIS_MIN){
//    vantoc_trai += 150/(-2*distanceCm_3);
//    vantoc_phai += 150/(-distanceCm_3);
//  }

  // Input Velocity Cm/s
  float inVelocity[NMOTORS];
  inVelocity[0] = vantoc_phai;
  inVelocity[1] = -vantoc_trai;
  
  float omega[NMOTORS];
  omega[0] = inVelocity[0]*0.333;
  omega[1] = inVelocity[1]*0.333;

  // Read the position and velocity of encoder
  int pos[NMOTORS] = {0,0};
  
  float velocity2[NMOTORS];//Unit: count/s

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    for(int k = 0; k < NMOTORS; k++){
      pos[k] = pos_i[k];
      velocity2[k] =  velocity_i[k];
    }
  }
  
  long currT[NMOTORS];
  float deltaT[NMOTORS];
  for(int k = 0; k < NMOTORS; k++){
      // Compute velocity with method 1
      currT[k] = micros();
      deltaT[k] = ((float) (currT[k]-prevT[k]))/1.0e6;
      prevT[k] = currT[k];
  }
  
  float velocity1[NMOTORS]; //Unit: count/s
  for(int k = 0; k < NMOTORS; k++){
      velocity1[k] = (pos[k] - posPrev[k])/deltaT[k];
      posPrev[k] = pos[k];
  }

  float v1[NMOTORS]; //Unit: RPM of Wheel 
  float v2[NMOTORS]; //Unit: RPM of Wheel

  // Turning số vòng của vận tốc theo method 1
  // Vận tốc được tính theo đổi từ count/s to RPM
  v1[0] = velocity1[0]/387.0*60.0;
  v1[1] = velocity1[1]/383.0*60.0;

  for(int k = 0; k < NMOTORS; k++){
    // Convert count/s to RPM
    //    v1[k] = velocity1[k]/379.0*60.0;
    v2[k] = velocity2[k]/379.0*60.0;

    // Low-pass filter (25 Hz cutoff)
    v1Filt[k] = 0.854*v1Filt[k] + 0.0728*v1[k] + 0.0728*v1Prev[k];
    v1Prev[k] = v1[k];
    v2Filt[k] = 0.854*v2Filt[k] + 0.0728*v2[k] + 0.0728*v2Prev[k];
    v2Prev[k] = v2[k];
  }

  //Set target velocity
  float vt[NMOTORS];
  vt[0] =  omega[0]*9.55;
  vt[1] =  omega[1]*9.55;

  for(int k = 0; k < NMOTORS; k++){
    int pwmVal, dir;
    // evaluate the control signal
    pi[k].evalu(v1Filt[k],vt[k],deltaT[k],pwmVal,dir);
    // signal the motor
    if(k == 0){
      if(dir == -1){
        pwmVal = constrain(pwmVal, MIN_SPEED, MAX_SPEED);
        digitalWrite(in1, HIGH);// chân này không có PWM
        analogWrite(in2, 255 - pwmVal);
      }
      else if(dir == 1){
        pwmVal = constrain(pwmVal, MIN_SPEED, MAX_SPEED);
        digitalWrite(in1, LOW);// chân này không có PWM
        analogWrite(in2, pwmVal);
      }
      else{
        digitalWrite(in1,LOW);
        digitalWrite(in2,LOW);
      }
    }else if(k == 1){
      dir = - dir;
      if(dir == 1){
        pwmVal = constrain(pwmVal, MIN_SPEED, MAX_SPEED);
        analogWrite(in3, pwmVal);
        digitalWrite(in4, LOW);// chân này không có PWM
      }
      else if(dir == -1){
        pwmVal = constrain(pwmVal, MIN_SPEED, MAX_SPEED);
        analogWrite(in3, 255 - pwmVal);
        digitalWrite(in4, HIGH);// chân này không có PWM
      }
      else{
        digitalWrite(in3,LOW);
        digitalWrite(in4,LOW);
      }
    }

    for(int k = 0; k < NMOTORS; k++){
      Serial.print(vt[k]);
      Serial.print(" ");
      Serial.print(v2Filt[k]);
      Serial.println(" ");
    }  
  }
  delay(10);
//  while(abs(delta_Pos[0]) <= 5 && abs(delta_Pos[1]) <= 5 && abs(delta_Pos[2]) <= 5){
//    /*
//      Kết nối multiple nRF24L01
  //    */
//    // This is what we receive from the other device (the transmitter)
//    network.update();
//    float incomingData[Dimension];
//    //-------Receiving--------
//    while(network.available()){
//      RF24NetworkHeader header_trans;
//      network.read(header_trans, &incomingData, sizeof(incomingData));
//    }
//
//    for(int i = 0; i < Dimension; i++){
//      target_Pos[i] = incomingData[i];
//    }
//    for(int i = 0; i < Dimension;i++ ){
//      delta_Pos[i]= target_Pos[i] - intal_Pos[i];
//    }
//    // //--------Sending---------
//    // // (Address where the data is going)
//    // float dataTarget[3];
//    // for(int i = 1; i  < Robot; i++){
//    //   RF24NetworkHeader header_recei(addresses[i]);
//    //   // Send the data  
//    //   bool ok = network.write(header_recei,&dataTarget, sizeof(dataTarget));
//    // }
//  }   
} 

// Hàm điều khiển động cơ
void setMotor_1(int dir,int pwmVal, int in1, int in2){
  if(dir == -1){
    pwmVal = constrain(pwmVal, MIN_SPEED, MAX_SPEED);
    digitalWrite(in1, HIGH);// chân này không có PWM
    analogWrite(in2, 255 - pwmVal);
  }
  else if(dir == 1){
    pwmVal = constrain(pwmVal, MIN_SPEED, MAX_SPEED);
    digitalWrite(in1, LOW);// chân này không có PWM
    analogWrite(in2, pwmVal);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  } 
}
void setMotor_2(int dir,int pwmVal, int in3, int in4){
  if(dir == 1){
    pwmVal = constrain(pwmVal, MIN_SPEED, MAX_SPEED);
    analogWrite(in3, pwmVal);
    digitalWrite(in4, LOW);// chân này không có PWM
  }
  else if(dir == -1){
    pwmVal = constrain(pwmVal, MIN_SPEED, MAX_SPEED);
    analogWrite(in3, 255 - pwmVal);
    digitalWrite(in4, HIGH);// chân này không có PWM
  }
  else{
    digitalWrite(in3,LOW);
    digitalWrite(in4,LOW);
  } 
}

template <int j>
void readEncoder(){
  //Read encoder B when ENCA rises
  int b = digitalRead(encb[j]);
  int increment = 0;
  if(b > 0){
    //If B is high, increment forward
    increment = 1;
  }else{
    increment = -1;
  }
  pos_i[j] = pos_i[j] + increment;

  //Compute velocity with method 2
  long currT = micros();
  float deltaT[NMOTORS]; 
  deltaT[j] = ((float)(currT - prevT_i[j]))/1.0e6;
  velocity_i[j] = increment/deltaT[j];
  prevT_i[j] = currT; 
}
