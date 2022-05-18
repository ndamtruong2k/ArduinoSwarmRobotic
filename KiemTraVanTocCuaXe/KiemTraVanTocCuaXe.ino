// Khai báo chân của động cơ
#include <util/atomic.h>
#include "Arduino.h"
#include <SPI.h>
#include <RF24.h>
#include <RF24Network.h>
double time_step;
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

void loop(){
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

    digitalWrite(in1, LOW);// chân này không có PWM
    analogWrite(in2, 255);
    analogWrite(in3, 255);
    digitalWrite(in4, LOW);
    delay(1000);
//    Serial.print("Count/s of Motor Right: ");
    Serial.println(velocity2[0]);
//    Serial.print("Count/s of Motor Left: ");
//    Serial.println(velocity2[1]);
//    Serial.println("");
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
