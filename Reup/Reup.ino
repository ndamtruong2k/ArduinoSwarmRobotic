      /**
 * Nguyễn Đam Trường - Lớp K63R
 * Thuật toán điều khiển vận tốc của hai bánh xe sử dụng PI
**/
#include <util/atomic.h>

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

// Motor
#define NMOTORS 2
#define MIN_SPEED 0
#define MAX_SPEED 255

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

void setup(){
  Serial.begin(115200);

  for(int k = 0; k < NMOTORS; k++){
    pinMode(enca[k],INPUT);
    pinMode(encb[k],INPUT);

    pi[k].setParams(1.5,10,0.005,255);
  }

  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);
  pinMode(in3,OUTPUT);
  pinMode(in4,OUTPUT);

  attachInterrupt(digitalPinToInterrupt(enca[0]),readEncoder<0>,RISING);
  attachInterrupt(digitalPinToInterrupt(enca[1]),readEncoder<1>,RISING);
}


void loop(){
    // Input Velocity Cm/s
    float inVelocity[NMOTORS];
    inVelocity[0] = 10;
    inVelocity[1] = -10;
    float omega[NMOTORS];
    omega[0] = inVelocity[0]/3;
    omega[1] = inVelocity[1]/3;
    
    // Read the position and velocity
    int pos[NMOTORS] = {0,0};
    float velocity2[NMOTORS];
    
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
    float velocity1[NMOTORS];
    for(int k = 0; k < NMOTORS; k++){
        velocity1[k] = (pos[k] - posPrev[k])/deltaT[k];
        posPrev[k] = pos[k];
    }
    float v1[NMOTORS];
    float v2[NMOTORS];
    
    // Turning số vòng của vận tốc theo method 1
    // Vận tốc được tính theo RPM
    v1[0] = velocity1[0]/387.0*60.0;
    v1[1] = velocity1[1]/383.0*60.0;
    
    for(int k = 0; k < NMOTORS; k++){
      // Convert count/s to RPM
//      v1[k] = velocity1[k]/379.0*60.0;
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
      int pwr, dir;
      // evaluate the control signal
      pi[k].evalu(v1Filt[k],vt[k],deltaT[k],pwr,dir);
      // signal the motor
      if(k == 0){
        setMotor_1(dir,pwr,in1,in2);
      }else if(k == 1){
        dir = - dir;
        setMotor_2(dir,pwr,in3,in4);
      }
    }
    
    for(int k = 0; k < NMOTORS; k++){
      Serial.print(vt[k]);
      Serial.print(" ");
      Serial.print(v2Filt[k]);
      Serial.println();
    };
}

// Điều khiển PI vận tốc hai bánh theo cm/s


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
