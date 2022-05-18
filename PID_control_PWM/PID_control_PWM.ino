// This alternate version of the code does not require
// atomic.h. Instead, interrupts() and noInterrupts() 
// are used. Please use this code if your 
// platform does not support ATOMIC_BLOCK.
                                          
// A class to compute the control signal
class SimplePID{   
  private:
    float kp, kd, ki, umax; // Parameters
    float eprev, eintegral; // Storage

  public:
  // Constructor
  SimplePID() : kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0){}

  // A function to set the parameters
  void setParams(float kpIn, float kdIn, float kiIn, float umaxIn){
    kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn;
  }

  // A function to compute the control signal
  void evalu(int value, int target, float deltaT, int &pwr, int &dir){
    // error
    int e = target - value;
  
    // derivative
    float dedt = (e-eprev)/(deltaT);
  
    // integral
    eintegral = eintegral + e*deltaT;
  
    // control signal
    float u = kp*e + kd*dedt + ki*eintegral;
  
    // motor power
    pwr = (int) fabs(u);
    if( pwr > umax ){
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

// How many motors
#define NMOTORS 2
#define MIN_SPEED 0
#define MAX_SPEED 255
// Pins
const int enca[] = {20,21};
const int encb[] = {18,19};
const int in1 = 7;
const int in2 = 6;
const int in3 = 5;
const int in4 = 4;

// Globals
long prevT = 0;
volatile int posi[] = {0,0};

// PID class instances
SimplePID pid[NMOTORS];

void setup() {
  Serial.begin(9600);

  for(int k = 0; k < NMOTORS; k++){
    pinMode(enca[k],INPUT);
    pinMode(encb[k],INPUT);
    pinMode(in1,OUTPUT);
    pinMode(in2,OUTPUT);
    pinMode(in3,OUTPUT);
    pinMode(in4,OUTPUT);
    
    pid[k].setParams(1,0,0,255);
  }
  
  attachInterrupt(digitalPinToInterrupt(enca[0]),readEncoder<0>,RISING);
  attachInterrupt(digitalPinToInterrupt(enca[1]),readEncoder<1>,RISING);
  
  Serial.println("target pos");
}
    
void loop() {

  // set target position
  int target[NMOTORS];
  target[0] = 10*prevT;
  target[1] = -10*prevT;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;


  // Read the position
  int pos[NMOTORS];
  noInterrupts(); // disable interrupts temporarily while reading
  for(int k = 0; k < NMOTORS; k++){
      pos[k] = posi[k];
    }
  interrupts(); // turn interrupts back on
  
  // loop through the motors
  for(int k = 0; k < NMOTORS; k++){
    int pwr, dir;
    // evaluate the control signal
    pid[k].evalu(pos[k],target[k],deltaT,pwr,dir);
    // signal the motor
    if(k == 0){
      setMotor_1(dir,pwr,in1,in2);
    }else if(k == 1){
      dir = - dir;
      setMotor_2(dir,pwr,in3,in4);
      }
  }

  for(int k = 0; k < NMOTORS; k++){
    Serial.print(target[k]);
    Serial.print(" ");
    Serial.print(pos[k]);
    Serial.print(" ");
      }          
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

template <int j>
void readEncoder(){
  int b = digitalRead(encb[j]);
  if(b > 0){
    posi[j]++;
  }
  else{
    posi[j]--;
  }
}
