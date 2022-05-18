// Khai báo chân của cảm biến siêu âm
#define trig1 15 // Cảm biến siêu âm trước thẳng
#define echo1 14

#define trig2 17 // Cảm biến siêu âm trước trái
#define echo2 16

#define trig3 27 // Cảm biến siêu âm trước phải
#define echo3 26
void setup() {
  Serial.begin(9600);
  pinMode(trig1,OUTPUT);
  pinMode(echo1,INPUT);
  pinMode(trig2,OUTPUT);
  pinMode(echo2,INPUT);
  pinMode(trig3,OUTPUT);
  pinMode(echo3,INPUT);
}
// Hàm đo khoảng cách tính theo m
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
  int distanceM = distanceCm/100;
  return distanceM;
}

void loop() {
  int vitribandau[2];
  int vitriketthuc[2];
  
  unsigned long duration1,duration2,duration3;
  int distance1,distance2,distance3;
  digitalWrite(trig1,0);
  delay(2);
  digitalWrite(trig1,1);
  delay(5);
  digitalWrite(trig1,0);
  duration1 = pulseIn(echo1,1);

  digitalWrite(trig2,0);
  delay(2);
  digitalWrite(trig2,1);
  delay(5);
  digitalWrite(trig2,0);
  duration2 = pulseIn(echo2,1);

  digitalWrite(trig3,0);
  delay(2);
  digitalWrite(trig3,1);
  delay(5);
  digitalWrite(trig3,0);
  duration3 = pulseIn(echo3,1);
  
  distance1 = int(duration1/2/29.412);
  distance2 = int(duration2/2/29.412);
  distance3 = int(duration3/2/29.412);

  
  Serial.print(distance1);
  Serial.print(' ');
  Serial.print(distance2);
  Serial.print(' ');
  Serial.println(distance3);
  delay(100);
}
