unsigned long time;

void setup(){
  Serial.begin(9600);
}
void loop(){
  Serial.print("Time: ");
  time = micros();
  // in ra thời gian kể từ lúc chương trình được bắt đầu 
  Serial.println(time);
  // đợi 1 giây trước khi tiếp tục in
  delay(1000);
}
