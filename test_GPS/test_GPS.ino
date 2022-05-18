#define rx 17
#define tx 16
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(rx,INPUT);
  pinMode(tx,INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print(rx);
  Serial.print("  ");
  Serial.println(tx);
  delay (1000);
}
