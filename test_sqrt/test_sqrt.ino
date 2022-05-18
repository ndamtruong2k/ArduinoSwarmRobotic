void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  int Dimension = 3;
  float delta_Pos[Dimension];
  delta_Pos[0] = 3;
  delta_Pos[1] = 4;  
  delta_Pos[2] = 0;
  // Calculate polar variables
  float rho = sqrt(delta_Pos[0]*delta_Pos[0] +  delta_Pos[1]*delta_Pos[1]);
  Serial.println(rho);

}
