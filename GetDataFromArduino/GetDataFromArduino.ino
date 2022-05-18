//#include<SPI.h>
//#include<SD.h>
//
//File myFile;
//const int chipSelect = 53;
//void setup(){
//    Serial.begin(9600);
//    while(!Serial){
//        ;//Wait for serial port to connect. Needed for native USB port only
//    }
//
//    Serial.print("Initializing SD card...");
//    pinMode(chipSelect, OUTPUT);
//    if(!SD.begin(chipSelect)){
//        Serial.println("initialization failed!");
//        while(1);
//    }
//    Serial.println("initialization done.");
//
//    //open the file, note that only one file can be open at a time,
//    //so you have to close this one before opening another
//    myFile = SD.open("test1.txt",FILE_WRITE);
//
//    //if the file opened okay, write to it:
//    if(myFile){
//        Serial.print("Writing to text1.txt...");
//        myFile.println("Testin 1,2,3,4.");
//        //close the file:
//        myFile.close();
//        Serial.println("done.");
//    }else{
//        // if the file didn't open, print an error:
//        Serial.println("error opening test.txt");
//    }
//
//    //re-open the file for reading
//    myFile = SD.open("test1.txt");
//    if(myFile){
//        Serial.println("test1.txt:");
//
//        //read from the file until there's nothing else in it:
//        while(myFile.available()){
//            Serial.write(myFile.read());
//        }
//
//        //close the file:
//        myFile.close();
//    }else{
//        //if the file didn't open, print an error:
//        Serial.println("error opening test.txt");
//    }
//}
//
//void loop(){
//    
//}

void setup(){
  Serial.begin(9600);
}
void loop(){
  for(int i = 0; i < 100;i++){
    Serial.println(i);
    delay(500);
  }
}
