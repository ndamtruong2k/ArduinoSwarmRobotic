#include <Adafruit_MPU6050.h>
#include <MPU6050.h>      
#include <MadgwickAHRS.h>
#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

MPU6050 mpu;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);


Madgwick filter;
unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;
float calibrated_values[3];
float xv, yv, zv;
  

void transformation(float uncalibrated_values[3])    
{
  //calibration_matrix[3][3] is the transformation matrix
  //replace M11, M12,..,M33 with your transformation matrix data
  double calibration_matrix[3][3] = 
  {
    {4.989, 0.687, 0.409},
    {1.316, 4.04, 0.874},
    {-0.53, 0.324, 5.086}  
  };
  //bias[3] is the bias
  //replace Bx, By, Bz with your bias data
  double bias[3] = 
  {
    -43.938,
    -14.427,
    -121.422
  };  
  //calculation
  for (int i=0; i<3; ++i) uncalibrated_values[i] = uncalibrated_values[i] - bias[i];
  float result[3] = {0, 0, 0};
  for (int i=0; i<3; ++i)
    for (int j=0; j<3; ++j)
      result[i] += calibration_matrix[i][j] * uncalibrated_values[j];
  for (int i=0; i<3; ++i) calibrated_values[i] = result[i];
}

float scaler;
boolean scaler_flag = false;
float normal_vector_length;
void vector_length_stabilasation(){
  //calculate the normal vector length
  if (scaler_flag == false)
  {
    getHeading();
    normal_vector_length = sqrt(calibrated_values[0]*calibrated_values[0] + calibrated_values[1]*calibrated_values[1] + calibrated_values[2]*calibrated_values[2]);
    scaler_flag = true;
  } 
  //calculate the current scaler
  scaler = normal_vector_length/sqrt(calibrated_values[0]*calibrated_values[0] + calibrated_values[1]*calibrated_values[1] + calibrated_values[2]*calibrated_values[2]);
  //apply the current scaler to the calibrated coordinates (global array calibrated_values)
  calibrated_values[0] = calibrated_values[0]*scaler;
  calibrated_values[1] = calibrated_values[1]*scaler;
  calibrated_values[2] = calibrated_values[2]*scaler;
}

void setup() {
  Serial.begin(9600);

  // start the IMU and filter
  filter.begin(25);
    while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  Serial.println("HMC5883 Magnetometer Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }
  mpu.calibrateGyro();
  mpu.setThreshold(3);

  // Set the accelerometer range to 2 g
  // Set the gyroscope range to 250 degrees/second

  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / 25;
  microsPrevious = micros();
}

 


void loop() {

  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  float roll, pitch, heading;
  unsigned long microsNow;

  // check if it's time to read data and update the filter
  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {

    // read raw data from CurieIMU

    Vector normGyro = mpu.readNormalizeGyro();
    Vector normAccel = mpu.readNormalizeAccel();
    float values_from_magnetometer[3];
  
    getHeading();
    values_from_magnetometer[0] = xv;
    values_from_magnetometer[1] = yv;
    values_from_magnetometer[2] = zv;
    transformation(values_from_magnetometer);
  
    vector_length_stabilasation();

    // convert from raw data to gravity and degrees/second units
    gx = normGyro.XAxis;
    gy = normGyro.YAxis;
    gz = normGyro.ZAxis;
    ax = normAccel.XAxis;
    ay = normAccel.YAxis;
    az = normAccel.ZAxis;
    mx = calibrated_values[1] ;
    my = -calibrated_values[0] ;
    mz = calibrated_values[2];



    // update the filter, which computes orientation
    filter.updateIMU(gx, gy, gz, ax, ay, az);
    //filter.update(gx, gy, gz, ax, ay, az,mx,my,mz);

    // print the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = atan2(my,mx);


//    heading = atan2(-my,mx)*(180/PI);
//    Serial.print("Orientation: ");
    Serial.print(heading);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.println(roll);

  

    // increment previous time, so we keep proper pace
    microsPrevious = microsPrevious + microsPerReading;
  }
}

float convertRawAcceleration(int aRaw) {
  // since we are using 2 g range
  // -2 g maps to a raw value of -32768
  // +2 g maps to a raw value of 32767
  
  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
  
  float g = (gRaw * 250.0) / 32768.0;
  return g;
}

void getHeading()
{ 
  sensors_event_t event; 
  mag.getEvent(&event);
  xv = event.magnetic.x;
  yv = event.magnetic.y;
  zv = event.magnetic.z;
}
