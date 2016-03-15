/*
 * This code is written to intergrate 3 compoents of the 3rd Arm Project.
   The three componets are: 

            1) IMU BNO005
            2) IR Distance Sensor VL6180X
            3) Hitting Servo 
 */

// 1. variables and library for accelerometer:
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SparkFun_VL6180X.h>
/* This driver reads raw data from the BNO055
   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   Output
   =======
    Front of BNO005 is considered to be where the 6 pin header is loacted.
    X - Yaw (flat rotation) right = positive
        Yaw (flat rotation) left   = negitive
        //dynamixel

    Y = Roll right = negitive
        Roll left   = positive

    Z = Pitch up     = negitive
        Pitch down = positive
        //servo
*/
/*
 TOF IR Distanc Sensor VL6180X

  The VL6180x by ST micro is a time of flight range finder that
  uses pulsed IR light to determine distances from object at close
  range.  The average range of a sensor is between 0-200mm
  
*/
// Defining IR Distance Sensor
#define VL6180X_ADDRESS 0x29

VL6180xIdentification identification;
VL6180x sensor(VL6180X_ADDRESS);

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055();
unsigned long previousTime=0;

//2. variables and libraries for the servo:
#include <Servo.h>
int order = 7;
Servo myservo;  // create servo object to control a servo
int angle=0; 

//3. setup:
void setup(void)
{
  Serial.begin(9600); // set baud rate
  Wire.begin(); //Start I2C library
  delay(100); // delay

  /* Initialise the sensor Distance*/
  sensor.getIdentification(&identification); // Retrieve manufacture info from device memory
  printIdentification(&identification); // Helper function to print all the Module informati
  
  if(sensor.VL6180xInit() != 0){
    Serial.println("FAILED TO INITALIZE"); //Initialize device and check for errors
  }; 

  sensor.VL6180xDefautSettings(); //Load default settings to get started.

  /* Initialise the sensor */
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");
  
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  myservo.attach(order);
  
  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
}


//4. Loop
void loop(void)
{
  unsigned long currentTime=millis();
  
  if (currentTime-previousTime>BNO055_SAMPLERATE_DELAY_MS){
    previousTime=currentTime;
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  /* Display the floating point data */
  Serial.print(" X: ");
  Serial.print(euler.x());
  Serial.print(" Y: ");
  Serial.print(euler.y());
  Serial.print(" Z: ");
  Serial.print(euler.z());
  Serial.print("\t\t");

 /* Distance Sensor*/
//Get Ambient Light level and report in LUX
  //Serial.print("\n");
  //Serial.print("Ambient Light Level (Lux) = ");
  
  //Input GAIN for light levels, 
  // GAIN_20     // Actual ALS Gain of 20
  // GAIN_10     // Actual ALS Gain of 10.32
  // GAIN_5      // Actual ALS Gain of 5.21
  // GAIN_2_5    // Actual ALS Gain of 2.60
  // GAIN_1_67   // Actual ALS Gain of 1.72
  // GAIN_1_25   // Actual ALS Gain of 1.28
  // GAIN_1      // Actual ALS Gain of 1.01
  // GAIN_40     // Actual ALS Gain of 40
  
  //Serial.println( sensor.getAmbientLight(GAIN_1) );

  //Get Distance and report in mm
  //Serial.print("Distance measured (mm) = ");
  Serial.println( sensor.getDistance() );    
  }
  
  while (Serial.available()) {
    // max sends byte values, so no need to reformat them
      angle=(int)Serial.read();
      myservo.write(angle);  
    }

};

/* A Function to help display start up data */

  void printIdentification(struct VL6180xIdentification *temp)
  {
  Serial.print("Model ID = ");
  Serial.println(temp->idModel);

  Serial.print("Model Rev = ");
  Serial.print(temp->idModelRevMajor);
  Serial.print(".");
  Serial.println(temp->idModelRevMinor);

  Serial.print("Module Rev = ");
  Serial.print(temp->idModuleRevMajor);
  Serial.print(".");
  Serial.println(temp->idModuleRevMinor);  

  Serial.print("Manufacture Date = ");
  Serial.print((temp->idDate >> 3) & 0x001F);
  Serial.print("/");
  Serial.print((temp->idDate >> 8) & 0x000F);
  Serial.print("/1");
  Serial.print((temp->idDate >> 12) & 0x000F);
  Serial.print(" Phase: ");
  Serial.println(temp->idDate & 0x0007);

  Serial.print("Manufacture Time (s)= ");
  Serial.println(temp->idTime * 2);
  Serial.println();
  Serial.println();
}
