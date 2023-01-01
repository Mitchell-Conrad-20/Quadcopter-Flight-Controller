#include <Wire.h>
#include <IBusBM.h>
#include "SoftPWM.h"

/*------------------------ Global Variables and Declarations ------------------------*/

// MPU-6050 gives you 16 bits data so you have to create some 16int constants to store the data for accelerations and gyro
int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;

float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];

// Store the elapsed time, current time, previous time
float elapsedTime, time, timePrev;

// Variable for easy conversions
float rad_to_deg = 180/3.141592654;


// IBus Object and Serial
IBusBM ibusRc;
HardwareSerial& ibusRcSerial = Serial1;

// Associate each motor with its PWM pin
int frontLeft  = 11;
int frontRight = 13;
int rearLeft   =  5;
int rearRight  = 12; 

// Initial pulse width for each motor (0 - 100)
int frontLeftPW   = 0;
int frontRightPW  = 0;
int rearLeftPW    = 0;
int rearRightPW   = 0;

/*------------------------ Helper Functions ------------------------*/

// readChannel(...)
// Takes 4 parameters: channel number, lower limit, upper limit, and default output
// Returns an output of the value for the particular channel mapped on the scale passed in via parameters
int readChannel(byte channelInput, int minLimit, int  maxLimit, int defaultValue){
  uint16_t ch = ibusRc.readChannel(channelInput);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}



/*------------------------ Setup Function ------------------------*/

void setup() {
  // IBUS SETUP 
  ibusRc.begin(ibusRcSerial);

  // Wire Setup
  Wire.begin(); //begin the wire comunication
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(19200);
  time = millis(); //Start counting time in milliseconds
}

/*------------------------ Loop Function ------------------------*/
void loop() {
/////////////////////////////I M U/////////////////////////////////////
  timePrev = time;  // the previous time is stored before the actual time read
  time = millis();  // actual time read
  elapsedTime = (time - timePrev) / 1000; 
  
/*The tiemStep is the time that elapsed since the previous loop. 
  * This is the value that we will use in the formulas as "elapsedTime" 
  * in seconds. We work in ms so we haveto divide the value by 1000 
  to obtain seconds*/

/*Reed the values that the accelerometre gives.
  * We know that the slave adress for this IMU is 0x68 in
  * hexadecimal. For that in the RequestFrom and the 
  * begin functions we have to put this value.*/

  Wire.beginTransmission(0x68);
  Wire.write(0x3B); //Ask for the 0x3B register- correspond to AcX
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,6,true); 
  
  /*We have asked for the 0x3B register. The IMU will send a brust of register.
  * The amount of register to read is specify in the requestFrom function.
  * In this case we request 6 registers. Each value of acceleration is made out of
  * two 8bits registers, low values and high values. For that we request the 6 of them  
  * and just make then sum of each pair. For that we shift to the left the high values 
  * register (<<) and make an or (|) operation to add the low values.*/
  
  Acc_rawX=Wire.read()<<8|Wire.read(); //each value needs two registres
  Acc_rawY=Wire.read()<<8|Wire.read();
  Acc_rawZ=Wire.read()<<8|Wire.read();

  /*///This is the part where you need to calculate the angles using Euler equations///*/
  
  /* - Now, to obtain the values of acceleration in "g" units we first have to divide the raw   
    * values that we have just read by 16384.0 because that is the value that the MPU6050 
    * datasheet gives us.*/
  /* - Next we have to calculate the radian to degree value by dividing 180º by the PI number
  * which is 3.141592654 and store this value in the rad_to_deg variable. In order to not have
  * to calculate this value in each loop we have done that just once before the setup void.
  */

  /* Now we can apply the Euler formula. The atan will calculate the arctangent. The
    *  pow(a,b) will elevate the a value to the b power. And finnaly sqrt function
    *  will calculate the rooth square.*/
  /*---X---*/
  Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
  /*---Y---*/
  Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;

  /*Now we read the Gyro data in the same way as the Acc data. The adress for the
  * gyro data starts at 0x43. We can see this adresses if we look at the register map
  * of the MPU6050. In this case we request just 4 values. W don¡t want the gyro for 
  * the Z axis (YAW).*/
  
  Wire.beginTransmission(0x68);
  Wire.write(0x43); //Gyro data first adress
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,4,true); //Just 4 registers
  
  Gyr_rawX=Wire.read()<<8|Wire.read(); //Once again we shif and sum
  Gyr_rawY=Wire.read()<<8|Wire.read();

  /*Now in order to obtain the gyro data in degrees/seconda we have to divide first
  the raw value by 131 because that's the value that the datasheet gives us*/

  /*---X---*/
  Gyro_angle[0] = Gyr_rawX/131.0; 
  /*---Y---*/
  Gyro_angle[1] = Gyr_rawY/131.0;

  /*Now in order to obtain degrees we have to multiply the degree/seconds
  *value by the elapsedTime.*/
  /*Finnaly we can apply the final filter where we add the acceleration
  *part that afects the angles and ofcourse multiply by 0.98 */

  /*---X axis angle---*/
  Total_angle[0] = 0.98 *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle[0];
  /*---Y axis angle---*/
  Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle[1];

  // Read the IBus channels for the controller input
  // CHANNEL 2 THROTTLE, CHANNEL 3 YAW, CHANNEL 0 ROLL, CHANNEL 1 PITCH
  int throttleCtrl = readChannel(2, 0, 80, 0);
  int yawCtrl = readChannel(3, -100, 100, 0);
  int rollCtrl = readChannel(0, -100, 100, 0);
  int pitchCtrl = readChannel(1, -100, 100, 0);

  /*------------------------ Serial Output for Testing------------------------*/
  
  /*Now we have our angles in degree and values from -100 to 100 degrees*/
  Serial.print("ANLGE: ");
  Serial.print(Total_angle[1]);
  Serial.print(", ");
  Serial.print(Total_angle[0]);

  Serial.print(" IBUS: ");
  Serial.print(throttleCtrl);
  Serial.print(", ");
  Serial.print(yawCtrl);
  Serial.print(", ");
  Serial.print(rollCtrl);
  Serial.print(", ");
  Serial.println(pitchCtrl);
  
  /*------------------------ PID ------------------------*/

  /*------------------------ PWM Motor Control ------------------------*/
  // Set the pulse width for each motor
  SoftPWMSetPercent(frontLeft, frontLeftPW);
  SoftPWMSetPercent(rearLeft, rearLeftPW);
  SoftPWMSetPercent(frontRight, frontRightPW);
  SoftPWMSetPercent(rearRight, rearRightPW);
}
