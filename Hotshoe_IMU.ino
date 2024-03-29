#define SerialDebug  // set to true to print serial output for debugging
//#define SendBLE    // set to true to send DATA over BLE
//#define SaveFlash  // set true to log data
//#define UseOLED   // set true to display data on OLED

#include "functions.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include <Wire.h>
#ifdef UseOLED
#include <SeeedOLED.h>
#endif

#ifdef SaveFlash
#include <SD.h>
#endif

#include <avr/pgmspace.h>
#include <Sleep_n0m1.h>
#include "sleep.h"
#include "TinyGPS++.h"


// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;

#ifdef SaveFlash
// On the Ethernet Shield, CS is pin 4. Note that even if it's not
// used as the CS pin, the hardware CS pin (10 on most Arduino boards,
// 53 on the Mega) must be left as an output or the SD library
// functions will not work.
const int chipSelect = A5;
File sdfile;

static char filename[13] = "AHRSBIN.DAT";
static bool fileOpened = false;
static int byteCount = 0;

#endif

// GPS Module on Serial1 wrapped with TinyGPS++
TinyGPSPlus gps;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

//Dual access exteriors
typedef union
{
  float number;
  uint8_t bytes[4];
} 
FLOATUNION_t;

//Time stamp
typedef union
{
  uint32_t timedate;
  uint8_t bytes[4];
} 
LONGUNION_t;

typedef struct
{
  char prefix;
  FLOATUNION_t x;
  FLOATUNION_t y;
  FLOATUNION_t z;
} 
MEASURE_t;

FLOATUNION_t pitch, yaw, roll;
FLOATUNION_t latitude,longitude,alt;
LONGUNION_t gps_time,gps_date;


float deltat = 0.0f;        // integration interval for both filter schemes

uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;        // used to calculate integration interval

float heading;
float tiltheading;

boolean dumpState = true;
unsigned char count=0;

//Scaled raw values
float Axyz[3];
float Gxyz[3];
float Mxyz[3];


float q[4] = {
  1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {
  0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

#define SerialBaud   38400
#define Serial1Baud  9600

void setup(void)
{
  Wire.begin();	

#ifdef UseOLED
  SeeedOled.init();  //initialze SEEED OLED display
  SeeedOled.clearDisplay();  // clear the screen and set start position to top left corner
#endif

  Serial.begin(SerialBaud);
  Serial1.begin(Serial1Baud);

  initIOs();
  // initialize device
#ifdef SerialDebug
  Serial.println("Initializing I2C devices...");
#endif
  accelgyro.initialize();

  // verify connection
#ifdef SerialDebug
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
#endif

  analogReference(INTERNAL);
  analogRead(4);

#ifdef SaveFlash
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    // don't do anything more:
    return;
  }
  if(!SD.exists(filename)) {
    sdfile = SD.open(filename, FILE_WRITE);
  }
  fileOpened = (boolean)sdfile;
  delay(3000);
#endif

  //while(!Serial.available());
  // set slave 
#ifdef SendBLE
  {
    Serial1.print("AT+ROLE0");
    delay(1000);
  }
#endif

  //while(!Serial.available());
}
void loop(void)
{
  if (Serial.available() > 0)
  {
    int incoming = Serial.read();
    if(incoming=='D')
    {
      dumpState = false;
    }
    if(incoming=='P')
    {
      dumpState = true;
    }
  }
  if(dumpState)
  {
    smartDelay(0);

    if (gps.location.isUpdated()||gps.date.isUpdated())
    {
      latitude.number = gps.location.lat();
      longitude.number = gps.location.lng();
      alt.number = gps.altitude.meters();

      gps_date.timedate = gps.date.value();
      gps_time.timedate = gps.time.value();



#ifdef SerialDebug
      {
        Serial.print("LT:"); 
        Serial.print(latitude.number, 6);
        Serial.print("LN:"); 
        Serial.print(longitude.number, 6);
        Serial.print("A:");
        Serial.println(alt.number,3);
        Serial.print(gps_date.timedate);
        Serial.println(gps_time.timedate);
      }
#endif

#ifdef UseOLED
      SeeedOled.setTextXY(4,0);
      //SeeedOled.putString("LT="); 
      SeeedOled.putFloat(latitude.number);

      SeeedOled.setTextXY(5,0);
      //SeeedOled.putString("LN="); 
      SeeedOled.putFloat(longitude.number);

      SeeedOled.setTextXY(6,0);
      //SeeedOled.putString("A="); 
      SeeedOled.putFloat(alt.number);

      SeeedOled.setTextXY(7,0);
      SeeedOled.putNumber(gps_date.timedate);
      SeeedOled.setTextXY(7,7);
      SeeedOled.putNumber(gps_time.timedate);
#endif

#ifdef SaveFlash
     if (fileOpened) {
        sdfile.write(latitude.bytes,4);
        sdfile.write(longitude.bytes,4);
        sdfile.write(alt.bytes,4);
        sdfile.write(gps_date.bytes,4);
        sdfile.write(gps_time.bytes,4);
        byteCount += 12;
        if (byteCount >= 1024) {
            // flush to file every 1KB
            sdfile.flush();
            byteCount = 0;
        }
    }
#endif

    }

    unsigned char ChS=readCharge();
    if(ChS == DONE)
    {
      rLEDport &=~ rLEDbit;
    }
    else if(ChS == CHARGE)
    {
      rLEDport ^= rLEDbit;
    }
    else
    {
      rLEDport |= rLEDbit;
    }
    if(count>9)
    {
      count=0;
      readBat();      
      gLEDport &=~ gLEDbit;
      smartDelay(100);
      gLEDport |= gLEDbit;
    }
    //delay(100);
    count++;

    Now = micros();
    deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
    lastUpdate = Now;

#ifdef UseOLED
    SeeedOled.setTextXY(0,5);
    SeeedOled.putFloat(1.0f/deltat);
    //SeeedOled.putString("Hz");
#endif


    int i;
    for(i=0;i<20;i++)
      // read raw accel/gyro measurements from device
    {
      accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
      getAccelValue();
      getGyroValue();
      getCompassValue();
      // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
      // the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
      // We have to make some allowance for this orientation mismatch in feeding the output to the quaternion filter.
      // For the MPU-9150, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
      // in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
      // This is ok by aircraft orientation standards!  
      // Pass gyro rate as rad/s
      MadgwickQuaternionUpdate(Axyz[0], Axyz[1], Axyz[2], Gxyz[0]*PI/180.0f, Gxyz[1]*PI/180.0f, Gxyz[2]*PI/180.0f,  Mxyz[0],  Mxyz[1], Mxyz[2]);
      // MahonyQuaternionUpdate(Axyz[0], Axyz[1], Axyz[2], Gxyz[0]*PI/180.0f, Gxyz[1]*PI/180.0f, Gxyz[2]*PI/180.0f,  Mxyz[0],  Mxyz[1], Mxyz[2]);
      smartDelay(0);
    }

    // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
    // In this coordinate system, the positive z-axis is down toward Earth. 
    // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
    // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
    // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
    // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
    // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
    // applied in the correct order which for this configuration is yaw, pitch, and then roll.
    // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
    yaw.number   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
    pitch.number = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll.number  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    pitch.number *= 180.0f / PI;
    yaw.number   *= 180.0f / PI; 
    yaw.number   -= 13.8; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    roll.number  *= 180.0f / PI;

#ifdef SendBLE
    {
      Serial1.print("Y:");
      Serial1.print(yaw.number, 2);
      Serial1.print("P:");
      Serial1.print(pitch.number, 2);
      Serial1.print("R:");
      Serial1.println(roll.number, 2);
    }
#endif

#ifdef SerialDebug
    {
      Serial.print("Y:");
      Serial.print(yaw.number, 2);
      Serial.print("P:");
      Serial.print(pitch.number, 2);
      Serial.print("R:");
      Serial.println(roll.number, 2);
      Serial.print("Rate:"); 
      Serial.print(1.0f/deltat, 2); 
      Serial.println(" Hz");
    }
#endif

#ifdef UseOLED
    SeeedOled.setTextXY(1,0);
    //SeeedOled.putString("Y=");
    //SeeedOled.setTextXY(1,2);
    SeeedOled.putFloat(yaw.number);

    SeeedOled.setTextXY(2,0);
    //SeeedOled.putString("R=");
    //SeeedOled.setTextXY(2,2);
    SeeedOled.putFloat(roll.number);

    SeeedOled.setTextXY(3,0);
    //SeeedOled.putString("P=");
    //SeeedOled.setTextXY(3,2);
    SeeedOled.putFloat(pitch.number);
#endif

  }
}

//=========================================
unsigned char readCharge(void)
{
  unsigned char Temp = CHRGpin & CHRGbit;
  if(!Temp)
  {
    return CHARGE;
  }
  Temp = DONEpin & DONEbit;
  if(!Temp)
  {
    return DONE;
  }
  return NONE;
}

//=========================================
unsigned char readBat(void)
{
  unsigned int rAD4 = analogRead(4);
  //Serial.println(rAD4);
#ifdef UseOLED
  SeeedOled.setTextXY(0,0);
  SeeedOled.putNumber(rAD4);
#endif

}

//=========================================
void initIOs(void)
{
  rLEDport |= rLEDbit;
  gLEDport |= gLEDbit;
  rLEDdir |= rLEDbit;
  gLEDdir |= gLEDbit;

  CHRGdir &=~ CHRGbit;
  CHRGport |= CHRGbit;
  DONEdir &=~ DONEbit;
  DONEport |= DONEbit;
}

void getAccelValue(void)
{
  Axyz[0] = (double) ax / 16384;
  Axyz[1] = (double) ay / 16384;
  Axyz[2] = (double) az / 16384; 
}
void getGyroValue(void)
{
  Gxyz[0] = (double) gx * 250 / 32768;
  Gxyz[1] = (double) gy * 250 / 32768;
  Gxyz[2] = (double) gz * 250 / 32768;
}
void getCompassValue(void)
{
  Mxyz[0] = (double) mx * 1200 / 4096;
  Mxyz[1] = (double) my * 1200 / 4096;
  Mxyz[2] = (double) mz * 1200 / 4096;
}

// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (Serial1.available())
      gps.encode(Serial1.read());
  } 
  while (millis() - start < ms);
}

