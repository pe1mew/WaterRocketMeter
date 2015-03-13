/**
 * \file version0.io
 * \date 13-3-2015
 * \author Remko Welling (PE1MEW) pe1mew@pe1mew.nl
 * \version 04
 * \license Creative Commons Attribution-ShareAlike 4.0 International https://creativecommons.org/licenses/by-sa/4.0/legalcode
 * 
 * This sketch allows a Arduino together with a accellerometer and a height sensor, on top of a 
 * water rocket (PET-bottle) to measure the acceleration during the flight, and height in meters of the flight.
 * Assumed that a water rocket will fly average 6 seconds (without a parachute) this sketch will collect 
 * at 200ms interval 50 measurements. This will result in 10 second of data and 5 measurements per second.
 * The collected data is stored and after flight being transmitted in a csv-style data stream that can be 
 * collected with any terminal program. When saved to a file, analysis is possible using a spreadsheet
 * or using a dedicated program.
 * The presentation of collected data will continue with a 1 second interval until a reset is performed.
 * 
 */
 
/**

 Version information:
 ====================
 Date       verssion  Comment
 ----------------------------------------------------------------------------------------------------
 12-3-2015  03        Initial commit to Github
 13-3-2015  04        Changed the use of doubles to float,
                      Change the use of sqrt() to sqrtf(), use single precision for double precision

 Future features and optimizations:
 ==================================
 \todo Add function to store unique measurement ID to measurement output and store in eeprom.
 \todo Add function to have exact timing interval for measurements. Use timer and interrupt.
           Calculate velocity using acceleration and time. 
 
 */

#include <Wire.h>                // Library for 2Wire (i2c)
#include <math.h>                // For Sqrt()

#include <Adafruit_BMP085.h>     // Library for Adafruit BMP085/BMP180 pressure sensor board
#include <Adafruit_Sensor.h>     // Generic Livrary for Adafruit sensors
#include <Adafruit_ADXL345_U.h>  // Library for Adafruit ADXL345 accelerometer sensor

#define DEBUG    // Comment to prevent debug messages on serial port

#define SAMPLES  50  // Number of samples taken by this sketch
#define INTERVAL 200 // Interval im ms between individual measurements
#define TRIGGER_TRESHOLD 2

#define AXIS_X  0  // Index to x-axis value
#define AXIS_Y  1  // Index to y-axis value
#define AXIS_Z  2  // Index to z-axis value
#define VECTOR  3  // Index to vector value

#define CALIBRATION_COUNT  10 // number of measurements over which measurements are being averaged

typedef struct sensorset_t {
float fAccX;       // X-axis accelerometer sensor in m/s
float fAccY;       // X-axis accelerometer sensor in m/s
float fAccZ;       // X-axis accelerometer sensor in m/s
float fAccV;       // vector accelerometer sensor in m/s
float fAltHeight;  // current height in meters
} sensorset_t;

Adafruit_BMP085 bmp;      // Create object for Adafruit BMP085 sensor
/* Assign a unique ID to this sensor at the same time */
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345); // Create object for Adafruit ADXL345 sensor

/** \brief blink function to blink LED x times y interval
 * This function is used to allow visual inspection on the operation of the program.
 * \variable int8_t iCount the number of times that the LED will blink
 * \varaible int16_t iDuration the time (T) in ms for a on-off cycle of the LED.
 * \note This function will affect the timing of the process calling this function.
 */
void blink(int8_t iCount, int16_t iDuration){
  for ( int i = 0; i < iCount; i++){ // Counter to count iCount times.
    digitalWrite(13, HIGH);  // turn the LED on (HIGH is the voltage level)
    delay(iDuration);        // Wait iduration ms
    digitalWrite(13, LOW);   // turn the LED on (HIGH is the voltage level)
    delay(iDuration);        // Wait iduration ms
  }
}

/** /brief setup Setup and initialisation of the Arduino and sensors.
 *
 */ 
void setup() {
  pinMode(13, OUTPUT); // initialize digital pin 13 as an output.
  Serial.begin(9600);  // Initialize serial and wait for port to open
  
  /* Initialise the BMP180 sensor */
  if (!bmp.begin()) {
#ifdef DEBUG
    Serial.println("NOK:BMP085");  // send message on serial port
#endif

    while(1) {
      blink(1, 250);  // Blink LED 1 time 
      delay(1000); // wait 1 second
    };
  }
  
  /* Initialise the ADXL345 sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
#ifdef DEBUG
    Serial.println("NOK:ADXL345");
#endif

    while(1) {
      blink(2, 250); // Blink ELED 2 times
      delay(1000); // wait 1 second
    };
  }
  
  /* Set the range to whatever is appropriate for your project */
  accel.setRange(ADXL345_RANGE_16_G);
  
#ifdef DEBUG
  Serial.println(" OK:Setup");
#endif
}

/** \brief main loop
 * The main program has three stages:
 * 1 - calibrate the sensors
 * 2 - Start measuring when movement is detected
 * 3 - When datacollection is finished start sending collected data over the serial port.
 * 
 */
void loop() {
  sensorset_t     aSensorSamples[SAMPLES] = { 0 };  // Array of sensor sets to store colleged data
  int32_t         i32ActualPreassure = 0; // variable for storing actual preassure in pascal. Used for calibrating
  float           fAxisCal[3] = { 0 };  // Array of floats to store calibration values. Used for calibrating
  sensors_event_t event;   // Pointer to event-struct for storing accelerometer data
  int8_t          i = 0;  // counter variable
  bool            bRunning = false; // flag to set state of data collection process.

  // Calibration of preassure sensor
  for (int i = 0; i < CALIBRATION_COUNT; i++){
    i32ActualPreassure += bmp.readPressure(); // Read local pressure for reference
    // delay(INTERVAL); // Wait to create longer averiging period
  };
  i32ActualPreassure /= CALIBRATION_COUNT; // average values.
  
  // calibration and averaging of acceleration sensor
  for (int i = 0; i < CALIBRATION_COUNT; i++){
    accel.getEvent(&event); // Get a new sensor event and store result in sensors event struct
    
    fAxisCal[AXIS_X] += event.acceleration.x; // Sum measured value with current value
    fAxisCal[AXIS_Y] += event.acceleration.y;
    fAxisCal[AXIS_Z] += event.acceleration.z;
    // delay(INTERVAL); // Wait to create longer averiging period
  };
  fAxisCal[AXIS_X] /= CALIBRATION_COUNT; // average all values.
  fAxisCal[AXIS_Y] /= CALIBRATION_COUNT;
  fAxisCal[AXIS_Z] /= CALIBRATION_COUNT;

#ifdef DEBUG
  Serial.println(" OK:Calibration");
#endif

#ifdef DEBUG
  Serial.println(" OK:Wait trigger");
#endif

  blink(3, 100);  // Blink LED 3 time 
  // Collect data
  while (i < SAMPLES){
    
    // Accelerometer sensor
    accel.getEvent(&event); // collect data
    aSensorSamples[i].fAccX = event.acceleration.x - fAxisCal[AXIS_X]; // store data in sensor array.
    aSensorSamples[i].fAccY = event.acceleration.y - fAxisCal[AXIS_Y];
    aSensorSamples[i].fAccZ = event.acceleration.z - fAxisCal[AXIS_Z];
    aSensorSamples[i].fAccV = sqrtf(pow(aSensorSamples[i].fAccX, 2) + // calculate the length of the vector 
                                    pow(aSensorSamples[i].fAccY, 2) +
                                    pow(aSensorSamples[i].fAccZ, 2)
                                   );
    // Height meter
    aSensorSamples[i].fAltHeight = bmp.readAltitude((float)i32ActualPreassure);
    
    // wait for trigger to running true
    if (aSensorSamples[i].fAccV > TRIGGER_TRESHOLD){     // test if accelerometer is sensing movement
      bRunning = true;
    }
    
    if ( bRunning == true){
      i++;
      delay(INTERVAL);
    }  
  }

#ifdef DEBUG
  Serial.println(" OK:Datacollection");
#endif

  // send collected data serial  
  while(1) {
    digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)

    Serial.println("");
    // Send start character
    Serial.println("S"); 
    // Send header of the table with sensor data including starting 'H' identifeier that Header is following
    Serial.println("H;Sample;X (m/s^2);Y (m/s^2);Z (m/s^2); Vector (m/s^2); Height (m)");

    // Send sensordata in table form semicolon separated.  
    for ( i = 0; i < SAMPLES; i++) {
      Serial.print("D;");  // Line identifier that data is following
      Serial.print(i);
      Serial.print(";");
      Serial.print(aSensorSamples[i].fAccX);
      Serial.print(";");
      Serial.print(aSensorSamples[i].fAccY);
      Serial.print(";");
      Serial.print(aSensorSamples[i].fAccZ);
      Serial.print(";");
      Serial.print(aSensorSamples[i].fAccV);
      Serial.print(";");
      Serial.println(aSensorSamples[i].fAltHeight); // print semicolon and terminate with /n
    };
    
    // Send stop character
    Serial.println("F");
    
    digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
    delay(1000);              // wait for a second to repeat sending data
  };
}
