/*!
 * @file BHwMMaKF.ino Version 13.0
 *
 * @mainpage Adafruit BME280 humidity, temperature & pressure sensor
 * @mainpage Adafruit Ultimate GPS modul
 * @mainpage Adafruit BNO055 IMU Sensor
 * @mainpage Embedded System
 *
 * @section intro_sec Introduction
 *
 * This is a program for an embedded system that is able to give the 3D Position of the user. 
 * The focus was mainly on improving the height measurement, since this is the one which has 
 * the greatest error compared to the other coordinates.
 * 
 * The main goal was to implement an Map-Matching algorithm for the arduino hardware. The Kalman Filter which is also included
 * in the code was simply a try, but since the Kalman Filter usually calculates the estimates with big matrices, the author decided therefore to simply implement
 * a 1D Kalman Filter. Besides this filter the author made also an alternative, which is called MyFilterEstimator.
 * 
 * The IMU is used to calculate the position by using the magnetometer for the orientation and the accelerometer for the acceleration. Combined with the GPS
 * it will be then processed by the Kalman-Filter in order to get a better estimate, which can be turned off. The MyFilterEstimator works in both cases: when the Kalman-Filter is on and also when it is off.
 * 
 * These sensors use I2C or SPI to communicate, 2 or 4 pins are required
 * to interface.
 *
 * @section author Author
 *
 * Written by Riccardo Orion Feingold Student of the Kirchenfeld-High-School.
 * --> Last Update: 13.07.19 11:33
 *
 */
//LIBRARIES
/*My libraries*/
#include "KalmanFilter.h"
#include "MyFilterEstimator.h"
#include "MapMatching.h"
#include "HEIGHT_DATA.h"

/*BNO055 Accelerometer*/
#include <Wire.h>
#include <Filter.h> 
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <EEPROM.h>

/*Adafruit GPS*/
#include <Adafruit_GPS.h>
#include <SD.h>

/*BME280 Sensor; Barometer*/
#include <Adafruit_BME280.h>

/*LCD Display*/
#include <LiquidCrystal_I2C.h>

/*Keypad*/
#include <Keypad.h>

//Height Model instantiation
HeightModel model;

//BNO055 IMU Sensor
#define BNO055_SAMPLERATE_DELAY_MS (1000)
Adafruit_BNO055 bno = Adafruit_BNO055(55);
double ACCEL_VEL_TRANSITION =  (double)(BNO055_SAMPLERATE_DELAY_MS) / 1000.0;
double xPos=7.35792, yPos=46.90985, xVel=0, yVel=0;
double METER_2_DEG = 0.000008995;
double DEG_2_RAD = 0.01745329251; //trig functions require radians, BNO055 outputs degrees
double longitude=7.35819, latitude=46.90988;//Initial starting point
double est_error = 0;

//correction variables for the BNO055 
double corrHeadingVel_gps=0, corrHeadingVel_imu=0;//those variables are used as bias

//Kalman Filter for BNO055
KalmanFilter velFilter(0,5,0.1,0.001);
KalmanFilter xPosFilter(7.35819,10,0.000016191,0);
KalmanFilter yPosFilter(46.90988,10,0.00008995,0);

//Kalman-Filter ON/OFF switch
bool KFSwitcher = false;

//MyFilterEstimator instantiation
MyFilterEstimator filter;

//Map-Matching
MapMatching mm_one;//for baro calibration
#define Pi 3.1415926

//Kalman Filter
float g;
uint32_t t;
KalmanFilter kf(602,10000,10,818);
KalmanFilter kf1(602,10000,10,818);

//SD Card Datalogger
File logFile;

//GPS stuff => important variables and constants
#define mySerial Serial2 //Serial2 are the pins 16(TX2) and 17(RX2)
Adafruit_GPS GPS(&mySerial);//Instantiate the GPS object
#define GPSECHO false

//SENSOR
float SEALEVELPRESSURE_HPA = 1013.25;
float SEALEVELPRESSURE_HPA_DWD = 1013.25;
float SEALEVELPRESSURE_HPA_RP = 1013.25;//For the reference-point-calculation
float T0 = 15;//Temperature at sea level
float sealevel = 1013.25, sealevelDWD = 1013.25;
float gradient = 0.0065;//Temperaturgradient T(auf Meereshöhe) = t(gemessen) + gradient*Höhe
Adafruit_BME280 bme;//I2C

uint16_t measurement_interval = 1000; //This is how fast the measured values are updated
uint16_t gps_interval = 100;
uint32_t GPSmeaPrev = 0;

//LCD
LiquidCrystal_I2C lcd(0x27,20,4);
bool Backlight = true;

//KEYPAD
const byte rows = 4;
const byte cols = 4;

char keys[rows][cols] = {
  {'1','4','7','A'},
  {'2','5','8','0'},
  {'3','6','9','B'},
  {'F','E','D','C'}
};//Keymap => this is how my keypad is looking

byte rowPins[rows] = {23, 25, 27, 29};//Pins where the cables for the row buttons are located
byte colPins[cols] = {37,35,33,31};//Pins where the cables for the column buttons are located
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, rows, cols); //Initialization fo the keypad object

//STATE_MACHINE
typedef enum {international=0, dwd, gnss, map_matching, comparing} State_type; //This enumeration contains all the main states
void internationaleFormel(), DWD(), gps(), MAP_MATCHING(), COMPARE();
void (*state_table[])() = {internationaleFormel, DWD, gps, MAP_MATCHING, COMPARE}; //This table conatins a pointer to the function to call in each state
State_type curr_state;
State_type prev_state;

volatile bool BUTTON_FLAG_PRESSED[6] = {false,false,false,false,false,false};//Sets a flag if a specific button is pressed
volatile bool BUTTON_FLAG_HOLD[2] = {false,false};//At the moment unused
volatile bool abgleich = true;//Is used to make a Höhenabgleich by changing the normal pressure and temperature
                              //If it's true then the temperature is changed, if false then the pressure
//Other functions
void displayCalStatus(void);
double xPosition(double ay, double headingVel_gps, double headingVel_imu, double heading, int fixquality);
double yPosition(double ay, double headingVel_gps, double headingVel_imu, double heading, int fixquality);
void InitializeSM();
void keypadEvent(KeypadEvent key);

//Interrupts
boolean usingInterrupt = false;
void useInterrupt(boolean);

/*********************************************************************************************************************************************************/
/*    SETUP    */
/*********************************************************************************************************************************************************/

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial){return;}
  
  //Initialization of the StateMachine
  InitializeSM();
}

/*********************************************************************************************************************************************************/
/*    Functions that are only called once    */
/*********************************************************************************************************************************************************/

/**************************************************************************/
/*    State Machine Initialization    */
/**************************************************************************/

void InitializeSM()
{ 
  /********************/
  /*SD Card Datalogger*/
  /********************/
  pinMode(10,OUTPUT);

  //Here we're creating a new logging file to store our data
  if (!SD.begin(10,11,12,13)) Serial.println("Card init. failed");
  char filename[15];
  strcpy(filename, "DELTAH00.TXT");
  for (uint8_t i = 0; i < 100; i++)
  {
    filename[6] = '0' + i/10;
    filename[7] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }

  logFile = SD.open(filename, FILE_WRITE);
  if( ! logFile ) {
    Serial.print("Couldnt create ");
    Serial.println(filename);
  }
  Serial.print("Writing to ");
  Serial.println(filename);
  
//LCD
  lcd.init();
  lcd.backlight();
  
//GPS 
  GPS.begin(9600);
  GPS.sendCommand(PMTK_ENABLE_SBAS);//Enable search for SBAS satellite (works only with 1Hz output rate)
  GPS.sendCommand(PMTK_ENABLE_WAAS);//Enable DGPS to correct the postion data
  //GPS.sendCommand(PMTK_DISABLE_SBAS);
  //GPS.sendCommand(PMTK_DISABLE_WAAS);
  //GPS.sendCommand(PMTK_SET_BAUD_57600);
  
  //mySerial.end();
  //GPS.begin(57600);//new baudrate, so that we can use an update rat of 10 Hz
  
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);//Means nothing else than: print only RMC (lon,lat) and GGA (height over sealevel) data => both have the necessary data
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);//Updating frequence 
  
  //GPS.sendCommand(PGCMD_ANTENNA);// Request updates on antenna status, comment out to keep quiet 
  
  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. 
  useInterrupt(true);
  
  delay(1000);
  //Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);
  
//STATE_MACHINE
  curr_state = international;//First state
  
//MyFilterEstimator: setting initial values for the filterLon and the filterLat
  filter.begin(xPos, yPos);//See initial values at the top under //BNO055 IMU
  
//BME280
  //checks the status of the barometric sensor
  bool status = bme.begin();
  if (!status)
  {
    lcd.setCursor(6,1);
    lcd.print("ERROR!");
    while(1);
  }

  bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                  Adafruit_BME280::SAMPLING_X2, //Sampling rate: Temperature
                  Adafruit_BME280::SAMPLING_X16, //Sampling rate: pressure
                  Adafruit_BME280::SAMPLING_X1, //Sampling rate: humidity
                  Adafruit_BME280::FILTER_X16, //Sampling rate: Filter
                  Adafruit_BME280::STANDBY_MS_0_5);
 
//KEYPAD
  keypad.begin(makeKeymap(keys));//Initialize the keypad object 

  keypad.addEventListener(keypadEvent);//This EventListener becomes active if any key is pressed

//IMU Sensor 
  if (!bno.begin())
  {
    Serial.print("No BNO055 detected");
    while (1);
  }
  bno.setMode(0X0C);
    
  int eeAddress = 0;
  long bnoID = 55;
  bool foundCalib = false;

  EEPROM.get(eeAddress, bnoID);

  adafruit_bno055_offsets_t calibrationData;
  sensor_t sensor;

  /*
  *  Look for the sensor's unique ID at the beginning oF EEPROM.
  *  This isn't foolproof, but it's better than nothing.
  */
  bno.getSensor(&sensor);
  if (bnoID != sensor.sensor_id)
  {
      Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
      delay(500);
  }
  else
  {
      Serial.println("\nFound Calibration for this sensor in EEPROM.");
      eeAddress += sizeof(long);
      EEPROM.get(eeAddress, calibrationData);

      Serial.println("\n\nRestoring Calibration data to the BNO055...");
      bno.setSensorOffsets(calibrationData);

      Serial.println("\n\nCalibration data loaded into BNO055");
      foundCalib = true;
  }

  delay(1000);
  
  bno.setExtCrystalUse(true);

  sensors_event_t event;
  bno.getEvent(&event);
  /* always recal the mag as It goes out of calibration very often */
  if (foundCalib){
      Serial.println("Move sensor slightly to calibrate magnetometers");
      while (!bno.isFullyCalibrated())
      {
          bno.getEvent(&event);
          displayCalStatus();
          delay(BNO055_SAMPLERATE_DELAY_MS);
      }
  }
  else
  {
      Serial.println("Please Calibrate Sensor: ");
      while (!bno.isFullyCalibrated())
      {
          bno.getEvent(&event);

          Serial.print("X: ");
          Serial.print(event.orientation.x, 4);
          Serial.print("\tY: ");
          Serial.print(event.orientation.y, 4);
          Serial.print("\tZ: ");
          Serial.print(event.orientation.z, 4);

          /* Optional: Display calibration status */
          displayCalStatus();

          /* New line for the next sample */
          Serial.println("");

          /* Wait the specified delay before requesting new data */
          delay(BNO055_SAMPLERATE_DELAY_MS);
      }
  }

  Serial.println("\nFully calibrated!");
  Serial.println("--------------------------------");
  Serial.println("Calibration Results: ");
  adafruit_bno055_offsets_t newCalib;
  bno.getSensorOffsets(newCalib);
  //displaySensorOffsets(newCalib);

  Serial.println("\n\nStoring calibration data to EEPROM...");
  
  eeAddress = 0;
  bno.getSensor(&sensor);
  bnoID = sensor.sensor_id;

  EEPROM.put(eeAddress, bnoID);

  eeAddress += sizeof(long);
  EEPROM.put(eeAddress, newCalib);
  Serial.println("Data stored to EEPROM.");

  Serial.println("\n--------------------------------\n");

  lcd.setCursor(5,1);
  lcd.print("I'm ready!");//System is ready for take off!!!
  delay(2000);
}

/**************************************************************************/
    //Display sensor calibration status
/**************************************************************************/
void displayCalStatus(void)
{
    /* Get the four calibration values (0..3) */
    /* Any sensor data reporting 0 should be ignored, */
    /* 3 means 'fully calibrated" */
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    /* The data should be ignored until the system calibration is > 0 */
    Serial.print("\t");
    if (!system)
    {
        Serial.print("! ");
    }

    /* Display the individual values */
    Serial.print("Sys:");
    Serial.print(system, DEC);
    Serial.print(" G:");
    Serial.print(gyro, DEC);
    Serial.print(" A:");
    Serial.print(accel, DEC);
    Serial.print(" M:");
    Serial.print(mag, DEC);
}

/*********************************************************************************************************************************************************/
/*    INTERRUPTS    */
/*********************************************************************************************************************************************************/

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect)
{
  //GPS
  uint8_t c = GPS.read(); //Read NMEA data sentence
  // if you want to debug, this is a good time to do it!
  if ((c) && (GPSECHO))
    Serial.write(c);
}

void useInterrupt(boolean v)
{
  if (v)
  {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0XAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  }else
  {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}
/*********************************************************************************************************************************************************/
/*    LOOP    */
/*********************************************************************************************************************************************************/

uint32_t MesurementPreviousMillis = millis();
uint8_t stopwatch = 0;
void loop() 
{
  // put your main code here, to run repeatedly:
  uint8_t key = keypad.getKey();//Without this the EventListener wouldn't work 
  //if (key) Serial.print(key);//Just to know which key I pressed => acutally useless 

  if (!usingInterrupt)
  {
    //GPS
    uint8_t c = GPS.read(); //Read NMEA data sentence
    // if you want to debug, this is a good time to do it!
    if ((c) && (GPSECHO))
      Serial.write(c);
  }
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) 
  {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
  
  // if millis() or timer wraps around, we'll just reset it
  if (MesurementPreviousMillis > millis())  MesurementPreviousMillis = millis();

  state_table[curr_state]();//Activate states

  //LCD BACKLIGHT ON/OFF
  if (Backlight) lcd.backlight();
  else 
  {
    lcd.noBacklight();
    stopwatch = 0;
  }
}

/*********************************************************************************************************************************************************/
/*    Main modes: internationale Höhenformel, DWD, GPS, Vergleich und Map-Matching    */
/*********************************************************************************************************************************************************/

/**************************************************************************/
/*    International Height Formulae    */
/**************************************************************************/

void internationaleFormel()
{  
 if (millis() - MesurementPreviousMillis >= measurement_interval)
 {
  MesurementPreviousMillis = millis();
  
  stopwatch += 1;
  if (stopwatch >= 10) Backlight = false;
 
  lcd.clear();
  float height = bme.readAltitude(SEALEVELPRESSURE_HPA, T0, gradient);//My own defined function in the Adafruit BME280 Library
  float temperature = bme.readTemperature();
  float pressure = bme.readPressure()/100;
  lcd.setCursor(0,0);
  lcd.print("Intern. Formel");
  lcd.setCursor(0,1);
  lcd.print("P0=");
  lcd.print(SEALEVELPRESSURE_HPA);
  lcd.print(" T0=");
  lcd.print(T0);
  lcd.setCursor(0,2);
  lcd.print("h=");
  lcd.print(height);
  lcd.print(" T=");
  lcd.print(temperature);
  lcd.setCursor(0,3);
  lcd.print("P=");
  lcd.print(pressure);
  lcd.print("hPa");
 }
  if (BUTTON_FLAG_PRESSED[0])
  {
    curr_state = dwd;
    stopwatch = 0;
    BUTTON_FLAG_PRESSED[0] = false;
  }

  if (BUTTON_FLAG_PRESSED[1])
  {
    curr_state = comparing;
    BUTTON_FLAG_PRESSED[1] = false;
  }
  
  //Höhenabgleich which is activated by the E button; After I pressed the E button, I can increase D and decrease C the height by changing either the sealevel Pressure or the Temperature at sealevel
  switch (abgleich)
  {
    case true:
      if (BUTTON_FLAG_PRESSED[2]) 
      {
        SEALEVELPRESSURE_HPA -= 0.1;
        BUTTON_FLAG_PRESSED[2] = false;
      }
      else if (BUTTON_FLAG_HOLD[0]) SEALEVELPRESSURE_HPA -= 0.001;

      if (BUTTON_FLAG_PRESSED[3]) 
      {
        SEALEVELPRESSURE_HPA += 0.1;
        BUTTON_FLAG_PRESSED[3] = false;
      }
      else if (BUTTON_FLAG_HOLD[1]) SEALEVELPRESSURE_HPA += 0.001;
      
    break;
    case false:
      if (BUTTON_FLAG_PRESSED[2]) 
      {
        T0 -= 0.1;
        BUTTON_FLAG_PRESSED[2] = false;
      }
      else if (BUTTON_FLAG_HOLD[0]) T0 -= 0.001;

      if (BUTTON_FLAG_PRESSED[3]) 
      {
        T0 += 0.1;
        BUTTON_FLAG_PRESSED[3] = false;
      }
      else if (BUTTON_FLAG_HOLD[1]) T0 += 0.001;
    break; 
  }

  //Activate Map-Matching mode
  if (BUTTON_FLAG_PRESSED[4])
  {
    prev_state = curr_state;
    curr_state = map_matching;
    BUTTON_FLAG_PRESSED[4] = false;
    BUTTON_FLAG_PRESSED[3] = false;
  }
}

/**************************************************************************/
/*    DWD    */
/**************************************************************************/

void DWD()
{
  if (millis() - MesurementPreviousMillis >= measurement_interval)
  {
    MesurementPreviousMillis = millis();
    lcd.clear();
    float height = bme.readAltitude_DWD(SEALEVELPRESSURE_HPA_DWD*100, gradient);//My own defined function in the Adafruit BME280 Library
    float temperature = bme.readTemperature();
    float pressure = bme.readPressure()/100;
    lcd.setCursor(0,0);
    lcd.print("DWD Formel");
    lcd.setCursor(0,1);
    lcd.print("P0=");
    lcd.print(SEALEVELPRESSURE_HPA_DWD);
    lcd.setCursor(0,2);
    lcd.print("h=");
    lcd.print(height);
    lcd.print(" T=");
    lcd.print(temperature);
    lcd.setCursor(0,3); 
    lcd.print("P=");
    lcd.print(pressure);
    lcd.print("hPa");

    stopwatch += 1;
    if (stopwatch >= 10) Backlight = false;
  }

  //Change the state
  if (BUTTON_FLAG_PRESSED[0])
  {
    curr_state = gnss;
    stopwatch = 0;
    BUTTON_FLAG_PRESSED[0] = false;
  }

  //Change the state
  if (BUTTON_FLAG_PRESSED[1])
  {
    curr_state = international;
    BUTTON_FLAG_PRESSED[1] = false;
  }

  //decrease the sealevel pressure 
  if (BUTTON_FLAG_PRESSED[2]) 
  {
    SEALEVELPRESSURE_HPA_DWD -= 0.1;
    BUTTON_FLAG_PRESSED[2] = false;
  }
  else if (BUTTON_FLAG_HOLD[0]) SEALEVELPRESSURE_HPA_DWD -= 0.001;

  //Increase the sealevel pressure
  if (BUTTON_FLAG_PRESSED[3]) 
  {
    SEALEVELPRESSURE_HPA_DWD += 0.1;
    BUTTON_FLAG_PRESSED[3] = false;
  }
  else if (BUTTON_FLAG_HOLD[1]) SEALEVELPRESSURE_HPA_DWD += 0.001;
  
  //Activate Map-Matching mode
  if (BUTTON_FLAG_PRESSED[4])
  {
    prev_state = curr_state;
    curr_state = map_matching;
    BUTTON_FLAG_PRESSED[4] = false;
    BUTTON_FLAG_PRESSED[3] = false;
  }
}

/**************************************************************************/
/*    GPS    */
/**************************************************************************/
bool change = false;
void gps()
{
  double x, y;
  if (millis() - MesurementPreviousMillis >= measurement_interval)
  {
    MesurementPreviousMillis = millis();
    
    //ON/OFF OF THE BACKLIGHT
    stopwatch += 1;
    if (stopwatch >= 10) Backlight = false;
    
    lcd.clear();
    lcd.print("Fix: "); lcd.print((int)GPS.fixquality);
    
    if (GPS.fix) 
    {
      lcd.setCursor(0,0);
      lcd.print("GPS");
      lcd.setCursor(0,1);
      lcd.print("Lat:");
      if (change) lcd.print(GPS.latitudeDegrees, 5); 
      else lcd.print(GPS.latitude);
      lcd.print(GPS.lat);
      lcd.setCursor(0,2);
      lcd.print("Lon:");
      if (change) lcd.print(GPS.longitudeDegrees, 5); 
      else lcd.print(GPS.longitude);
      lcd.print(GPS.lon);
      lcd.setCursor(0,3);
      lcd.print("h="); lcd.println(GPS.altitude,1);
      lcd.print(" Sat.="); lcd.print((int)GPS.satellites);
    }
  }

  //Change the state
  if (BUTTON_FLAG_PRESSED[0])
  {
    curr_state = comparing;
    stopwatch = 0;
    BUTTON_FLAG_PRESSED[0] = false;
    BUTTON_FLAG_PRESSED[2] = false;
    BUTTON_FLAG_PRESSED[4] = false;
    BUTTON_FLAG_HOLD[0] = false;
  }

  //Change the state
  if (BUTTON_FLAG_PRESSED[1])
  {
    curr_state = dwd;
    BUTTON_FLAG_PRESSED[1] = false;
    BUTTON_FLAG_PRESSED[2] = false;
    BUTTON_FLAG_PRESSED[4] = false;
    BUTTON_FLAG_HOLD[0] = false;
  }

  //change the Position output (degrees or degrees and minutes)
  if (BUTTON_FLAG_PRESSED[2])
  {
    change = !change;
    BUTTON_FLAG_PRESSED[2] = false;
  }
}

/**************************************************************************/
/*    Comparison of the different height estimation    */
/**************************************************************************/
bool measurement_control = false;
bool long_measurement = false;
void COMPARE()
{
  float height_I;
  float height_dwd;
  float height_gps;
  float height_model;
  
  if (millis() - MesurementPreviousMillis >= measurement_interval)
  {
    lcd.clear();
    MesurementPreviousMillis = millis();
    if (GPS.fix)
    { 
      //International Height
      height_I = bme.readAltitude(SEALEVELPRESSURE_HPA, T0, gradient);
  
      //DWD Height
      height_dwd = bme.readAltitude_DWD(SEALEVELPRESSURE_HPA_DWD*100, gradient);//My own defined function in the Adafruit BME280 Library
  
      //GPS Height
      height_gps = GPS.altitude;
  
      //Height Model Test 
      height_model = model.search(GPS.longitudeDegrees,GPS.latitudeDegrees);
      
      //International Height
      lcd.setCursor(0,1);
      lcd.print("hI=");
      lcd.print(height_I);
      
      //DWD Height
      lcd.setCursor(0,2);
      lcd.print("hD=");
      lcd.print(height_dwd);
      
      //GPS Height 
      lcd.setCursor(0,3);
      lcd.print("hG="); lcd.print(height_gps);
      
      //Height Model Test 
      lcd.setCursor(0,0);
      lcd.print("hM=");
      lcd.print(height_model);

      if (measurement_control)
      {
        lcd.setCursor(11,0);
        lcd.print("READ");
        logFile.print(height_I);
        logFile.print(",");
        logFile.print(height_dwd);
        logFile.print(",");
        logFile.print(height_gps);
        logFile.print(",");
        logFile.print(height_model);
        logFile.println(",");
        logFile.flush();
        measurement_control = false;
      }else if (long_measurement)
      {
        lcd.setCursor(11,0);
        lcd.print("READING");
        logFile.print(height_I);
        logFile.print(",");
        logFile.print(height_dwd);
        logFile.print(",");
        logFile.print(height_gps);
        logFile.print(",");
        logFile.print(height_model);
        logFile.println(",");
        logFile.flush();
      }
    }
  }

  //Button b
  if (BUTTON_FLAG_PRESSED[0])
  {
    curr_state = international;
    BUTTON_FLAG_PRESSED[0] = false;
    BUTTON_FLAG_HOLD[0] = false;
    BUTTON_FLAG_PRESSED[2] = false;
    BUTTON_FLAG_PRESSED[4] = false;
  }
 
  //Button A
  if (BUTTON_FLAG_PRESSED[1])
  {
    curr_state = gnss;
    BUTTON_FLAG_HOLD[0] = false;
    BUTTON_FLAG_PRESSED[1] = false;
    BUTTON_FLAG_PRESSED[2] = false;
    BUTTON_FLAG_PRESSED[4] = false;
  }

  //Button C
  if (BUTTON_FLAG_PRESSED[2])
  {
    measurement_control = true;
    BUTTON_FLAG_PRESSED[2] = false;
  }

  //Button C
  if (BUTTON_FLAG_HOLD[0])
  {
    long_measurement = !long_measurement;
    BUTTON_FLAG_HOLD[0] = false;
  }
}


/**************************************************************************/
/*    Map-Matching-State    */
/**************************************************************************/

bool measurement_control_mm = false;
bool long_measurement_mm = false;
bool show_vel = false;
void MAP_MATCHING()
{
  double checker = 0; //if it's is 1 that means we reached a fixpoint otherwise it prints 0
  double mm_one_dist, mm_dist;
  double raw_lon, raw_lat;//So that we can reset our mapmatching alogrithm in case of not-satisfing measurement results
  int fixquality; //Determines how good the connection is of the gps reciever and the satelites: e.g. in a tunnel = low fixquality, on a landscape = high fixquality
  float headingVel_gps;//Heading velocity calculated by gps
  float headingVel_imu;//heading velocity calculated by imu
  float gps_height, model_height;
  

  //Used sensor for estimating the position and orientation  
  imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  
  //read the magnetic field
  float mag_y = mag.y();
  float mag_x = mag.x();
  //Rotation around the Z axis which is basically pointing towards the sky
  float heading_direction = 0;
  if (int(mag_y) > 0) heading_direction = atan2(-mag_x,-mag_y)*180/Pi;
  if (int(mag_y) == 0 && int(mag_x) > 0) heading_direction = -90;
  if (int(mag_y) == 0 && int(mag_x) < 0) heading_direction = 90;
  if (int(mag_x) == 0 && int(mag_y) > 0) heading_direction = 180;
  if (int(mag_x) == 0 && int(mag_y) < 0) heading_direction = 0;
  if (int(mag_y) < 0 && int(mag_x) > 0) heading_direction = atan2(-mag_x, -mag_y)*180/Pi;
  if (int(mag_y) < 0 && int(mag_x) < 0) heading_direction = atan2(-mag_x, -mag_y)*180/Pi;

  //Updates every seconds the display and stores data on the SD card
  if (millis() - MesurementPreviousMillis >= measurement_interval)
  {
    MesurementPreviousMillis = millis();
    lcd.clear();
    //ON/OFF OF THE BACKLIGHT
    stopwatch += 1;
    if (stopwatch >= 10) Backlight = false;

    //Read velocity from imu 
    headingVel_imu = ACCEL_VEL_TRANSITION*accel.y()-corrHeadingVel_imu;//Calculating the heading velocity based on measurements of the imu sensor
    
    //bias of the headingVel_imu
    if ((int)headingVel_imu*10==0) corrHeadingVel_imu = headingVel_imu;
    
    //If we have a GPS Fix we read the speed in knots, convert it to m/s; MapMatching occurs in this if-statement
    if (GPS.fix) 
    {
      //GPS height, raw lon and lat 
      gps_height = GPS.altitude;
      raw_lon = GPS.longitudeDegrees;
      raw_lat = GPS.latitudeDegrees;
    
      //GPS velocity and fixquality
      //headingVel_gps = (GPS.speed-corrHeadingVel_gps)*1.852/3.6; //Estimation of velocity; vel_knots*1.852/3.6=velocity in m/s 
      //if ((int)headingVel_gps*10==0) corrHeadingVel_gps = GPS.speed;//If no movement is done store the bias 
      fixquality = GPS.fixquality;//0 = no fix, 1 = GPS and 2 = DGPS
    }
    
    //model height
    model_height = model.search(raw_lon,raw_lat);

    //change the cali_dist to 15 if dgps is available
    if (fixquality==2) mm_one.cali_dist = 15;
    else mm_one.cali_dist = 20;
    
    //If this is false the Map-Matching System will start.
    if (mm_one.RESET)
    {
      lcd.setCursor(6,0);
      lcd.print("START");

      //MapMatching
      mm_one.begin(raw_lon, raw_lat, true);
      latitude = mm_one.yPos;
      longitude = mm_one.xPos;
   
      //Calibrate barometer if a fix point is reached
      sealevel = bme.seaLevelForAltitude(mm_one.calibrate_BARO(), bme.readPressure()/100);
      sealevelDWD = bme.seaLevelForAltitudeDWD(mm_one.calibrate_BARO(), bme.readPressure())/100;
      xPos = raw_lon;
      yPos = raw_lat;
    }else
    {
      //Measurement input for the MyFilterEstimator
      filter.compare(raw_lon,raw_lat);
      
      //Getting heading velocity from MapMatching library
      headingVel_gps = sqrt(filter.distance);

      //Calculating the xPos and yPos in degrees
      //angle_gps and angle_gyro are used in a complementary filter to estimate heading direction
      //xPos, yPos, xVel and yVel are global variables
      longitude = xPosition(accel.y(), headingVel_gps, headingVel_imu, heading_direction, fixquality);//We use here the y acceleration, because it is the axis which is always pointing into the direction of movement
      latitude = yPosition(accel.y(), headingVel_gps, headingVel_imu, heading_direction, fixquality);

      //calibrate GPS
      mm_one.calibrate_GPS(longitude, latitude, true);
      mm_one_dist = mm_one.distance();
      latitude = mm_one.yPos;
      longitude = mm_one.xPos;
      
      //as a control, in the case if the calibrated GPS values are worse than the normal ones 
      mm_one.calibrate_GPS(raw_lon,raw_lat,false);
      mm_dist = mm_one.distance();
      
      //Get calibration status
      if (mm_one_dist <= mm_one.cali_dist || mm_dist <= mm_one.cali_dist)
      {
        if (!mm_one.inside_cal_area)
        {
          //Calibrate barometer if a fix point is reached
          sealevel = bme.seaLevelForAltitude(mm_one.calibrate_BARO(), bme.readPressure()/100);
          sealevelDWD = bme.seaLevelForAltitudeDWD(mm_one.calibrate_BARO(), bme.readPressure())/100;
        }
        xPos = raw_lon;
        yPos = raw_lat;
        checker = 1;
      }

      lcd.setCursor(6,0);
      lcd.print("Checker: ");
      lcd.print(checker,0);
    }
    
    switch (show_vel)
    {
      case false:
        //Print the estimated Position
        lcd.setCursor(0,1);
        lcd.print("M:");
        lcd.print(latitude, 5); lcd.print(GPS.lat);
        lcd.print(",");
        lcd.print(longitude, 5); lcd.print(GPS.lon);
      break;
      
      case true:
        //Print the estimated Position
        lcd.setCursor(0,1);
        lcd.print("Vel:");
        lcd.print(headingVel_gps*3.6, 2); 
        lcd.print(",");
        lcd.print(sqrt(xVel*xVel+yVel*yVel)*3.6, 2);
      break;
    }

    lcd.setCursor(0,0);
    lcd.print("GPS");
    lcd.setCursor(4,0);
    lcd.print(GPS.fixquality);
    lcd.setCursor(0,2);
    lcd.print("P:"); lcd.print(raw_lat,5); lcd.print(GPS.lat);
    lcd.print(","); lcd.print(raw_lon,5); lcd.print(GPS.lon);
    
    lcd.setCursor(0,3);
    lcd.print("HI=");
    lcd.print(bme.readAltitude(sealevel, T0, gradient));
    lcd.print(" DWD=");
    lcd.print(bme.readAltitude_DWD(sealevelDWD*100, gradient)); 

    //Logging data latitude, longitude and height
    if (measurement_control_mm)
    {
      measurement_control_mm = false;
      lcd.setCursor(17,0);
      lcd.print("R");//short reding
      logFile.print(longitude,6);
      logFile.print(",");
      logFile.print(latitude,6);
      logFile.print(",");
      logFile.print(raw_lon,6);
      logFile.print(",");
      logFile.print(raw_lat,6);
      logFile.print(",");
      logFile.print(xPos,6);
      logFile.print(",");
      logFile.print(yPos,6);
      logFile.print(",");
      logFile.print(filter.distance);
      logFile.print(",");
      logFile.print(headingVel_gps);
      logFile.print(",");
      logFile.print(headingVel_imu);
      logFile.print(",");
      logFile.print(checker);
      logFile.print(",");
      logFile.print(bme.readAltitude(sealevel, T0, gradient));
      logFile.print(",");
      logFile.print(bme.readAltitude_DWD(sealevelDWD*100, gradient));
      logFile.print(",");
      logFile.print(bme.readAltitude(SEALEVELPRESSURE_HPA, T0, gradient));
      logFile.print(",");
      logFile.print(bme.readAltitude_DWD(SEALEVELPRESSURE_HPA_DWD*100, gradient));
      logFile.print(",");
      logFile.print(gps_height);
      logFile.print(",");
      logFile.print(model_height);
      logFile.println(",");
      logFile.flush();
    }else if (long_measurement_mm)
    {
      lcd.setCursor(17,0);
      lcd.print("LR");//long reading
      logFile.print(latitude,6);
      logFile.print(",");
      logFile.print(longitude,6);
      logFile.print(",");
      logFile.print(raw_lon,6);
      logFile.print(",");
      logFile.print(raw_lat,6);
      logFile.print(",");
      logFile.print(xPos,6);
      logFile.print(",");
      logFile.print(yPos,6);
      logFile.print(",");
      logFile.print(filter.distance);
      logFile.print(",");
      logFile.print(headingVel_gps);
      logFile.print(",");
      logFile.print(headingVel_imu);
      logFile.print(",");
      logFile.print(checker);
      logFile.print(",");
      logFile.print(bme.readAltitude(sealevel, T0, gradient));
      logFile.print(",");
      logFile.print(bme.readAltitude_DWD(sealevelDWD*100, gradient));
      logFile.print(",");
      logFile.print(bme.readAltitude(SEALEVELPRESSURE_HPA, T0, gradient));
      logFile.print(",");
      logFile.print(bme.readAltitude_DWD(SEALEVELPRESSURE_HPA_DWD*100, gradient));
      logFile.print(",");
      logFile.print(gps_height);
      logFile.print(",");
      logFile.print(model_height);
      logFile.println(",");
      logFile.flush();
    }
  }
  //automatic Map_Matching: The reference points are selceted by the program; Button C
  if (BUTTON_FLAG_PRESSED[2])
  {
    mm_one.RESET = !mm_one.RESET;
    BUTTON_FLAG_PRESSED[2] = false;
  }

  //Exit Map-Matching mode
  if (BUTTON_FLAG_PRESSED[4])
  {
    curr_state = prev_state;//Button F
    BUTTON_FLAG_HOLD[0] = false;
    BUTTON_FLAG_PRESSED[0] = false;
    BUTTON_FLAG_PRESSED[1] = false;
    BUTTON_FLAG_PRESSED[2] = false;
    BUTTON_FLAG_PRESSED[3] = false;
    BUTTON_FLAG_PRESSED[4] = false;
  }

  //Button 0
  if (BUTTON_FLAG_PRESSED[5])
  {
    show_vel = !show_vel; //boolean: through this you can see the headingVel_gps and the velocity, which is fusion of the velocity of the imu and the gps
    BUTTON_FLAG_PRESSED[5] = false;
  }
  //Button D
  if (BUTTON_FLAG_PRESSED[3])
  {
    measurement_control_mm = true;
    BUTTON_FLAG_PRESSED[3] = false;
  }

  //Button D
  if (BUTTON_FLAG_HOLD[1])
  {
    long_measurement_mm = !long_measurement_mm;
    BUTTON_FLAG_HOLD[1] = false;
  }

  //Button 0 in case of something goes wrong there is the possibility to reset everything
  if (BUTTON_FLAG_PRESSED[5])
  {
    filter.prev_lon = raw_lon;
    filter.prev_lat = raw_lat;
    mm_one.compensation_lon = 0;
    mm_one.compensation_lat = 0;
    BUTTON_FLAG_PRESSED[5] = false;
  }
}

/*********************************************************************************************************************************************************/
/*    Other functions which are used more than once: xPosition, yPosition, KeypadEventListener    */
/*********************************************************************************************************************************************************/

/**************************************************************************/
/*    KeypadEventListener    */
/**************************************************************************/

void keypadEvent(KeypadEvent key)
{
  switch (keypad.getState())
  {
    case PRESSED:
      Backlight = true;
      stopwatch = 0; 
      if (key=='B') BUTTON_FLAG_PRESSED[0] = true;//Go one state forward
      if (key=='A') BUTTON_FLAG_PRESSED[1] = true;//Go one state backward
      if (key=='C') BUTTON_FLAG_PRESSED[2] = true;//Decrease
      if (key=='D') BUTTON_FLAG_PRESSED[3] = true;//Increase
      if (key=='F') BUTTON_FLAG_PRESSED[4] = true;//Start Map-Matching state
      if (key=='0') BUTTON_FLAG_PRESSED[5] = true;//Reset the compensation values, which are calculated in the MapMatching library
      if (key=='E') 
      { 
        abgleich = !abgleich;
        BUTTON_FLAG_PRESSED[5] = false;
      }
    break;
    
    case HOLD:
      if (key=='C') BUTTON_FLAG_HOLD[0] = !BUTTON_FLAG_HOLD[0];//Decrease
      if (key=='D') BUTTON_FLAG_HOLD[1] = !BUTTON_FLAG_HOLD[1];//Increase
    break;
  }
}

/**************************************************************************/
    //X-Position Estimator
/**************************************************************************/
double xPosition(double ay, double headingVel_gps, double headingVel_imu, double heading, int fixquality)
{
  float estimation;
  double lon;
  
  //Zeroing the velocity if no movement is present, because the gps sends always some small velocity values although nothing is moving (random process)
  if (fixquality==0)
  {
    if ((int)headingVel_imu*10==0) 
    {
      xVel = 0;
      yVel = 0;
      headingVel_gps = 0;
      headingVel_imu = 0;
    }
  }else
  {
    if ((int)headingVel_gps*10==0) 
    {
      xVel = 0;
      yVel = 0;
      headingVel_gps = 0;
      headingVel_imu = 0;
    }
  }
  
  //calulating the longitudenal acceleration and velocity
  float accel_x = ay*cos(heading*Pi/180);
  float velocity = headingVel_gps*cos(heading*Pi/180);
  
  //calculating the longitude velocity
  xVel = xVel + accel_x*ACCEL_VEL_TRANSITION;
  
  //Increase the tolerated deviation for the Filter
  if (headingVel_gps*headingVel_gps > 2500) filter.max_distance = headingVel_gps*headingVel_gps;
  else filter.max_distance = 2500;//the distances are squared so that the board has to calculate less; here we have a max. distance of 50m
  
  //calculate the position in degrees (Our model for the Kalman Filter)
  float xPos1 = xPos + velocity*ACCEL_VEL_TRANSITION*METER_2_DEG;//calculating the position with the measure velocity
  xPos = xPos + xVel*ACCEL_VEL_TRANSITION*METER_2_DEG;//calculating the position with the calculated velocity
  xPos = 0.1*xPos+0.9*xPos1;//Measurements have shown that the velocity calculated from the previous and the current point is more precise than the velocity from the imu sensor
  
  //Model Input for the MyFilterEstimator
  filter.modelInputLon(xPos);

  //Start Filtering the GPS data
  lon = filter.est_lon;
  
  //If KFSwitcher is true (1) then a simple Kalman Filter is used when we only have a fixquality of 1 (=GPS) else we only use the MapMatching
  if (KFSwitcher)
  {
    if (fixquality==1)
    {
      estimation = xPosFilter.update(xPos);
      est_error = xPosFilter.predict(lon);
    }
    else if (fixquality==2)
    {
      estimation = lon; //if we have a fixquality of 2 then use only the position from the gps
    }else
    {
      estimation = xPos;
    }
  }else
  {
    if (fixquality)
    {
      estimation = lon;
      xPos = lon;
    }
    else
    {
      estimation = xPos;
    }
  }
  return estimation;
}

/**************************************************************************/
    //Y-Position Estimator
/**************************************************************************/

double yPosition(double ay, double headingVel_gps, double headingVel_imu, double heading, int fixquality)
{
  float estimation;
  double lat;
  
  //Zeroing the velocity if no movement is present, because the gps sends always some small velocity values although nothing is moving (random process)  
  if (fixquality==0)
  {
    if ((int)headingVel_imu*10==0) 
    {
      xVel = 0;
      yVel = 0;
      headingVel_gps = 0;
      headingVel_imu = 0;
    }
  }else
  {
    if ((int)headingVel_gps*10==0) 
    {
      xVel = 0;
      yVel = 0;
      headingVel_gps = 0;
      headingVel_imu = 0;
    }
  }
  
  //calulating the latitudenal acceleration and velocity
  float accel_y = ay*sin(heading*Pi/180);
  float velocity = headingVel_gps*sin(heading*Pi/180);
  
  //calculating the latitude velocity
  yVel = yVel + accel_y*ACCEL_VEL_TRANSITION;
  
  //Increase the tolerated deviation for the Filter
  if (headingVel_gps*headingVel_gps > 10000) filter.max_distance = headingVel_gps*headingVel_gps;
  else filter.max_distance = 10000;//the distances are squared so that the board has to calculate less; here we have a max. distance of 50m
  
  //calculating the latitude in degrees (our model for the Kalman Filter)
  float yPos1 = yPos + velocity*ACCEL_VEL_TRANSITION*METER_2_DEG;//calculating the position with the measured velocity 
  yPos = yPos + yVel*ACCEL_VEL_TRANSITION*METER_2_DEG;//calculating the position with the calculated velocity
  yPos = 0.1*yPos+0.9*yPos1;//Measurements have shown that the velocity calculated from the previous and the current point is more precise than the velocity from the imu sensor 
  
  //Model Input for the MyFilterEstimator
  filter.modelInputLat(yPos);

  //Start Filtering the GPS data
  lat = filter.est_lat;
  
  //we use only the Kalman Filter if we also have a gps signal
  if (KFSwitcher)
  {
    if (fixquality==1)
    {
      estimation = yPosFilter.update(yPos);
      float est_error = yPosFilter.predict(lat);
    }
    else if (fixquality==2)
    {
      estimation = lat;//if we have a fixquality of 2 then use the position from the gps
    }else
    {
      estimation = yPos;  
    } 
  }else
  {
    if (fixquality) 
    {
      estimation = lat;
      yPos = lat;
    }
    else
    {
      yPos = yPos + yVel*ACCEL_VEL_TRANSITION*METER_2_DEG;
      estimation = yPos;
    }
  }
  return estimation;
}
