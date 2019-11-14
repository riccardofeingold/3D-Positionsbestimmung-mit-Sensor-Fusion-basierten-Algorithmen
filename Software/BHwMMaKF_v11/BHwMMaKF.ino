/*!
 * @file BHwMMaKF.ino Version 9.0
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
 * One of the main goal was to program the embedded system in such a way that is almost to every
 * situation adaptive. This was established by using Sensor Fusion based algorithms such as Map-Matching,
 * Kalman-Filtering and Complementary-Filter.
 * 
 * The IMU is used to calculate the position by using the orientation and the acceleration. Combined with the GPS
 * it will be then processed by the Kalman-Filter in order to get a better estimate. 
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
#include "MapMatching.h"
#include "HEIGHT_DATA.h"

/*BNO055 Accelerometer*/
#include <Wire.h>
#include <Filter.h> 
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
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
double longitude=7.35819, latitude=46.90988;
double est_error = 0;

//Rcursive Filters for BNO055
double corrY=0, corrZ=0, corrHeadingVel_gps=0, corrHeadingVel_imu=0;//those variables are used as bias
ExponentialFilter<float> filterY(20,0);
ExponentialFilter<float> filterZ(15,0);
ExponentialFilter<float> GPSVEL(20,0);

//Kalman Filter for BNO055
KalmanFilter velFilter(0,5,0.1,0.001);
KalmanFilter xPosFilter(7.35819,10,0.000016191,0);
KalmanFilter yPosFilter(46.90988,10,0.00008995,0);

//Kalman-Filter ON/OFF switch
#define KFSwitcher 0

//Map-Matching
MapMatching mm; 

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
uint32_t MesurementPreviousMillis = 0; //Stores the last time when BME280 measured somthing
float gradient = 0.0065;//Temperaturgradient T(auf Meereshöhe) = t(gemessen) + gradient*Höhe
Adafruit_BME280 bme;//I2C

uint16_t measurement_interval = 1000; //This is how fast the measured values are updated

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
typedef enum {international=0, dwd, gnss, map_matching, learning} State_type; //This enumeration contains all the main states
void internationaleFormel(), DWD(), gps(), MAP_MATCHING(), LEARN();
void (*state_table[])() = {internationaleFormel, DWD, gps, MAP_MATCHING, LEARN}; //This table conatins a pointer to the function to call in each state
State_type curr_state;
State_type prev_state;

volatile bool BUTTON_FLAG_PRESSED[6] = {false,false,false,false,false,false};//Sets a flag if a specific button is pressed
volatile bool BUTTON_FLAG_HOLD[2] = {false,false};//At the moment unused
volatile bool abgleich = true;//Is used to make a Höhenabgleich by changing the normal pressure and temperature
                              //If it's true then the temperature is changed, if false then the pressure

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


/**************************************************************************/
    //X-Position Estimator
/**************************************************************************/
double xPosition(double ay, double az, double headingVel_gps, double headingVel_imu, double lon, double angle_gps, double angle_gyro, double angle_pitch, int fixquality)
{
  float a;
  float estimation;
  
  //Decide whether we take the Z-Acceleration into account or not
  if (angle_pitch>10||angle_pitch<-10) a = sqrt(sq(ay)+sq(az));
  else a = abs(ay);

  //Complementary Filter for velocity
  float velocity = 0.98*headingVel_imu + 0.02*headingVel_gps;
  
  //Zeroing the velocity if no movement is present, because the gps sends always some small velocity values although nothing is moving (random process)
  if (fixquality==0)
  {
    if ((int)headingVel_gps*10==0||(int)headingVel_imu*10==0||(int)velocity*10==0) 
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
  
  //complementary filter for estimating the angle 
  float angle_in_rad;
  if (fixquality==2) angle_in_rad = (0.98*angle_gyro + 0.02*angle_gps)*DEG_2_RAD;
  else angle_in_rad = angle_gyro;
  
  //calulating the longitudenal acceleration
  float accel_x = a*cos(angle_in_rad);

  //calculating the longitude velocity
  xVel = xVel + accel_x*ACCEL_VEL_TRANSITION;

  //If KFSwitcher is true (1) then a simple Kalman Filter is used when we only have a fixquality of 1 (=GPS) else we only use the MapMatching
  if (KFSwitcher)
  {
    if (fixquality==1)
    {
      //calculate the position in degrees (Our model for the Kalman Filter)
      xPos = xPos + xVel*ACCEL_VEL_TRANSITION*METER_2_DEG;
      estimation = xPosFilter.update(xPos);
      est_error = xPosFilter.predict(lon);
    }
    else if (fixquality==2)
    {
      estimation = lon; //if we have a fixquality of 2 then use only the position from the gps
    }else
    {
      xPos = xPos + xVel*ACCEL_VEL_TRANSITION*METER_2_DEG;
      estimation = xPos;
    }
  }else
  {
    if (fixquality>0)  estimation = lon;
    else
    {
      xPos = xPos + xVel*ACCEL_VEL_TRANSITION*METER_2_DEG;
      estimation = xPos;
    }
  }
  return estimation;
}

/**************************************************************************/
    //Y-Position Estimator
/**************************************************************************/

double yPosition(double ay, double az, double headingVel_gps, double headingVel_imu, double lat, double angle_gps, double angle_gyro, double angle_pitch, int fixquality)
{
  float a;
  float estimation;
  
  //Decide whether we take the Z-Acceleration into account or not
  if (angle_pitch>10||angle_pitch<-10) a = sqrt(sq(ay)+sq(az));
  else a = abs(ay);

  //Complementary Filter for velocity
  float velocity = 0.98*headingVel_imu + 0.02*headingVel_gps;
  
  //Zeroing the velocity if no movement is present, because the gps sends always some small velocity values although nothing is moving (random process)  
  if (fixquality==0)
  {
    if ((int)headingVel_gps*10==0||(int)headingVel_imu*10==0||(int)velocity*10==0) 
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

  //complementary filter for estimating the angle 
  float angle_in_rad;
  if (fixquality==2) angle_in_rad = (0.98*angle_gyro + 0.02*angle_gps)*DEG_2_RAD;
  else angle_in_rad = angle_gyro;
  
  //calulating the latitudenal acceleration
  float accel_y = a*sin(angle_in_rad);

  //calculating the latitude velocity
  yVel = yVel + accel_y*ACCEL_VEL_TRANSITION;
 
  //we use only the Kalman Filter if we also have a gps signal
  if (KFSwitcher)
  {
    if (fixquality==1)
    {
      //calculating the latitude in degrees (our model for the Kalman Filter)
      yPos = yPos + yVel*ACCEL_VEL_TRANSITION*METER_2_DEG;
      estimation = yPosFilter.update(yPos);
      float est_error = yPosFilter.predict(lat);
    }
    else if (fixquality==2)
    {
      estimation = lat;//if we have a fixquality of 2 then use the position from the gps
    }else
    {
      yPos = yPos + yVel*ACCEL_VEL_TRANSITION*METER_2_DEG;
      estimation = yPos;  
    } 
  }else 
  {
    if (fixquality>0) estimation = lat;
    else
    {
      yPos = yPos + yVel*ACCEL_VEL_TRANSITION*METER_2_DEG;
      estimation = yPos;
    }
  }
  return estimation;
}
/**************************************************************************/
/*    SETUP    */
/**************************************************************************/

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial){return;}
  
  //Initialization of the StateMachine
  InitializeSM();
}

/**************************************************************************/
/*    State Machine Initialization    */
/**************************************************************************/

void InitializeSM()
{ 
  /********************/
  /*SD Card Datalogger*/
  /********************/
  pinMode(10,OUTPUT);

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
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);//Means nothing else than: print only RMC (lon,lat) and GGA (height over sealevel) data => both have the necessary data
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);//Updating frequence 
  GPS.sendCommand(PGCMD_ANTENNA);//I assume this says the GPS-shield that an antenna is attached 
  delay(1000);
  mySerial.println(PMTK_Q_RELEASE);
  
//STATE_MACHINE
  curr_state = international;//First state

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
/*    LOOP    */
/**************************************************************************/

uint32_t timer = millis();
uint8_t stopwatch = 0;
void loop() 
{
  t = millis();
  // put your main code here, to run repeatedly:
  uint8_t key = keypad.getKey();//Without this the EventListener wouldn't work 
  //if (key) Serial.print(key);//Just to know which key I pressed => acutally useless 
  
  //GPS
  uint8_t c = GPS.read(); //Read NMEA data sentence
  // if you want to debug, this is a good time to do it!
  if ((c) && (GPSECHO))
    Serial.write(c);

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
  if (timer > millis())  timer = millis();

  state_table[curr_state]();//Activate states

  //LCD BACKLIGHT ON/OFF
  if (Backlight) lcd.backlight();
  else 
  {
    lcd.noBacklight();
    stopwatch = 0;
  }
}

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
/*    International Height Formulae    */
/**************************************************************************/

void internationaleFormel()
{ 
 float current_time = millis();
 
 if (current_time - MesurementPreviousMillis >= measurement_interval)
 {
  MesurementPreviousMillis = current_time;
  
  stopwatch += 1;
  //Serial.print(stopwatch);
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
    curr_state = learning;
    BUTTON_FLAG_PRESSED[1] = false;
  }
//Höhenabgleich which is activated by the E button; After I pressed the E button, I can increase D and decrease C the height by changing either the sealevel Pressure or the Temperature at sealevel
  switch (abgleich)
  {
    case false:
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
    case true:
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
  float current_time = millis();
  if (current_time - MesurementPreviousMillis >= measurement_interval)
  {
    MesurementPreviousMillis = current_time;
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
  
  if (BUTTON_FLAG_PRESSED[0])
  {
    curr_state = gnss;
    stopwatch = 0;
    BUTTON_FLAG_PRESSED[0] = false;
  }
  if (BUTTON_FLAG_PRESSED[1])
  {
    curr_state = international;
    BUTTON_FLAG_PRESSED[1] = false;
  }
  
  if (BUTTON_FLAG_PRESSED[2]) 
  {
    SEALEVELPRESSURE_HPA_DWD -= 0.1;
    BUTTON_FLAG_PRESSED[2] = false;
  }
  else if (BUTTON_FLAG_HOLD[0]) SEALEVELPRESSURE_HPA_DWD -= 0.001;

  if (BUTTON_FLAG_PRESSED[3]) 
  {
    SEALEVELPRESSURE_HPA_DWD += 0.1;
    BUTTON_FLAG_PRESSED[3] = false;
  }
  else if (BUTTON_FLAG_HOLD[1]) SEALEVELPRESSURE_HPA_DWD += 0.001;
  
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

void gps()
{
  float current_time = millis();
  double x, y;
  if (current_time - MesurementPreviousMillis >= measurement_interval)
  {
    MesurementPreviousMillis = current_time;
    
    //ON/OFF OF THE BACKLIGHT
    stopwatch += 1;
    if (stopwatch >= 10) Backlight = false;
    
    lcd.clear();
    lcd.print("Fix: "); lcd.print((int)GPS.fix);
    
    if (GPS.fix) {
      lcd.setCursor(0,0);
      lcd.print("GPS");
      lcd.setCursor(0,1);
      lcd.print("L:");
      lcd.print(GPS.latitude, 1); lcd.print(GPS.lat);
      lcd.print(", ");
      lcd.print(GPS.longitude, 1); lcd.print(GPS.lon);

      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      lcd.setCursor(0,2);
      lcd.print("h="); lcd.println(GPS.altitude,1);
      lcd.print(" Sat.="); lcd.print((int)GPS.satellites);
    }
  }
  
  if (BUTTON_FLAG_PRESSED[0])
  {
    curr_state = learning;
    stopwatch = 0;
    BUTTON_FLAG_PRESSED[0] = false;
  }
  
  if (BUTTON_FLAG_PRESSED[1])
  {
    curr_state = dwd;
    BUTTON_FLAG_PRESSED[1] = false;
  }
  
  if (BUTTON_FLAG_PRESSED[4])
  {
    curr_state = international;
    BUTTON_FLAG_PRESSED[4] = false;
  }
  
  if (BUTTON_FLAG_PRESSED[3])
  {
    BUTTON_FLAG_PRESSED[3] = false;
  } 
}

/**************************************************************************/
/*    Learning-Phase    */
/**************************************************************************/

int n_of_measurements = 50;
int runs = 0;//Indicates how many times I make a measurement
bool yes = true;
float x_lon = 7, y_lat = 46;
void LEARN()
{
  float current_time = millis();
  
  if (current_time - MesurementPreviousMillis >= measurement_interval)
  {
    lcd.clear();
    MesurementPreviousMillis = current_time;
    if (GPS.fix==1)
    { 
      //International Height
      float height_I = bme.readAltitude(SEALEVELPRESSURE_HPA, T0, gradient);
      lcd.setCursor(0,1);
      lcd.print("hI=");
      lcd.print(height_I);

      lcd.setCursor(10,0);
      lcd.print("runs=");
      lcd.print(runs);
      
      //DWD Height
      float height_dwd = bme.readAltitude_DWD(SEALEVELPRESSURE_HPA_DWD*100, gradient);//My own defined function in the Adafruit BME280 Library
      lcd.setCursor(0,2);
      lcd.print("hD=");
      lcd.print(height_dwd);
      
      //GPS Height 
      lcd.setCursor(0,3);
      float height_gps = GPS.altitude;
      lcd.print("hG="); lcd.print(height_gps);
      
      //Height Model Test 
      float altitude = model.search(GPS.longitudeDegrees,GPS.latitudeDegrees);
      lcd.setCursor(0,0);
      lcd.print("hM: ");
      lcd.print(altitude);
      Serial.print("HM: ");
      Serial.println(altitude);
    }
  }

  if (BUTTON_FLAG_PRESSED[0])
  {
    curr_state = international;
    BUTTON_FLAG_PRESSED[0] = false;
  }

  if (BUTTON_FLAG_PRESSED[1])
  {
    curr_state = gnss;
    BUTTON_FLAG_PRESSED[1] = false;
  }
}


/**************************************************************************/
/*    Map-Matching-State    */
/**************************************************************************/

long counter = 0;
void MAP_MATCHING()
{
  float current_time = millis();
  double calibration_counter = 0; //Counts how many times the barometer and the gps is calibrated
  double x_coordinate,y_coordinate;//Longitude und Latitude in degrees
  double last_x_coordinate, last_y_coordinate;//last "good" coordinate from gps befor FIX==0 is reached.
  int fixquality; //Determines how good the connection is of the gps reciever and the satelites: e.g. in a tunnel = low fixquality, on a landscape = high fixquality
  float angle_gps;//Heading direction in degrees
  float headingVel_gps;//Heading velocity
  bool START = false;

  //Used sensor for estimating the position and orientation  
  unsigned long tStart = millis();  
  imu::Vector<3> orient = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  //Rotation around the Z axis which is basically pointing towards the sky
  float oZ = orient.x();
  
  //Convert degrees into radians AND change the range to [-180,180]
  if (oZ>180) oZ = (-360 + oZ)*0.0174533;
  else oZ = oZ*0.0174533;
  
  //Start Filtering
  filterY.Filter(accel.y()+corrY);
  filterZ.Filter(accel.z()+corrZ);
  
  //Access Current estimation
  float filteredY = filterY.Current();//Heading acceleration
  float filteredZ = filterZ.Current();//is used in combination with the y acceleration if the pitch angle is greater/less 10 degrees 
  
  //Zeroing the z and y acceleration if no acceleration is present
  int checkY = abs(filteredY*10);
  int checkZ = abs(filteredZ*10);
  if (!checkY||checkY==1) 
  {
    corrY = abs(filterZ.Current());
    filteredY = 0;
  }
  if (!checkZ||checkZ==1) 
  {
    corrZ = abs(filterY.Current());
    filteredZ = 0;
  }
  
  //Read orientation and velocity from imu 
  float angle_gyro_orient = orient.x();//Heading dircetion with respect to azimut 
  float angle_pitch = orient.z();//Used to determine when to take z acceleration in the calculation too; if greater than 10 or smaller than -10
  float headingVel_imu = ACCEL_VEL_TRANSITION*filteredY/cos(DEG_2_RAD*orient.x())-corrHeadingVel_imu;//Calculating the heading velocity based on measurements of the imu sensor
  
  //bias of the headingVel_imu
  if ((int)headingVel_imu*10==0) corrHeadingVel_imu = headingVel_imu;
  
  if (current_time - MesurementPreviousMillis >= measurement_interval)
  {
    lcd.clear();
    MesurementPreviousMillis = current_time;
    
    //ON/OFF OF THE BACKLIGHT
    stopwatch += 1;
    if (stopwatch >= 10) Backlight = false;

    //If we have a GPS Fix we read the speed in knots, convert it to m/s and Filter the random process noise of it with a so called recursive filter
    if (GPS.fix==1) 
    {
      latitude = mm.begin(latitude, longitude, true);
      longitude = mm.begin(latitude, longitude, false);
      
      x_coordinate = mm.begin(GPS.latitudeDegrees,GPS.longitudeDegrees,false);
      y_coordinate = mm.begin(GPS.latitudeDegrees,GPS.longitudeDegrees,true);
      
      lcd.setCursor(0,0);
      lcd.print("GPS");
      lcd.setCursor(4,0);
      lcd.print(GPS.fixquality);
      lcd.setCursor(0,2);
      lcd.print("P="); lcd.print(GPS.latitudeDegrees,5); lcd.print(GPS.lat);
      lcd.print(","); lcd.print(GPS.longitudeDegrees,5); lcd.print(GPS.lon);

      //GPS velocity, heading direction and fixquality
      GPSVEL.Filter((GPS.speed-corrHeadingVel_gps)*1.852/3.6); //vel_knots*1.852/3.6=velocity in m/s
      headingVel_gps = GPSVEL.Current(); //Estimation of velocity
      if ((int)headingVel_gps*10==0) corrHeadingVel_gps = GPS.speed;//If no movement is done store the bias 
      angle_gps = GPS.angle; //Heading direction with respect to azimut
      fixquality = GPS.fixquality;//0 = no fix, 1 = GPS and 2 = DGPS
    }else
    {
      fixquality = 0;//So that it is really giving 0, because without this it could be also the case that an other number gets printed.
    }
    
    //Storing the last valid gps coordinates
    last_x_coordinate = x_coordinate;
    last_y_coordinate = y_coordinate;
    
    //Get calibration status
    if (mm.distance() <= mm.cali_dist)
    {
      //Calibrate barometer if a fix point is reached
      sealevel = bme.seaLevelForAltitude(mm.calibrate_BARO(), bme.readPressure()/100);
      sealevelDWD = bme.seaLevelForAltitudeDWD(mm.calibrate_BARO(), bme.readPressure())/100;
      xPos = last_x_coordinate;
      yPos = last_y_coordinate;
      calibration_counter = 1;
    }
    
    //If this is false the Map-Matching System will start.
    if (mm.RESET)
    {
      lcd.setCursor(4,0);
      lcd.print("START");
    }else
    {
      lcd.setCursor(6,0);
      lcd.print("CCounter: ");
      lcd.print(calibration_counter);
      
      //Calculating the xPos and yPos in degrees
      //angle_gps and angle_gyro are used in a complementary filter to estimate heading direction
      //xPos, yPos, xVel and yVel are global variables
      longitude = xPosition(filteredY, filteredZ, headingVel_gps, headingVel_imu, last_x_coordinate, angle_gps, angle_gyro_orient, angle_pitch, fixquality);//We use here the y acceleration, because it is the axis which is always pointing into the direction of movement
      latitude = yPosition(filteredY, filteredZ, headingVel_gps, headingVel_imu, last_y_coordinate, angle_gps, angle_gyro_orient, angle_pitch, fixquality);

      if (fixquality==1)
      {
        //Logging data latitude, longitude and est_error
        logFile.print(latitude,10);
        logFile.print(",");
        logFile.print(longitude,10);
        logFile.print(",");
        logFile.print(est_error);
        logFile.println(",");
        logFile.flush();
      }
    }
   
    lcd.setCursor(0,3);
    lcd.print(bme.readAltitude(sealevel, T0, gradient));
    lcd.print(", ");
    lcd.print(bme.readAltitude_DWD(sealevelDWD*100, gradient)); 
    float ave = (bme.readAltitude(SEALEVELPRESSURE_HPA, T0, gradient) + bme.readAltitude_DWD(SEALEVELPRESSURE_HPA_DWD*100, gradient))/2;
    float ave1 = (bme.readAltitude(sealevel, T0, gradient) + bme.readAltitude_DWD(sealevelDWD*100, gradient))/2;
    
    //Print the estimated Position
    lcd.setCursor(0,1);
    lcd.print("M:");
    lcd.print(latitude, 5); lcd.print(GPS.lat);
    lcd.print(",");
    lcd.print(longitude, 5); lcd.print(GPS.lon);

    //Logging data latitude, longitude and est_error
    logFile.print(latitude,10);
    logFile.print(",");
    logFile.print(longitude,10);
    logFile.print(",");
    logFile.print(mm.distance());
    logFile.println(",");
    logFile.flush();
  }

  if (BUTTON_FLAG_PRESSED[2])//automatic Map_Matching: The reference points are selceted by the program; Button C
  {
    mm.RESET = !mm.RESET;
    BUTTON_FLAG_PRESSED[2] = false;
  }
  
  if (BUTTON_FLAG_PRESSED[4])
  {
    curr_state = international;//Button F
    BUTTON_FLAG_PRESSED[4] = false;
  }
}
