//LIBRARIES
/*My libraries*/
#include "HEIGHT_DATA.c"
#include "KalmanFilter.h"
#include "MapMatching.h"

/*BNO055 Accelerometer*/
#include <Wire.h> 
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/*Adafruit GPS*/
#include <Adafruit_GPS.h>
#include <SD.h>

/*BME280 Sensor; Barometer*/
#include <Adafruit_BME280.h>

/*LCD Display*/
#include <LiquidCrystal_I2C.h>

/*Keypad*/
#include <Keypad.h>

//Accelerometer: Measurement of g vector
/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
Adafruit_BNO055 bno = Adafruit_BNO055(55);

//Map-Matching
MapMatching mm; 

//Kalman Filter 
float g;
uint32_t t;
KalmanFilter kf(602,10000,10,818);
float lon_in_meters, lat_in_meters;//Initialization coordinates

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
typedef enum {international=0, dwd, gnss, map_matching, reference_height, learning} State_type; //This enumeration contains all the main states
void internationaleFormel(), DWD(), gps(), MAP_MATCHING(), RHEIGHT(), LEARN();
void (*state_table[])() = {internationaleFormel, DWD, gps, MAP_MATCHING, RHEIGHT, LEARN}; //This table conatins a pointer to the function to call in each state
State_type curr_state;
State_type prev_state;

volatile bool BUTTON_FLAG_PRESSED[6] = {false,false,false,false,false,false};//Sets a flag if a specific button is pressed
volatile bool BUTTON_FLAG_HOLD[2] = {false,false};//At the moment unused
volatile bool abgleich = true;//Is used to make a Höhenabgleich by changing the normal pressure and temperature
                              //If it's true then the temperature is changed, if false then the pressure
                              
/****************************************************************************************************/
/*    SETUP    */
/****************************************************************************************************/

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.begin(57600);
  while (!Serial){return;}
  
  //Initialization of the longitude array and the height array
  int id;
  for(id=0; id<MAX; id++)
  {
      lonStore[id] = 0;
      hStore[id] = 0;
  }
  
  //Initialization of the StateMachine
  InitializeSM();
}

/****************************************************************************************************/
/*    State Machine Initialization    */
/****************************************************************************************************/

void InitializeSM()
{
//User's starting point 
  lat_in_meters = 46.90972222*3.14/180*6371000;
  lon_in_meters = 7.35805556*3.14/180*6371000; 
  
//SD Card Datalogger
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
  
  lcd.setCursor(5,1);
  lcd.print("I'm ready!");//System is ready for take off!!!
  delay(2000);
//KEYPAD
  keypad.begin(makeKeymap(keys));//Initialize the keypad object 

  keypad.addEventListener(keypadEvent);//This EventListener becomes active if any key is pressed
}

/****************************************************************************************************/
/*    LOOP    */
/****************************************************************************************************/

uint32_t timer = millis();
uint8_t stopwatch = 0;
void loop() 
{
  t = millis();
  // put your main code here, to run repeatedly:
  uint8_t key = keypad.getKey();//Without this the EventListener wouldn't work 
  if (key) Serial.print(key);//Just to know which key I pressed => acutally useless 
  
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

/****************************************************************************************************/
/*    KeypadEventListener    */
/****************************************************************************************************/

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

/****************************************************************************************************/
/*    International Height Formulae    */
/****************************************************************************************************/

void internationaleFormel()
{ 
 float current_time = millis();
 
 if (current_time - MesurementPreviousMillis >= measurement_interval)
 {
  MesurementPreviousMillis = current_time;
  
  stopwatch += 1;
  Serial.print(stopwatch);
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

/****************************************************************************************************/
/*    DWD    */
/****************************************************************************************************/

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

/****************************************************************************************************/
/*    GPS    */
/****************************************************************************************************/

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

      //Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      //Serial.print("Angle: "); Serial.println(GPS.angle);
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
    curr_state = reference_height;
    BUTTON_FLAG_PRESSED[4] = false;
  }
  
  if (BUTTON_FLAG_PRESSED[3])
  {
    BUTTON_FLAG_PRESSED[3] = false;
  } 
}

/****************************************************************************************************/
/*    Learning-Phase    */
/****************************************************************************************************/

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
      //Map-Matching Height 
      float lon = GPS.longitude;
      float lat = GPS.latitude;
      uint16_t height_map_matching = searchPrecise(lat, lon, true);//Height estimator: Mathematical Model

      float v_gps = GPS.speed;//This velocity is in knotes/s
      float v = (v_gps * 1.852)/3.6;
      float x_y = v*(t/1000);
      float x_lon = (lon_in_meters + x_y)*180/(6371000*3.14);
      float y_lat = (lat_in_meters + x_y)*180/(6371000*3.14);
      uint16_t estimated_height = searchPrecise(y_lat, x_lon, false);
      
      lcd.setCursor(0,0);
      lcd.print("hM=");
      lcd.print(height_map_matching);//Estimated height
      
      lcd.setCursor(10,1);
      lcd.print("x=");
      lcd.print(x_lon);

      lcd.setCursor(11,2);
      lcd.print("y=");
      lcd.print(y_lat);

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
      
      if (n_of_measurements < 40)
      {
        logFile.print(x_lon);
        logFile.print(",");
        logFile.print(y_lat);
        logFile.print(",");
        logFile.print(lon);
        logFile.print(",");
        logFile.print(lat);
        logFile.print(",");
        logFile.print(v);
        logFile.print(",");
        logFile.print(estimated_height);
        logFile.print(",");
        logFile.print(height_map_matching);
        logFile.print(",");
        logFile.print(height_I);
        logFile.print(",");
        logFile.print(height_gps);
        logFile.print(",");
        logFile.print(height_dwd);
        logFile.println(",");
        
        logFile.flush();

        n_of_measurements += 1;
      }else
      {
        lcd.setCursor(11,3);
        lcd.print("END");
      }

      float moyen = (height_gps + height_I + height_dwd)/3;
      float model_height = 586 + 0.5*(t/1000) - 0.5*g*pow(t/1000,2);
      float kf_height = kf.update(moyen);
      float kf_estimation_error = kf.predict(model_height);

      Serial.print("KF_HEIGHT: ");
      Serial.println(kf_height);
      Serial.print("kf_estimation_error: ");
      Serial.println(kf_estimation_error);
      
      /*
      //Starting Kalman Filtering 
      float y = (GPS.altitude + height_dwd + height_I + hoehe)/4; //Durch Gewichtung besser Ouputs?
      Serial.print(y);
      float err_measure = (pow(GPS.altitude - y,2) + pow(height_dwd - y,2) + pow(height_I - y,2) + pow(hoehe - y,2))/4;
      float height_kf = (KF.updateEstimate(hoehe) + 2*hoehe)/3;
      float height_kf1 = KALMAN_FILTERING(y, err_measure, 1000, 0.001, height_kf);
      
      //Serial.print("height_kf1");
      Serial.println(height_kf1);
      Serial.print(" ");
      
      
      //Serial.print("KF height: ");
      //Serial.println(height_kf);
      //Serial.print(" ");
      //Serial.println(589);
      */
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

  if (BUTTON_FLAG_PRESSED[4])
  {
    n_of_measurements = 0;
    runs += 1;//Indicates how many times I make a measurement
    logFile.println();
    logFile.print(runs);
    logFile.print(": ");
    logFile.print("x_lon");
    logFile.print(",");
    logFile.print("x_lat");
    logFile.print(",");
    logFile.print("lon");
    logFile.print(",");
    logFile.print("lat");
    logFile.print(",");
    logFile.print("v");
    logFile.print(",");
    logFile.print("estimated_height");
    logFile.print(",");
    logFile.print("height_map_matching");
    logFile.print(",");
    logFile.print("height_I");
    logFile.print(",");
    logFile.print("height_gps");
    logFile.print(",");
    logFile.print("height_dwd");
    logFile.println(",");

    BUTTON_FLAG_PRESSED[4] = false;
  }
  
  switch (abgleich)
  {
    case false:
      if (BUTTON_FLAG_PRESSED[2]) 
      {
        y_lat -= 0.01;
        BUTTON_FLAG_PRESSED[2] = false;
      }
      else if (BUTTON_FLAG_HOLD[0]) y_lat -= 0.001;

      if (BUTTON_FLAG_PRESSED[3]) 
      {
        y_lat += 0.01;
        BUTTON_FLAG_PRESSED[3] = false;
      }
      else if (BUTTON_FLAG_HOLD[1]) y_lat += 0.001;
      
    break;
    case true:
      if (BUTTON_FLAG_PRESSED[2]) 
      {
        x_lon -= 0.01;
        BUTTON_FLAG_PRESSED[2] = false;
      }
      else if (BUTTON_FLAG_HOLD[0]) x_lon -= 0.001;

      if (BUTTON_FLAG_PRESSED[3]) 
      {
        x_lon += 0.01;
        BUTTON_FLAG_PRESSED[3] = false;
      }
      else if (BUTTON_FLAG_HOLD[1]) x_lon += 0.001;
    break; 
  }
}

/****************************************************************************************************/
/*    Kalman-Filtering    */
/****************************************************************************************************/

/*float KALMAN_FILTERING(float mea)
{ 
  float kalman_gain = err_estimate/(err_estimate+err_measure);
  float current_estimate = last_estimate + kalman_gain*(mea-last_estimate);
  err_estimate = (1.0 - kalman_gain)*err_estimate + fabs(last_estimate - current_estimate)*q;
  last_estimate = current_estimate;
  return current_estimate;
}*/
float P1;
float KALMAN_FILTERING(float y, float R, float P0, float q, float X0)
{
  if (yes) P1 = P0 + q;
  float K = P1/(P1+R);
  float X1 = X0 + K*(y-X0);
  P1 = P1 + q;
  X0 = X1;
  return X1;
}

/****************************************************************************************************/
/*    Map-Matching-State    */
/****************************************************************************************************/

//Basic structure of Map-Matching state
void MAP_MATCHING()
{
  float current_time = millis();
  double x_coordinate,y_coordinate;//Latitude und Longitude
  double new_x, new_y;
  bool START = false;
  if (current_time - MesurementPreviousMillis >= measurement_interval)
  {
    lcd.clear();
    MesurementPreviousMillis = current_time;
    
    //ON/OFF OF THE BACKLIGHT
    stopwatch += 1;
    if (stopwatch >= 10) Backlight = false;

    if (GPS.fix) 
    {
      x_coordinate = mm.begin(GPS.latitude, GPS.longitude, true);
      y_coordinate = mm.begin(GPS.latitude, GPS.longitude, false);
      new_x = (GPS.latitude-4600)/60 + 46;
      new_y = (GPS.longitude-700)/60 + 7;
      lcd.setCursor(0,0);
      lcd.print("GPS");
      lcd.setCursor(0,2);
      lcd.print("l="); lcd.print(new_x,5); lcd.print(GPS.lat);
      lcd.print(", "); lcd.print(new_y,5); lcd.print(GPS.lon);
    }
    if (mm.RESET)
    {
      lcd.setCursor(4,0);
      lcd.print("START");
    }
    lcd.setCursor(0,1);
    lcd.print("L:");
    lcd.print(x_coordinate, 5); lcd.print(GPS.lat);
    lcd.print(",");
    lcd.print(y_coordinate, 5); lcd.print(GPS.lon);
    lcd.setCursor(0,3);
    
    if (mm.dist <= mm.cali_dist)
    {
      sealevel = bme.seaLevelForAltitude(mm.calibrate_BARO(), bme.readPressure()/100);
      sealevelDWD = bme.seaLevelForAltitudeDWD(mm.calibrate_BARO(), bme.readPressure())/100; 
    }
    
    lcd.print(bme.readAltitude(sealevel, T0, gradient));
    lcd.print(", ");
    lcd.print(bme.readAltitude_DWD(sealevelDWD*100, gradient)); 
    float ave = (bme.readAltitude(SEALEVELPRESSURE_HPA, T0, gradient) + bme.readAltitude_DWD(SEALEVELPRESSURE_HPA_DWD*100, gradient))/2;
    float ave1 = (bme.readAltitude(sealevel, T0, gradient) + bme.readAltitude_DWD(sealevelDWD*100, gradient))/2;
    logFile.print(ave);
    logFile.print(",");
    logFile.print(ave1);
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

/****************************************************************************************************/
/*    Select REFERENZ-POINT    */
/****************************************************************************************************/

void RHEIGHT()//Search a reference height in the data file 
{
  float current_time = millis();
  
  if (current_time - MesurementPreviousMillis >= measurement_interval)
  {
    lcd.clear();
    MesurementPreviousMillis = current_time;
    if (GPS.fix==1)
    {
      lcd.setCursor(0,0);
      lcd.print(GPS.longitude);
      lcd.print(" ");
      lcd.print(GPS.latitude);
      
      float lon = GPS.longitude;
      float lat = GPS.latitude;
      uint16_t hoehe = searchPrecise(lat, lon, true);
      lcd.setCursor(0,1);
      lcd.print("h=");
      lcd.print(hoehe);

      float temp = bme.readTemperature();
      float normPressure = bme.readPressureOnHeight(SEALEVELPRESSURE_HPA_RP, hoehe, gradient, T0+273.15);
      //float newGradient = (15 - temp)/hoehe;
      //Serial.println(newGradient);
      float delta_height = bme.readAltitude(normPressure,15,0.0065);
      
      lcd.setCursor(0,2);
      lcd.print("dh=");
      lcd.print(delta_height);
      
      
      float actual_height = hoehe + delta_height;
      Serial.println(actual_height);
      lcd.setCursor(0,3);
      lcd.print("H=");
      lcd.print(actual_height);
    }
  }

  if (BUTTON_FLAG_PRESSED[4])
  {
    curr_state = gnss;
    BUTTON_FLAG_PRESSED[4] = false;
  }
}
