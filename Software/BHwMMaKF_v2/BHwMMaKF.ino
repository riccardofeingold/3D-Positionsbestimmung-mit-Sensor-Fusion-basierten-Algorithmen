//LIBRARIES
#include "HEIGHT_DATA.c"
#include <SimpleKalmanFilter.h>
#include <Adafruit_GPS.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>

//GPS stuff => important variables and constants
#define mySerial Serial2 //Serial2 are the pins 16(TX2) and 17(RX2)
Adafruit_GPS GPS(&mySerial);//Instantiate the GPS object
#define GPSECHO false

//SENSOR
float SEALEVELPRESSURE_HPA = 1013.25;//For the international formulae
float SEALEVELPRESSURE_HPA_DWD = 1013.25;//For the DWD formulae
float SEALEVELPRESSURE_HPA_RP = 1013.25;//For the reference-point-calculation
float T0 = 15;//Temperature at sea level
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
typedef enum {international=0, dwd, gnss, map_matching, reference_height} State_type; //This enumeration contains all the main states
void (*state_table[])() = {internationaleFormel, DWD, gps, MAP_MATCHING, RHEIGHT}; //This table conatins a pointer to the function to call in each state
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
  while (!Serial){;}
  
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
    if (key=='E') //Activates C and D Buttons
    {
      BUTTON_FLAG_PRESSED[5] = true;
      abgleich = !abgleich;
    }
    break;
    
    case HOLD:
    if (key=='C') BUTTON_FLAG_HOLD[0] = !BUTTON_FLAG_HOLD[0];//Decrease
    if (key=='D') BUTTON_FLAG_HOLD[1] = !BUTTON_FLAG_HOLD[1];//Increase
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
    curr_state = gnss;
    BUTTON_FLAG_PRESSED[1] = false;
  }
//Höhenabgleich which is activated by the E button; After I pressed the E button, I can increase D and decrease C the height by changing either the sealevel Pressure or the Temperature at sealevel
  if (BUTTON_FLAG_PRESSED[5])
  {
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
  }
  
  if (BUTTON_FLAG_PRESSED[4])
  {
    prev_state = curr_state;
    curr_state = map_matching;
    BUTTON_FLAG_PRESSED[4] = false;
    BUTTON_FLAG_PRESSED[3] = false;
    lcd.clear();
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
    lcd.clear();
  }
}

/****************************************************************************************************/
/*    GPS    */
/****************************************************************************************************/

void gps()
{
  float current_time = millis();
  if (current_time - MesurementPreviousMillis >= measurement_interval)
  {
    MesurementPreviousMillis = current_time;

    //ON/OFF OF THE BACKLIGHT
    stopwatch += 1;
    if (stopwatch >= 10) Backlight = false;
    
    lcd.clear();
    // approximately every 2 seconds or so, print out the current stats
    if (millis() - timer > 500) {
      timer = millis(); // reset the timer
      
      lcd.print("Fix: "); lcd.print((int)GPS.fix);
      
      if (GPS.fix == 1) {
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
  }
  
  if (BUTTON_FLAG_PRESSED[0])
  {
    curr_state = international;
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
/*    Map-Matching-State    */
/****************************************************************************************************/

//Basic structure of Map-Matching state; Two Modes: Auto refrence point selection, Manual reference point selection
void MAP_MATCHING()
{
  if (prev_state==international)
  {
    //lcd.print("MAP_MATCHING");
    if (BUTTON_FLAG_PRESSED[1])//automatic Map_Matching: The reference points are selceted by the program; Button A
    {
      lcd.clear();
      lcd.print("auto");
    }
    if (BUTTON_FLAG_PRESSED[0])//manual Map_Matching: You can choose the reference points; Button B
    {
      lcd.clear();
      lcd.print("manual");
    }
    if (BUTTON_FLAG_PRESSED[4]) curr_state = international;//Button F
    BUTTON_FLAG_PRESSED[3] = false; //Reset button state of E
    BUTTON_FLAG_PRESSED[4] = false; //Reset button state of F
    BUTTON_FLAG_PRESSED[0] = false; //Reset button state of B
    BUTTON_FLAG_PRESSED[1] = false; //Reset button state of A
  }
  if (prev_state==dwd)
  {
    if (BUTTON_FLAG_PRESSED[1])//automatic Map_Matching: The reference points are selceted by the program
    {
      lcd.clear();
      lcd.print("auto");
    }
    if (BUTTON_FLAG_PRESSED[0])//manual Map_Matching: You can choose the reference points
    {
      lcd.clear();
      lcd.print("manual");
    }
    if (BUTTON_FLAG_PRESSED[4]) curr_state = dwd;
    stopwatch = 0;
    BUTTON_FLAG_PRESSED[3] = false;//Reset button state of E
    BUTTON_FLAG_PRESSED[4] = false; //Reset button state of F
    BUTTON_FLAG_PRESSED[0] = false; //Reset button state of B
    BUTTON_FLAG_PRESSED[1] = false; //Reset button state of A
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
      uint16_t hoehe = searchPrecise(lat, lon);
      lcd.setCursor(0,1);
      lcd.print("h=");
      lcd.print(hoehe);
      float normPressure = bme.readPressureOnHeight(SEALEVELPRESSURE_HPA_RP, hoehe, gradient, T0+273.15);
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
