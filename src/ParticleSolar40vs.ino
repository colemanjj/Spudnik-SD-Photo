/* A project to monitor water quality in remote locations
  Uses the Particle Electron powered by a solar panel and the LiPo battery that comes with the Electron.
  parameters include:
  Water   Temperature                       --from BME280 in oil for depth
          Specific conductance              --Gravity: Analog TDS Sensor/Meter
          Water Depth                       --a BME280 in mineral oil measuring pressure
  Air     Temperature, Humidity, Pressure   --BME280
  Rain	  Index of intensity                --generic Rain / Water Level Sensor
  Battery Percent charge                    --internal to Electron
          Charge voltage                    --internal to Electron
  Cell	  Signal strength                   --internal to Electron

  Parameter values are sent to:
    Ubidots for storage and plotting
    Particle for tracking of unit activity

  Pin IDs and code are included for an analog vented depth sensor and a BME280 based depth sensor but the
  analog vented sensor is not implemented in Spudnik-09 and later.
  The code will run with the depth sensors missing. If a sensor reading fails, dummy results are reported
  to Ubidots and/or particle.

  In general the code is written to, hopefully, not "hang" if a sensor is missing or fails.
    This code has been run on a "bare" electron with only a battery and antenna and "FuelGauge" values
    are reported to the Particle Console and dummy values are reported to Ubidots sucessfully.

  Delays and Particle.process() are implemented after upload of the data so that there is time for
  OTA software updates. In any case OTA updates seem to be sensitive to timing.  The delay for OTA upload
  requires SYSTEM_THREAD(ENABLED) otherwise the code gets stuck while checking for Cellular.connect();

  Frequency of sensor reading and data reporting is dependent on the battery charge.  Frequency decreases
  as the battery charge decreases.  If charge (SOC) is below 20% the Particle blinks the blue LED slowly then
  goes to sleep for 9 hours, during which it is hoped that it will get some sun to recharge.

  If cell signal is weak and the Particle can not connect for 2 minutes it flashed blue LED, writes a message
  to serial, tries 1 more minute and then goes back to sleep.

*/
//#define ver = "1.1.0-alpha.1"
// @ts-check
// for logging to SD------------
#include <SPI.h>        //**
#include <SdFat.h>      //**
// SD chip select pin.  Be sure to disable any other SPI devices such as Enet.
const uint8_t chipSelect = SS;  //**
// create filename as a global variable for use in several functions
String fileName ;       //**
// File system object.
SdFat sd;               //**
// identify a Logging file.
SdFile file;            //**
// Error messages stored in flash.
#define error(msg) sd.errorHalt(F(msg))

ArduinoOutStream cout(Serial);
// for camera-------------------------------
#include "camera_VC0706.h"
//camera_VC0706 cam(&Serial1);
///camera_VC0706 cam = camera_VC0706(&Serial1);
//-------------------------------
#include "sensitive_definitions.h"  // this file contains ubidots token definition
                               // e.g.  #define your_token ""  // Put here your Ubidots TOKEN
// for the temp. humidity, pressure BME280 sensor
#include <CE_BME280.h>
// Create two BME280 instances
CE_BME280 bme1; // I2C  for air temp. pressure, humidity
CE_BME280 bme2; // I2C   for WATER temp. & pressure
//BME280_I2C bme1(0x76); // I2C using address 0x76
//BME280_I2C bme2(0x77); // I2C using address 0x77

//---#include <Adafruit_DHT_Particle.h>  // air and humidity sensor.   includes "isnan()" function
//---#include <math.h>
#include <Ubidots.h>   // using here Ubidots=3.1.4
//SYSTEM_MODE(AUTOMATIC); 
SYSTEM_MODE(SEMI_AUTOMATIC);   // was set at semi_automatic but I could not flash remotely, tried automatic then set back to semi-automatic
//The only main difference between SEMI_AUTOMATIC mode and AUTOMATIC mode is that for semi-automatic
    // Particle.connect() is not called at the beginning of your code;  With semi-automatic you must call Particle.connect() yourself
///SYSTEM_THREAD(ENABLED);       // seems to make the checking for connection routine work, keep an eye on this ****
//#define your_token "xyz..."  // for privacy, the Ubidots token is defined in the included .h file as "your_token"
#define DATA_SOURCE_NAME "Spudnik-08b"
#define unit_name "Spudnik-08"
//String unit_name = "Spudnik-08b";
#define code_name "particlesolar40vs"

//SerialLogHandler logHandler;

ApplicationWatchdog *wd;

Ubidots ubidots(your_token, UBI_TCP); // A data source with particle name will be created in your Ubidots account
                    // your_token is defined in sensitive_definitions.h

//*****************************************************
//*****************************************************
  #define t1_offset  -0.7 // correction offset for the AIR tmp. sensor  Set with thermometer before deployment
  #define t2_offset  0.05 // correction offset for the H2O tmp. sensor  Set with thermometer before deployment
  #define k  1.25  // ***** K is a crude calibration factor that can be used to tune the Sp.C. readings
//*****************************************************
  char publishStr[150]; /// was 30,  for publishing strings of info
  char works[5];  /// was 4,  for reporting if SD-card setup worked
  char _json[150];  // for reporting data to SD-card, and Particle
//initialize for BME280 bme2 AIR temp, humidity, pressure readings
  float t1 = -99.9;
  float h1 = -99.9;
  float p1 = -99.9;
//initialize for BME280  bme2 WATER temp, pressure readings
  float t2 = -99.9;
  float p2 = -99.9;
  float depth = -99.9;
// initialize variable for reading sensors
  float Sp_C;     // Specific Conductance returned from TDS sensor routine
  float Avolts;  //raw voltage returned from TDS sensor
  float rain;     //
// initialize battery variables
  float volts;  // battery voltage
  float SoC;    // battery charge
//initialize timing variables
  int sleepInterval = 60;  // This is used below for sleep times and is equal to 60 seconds of time.
  int a_minute = 60000; // define a minute as 60000 milli-seconds
  int minutes = 60;  // default minutes to sleep
  int seconds = 5;
/*
D0 = SDA for temp/humid/pressure sensor (BME280)
D1 = SCL for temp/humid/pressure sensor (BME280)
// D2 =
D3 = power for digital sensors (BME280, )
// D4 = water temperature sensor (ds18b20 )
// D5 = power for 1st analog sensor (  )
// D6 = power for 2nd analog sensor (TDS)  not needed, TDS runs off of B1
D7 = ledPin  to flash LED

A0 = analog pin for TDS sensor
A1 = analog pin for rain sensor

A2 = analog pin for SPI (SS/SPI)   connected to mini-SD-card  CS
A3 = analog pin for SPI (SCK)   connected to mini-SD-card  SCK
A4 = analog pin for SPI (MISO)  connected to mini-SD-card  D0
A5 = analog pin for SPI (MOSI)  connected to mini-SD-card  D1

B0 = used as digital power pin to supply 3.3 volts to RAIN analog sensor
B1 = used as digital power pin to supply 3.3 volts to TDS analog sensor
B2 = used as a digital signal pin to switch a NpN transister to turn on/off ground 
        for SD-card and camera
*/
//int DepthPin = A0;  //unused analog depth sensor
int RainPin = A1;
int SpCSensorPin  = A0;
int ledPin = D7;         // LED connected to D7

  //ApplicationWatchdog wDog(90000, watchdogHandler, 512);
int usbOn = 0;

// ---------SETUP------------
void setup() 
{
  Serial.begin(9600);
  // waitSec(0.5);
   Serial.println();
   waitSec(1.0);
  // reset the system after 10 min if the application is stuck.  set as an escape from some hangup.
  // watchDog is petted after cell connection estsblished
  wd = new ApplicationWatchdog(10min, watchdogHandler, 1536);
  // set date time callback function. Do in setup() or loop()?  used to write file datetime to SD-card
  SdFile::dateTimeCallback(dateTime);
  Time.zone(-6);  // setup to CST time zone, which is part of the ISO8601 format        //**

//  Do I need to set up D0 and D1 in some way for the BME280s   ????????????
  pinMode(ledPin, OUTPUT);          // Sets pin as output
// create power on digital pins for stable power supply AND so that sensors are shut down
  // when processor is shut down
	pinMode(D3, OUTPUT);     // power for the all digital sensors
	digitalWrite(D3, HIGH);

	pinMode(D5, OUTPUT);     // power for SD-card and camera
	digitalWrite(D5, HIGH);

	pinMode(B0, OUTPUT);     // power for analog rain sensor
	digitalWrite(B0, HIGH);	//
  pinMode(B1, OUTPUT);     // power for analog TDS sensor
  digitalWrite(B1, HIGH);	//
  pinMode(B2, OUTPUT);     // set high to trigger 3V3 grounding for camera, set high later
  digitalWrite(B2, LOW);	//

  waitSec(0.1); // delay to give time for power to turn on, don't know if this is needed   

  customPower();
      // {  //turn off battery charging
      //   PMIC _pmic; // instantiate an object
      //   _pmic.disableCharging();
      // }
 // setup two BME280s
    if (!bme1.begin(0x77)) // the air sensor BME280 for temp, humidity, pressure
                  //  with SD0 held high by wire to 3.3 V. see HiLetGo_BME280.txt   check which bme has SD0 held high
    {
///      Serial.println("Could not find 1st valid BME280 sensor, check wiring!");
      Particle.publish("ObiDots", "could not find bme1",60,PRIVATE);
        //  while (1);  // original code had this but seems like an endless loop if the BME is not detected.
      }
    if (!bme2.begin(0x76))  // the water depth sensor in oil made from a BME280. Temp and pressure
    {
///        Serial.println("Could not find 2nd valid BME280 sensor, check wiring!");
        Particle.publish("ObiDots", "could not find bme1",60,PRIVATE);
      }

// register a Particle cloud Function.  "Delay" is used from the Particle console to set the delay 
     // to "long", i.e. delay of 180 seconds for OTA software uploads. otherwise delay defaults to 2 seconds.
  Particle.function("Delay", delayTime);

  usbOn = Serial.isConnected();
  if(usbOn)  {Serial.println("ending setup"); waitMS(100);}

} // end setup()

//-----------LOOP
void loop() 
{  
  if(usbOn) {Serial.println("in loop"); waitSec(0.1);}
  FuelGauge fuel; // Initalize the Fuel Gauge so we can call the fuel gauge functions below.
  //--- get battery info
  waitSec(0.5);
  // fuel.quickStart();
  waitSec(1);
  volts = fuel.getVCell();
  SoC = fuel.getSoC();
  if(usbOn)
    { 
      //Serial.println(fuel.getVersion());
    //  Serial.printlnf("quickstart_success= %d i.e. %s", fuel.quickStart(), (fuel.quickStart()?"false":"true"));

      Serial.printlnf("SoC=%6.2f,  volts=%6.2f,  volts/4.4=%5.2f" , SoC,volts,(volts/4.4));
      Serial.printlnf("difference= %5.2f%%", 100*abs(1-(100*volts/4.304)/SoC));
      Serial.printlnf("mapped volts= %5.2f%%", map((double)volts,3.5,4.304,0.0,100.0));
      
      //SoC = System.batteryCharge();    
      waitSec(0.2);
      showPMIC();
    }
  
//  set the deep sleep time based on the battery charge
 minutes = checkBattery(SoC,volts);

// setup the SD for logging the data
  setup_SD();

  {    // read sensors
  rain = analogRead(RainPin);
  digitalWrite(B0, LOW);     //turn off power to the rain sensor, otherwise it interfears
                             // with the next analog sensor (i.e. TDS/Sp.C)
  delay(200);

    {   // read BME sensors
      // ---- get AIR temperature and humidity and pressure  // from BME280 using I2C connection
        int i = 0;
        while(i<2)  // read 2 times to be sure of a good read
             {
               t1 = bme1.readTemperature();
               h1 = bme1.readHumidity();
               p1 = bme1.readPressure()/100.0;
               delay(200);
               i++;
             }
          if (isnan(p1) || p1<1.0 )
            { h1 = -99.1; t1 = -99.1; p1 = -99.1;  }   // -99.1 is flag for bme read error
      // ---- get WATER temperature and pressure // from the BME280 using I2C connection. 
          // being used underwater (enclosed in mineral oil) for depth sensor
        i = 0;
        while(i<2)  // read 2 times to be sure of a good read
             {
              t2 = bme2.readTemperature();
              p2 = bme2.readPressure()/100.0;
              delay(200);
              i++;
            }
          // Check if any reads failed but don't hold things up
       	    if (isnan(p1) || p2<1.0 )
              {  t2 = -99.1; p2 = -99.1;  }    // -99.1 is flag for bme read error
        t1 = t1+t1_offset;
        t2 = t2+t2_offset;
    }
    /////////////   check for too low or too high temperature  //////////////////////
    if (t1 < -15 || t1 > 40) 
      {
        PMIC _pmic; // instantiate an object
        _pmic.disableCharging();  //stops charging which carries on into sleep
                                  // but every time unit starts up charging is 
                                  // re-started and this check is done again
      }
  // ---- get WATER calculated Specific Conductance and median voltage on sensor
    Sp_C = getSpC() * k;
    Avolts = getAvolts();
  // turn off sensor POWER pins after sensors are read
    digitalWrite(D3, LOW);	 // for the digital sensors, BME280s  and camera
   //  digitalWrite(D6, LOW);	// not needed because all digital sensors run off D3
    ///digitalWrite(B0, LOW);	// for the rain sensor
    digitalWrite(B1, LOW);     //for the TDS-Sp.C sensor  
  //        digitalWrite(D5, LOW);	 // for the  camera
        
  ///char context[90];
  //sprintf(context, "tries=%02i", ii);
  // add values to que of data to be uploaded to Ubidots
  ///	ubidots.add("time(UTC)",Time.now()/60);
  }

{   //write to ubidots
//char Rain = "Rain";
  char Rain[] = "Rain";
  char AirT[] = "Air-Temp_C";
  char Humid[] = "Humidity_%";
  depth = (p2-p1)*0.40147;  // Hectopascals (hPa) to	Inches Of Water (inH2O)*

  ubidots.add(Rain, rain);
	ubidots.add(Humid, h1);
	ubidots.add(AirT, t1);
  ubidots.add("Pressure_hPA", p1);
  ubidots.add("H2O-Temp_C", t2);
  ubidots.add("H2O_hPA", p2);
  ubidots.add("Depth_in", depth);
  ubidots.add("Volts", volts);
  ubidots.add("SOC", SoC);
   //if (t2 > -99.0)   // if reading water temperature was successful, send temp and Sp_Cond to Ubidots
  ubidots.add("Sp_Cond", Sp_C);
  ubidots.add("A.volts", Avolts);
}
//------------------ log data and take photo-------------------------------------------------

// digitalWrite(B2, HIGH);	//   turn on ground for the SD-card
//  write the data to a SD card before trying to connect
  snprintf(_json, sizeof(_json), ", %05.2f, %05.2f, %06.1f, %05.3f, %04.0f, %06.3f, %05.2f, %06.1f, %06.1f, %05.2f, %04.2f",
                         t1, t2, Sp_C ,Avolts, rain, depth, h1, p1, p2, SoC, volts);
  logData(_json);
    if(usbOn)  {Serial.println("logged following data to SD-card"); waitMS(100);}
    if(usbOn) {Serial.println(_json); waitMS(100);}
    waitSec(0.5);
  close_SD();

 //--------------take a photo  --------------------
if ( (SoC > 60.0) && ( (Time.hour()>=9) || (Time.hour()==11) ||
                       (Time.hour()==14) || (Time.hour()==15) ) )
  {  
    digitalWrite(B2, HIGH);	//   turn on ground for the camera
    waitSec(2);
    takePhoto(); 
    waitSec(1);
    digitalWrite(B2, LOW);     //disconnect ground for the camera
  }
/* 
//list files on SD to terminal
  cout <<  F("\nList of files on the SD.\n");
  // (sd.ls("/", LS_R) );
  waitSec(0.5);
  sd.ls("/",LS_R | LS_DATE | LS_SIZE | LS_A);
  //  file.timestamp()

  // sprintf(publishStr, " this forces the files to be written to SD %2i minutes", minutes);  
                      // try to get rid of this
  // waitSec(0.5);
 */
//--------------------------------------------------------------------------------------------

if (SoC >40)   // if enough charge connect and upload to Particle and Ubidots, set to 50 ??????????
  { connectToWeb();
    uploadToUbi();
    uploadToParticle();
  }
  // send warning message to particle console
    sprintf(publishStr, "uploaded, will sleep in %2i seconds", seconds);
      Particle.publish(unit_name, publishStr,60,PRIVATE);
    if(usbOn) {Serial.println("sleeping " + String(minutes)); waitMS(100);}
   waitSec(seconds);  //wait seconds. seconds is set at beginning to 5 or else by call 
                     // of "long" to function "delay" from Particle console to 180 seconds
                     // using function  int delayTime(String delay)
                     // call "long" from particle console to give a long time to software update
    waitMS(1000);  // 1 second delay with call to Particle.process() to allow time for OTA flashing
      // send message to particle console
    sprintf(publishStr, "sleeping %2i minutes", minutes);
      ///sprintf(event_name, " %s_on_%s", unit_name.c_str(), code_name);
    char event_name[40];
    sprintf(event_name, " %s_on_%s", unit_name, code_name);
      Particle.publish(event_name, publishStr,60,PRIVATE);

//  Go to sleep for the amount of time determined by the battery charge
//  for sleep modes see:https://community.particle.io/t/choosing-an-electron-sleep-mode/41822?u=colemanjj

    System.sleep(SLEEP_MODE_DEEP, sleepInterval * minutes);   //keeps SOC meter running
    // System.sleep(SLEEP_MODE_SOFTPOWEROFF, sleepInterval * minutes);  // shuts down SOC meter
    // SLEEP_MODE_DEEP = 161 μA
    // SLEEP_MODE_SOFTPOWEROFF = 110 μA

} // end loop()

//***********************************************************************************************
//***********************************************************************************************
//************************************               *******************************************************
//************************************               *******************************************************
//************************************               *******************************************************
//***********************************************************************************************
//--------------------------------------Functions --------------------------------------------------
//
void Flicker(size_t n=1)
    {
        for (size_t i = 0; i < n; i++)
        {
          digitalWrite(ledPin, HIGH);   // Sets the LED on
          delay(10);                   // Waits for a sec
          digitalWrite(ledPin, LOW);   // Sets the LED off
          if (n>1)  delay(60);
        }
    }

void LowBattBlink() //slow blink blue twice
    {
        for (size_t i = 0; i < 2; i++)
        {
          digitalWrite(ledPin, HIGH);   // Sets the LED on
          delay(2000);                   // Waits for a sec
          digitalWrite(ledPin, LOW);   // Sets the LED on
          delay(2000);
        }
    }

void WeakSignalBlink()
    {
        for (size_t i = 0; i < 10; i++)
        {
          digitalWrite(ledPin, HIGH);   // Sets the LED on
          delay(150);                   // Waits for a sec
          digitalWrite(ledPin, LOW);    // Sets the LED off
          delay(150);
        }
          digitalWrite(ledPin, HIGH);   // Sets the LED on
          delay(550);                   // Waits for a sec
          digitalWrite(ledPin, LOW);    // Sets the LED off
    }

void UploadBlink()
    {
        for (size_t i = 0; i < 1; i++)
        {
          digitalWrite(ledPin, HIGH);   // Sets the LED on
          delay(500);                   // Waits for a sec
          digitalWrite(ledPin, LOW);   // Sets the LED on
          delay(1000);
        }
        for (size_t i = 0; i < 4; i++)
        {
          digitalWrite(ledPin, HIGH);   // Sets the LED on
          delay(50);                   // Waits for a sec
          digitalWrite(ledPin, LOW);   // Sets the LED on
          delay(50);
        }
    }
// set sleep time based on battery charge----------------------------
int checkBattery(float charge,float V)          // redo this based on 29.ino ???????????????????
      {
        if (charge < 20) {
          LowBattBlink();
          PMIC pmic;
          pmic.disableBATFET();
          // to prevent complete discharge of battery
          // turns off the battery feed. Unit will still run if power is supplied to VIN,
              // i.e. a solar panel+light
          // unit will stay on programed schedule of waking if power to VIN maintained
          // if no power to VIN, i.e. no light, then unit stays off
          // if power re-applied to VIN, unit boots up, disables battery again but continues with
              //program, including reporting to web
          // pwerer to VIN, i.e. solar+light, will charge battery even if disableBATFET()
          // this routine will:
              //--disable battery if SOC is very low
              //--wake and run the unit if solar powers VIN (what happens if solar fades?)
              //--run on programed schedule if solar powers VIN constantly even if batt < 20%
              //--charge the battery if solar powers VIN (and it is not to cold or hot)
              //--be skipped if power to VIN brings battery charge above 20%

              // does the pmic go back to default when waking from deepsleep?  seems so see:
              // https://community.particle.io/t/electron-solar-charging-rates-and-sleep/37351/6
          }
      
        int min;
        if (charge>25)   //  testing seems to indicate unit stops connecting to internet when too low
          // with a FLCapacitor in parallel with battery, connection continues even when as low as 10%
          // discharging the Electron completely can render it "bricked".
          //   see: https://community.particle.io/t/bug-bounty-electron-not-booting-after-battery-discharges-completely/
          //  Getting it wet will do that also. //   see: https://community.particle.io/t/recover-electron-from-beaver-attack/
              {
                min = 420;  // 7 hours (420 min)  // values set to shorter intervals during code testing
                  if (charge>35 )   min = 300;    // 5 hours (300 min)
                      if (charge>40 )   min =115;     // 2 hours (120 min)
                          if (charge>60 )   min = 90;   // 1.5 hours (90 min)
                                if (charge>70 )   min = 60;     // 60 minutes
                                    if (charge>80 )   min = 30;      // 30 minutes;
                  // after sleep time is set based on battery charge, go on to read sensors and report to internet
              }
              else
              { // if battery below 25%, don't even try to run but go to sleep for 
                  min = 5040;   // sleep 3.5 days (5040 min) if battery very low
            //   sprintf(publishStr, "not connecting, sleeping for %2i min to charge battery ", min);
            //     Serial.println(publishStr);
                  LowBattBlink();
                  System.sleep(SLEEP_MODE_DEEP, sleepInterval * min);
                }
          return min;
      }  // end of checkBattery

// get SpC value from sensor----------------------------------------------
float getSpC()
    {
        #define VREF 3.3      // analog reference voltage(Volt) of the ADC
        #define SCOUNT  40           // number of sample points to collect for averaging
        #define resolution 4095.0  // analog resolution of 4095 with Particle electron
        int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
        int analogBufferTemp[SCOUNT];
        int analogBufferIndex = 0,  copyIndex = 0;
        float averageVoltage = 0;
        float SpC = -1.1;

        while (analogBufferIndex < SCOUNT)   // read the sensor every 50 milliseconds, SCOUNT times and store in array
          {
            analogBuffer[analogBufferIndex] = analogRead(SpCSensorPin);    //read the analog value and store into the buffer
            analogBufferIndex++;
    //         if(analogBufferIndex == SCOUNT)
              delay(50u);  //delay 50 milliseconds between taking sample
          }
        analogBufferIndex = 0;

        for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)  // for coppyIndex = 0 to SCOUNT-1
                  analogBufferTemp[copyIndex]= analogBuffer[copyIndex]; // copy analogBuffer to analogBufferTemp
        averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / resolution; // read the analog value,
                                              // remember particle board has analog resolution of 4095
                                              //made more stable by the median filtering algorithm, and convert to voltage value
    ///      Serial.print(t2);   // temperature comes from a different sensor, outside this function.
    ///      Serial.println(" deg.C at start");
    ///      Serial.print("median analog reading= "); Serial.println(getMedianNum(analogBufferTemp,SCOUNT));
    ///      Serial.print("averageVoltage= "); Serial.println(averageVoltage);
        float compensationCoefficient=1.0+0.019*(t2-25.0);    //temperature compensation formula: 0.019 used by YSI
                  //fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    // coefficients given by DFROBOT on their webpage.  Error in that the temp. compensation. should be after using the equation
    /* TDS=(133.42*compensationVolatge*compensationVolatge*compensationVolatge
              - 255.86*compensationVolatge*compensationVolatge
              + 857.39*compensationVolatge)*0.5*K; //convert voltage value to tds value and multiply by calibration K.
  */
    // coefficients for the following equation derived from calibration to
    // hundreds of specific conductance readings taken at SandL04 by an Onset logger running in parallel with the Spudnik
        SpC= ( 18.835*averageVoltage*averageVoltage*averageVoltage
              + 24.823*averageVoltage*averageVoltage
              + 624.194*averageVoltage) /compensationCoefficient; //compensationCoefficient //convert voltage value to SpC value, then correct for temp

      //  Serial.print("SpC Value: ");
      //  Serial.println(SpC,2);
        return SpC;  //adjust SpC by correction factor
    }  // end of getSpC

// get averageVolts value from sensor.  -----------------------------------------------
      //  This can be sent to Ubidots for use later to calculate Specific Conductance
float getAvolts()
    {
      #define VREF 3.3      // analog reference voltage(Volt) of the ADC
      #define SCOUNT  40           // number of sample points to collect for averaging
      #define resolution 4095.0  // analog resolution of 4095 with Particle electron
      int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
      int analogBufferTemp[SCOUNT];
      int analogBufferIndex = 0, copyIndex = 0;
      float averageVoltage = 0;

      while (analogBufferIndex < SCOUNT)   // read the sensor every 50 milliseconds, SCOUNT times and store in array
        {
            analogBuffer[analogBufferIndex] = analogRead(SpCSensorPin);    //read the analog value and store into the buffer
            analogBufferIndex++;
    //         if(analogBufferIndex == SCOUNT)
            delay(50u);  //delay 50 milliseconds between taking sample
        }
        // copy one array to another
      for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)  // for coppyIndex = 0 to SCOUNT-1  // old way of copying an array
          {
              analogBufferTemp[copyIndex]= analogBuffer[copyIndex];   // copy analogBuffer to analogBufferTemp
          }
      averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / resolution; // read the analog value,
              // remember particle board has analog resolution of 4095
              //made more stable by the median filtering algorithm, and convert to voltage value
    return averageVoltage;
    }  // end of getAvolts

// calculate a median for set of values in buffer ----------------------------------------
int getMedianNum(int bArray[], int iFilterLen)
    {     int bTab[iFilterLen];
        for (byte i = 0; i<iFilterLen; i++)
                bTab[i] = bArray[i];                  // copy input array into BTab[] array
        int i, j, bTemp;
        for (j = 0; j < iFilterLen - 1; j++)        // put array in ascending order
            {  for (i = 0; i < iFilterLen - j - 1; i++)
              {  if (bTab[i] > bTab[i + 1])
                  {  bTemp = bTab[i];
                    bTab[i] = bTab[i + 1];
                    bTab[i + 1] = bTemp;
                  }
                }
              }
      if ((iFilterLen & 1) > 0)  // check to see if iFilterlen is odd or even using & (bitwise AND) i.e if length &AND 1 is TRUE (>0)
            bTemp = bTab[(iFilterLen - 1) / 2];     // then then it is odd, and should take the central value
        else
          bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;  // if even then take aveage of two central values
      return bTemp;
    } //end getmedianNum

// a delay times using ms
inline void waitMS(uint32_t timeout)   // function to delay the system thread for the timeout period
      { // timeout == 0 waits forever
        uint32_t ms = millis();
        while (timeout == 0 || millis() - ms < timeout)
          Particle.process();
      }

 //  create a Delay using timer and seconds, safer than delay()
inline void waitSec(uint32_t seconds) 
    {
      for (uint32_t sec = (millis()/1000); (millis()/1000) - sec < seconds; Particle.process());
    }

//===========================SD FUNCTIONS=====================================

//setup the sd card -----------------------------------------
void setup_SD()
    {
       if (!sd.begin(chipSelect, SD_SCK_MHZ(20))) {  sprintf(works,"No ");   }
          else { sprintf(works,"Yes "); }
  //      Time.zone(-6);  // setup to CST time zone, which is part of the ISO8601 format        //**
        //if(Time.year() < 2020)

        if( !Time.isValid())
        {
               fileName = String("lost-time000.csv");       
        for (int i = 0; i < 1000; i++) {
            fileName.String::operator[](9) = '0' + i/100;
            //           fileName[9] = '0' + i/100;
            fileName.String::operator[](10) = '0' + i/10;
            fileName.String::operator[](11) = '0' + i%10;
            // create if does not exist, do not open existing, write, sync after write
          if (!sd.exists(fileName)) {  break;  }
          }
        }
          else
             fileName =  String(String(unit_name) + "_" + Time.format(Time.now(),"%Y-%m-%d") + ".csv");    //**
          
        { 
           if(usbOn) {Serial.println("card filename " + fileName); waitMS(100);}
           if(usbOn) {Serial.println("card works " + String(works)); waitMS(100);}
        }  
        if(sd.exists(fileName))
            {  file.open(fileName, O_WRONLY | O_APPEND);  }
            else
            {
                file.open(fileName, O_WRONLY |O_CREAT | O_EXCL);
                writeHeader();
            }
    }
//------------------------------------------------------------------------------
// Write data header.
void writeHeader()
    {
      //  file.print(F("datetime, t1, t2, Sp_C ,Avolts, rain, depth, SoC, volts"));
      file.print(F("datetime, Atemp, H2Otemp, Sp_C , Avolts, rain, depth_in, humid, Apressure, H2Opressure, SoC, volts"));
      file.println();
    }
//------------------------------------------------------------------------------
// Log a data record.
void logData(char data[150])
    {
    // Write data to file.
        time_t time = Time.now();
        waitSec(0.5);
        file.print(Time.format(time, TIME_FORMAT_ISO8601_FULL)); // e.g. 2004-01-10T08:22:04-06:00
        waitSec(0.5);
        file.print(data);
        file.println();
        waitSec(0.5);
    }
//--------------------------------------------------------------------------------
//close down the SD card
void close_SD()
    {
    // Force data to SD and update the directory entry to avoid data loss.
     /// if (!file.sync() || file.getWriteError()) {   Particle.publish("Log", "write error", 60, PRIVATE);  }
      if (!file.sync() ) {   Particle.publish("Log", "write error", 60, PRIVATE);  }
        //  if (Serial.available()) {
        // Close file and stop.
        file.flush();
        waitMS(200);
      //  file.close();
        
      if ( file.close() && sd.exists(fileName) )  {
        sprintf(publishStr, "SD-write worked at %s", 
                            Time.format(Time.now(),"%Y-%m-%d_%H-%M").c_str());
           if(usbOn) {Serial.println((publishStr)); waitMS(100);}
        }
        else {
        sprintf(publishStr, "SD-write FAILED at %s", 
                            Time.format(Time.now(),"%Y-%m-%d_%H-%M").c_str());
           if(usbOn) {Serial.println((publishStr)); waitMS(100);}
        }
    }

void watchdogHandler() 
    {
      // Do as little as possible in this function, preferably just calling System.reset().
      // Do not attempt to Particle.publish(), use Cellular.command() 
      // or similar functions. You can save data to a retained variable
      // here safetly so you know the watchdog triggered when you restart.
      // In 2.0.0 and later, System.reset(RESET_NO_WAIT); prevents notifying the cloud of a pending reset
      System.reset();
    }

int delayTime(String delay)
    { if(delay == "long")
        {seconds=180;   // creat enough delay time to flash the unit
        Particle.publish("Particle", "in delayTime",60,PRIVATE);
        return 1; }
      else 
        {seconds=5; return -1; }
    }

//--------take Photo and store on SD--------------------------------------------
void takePhoto()
    {
      camera_VC0706 cam(&Serial1);
      waitSec(0.5);
      // locatecamera
      if (cam.begin()) {
         if(usbOn) {Serial.println("Camera Found:"); waitMS(100);}
      } else {
         if(usbOn) {Serial.println("No camera found?"); waitMS(100);}
        }
      waitSec(0.5);
      // Print out the camera version information (optional)
      char *reply = cam.getVersion();
      if (reply == 0) {
    ///   Serial.print("Failed to get version");
        } else {
        //  Serial.println("-----------------");
            if(usbOn) {Serial.print(reply); waitMS(100);}
        //  Serial.println("-----------------");
        }
       cam.setImageSize(VC0706_640x480);        // biggest
      //cam.setImageSize(VC0706_320x240);        // medium
      //cam.setImageSize(VC0706_160x120);          // small
      // You can read the size back from the camera (optional, but maybe useful?)
        uint8_t imgsize = cam.getImageSize();
        Serial.print("Image size: ");
        if (imgsize == VC0706_640x480) Serial.println("640x480");
        if (imgsize == VC0706_320x240) Serial.println("320x240");
        if (imgsize == VC0706_160x120) Serial.println("160x120");

        if(usbOn) {Serial.println("Snap in 1/2 secs..."); waitMS(100);}
      delay(500);
      if (! cam.takePicture()) 
           if(usbOn) {Serial.println("Failed to snap!"); waitMS(100);}
        else 
           if(usbOn) {Serial.println("Picture taken!"); waitMS(100);}  

      // setupFile
      if(! Time.isValid()) 
            {
              fileName = String("lost-time000.jpg");       
              for (int i = 0; i < 1000; i++) {
                fileName.String::operator[](9) = '0' + i/100;
      //           strcpy(fileName, "lost-time000.jpg");  
      //           fileName[9] = '0' + i/100;
                fileName.String::operator[](10) = '0' + i/10;
                fileName.String::operator[](11) = '0' + i%10;
                // create if does not exist, do not open existing, write, sync after write
              if (!sd.exists(fileName)) {  break;  }
              }
            }
            else
              {
              fileName =  String(String(unit_name) + "_" + Time.format(Time.now(),"%Y-%m-%d-%H-%M") + ".jpg");    
              ///  strcpy(fileName, hold); 
              }
      // Open the file for writing
        file.open(fileName, FILE_WRITE);

      // writePhotoToFile
      // Get the size of the image (frame) taken  
        uint16_t jpglen = cam.frameLength();
         
        {
           if(usbOn) {Serial.print(jpglen, DEC); waitMS(100);}
           if(usbOn) {Serial.println(" byte image. "); waitMS(100);}
           if(usbOn) {Serial.println("photo filename " + fileName); waitMS(100);}
        }

        int32_t time = millis();
        pinMode(8, OUTPUT);
      // Read all the data up to # bytes!
        byte wCount = 0; // For counting # of writes
        while (jpglen > 0) 
        {
          // read 32 bytes at a time;
          uint8_t *buffer;
          uint8_t bytesToRead = min(32, jpglen); // change 32 to 64 for a speedup but may not work with all setups!
          buffer = cam.readPicture(bytesToRead);
          file.write(buffer, bytesToRead);
          if(++wCount >= 64) 
          { // Every 2K, give a little feedback so it doesn't appear locked up
          //  Serial.print('.');
            Flicker(1);
            wCount = 0;
          }
          jpglen -= bytesToRead;
        }
        waitSec(0.1);
      file.sync();  // to update file date
      if ( file.close() && sd.exists(fileName) )  
        {
        sprintf(publishStr, "Photo-save worked at %s", 
                            Time.format(Time.now(),"%Y-%m-%d_%H-%M").c_str());
           if(usbOn) {Serial.println((publishStr)); waitMS(100);}
        }
        else 
        {
        sprintf(publishStr, "Photo-save FAILED at %s", 
                            Time.format(Time.now(),"%Y-%m-%d_%H-%M").c_str());
           if(usbOn) {Serial.println((publishStr)); waitMS(100);}
        }
       // file.close();
        
       {
          time = millis() - time;
           if(usbOn) {Serial.print(time); Serial.println(" ms elapsed"); waitMS(100);}
       }
    }

// dateTime stores current datetime in the right format for FAT
// See SdFile::dateTimeCallback() for usage.
void dateTime(uint16_t* date, uint16_t* time) 
    {
     // return date using FAT_DATE macro to format fields
     *date = FAT_DATE(Time.year(), Time.month(), Time.day());

     // return time using FAT_TIME macro to format fields
     *time = FAT_TIME(Time.hour(), Time.minute(), Time.second());
    }

void setPMIC()
    {
        // Initalize the PMIC class so you can call the Power Management functions below.
      // Particle.publish("PMIC", "setting charge in setup",60,PRIVATE);
    PMIC pmic;
      // pmic.setInputCurrentLimit(150);
      /*******************************************************************************
        Function Name : setInputCurrentLimit
        Description : Sets the input current limit for the PMIC
        Input : 100,150,500,900,1200,1500,2000,3000 (mAmp)
        Return : 0 Error, 1 Success
        use pmic.setInputCurrentLimit(uint16_t current);
        // from spark_wiring_power.cpp
        @ https://github.com/spark/firmware/blob/develop/wiring/src/spark_wiring_power.cpp
        This will be overridden if the input voltage drops out and comes back though (with something like a solar cell)
        and it will be set back to the default 900mA level. To counteract that you could set it in a Software Timer every 60 seconds or so.
        *******************************************************************************/
    pmic.setChargeCurrent(0, 0, 1, 0, 0, 0);      // Set charging current to 1024mA (512 + 512 offset)    //???????? is this good idea?
        //pmic.setChargeCurrent(0, 0, 0, 0, 1, 0);  // Set charging current to 640mA (512 + 128)
      /* Function Name  : setChargeCurrent  // from spark_wiring_power.cpp
        @ https://github.com/spark/firmware/blob/develop/wiring/src/spark_wiring_power.cpp
      * Description    : The total charge current is the 512mA + the combination of the
                        current that the following bits represent
                        bit7 = 2048mA
                        bit6 = 1024mA
                        bit5 = 512mA
                        bit4 = 256mA
                        bit3 = 128mA
                        bit2 = 64mA
    * Input          : six boolean values
                        For example,
                        setChargeCurrent(0,0,1,1,1,0) will set the charge current to
                        512mA + [0+0+512mA+256mA+128mA+0] = 1408mA
        */
      // Set the lowest input voltage to 4.84 volts. This keeps the solar panel from operating below 4.84 volts.
    pmic.setInputVoltageLimit(4840);  //  taken from code suggested by RyanB in the https://community.particle.io forum
          // see: https://community.particle.io/t/pmic-only-sometimes-not-charging-when-battery-voltage-is-below-3-5v/30346
      //      pmic.setInputVoltageLimit(4040); //to get some charge in low light? not sure this helps
      ///pmic.setInputVoltageLimit(5080);
      /*************************from: https://github.com/particle-iot/firmware/blob/develop/wiring/src/spark_wiring_power.cpp
      * Function Name  : setInputVoltageLimit
      * Description    : set the minimum acceptable input voltage
      * Input          : 3880mV to 5080mV in the increments of 80mV
                        3880
                        3960
                        4040
                        4120
                        4200
                        4280
                        4360
                        4440
                        4520
                        4600
                        4680
                        4760
                        4840
                        4920
                        5000
                        5080
      * Return         : 0 Error, 1 Success
    *******************************************************************************/
      //pmic.setChargeVoltage(4512);  // for sealed lead-acit (SLA) battery. may not be implemented in spark_wiring_power.cpp
    pmic.setChargeVoltage(4208); // set upper limit on charge voltage. this limits the
      // max charge that will be given to the battery.
      // default is 4112 in Particle Electron which gives 80% charge. set to 4208 to get charge to go up to 90%
      /*******************************************************************************
      * Function Name  : setChargeVoltage
      * Description    : The total charge voltage is the 3.504V + the combination of the
                        voltage that the following bits represent
                        bit7 = 512mV
                        bit6 = 256mV
                        bit5 = 128mV
                        bit4 = 64mV
                        bit3 = 32mV
                        bit2 = 16mV
      * Input          : desired voltage (4208 or 4112 are the only options currently)
                        4208 is the default // this doesn't seem to be true for the Electron
                        4112 is a safer termination voltage if exposing the
                    battery to temperatures above 45°C & the Particle Electron default
      * Return         : 0 Error, 1 Success
      e.g  case 4112:    writeRegister(CHARGE_VOLTAGE_CONTROL_REGISTER, (mask | 0b10011000));
                                                                                  76543 = 3504+512+64+32=4112
        0b111111000 = max = 4.512 if  spark_wiring_power.cpp gets modified
      *******************************************************************************
      bool PMIC::setChargeVoltage(uint16_t voltage) {.......................
    *******************************************************************************/
    }

void customPower()
{
  // Apply a custom power configuration
  SystemPowerConfiguration conf;
    conf.powerSourceMaxCurrent(1200)   //default 900 mA. Set maximum current the power source can provide when powered through VIN.
                                          //1024 results in 900, 1100 results in 900, 1200 results in 1200, 1160 results in 900
        .powerSourceMinVoltage(4840)  //default 3880 (3.88 v). Set minimum voltage required for VIN to be used. 
                                        // 4840 suggested by RyanB  < https://community.particle.io/t/powering-electron-via-solar-power/30399/2?u=colemanjj >
                                        // and 5080 by Rftop <https://community.particle.io/t/boron-solar-charging-with-1-5-0-rc1/54680/20?u=colemanjj >
        .batteryChargeCurrent(1000)  //default 896 mA. Sets the battery charge current. The actual charge current is the lesser of powerSourceMaxCurrent and batteryChargeCurrent.
                                          // 1200 results in 1152, 1000 results in 960
        .batteryChargeVoltage(4512) //default 4112 (4.112 v) use 4208 to get 90% charge. Sets the battery charge termination voltage.
                                      // set to 3504 to stop charging from usb
                                      // set at 4400 (the max allowed) for charging Lead Acid battery
        .feature(SystemPowerFeature::USE_VIN_SETTINGS_WITH_USB_HOST);

  Serial.println(System.setPowerConfiguration(conf)); // 0 means no error 
  // / int res = System.setPowerConfiguration(conf); 
  // / Log.info("setPowerConfiguration=%d", res);
    waitSec(0);
    // returns SYSTEM_ERROR_NONE (0) in case of success
    // Settings are persisted, you normally wouldn't do this on every startup.
}
void showPMIC()
  {
    PMIC power(true);
    Log.info("Current PMIC settings:");
    Log.info("VIN Vmin_V_input_lowest: %u", power.getInputVoltageLimit());
    Log.info("VIN Imax_current_mA_max_limit: %u", power.getInputCurrentLimit());
    Log.info("Ichg_current_mA_value: %u", power.getChargeCurrentValue());
    Log.info("Iterm_charge_termination_V: %u", power.getChargeVoltageValue());

    int powerSource = System.powerSource();
    int batteryState = System.batteryState();
    float batterySoc = System.batteryCharge();

    constexpr char const* batteryStates[] = {
        "unknown", "not charging", "charging",
        "charged", "discharging", "fault", "disconnected"
      };
    constexpr char const* powerSources[] = {
        "unknown", "vin", "usb host", "usb adapter",
        "usb otg", "battery"
      };

    Log.info("Power source: %s", powerSources[std::max(0, powerSource)]);
    Log.info("Battery state: %s", batteryStates[std::max(0, batteryState)]);
    Log.info("Battery charge: %f", batterySoc);
    waitSec(0.5);
  }

void connectToWeb()
    {
    //----------------------------------------------------------------------------------
    // This command turns on the Cellular Modem and tells it to connect to the cellular network. requires SYSTEM_THREAD(ENABLED)
      //Serial.println("just prior to the Cellular.connect() command");
      //delay(100);
      Cellular.connect();   // this blocks further code execution (see reference) until connection
                              // when in SYSTEM_MODE(semi_automatic),
                              // unless SYSTEM_THREAD(ENABLED). I have SYSTEM_THREAD(ENABLED);
                              //  in any case, after 5 mins of not successfuly connecting the modem
                              // will give up and stop blocking code execution
      delay(200);
    ///   Serial.println("done the Cellular.connect() command, Waiting for Cellular.ready");
          // If the cellular modem does not successfuly connect to the cellular network in
          // 2 mins then blink blue LED and write message to serial below.
          // Regardless of code, after 5 mins of not successfuly connecting the modem will give up.
          if (!waitFor(Cellular.ready, a_minute * 1.5))
            {
                WeakSignalBlink();
                delay(500);
                WeakSignalBlink();
                  if(usbOn) {Serial.println("Difficulty connecting. Will try for 1 more min"); waitMS(100);}
                delay(500);
            }   
          // check a second time to make sure it is connected, if not, try for 1 more minute
          if (!waitFor(Cellular.ready, a_minute * 0.5))
            {
                WeakSignalBlink();
                delay(500);
                WeakSignalBlink();
                  sprintf(publishStr, " sleeping for %2i minutes to wait for better time ", minutes);
                   
                  {
                     if(usbOn) 
                     {  Serial.print("Difficulty connecting, sleeping");   
                        Serial.println(publishStr);
                        waitMS(100);
                     }
                  }
                delay(200);
                //System.sleep(SLEEP_MODE_SOFTPOWEROFF, sleepInterval*minutes);
                System.sleep(SLEEP_MODE_DEEP, sleepInterval * minutes);
                // if can't connect for a second time, go to deep sleep for
                // for "minutes" minutes and on wake the program starts from the beginning
              }       
    ///  Serial.println("passed the Cellular.ready test");
      Particle.connect();
    /// if(Particle.connected()) { wDog.checkin();  } // resets the ApplicationWatchdog count if connected
    ///   if(Particle.connected()) {  
      
          wd->checkin();  
          
          Particle.publish("particle", "connected",60,PRIVATE);
             if(usbOn) {Serial.println("connected"); waitMS(100);}
    ///     } // resets the ApplicationWatchdog count if connected
                  // to cell and connected to Particle cloud.
      
    // if you want to set a position for mapping in Ubidots
    //char context[25];
    //sprintf(context, "lat= 47.6162$lng=-91.595190"); //Sends latitude and longitude for watching position in a map
    ///  sprintf(context, "AirTemp=%05.2f$H2OTemp=%05.2f$A.volts=%05.3f$Depth=%05.2f$tries=%1.1i", t1,t2,Avolts,depth,ii);
    //  ubidots.add("Position", 47.6162, context); // need variable named "Position" to set device location
    // add data to list of items to be sent to Ubidots. Max of 10 items in que. 
        //Limit set in include file ubidots.h  , modified to take 15 adds
    }

void uploadToUbi()
      {
    // ---- get cell signal strength & quality
          CellularSignal sig = Cellular.RSSI();  //this may hang up the system if no connection.
                                        //So this line has been moved to after the if Cellular.ready statement
      //    ubidots.add("CellQual", sig.qual); //value location will show up as Ubidots "context"
      //    ubidots.add("CellStren", sig.rssi);
      ubidots.add("CellQual", sig.getQuality()); //value location will show up as Ubidots "context"
      ubidots.add("CellStren", sig.getStrength());
    //
    //  send the the data to Ubidots after it has been added
          ubidots.send(DATA_SOURCE_NAME,DATA_SOURCE_NAME); // Send rest of the data to your Ubidots account.
        //2020-01-12 modified UbiConstants.h to allow for sending up to 15 variables
                      // but unibots doesn't seem to accept well more than 14 for a device
        waitSec(5);  //give enough time for unit to receive Function call to set the delayTime in seconds

        UploadBlink();
      }  

void  uploadToParticle()
    {
    sprintf(publishStr, 
      "works,%s, t1_offset,%05.2f, t2_offset,%05.2f, k_correction,%05.2f, AtempC,%05.2f, H2Otemp,%05.2f, SpC,%06.1f, rain,%06.0f, Depth_in,%06.3f",
                works, t1_offset, t2_offset, k, t1, t2, Sp_C, rain, depth);
      Particle.publish(unit_name, publishStr, 60, PRIVATE);
      delay(500);
      //    snprintf(_json, sizeof(_json), "%s,{\"AtempC\":\"%05.2f\",\"H2Otemp\":\"%05.2f\",\"SpC\":\"%06.1f\", \"Avolts\":\"%05.3f\",\"rain\":\"%04.0f\",\"depth\":\"%06.3f\",\"SOC\":\"%05.2f\",\"volts\":\"%04.2f\"}",
      //                           unit_name.c_str(), t1, t2, Sp_C ,Avolts, rain, depth, SoC, volts);
    snprintf(_json, sizeof(_json), 
      "{\"AtempC\":\"%05.2f\",\"H2Otemp\":\"%05.2f\",\"SpC\":\"%06.1f\", \"Avolts\":\"%05.3f\",\"rain\":\"%04.0f\",\"depth\":\"%06.3f\",\"SOC\":\"%05.2f\",\"volts\":\"%04.2f\"}",
                t1, t2, Sp_C ,Avolts, rain, depth, SoC, volts );
        Particle.publish("data", _json, PRIVATE);
      delay(500);
       if(usbOn) {Serial.println("finished uploading"); waitMS(100);}
    
    waitSec(1); //wait 1 more seconds
    }     