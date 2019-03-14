// UAT SPACE M2 FLIGHT COMPUTER MAIN PROGRAM
// Copyright (c) 2018, UAT Space. All rights reserved.
// Copyrights licensed under the Apache-2.0 License.
// Author : Brandon Nay, branay@uat.edu, linkedin.com/in/brandon-nay/

// NOTE: The SparkFunCCS811 library has a compile error on its end.
//       To fix this, comment out line 61:64 inside the cpp source.

#include <SparkFunCCS811.h>   // Air quality sensor on the environmental combo board
#include <SparkFunBME280.h>   // Pressure sensor on the environmental combo board
#include <Adafruit_Sensor.h>  // General library required by Adafruit sensors
#include <Adafruit_LSM9DS1.h> // Gyro board (gyro, accelerometer, magnetometer)
#include <Adafruit_GPS.h>     // GPS
#include <SD.h>               // SD
//#include <Wire.h>           // disabled unless needed (I2C related)

#define PRINTER   Serial2      // Serial for debugging :: Serial2 for flight
#define gpsSerial Serial1     // GPS Serial
#define GPSECHO     false     // Toggle to enable/disable GPS debugging
#define solarPanel      5     // Analog pin 5
#define wind1           4     // Analog pin 5
#define wind2           3     // Analog pin 5
#define SDCS            8     // CHECK THAT THIS IS ACCURATE (SD Shield)
#define COMtx          16     // Serial2 TX : connect to transceiver RX
#define COMrx          17     // Serial2 RX : connect to transceiver TX
#define GPStx          18     // Serial1 TX : connect to GPS RX
#define GPSrx          19     // Serial1 RX : connect to GPS TX
// STILL NEED ANALOG READ FOR SOLAR AND WIND TURBINES

// GPS
Adafruit_GPS GPS(&gpsSerial);       // create 'Adafruit_GPS' object named 'GPS'
bool usingInterrupt = false;        // variable to see if we're using interrupts

SIGNAL(TIMER0_COMPA_vect)
{
  // Interrupt is called once a millisecond, looks for any new GPS data, and stores it
  char c = GPS.read();
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;
#endif
}

// I2C
Adafruit_LSM9DS1 gyro = Adafruit_LSM9DS1(); // create 'Adafruit_LSM9DS1' object named 'gyro'
BME280 bme;                                 // define 'BME280'           object named 'bme'

// SD
File data;  // always open - write with data.print("") - save with data.flush()
String logString;

// clock
//uint32_t timer = millis();

////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTION PROTOTYPES /////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
void createLog();
void printError(int);
void readBME();
void readGyro();
void startBME();  // can throw error 1
void startComms();
void startDebug();
void startGPS();
void startGyro(); // can throw error 2
void startSD();   // can throw error 3 4 5
void useInterrupt(bool);

////////////////////////////////////////////////////////////////////////////////////////////
// SETUP ///////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  if (PRINTER == Serial) startDebug();
  startComms();
  startGPS();
  startSD();
  //  open data file - file remains open and is saved to via data.flush()
  data = SD.open("data.txt", FILE_WRITE);
  startGyro();
  startBME();
}

////////////////////////////////////////////////////////////////////////////////////////////
// LOOP ////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

void loop()
{
  if (GPS.newNMEAreceived())
  {
    GPS.parse(GPS.lastNMEA());
  }

  createLog();
  Serial2.print(GPS.latitudeDegrees, 4);
  Serial2.print(',');
  Serial2.print(GPS.longitudeDegrees, 4);
  Serial2.println(logString);
  //  PRINTER.println(logString);
  data.print(GPS.latitudeDegrees, 4);
  data.print(',');
  data.print(GPS.longitudeDegrees, 4);
  data.println(logString);
  data.flush();

  delay(5000);
}

//  PRINTER.print(GPS.hour - 7);  // -7 accounts for arizona's timezone offset
//  PRINTER.print(GPS.minute);
//  PRINTER.print(GPS.seconds);
//  PRINTER.print(',');
//  PRINTER.print(GPS.latitudeDegrees, 4);    // 4 specifies 4 decimal places; default 2
//  PRINTER.print(',');
//  PRINTER.print(GPS.longitudeDegrees, 4);   // 4 specifies 4 decimal places; default 2
//  PRINTER.print(',');
//  PRINTER.print(GPS.altitude);
//  PRINTER.print(',');
//  readBME();
//  PRINTER.print(',');
//  readGyro();
//  PRINTER.println();

//  Retrievable GPS data formats

//  GPS.fix
//  GPS.hour
//  GPS.minute
//  GPS.seconds
//  GPS.milliseconds
//  if (GPS.fix)
//  {
//    GPS.latitude
//    GPS.lat
//    GPS.longitude
//    GPS.lon
//    GPS.latitudeDegrees   // works with google maps
//    GPS.longitudeDegrees  // works with google maps
//    GPS.speed
//    GPS.altitude
//  }

////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTION DEFINITIONS ////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

void createLog()
{
  gyro.read();
  sensors_event_t a, m, g, temp;
  gyro.getEvent(&a, &m, &g, &temp);

  logString = "";

  logString += ',';
  logString += GPS.hour;
  logString += GPS.minute;
  logString += GPS.seconds;
  logString += ',';
  logString += GPS.altitude;
  logString += ',';
  logString += bme.readTempF();
  logString += ',';
  logString += bme.readFloatPressure();
  logString += ',';
  logString += bme.readFloatAltitudeMeters();
  logString += ',';
  logString += bme.readFloatHumidity();
  logString += ',';
  logString += analogRead(solarPanel);
  logString += ',';
  logString += analogRead(wind1);
  logString += ',';
  logString += analogRead(wind2);
//  logString += a.acceleration.x;
//  logString += ',';
//  logString += a.acceleration.y;
//  logString += ',';
//  logString += a.acceleration.z;
//  logString += ',';
//  logString += m.magnetic.x;
//  logString += ',';
//  logString += m.magnetic.y;
//  logString += ',';
//  logString += m.magnetic.z;
//  logString += ',';
//  logString += g.gyro.x;
//  logString += ',';
//  logString += g.gyro.y;
//  logString += ',';
//  logString += g.gyro.z;
  return;
}

void printError(int n)
{
  PRINTER.println();
  PRINTER.print(">>ERROR: ");
  PRINTER.println(n);
  return;
}

void readBME()
{
  // always read temp first - reading temp triggeres all sensors to read
  PRINTER.print(bme.readTempF());
  PRINTER.print(',');
  PRINTER.print(bme.readFloatPressure());
  PRINTER.print(',');
  PRINTER.print(bme.readFloatAltitudeMeters());
  PRINTER.print(',');
  PRINTER.print(bme.readFloatHumidity());
}

void readGyro()
{
  // Getting event data:

  gyro.read();
  sensors_event_t a, m, g, temp;
  gyro.getEvent(&a, &m, &g, &temp);

  PRINTER.print(a.acceleration.x);
  PRINTER.print(',');
  PRINTER.print(a.acceleration.y);
  PRINTER.print(',');
  PRINTER.print(a.acceleration.z);
  PRINTER.print(',');
  PRINTER.print(m.magnetic.x);
  PRINTER.print(',');
  PRINTER.print(m.magnetic.y);
  PRINTER.print(',');
  PRINTER.print(m.magnetic.z);
  PRINTER.print(',');
  PRINTER.print(g.gyro.x);
  PRINTER.print(',');
  PRINTER.print(g.gyro.y);
  PRINTER.print(',');
  PRINTER.print(g.gyro.z);

  // Access event data the following ways:

  // Acceleration (M/s^2)
  // a.acceleration.x
  // a.acceleration.y
  // a.acceleration.z

  // Magnetometer (gauss)
  // m.magnetic.x
  // m.magnetic.y
  // m.magnetic.z

  // Gyroscope (degrees per second)
  // g.gyro.x
  // g.gyro.y
  // g.gyro.z
}

void startBME()
{
  if (!bme.beginI2C())
  {
    printError(1);
    return;
  }
}

void startComms()
{
  //  pinMode(COMtx, OUTPUT); // uncomment if necessary
  //  pinMode(COMrx, INPUT);  // uncomment if necessary
  Serial2.begin(9600);
  while (!Serial2);
}

void startDebug()
{
  Serial.begin(9600);
  while (!Serial);
}

void startGPS()
{
  GPS.begin(9600);
  while (!Serial1);

  // these 3 options set which NMEA strings the GPS outputs
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);

  // these 3 options set how fast the GPS sends updates
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
  //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);

  // disable updates on antenna status
  GPS.sendCommand(PGCMD_NOANTENNA);

  // enable using an interrupt to store incoming GPS data
  useInterrupt(true);
}

void startGyro()
{
  if (!gyro.begin())
  {
    printError(2);
    return;
  }

  gyro.setupAccel(gyro.LSM9DS1_ACCELRANGE_2G);
  //gyro.setupAccel(gyro.LSM9DS1_ACCELRANGE_4G);
  //gyro.setupAccel(gyro.LSM9DS1_ACCELRANGE_8G);
  //gyro.setupAccel(gyro.LSM9DS1_ACCELRANGE_16G);

  gyro.setupMag(gyro.LSM9DS1_MAGGAIN_4GAUSS);
  //gyro.setupMag(gyro.LSM9DS1_MAGGAIN_8GAUSS);
  //gyro.setupMag(gyro.LSM9DS1_MAGGAIN_12GAUSS);
  //gyro.setupMag(gyro.LSM9DS1_MAGGAIN_16GAUSS);

  //gyro.setupGyro(gyro.LSM9DS1_GYROSCALE_245DPS);
  gyro.setupGyro(gyro.LSM9DS1_GYROSCALE_500DPS);
  //gyro.setupGyro(gyro.LSM9DS1_GYROSCALE_2000DPS);
}

void startSD()
{
  pinMode(SDCS, OUTPUT);

  // attempt to start SD card up to 3 times
  for (int i = 1; i <= 3; i++)
  {
    if (SD.begin(SDCS)) break;
    if (i == 3)
    {
      printError(3);
      return;
    }
  }

  //====CHECK FOR DATA FILE====//
  if (SD.exists("data.txt"))
  {
    // if dataDump doesn't exist, create it
    if (!SD.exists("dataDump.txt"))
    {
      File f = SD.open("dataDump.txt");
      f.close();
    }

    // copy content of data to dataDump
    File readFile = SD.open("data.txt", FILE_READ);
    File writeFile = SD.open("dataDump.txt", FILE_WRITE);
    if ((readFile) && (writeFile))
    {
      writeFile.println(" ");
      writeFile.println("NEW DUMP");
      writeFile.println(" ");
      byte data;
      while (readFile.available())
      {
        writeFile.write(readFile.read());
      }

      readFile.close();
      writeFile.close();
    }
    else
    {
      printError(4);
      return;
    }

    // remove original data file
    if (!SD.remove("data.txt"))
    {
      printError(5);
      return;
    }
  }

  // if the data file doesn't exist, create it
  if (!SD.exists("data.txt"))
  {
    File f = SD.open("data.txt", FILE_WRITE);
    f.close();
  }
  return;
}

// low-level arduino code pulled from an example--uses hardware addresses
void useInterrupt(bool v)
{
  if (v)
  {
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  }
  else
  {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}
