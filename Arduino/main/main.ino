// UAT SPACE M3 FLIGHT COMPUTER MAIN PROGRAM
// Copyright (c) 2019, UAT Space.
// Copyrights licensed under the Apache-2.0 License.
// Author : Brandon Nay | branay@uat.edu | linkedin.com/in/brandon-nay/

// Compiles on AVR Boards driver v 1.6.21. | lto-wrapper issue on 1.6.22 and up

#include <SD.h>                 // SD
#include <Adafruit_GPS.h>       // GPS
#include <Adafruit_Sensor.h>    // Driver library required by Adafruit sensors
#include <Adafruit_CCS811.h>    // I2C Air quality sensor breakout (eC02, TVOC)
#include <Adafruit_LSM9DS1.h>   // I2C Gyro board (gyro, accelerometer, magnetometer)
#include <Adafruit_AMG88xx.h>   // I2C 8x8 grideye thermal cam
#include <Adafruit_BMP085_U.h>  // I2C Pressure sensor breakout (pressure, temp, humidity, VOC)
#include <DallasTemperature.h>  // Digital Temperature Sensor

#define printer Serial1   // Serial for USB | Serial1 for radio
#define gpsSerial Serial3 // GPS Serial line
#define SDCS 8            // SD card Chip selct pin
#define gpsTX 14          // TX3
#define gpsRX 15          // RX3
#define radioTX 18        // TX1
#define radioRX 19        // RX1
#define tempPin 40        // Digital temp sensor
#define uvPin A0          // UV sensor analog in
#define FILE_BASE_NAME "Data"
#define PRESSURE_HPA (1019.3)   // needs to be updated for launch day (sea level hpa)

// interval between data records in millis
const uint32_t SAMPLE_INTERVAL_MS = 2000;

// time in micros for next data record/transmit
uint32_t logTime;

// AMG88xx pixel buffer
float pixels[64];

// UV sensor value;
float uvValue;

// GPS
Adafruit_GPS gps(&gpsSerial);

// I2C Sensors
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
Adafruit_CCS811 ccs;
Adafruit_AMG88xx amg;

// Digital Temp
OneWire oneWire(tempPin);
DallasTemperature digitalTemp(&oneWire);

// SD
File file;


/////////////////////////
// FUNCTION PROTOTYPES //
/////////////////////////

void error(uint8_t c);
void processData();
void startAMG();
void startBMP();
void startCCS();
void startGPS();
void startLSM();
void startTemp();
void startComponents();

///////////
// SETUP //
///////////

void setup() {
  // start either Serial or Serial1
  printer.begin(9600);
  while (!printer);

  // start all sensors
  startComponents();
  pinMode(uvPin, INPUT);

  // start on a multiple of the sample interval.
  logTime = micros() / (1000UL * SAMPLE_INTERVAL_MS) + 1;
  logTime *= 1000UL * SAMPLE_INTERVAL_MS;
}

//////////
// LOOP //
//////////

void loop() {
  // time for next record.
  logTime += 1000UL * SAMPLE_INTERVAL_MS;

  // wait for log time.
  int32_t diff;
  do {
    diff = micros() - logTime;
  } while (diff < 0);

  // check for data rate too high.
  if (diff > 10) {
    error(1);
  }

  // LOG
  processData();
}

//////////////////////////
// FUNCTION DEFINITIONS //
//////////////////////////

/// sends error code (int) over printMode
void error(uint8_t c) {
  printer.print(F("ERROR: "));
  printer.println(c);
}

/// collects data from sensors, transmits to ground and saves to SD
void processData() {
  // perform sensor readings
  
  // parse new GPS string
  if (gps.newNMEAreceived()) {
    gps.parse(gps.lastNMEA());
  }

  // UV sensor
  uvValue = analogRead(uvPin);

  // AMG88xx
  amg.readPixels(pixels);
  
  // LSM9DS1 gyro
  sensors_event_t a, m, g, temp;
  lsm.read();
  lsm.getEvent(&a, &m, &g, &temp);

  // digital temp sensor
  digitalTemp.requestTemperatures();

  // BMP180
  sensors_event_t bmpEvent;
  float bmpTemperature;
  float bmpAltitude;
  bmp.getEvent(&bmpEvent);
  bmp.getTemperature(&bmpTemperature);
  bmpAltitude = bmp.pressureToAltitude(PRESSURE_HPA, bmpEvent.pressure);

  // CCS811
  float ccsTemp = ccs.calculateTemperature();
  ccs.readData();

  // report sensor readings (printer -> radio and SD)

  // logTime/1M -> seconds elapsed
  printer.print((logTime / 1000000));             printer.print(F(","));
  file.print((logTime / 1000000));             printer.print(F(","));
  printer.print(gps.hour);                        printer.print(F(":"));
  file.print(gps.hour);                        printer.print(F(":"));
  printer.print(gps.minute);                      printer.print(F(":"));
  file.print(gps.minute);                      printer.print(F(":"));
  printer.print(gps.seconds);                     printer.print(F(","));
  file.print(gps.seconds);                     printer.print(F(","));
  printer.print(gps.latitudeDegrees, 4);          printer.print(F(","));
  file.print(gps.latitudeDegrees, 4);          printer.print(F(","));
  printer.print(gps.longitudeDegrees, 4);         printer.print(F(","));
  file.print(gps.longitudeDegrees, 4);         printer.print(F(","));
  printer.print(gps.altitude);                    printer.print(F(","));
  file.print(gps.altitude);                    printer.print(F(","));
  printer.print(bmpAltitude);                     printer.print(F(","));
  file.print(bmpAltitude);                     printer.print(F(","));
  printer.print(bmpEvent.pressure);               printer.print(F(","));
  file.print(bmpEvent.pressure);               printer.print(F(","));
  printer.print(bmpTemperature);                  printer.print(F(","));
  file.print(bmpTemperature);                  printer.print(F(","));
  printer.print(digitalTemp.getTempCByIndex(0));  printer.print(F(","));
  file.print(digitalTemp.getTempCByIndex(0));  printer.print(F(","));
  printer.print(ccsTemp);                         printer.print(F(","));
  file.print(ccsTemp);                         printer.print(F(","));
  printer.print(ccs.geteCO2());                   printer.print(F(","));
  file.print(ccs.geteCO2());                   printer.print(F(","));
  printer.print(ccs.getTVOC());                   printer.print(F(","));
  file.print(ccs.getTVOC());                   printer.print(F(","));
  printer.print(uvValue);                         printer.print(F(","));
  file.print(uvValue);                         printer.print(F(","));
  printer.print(a.acceleration.x);                printer.print(F(","));
  file.print(a.acceleration.x);                printer.print(F(","));
  printer.print(a.acceleration.y);                printer.print(F(","));
  file.print(a.acceleration.y);                printer.print(F(","));
  printer.print(a.acceleration.z);                printer.print(F(","));
  file.print(a.acceleration.z);                printer.print(F(","));
  printer.print(m.magnetic.x);                    printer.print(F(","));
  file.print(m.magnetic.x);                    printer.print(F(","));
  printer.print(m.magnetic.y);                    printer.print(F(","));
  file.print(m.magnetic.y);                    printer.print(F(","));
  printer.print(m.magnetic.z);                    printer.print(F(","));
  file.print(m.magnetic.z);                    printer.print(F(","));
  printer.print(g.gyro.x);                        printer.print(F(","));
  file.print(g.gyro.x);                        printer.print(F(","));
  printer.print(g.gyro.y);                        printer.print(F(","));
  file.print(g.gyro.y);                        printer.print(F(","));
  printer.print(g.gyro.z);                        printer.print(F(","));
  file.print(g.gyro.z);                        printer.print(F(","));

  file.flush();
  
  // AMG pixel array
  printer.print(F("["));
  for (uint8_t i = 0; i < 63; i++) {
    printer.print(pixels[i]);
    printer.print(F(","));
  }
  printer.print(pixels[63]);
  printer.println(F("]"));
}

void startAMG() {
  if (!amg.begin()) {
    error(2);
  }
}

/// starts the BMP180 sensor
void startBMP() {
  if (!bmp.begin()) {
    error(3);
  }
}

/// starts the CCS811 sensor
void startCCS(){
  if (!ccs.begin()) {
    error(4);
  }
  while(!ccs.available());
  
  //calibrate temperature sensor
  float temp = ccs.calculateTemperature();
  ccs.setTempOffset(temp - 25.0);
}

void startGPS() {
  gps.begin(9600);
  while (!gpsSerial);
  
  gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
}

/// starts the LSM9DS1 sensor and sets sensitivities
void startLSM() {
  if (!lsm.begin()) {
    error(5);
    return;
  }
  
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}

void startSD() {
  if (!SD.begin(SDCS)) {
    error(6);
  }

  // find unused file name
  const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
  char fileName[13] = FILE_BASE_NAME "00.csv";
  if (BASE_NAME_SIZE > 6) {
    error(8);
  }
  while (SD.exists(fileName)) {
    if (fileName[BASE_NAME_SIZE + 1] != '9') {
      fileName[BASE_NAME_SIZE + 1]++;
    }
    else if (fileName[BASE_NAME_SIZE] != '9') {
      fileName[BASE_NAME_SIZE + 1] = '0';
      fileName[BASE_NAME_SIZE]++;
    }
    else {
      error(9);
    }
  }
  file = SD.open(fileName, "W+");
  if (!file) {
    error(24);
  }
}

void startTemp() {
  digitalTemp.begin();
}

void startComponents() {
  startAMG();
  startBMP();
  startCCS();
  startGPS();
  startLSM();
  startSD();
  startTemp();
}
