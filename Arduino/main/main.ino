

// UAT SPACE M3 FLIGHT COMPUTER MAIN PROGRAM
// Copyright (c) 2019, UAT Space.
// Copyrights licensed under the Apache-2.0 License.
// Author : Brandon Nay | branay@uat.edu | linkedin.com/in/brandon-nay/

#include <SdFat.h>              // SD
#include <Adafruit_GPS.h>       // GPS
#include <Adafruit_Sensor.h>    // Driver library required by Adafruit sensors
#include <Adafruit_CCS811.h>    // I2C Air quality sensor breakout (eC02, TVOC)
#include <Adafruit_BME680.h>    // I2C Pressure sensor breakout (pressure, temp, humidity, VOC)
#include <Adafruit_LSM9DS1.h>   // I2C Gyro board (gyro, accelerometer, magnetometer)
#include <DallasTemperature.h>  // Digital Temperature Sensor

#define printMode Serial      // Serial for USB | Serial1 for radio
#define gpsSerial Serial3     // GPS Serial line
#define gpsTX          14     // TX3
#define gpsRX          15     // RX3
#define radioTX        18     // TX1
#define radioRX        19     // RX1
#define tempPin        40     // Digital temp sensor
#define chipSelect     53     // SD CS
#define FILE_BASE_NAME "Data"
#define PRESSURE_HPA (1018.0) // needs to be updated for launch day (sea level hpa)

// interval between data records in millis
const uint32_t SAMPLE_INTERVAL_MS = 3000;

// time in micros for next data record/transmit
uint32_t logTime;

// GPS
Adafruit_GPS gps(&gpsSerial);

// I2C Sensors
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
Adafruit_BME680 bme;
Adafruit_CCS811 ccs;

// SD
SdFat sd;
SdFile file;


/* Class feeds one input to two output channels (ex. radio and SD).
   Inherits from Print: instances use instance.print/println/write.
   In this file we use it for sending the same data to the SD and
   radio with one function call. */
class Tee : public Print {
  public:
    Tee(Print &_p1, Print &_p2) : p1(_p1), p2(_p2) {}

    size_t write(uint8_t c) {
      size_t count1 = p1.write(c);
      size_t count2 = p2.write(c);
      return min(count1, count2);
    }
  private:
    Print &p1, &p2;
};


// printer.print(data) -> SD and radio output
Tee printer(printMode, file);

/////////////////////////
// FUNCTION PROTOTYPES //
/////////////////////////

void error(uint8_t c);
void processData();
void startBME();
void startCCS();
void startGPS();
void startLSM();
void startSD();
void startComponents();

///////////
// SETUP //
///////////

void setup() {
  // start either Serial or Serial1
  printMode.begin(9600);
  while (!printMode) {
    SysCall::yield();
  }

  // start BME/CCS/GPS/GYRO/SD
  startComponents();

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
    error(5);
  }

  // LOG
  processData();

  // force data to SD and update the directory entry to avoid data loss.
  if (!file.sync() || file.getWriteError()) {
    error(6);
  }
}

//////////////////////////
// FUNCTION DEFINITIONS //
//////////////////////////

/// sends error code (int) over printMode
void error(uint8_t c) {
  printMode.print(F("ERROR: "));
  printMode.println(c);
}

/// collects data from sensors, transmits to ground and saves to SD
void processData() {
  // perform sensor readings
  
  // parse new GPS string
  if (gps.newNMEAreceived()) {
    gps.parse(gps.lastNMEA());
  }
  
  // LSM9DS1 gyro
  sensors_event_t a, m, g, temp;
  lsm.read();
  lsm.getEvent(&a, &m, &g, &temp);

  // digital temp sensor

  // BME680
  if (!bme.performReading()) {
    error(1);
  }

  // CCS811
  float ccsTemp = ccs.calculateTemperature();
  ccs.readData();

  // report sensor readings (printer -> radio and SD)

  // logTime/1M -> seconds elapsed
  printer.print(logTime / 1000000);               printer.print(F(","));
  printer.print(gps.hour);                        printer.print(F(":"));
  printer.print(gps.minute);                      printer.print(F(":"));
  printer.print(gps.seconds);                     printer.print(F(","));
  printer.print(gps.latitudeDegrees, 4);          printer.print(F(","));
  printer.print(gps.longitudeDegrees, 4);         printer.print(F(","));
  printer.print(gps.altitude);                    printer.print(F(","));
  printer.print(bme.pressure / 100.0);            printer.print(F(","));
  printer.print(bme.humidity);                    printer.print(F(","));
  printer.print(bme.gas_resistance / 1000.0);     printer.print(F(","));
  printer.print(bme.readAltitude(PRESSURE_HPA));  printer.print(F(","));
  printer.print(bme.temperature);                 printer.print(F(","));
  // digital temp here
  printer.print(ccsTemp);                         printer.print(F(","));
  printer.print(ccs.geteCO2());                   printer.print(F(","));
  printer.print(ccs.getTVOC());                   printer.print(F(","));
  printer.print(a.acceleration.x);                printer.print(F(","));
  printer.print(a.acceleration.y);                printer.print(F(","));
  printer.print(a.acceleration.z);                printer.print(F(","));
  printer.print(m.magnetic.x);                    printer.print(F(","));
  printer.print(m.magnetic.y);                    printer.print(F(","));
  printer.print(m.magnetic.z);                    printer.print(F(","));
  printer.print(g.gyro.x);                        printer.print(F(","));
  printer.print(g.gyro.y);                        printer.print(F(","));
  printer.print(g.gyro.z);
}

/// starts the BME680 sensor
void startBME() {
  if (!bme.begin()) {
    error(1);
  }

  // set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
}

/// starts the CCS811 sensor
void startCCS(){
  if (!ccs.begin()) {
    error(1);
  }
  while(!ccs.available()) {
    SysCall::yield();
  }
  
  //calibrate temperature sensor
  float temp = ccs.calculateTemperature();
  ccs.setTempOffset(temp - 25.0);
}

void startGPS() {
  gps.begin(9600);
  gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
}

/// starts the LSM9DS1 sensor and sets sensitivities
void startLSM() {
  if (!lsm.begin()) {
    error(1);
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

/// starts the SD card and creates a new file to avoid overwriting
void startSD() {
  // try speed lower than 50 if SPI errors occur
  if (!sd.begin(chipSelect, SD_SCK_MHZ(50))) {
    error(1);
  }

  // find unused file name
  const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
  char fileName[13] = FILE_BASE_NAME "00.csv";
  if (BASE_NAME_SIZE > 6) {
    error(2);
  }
  while (sd.exists(fileName)) {
    if (fileName[BASE_NAME_SIZE + 1] != '9') {
      fileName[BASE_NAME_SIZE + 1]++;
    }
    else if (fileName[BASE_NAME_SIZE] != '9') {
      fileName[BASE_NAME_SIZE + 1] = '0';
      fileName[BASE_NAME_SIZE]++;
    }
    else {
      error(3);
    }
  }
  if (!file.open(fileName, O_WRONLY | O_CREAT | O_EXCL)) {
    error(4);
  }
}

void startComponents() {
  startBME();
  startCCS();
  startGPS();
  startLSM();
  startSD();
}
