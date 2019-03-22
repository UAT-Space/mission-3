// UAT SPACE M3 FLIGHT COMPUTER MAIN PROGRAM
// Copyright (c) 2019, UAT Space.
// Copyrights licensed under the Apache-2.0 License.
// Author : Brandon Nay | branay@uat.edu | linkedin.com/in/brandon-nay/

#include <SPI.h>
#include <Wire.h>
#include <SdFat.h>            // SD
#include <Adafruit_GPS.h>     // GPS
#include <Adafruit_Sensor.h>  // General library required by Adafruit sensors
#include <Adafruit_CCS811.h>  // Air quality sensor breakout (eC02, TVOC)
#include <Adafruit_BME680.h>  // Pressure sensor breakout (pressure, temp, humidity, VOC)
#include <Adafruit_LSM9DS1.h> // Gyro board (gyro, accelerometer, magnetometer)

#define printMode Serial      // Serial for USB | Serial1 for radio
#define gpsTX         14      // TX3
#define gpsRX         15      // RX3
#define radioTX       18      // TX1
#define radioRX       19      // RX1
#define tempPin       40      // Digital temp sensor
#define chipSelect    53      // SD CS
#define FILE_BASE_NAME "Data"

// Interval between data records in millis
const uint32_t SAMPLE_INTERVAL_MS = 3000;

// time in micros for next data record/transmit
uint32_t logTime;

// I2C
Adafruit_LSM9DS1 gyro = Adafruit_LSM9DS1();
Adafruit_BME680 bme;

// SD
SdFat sd;
SdFile file;


// class feeds one input to two output channels (ex. radio and SD)
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


Tee printer(printMode, file);

/////////////////////////
// FUNCTION PROTOTYPES //
/////////////////////////

void error(uint8_t c);

///////////
// SETUP //
///////////

void setup() {
  if (printMode == Serial) {
    Serial.begin(9600);
    while (!Serial) SysCall::yield();
    delay(1000);  // necessary?
  }

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

  // Start on a multiple of the sample interval.
  logTime = micros()/(1000UL*SAMPLE_INTERVAL_MS) + 1;
  logTime *= 1000UL*SAMPLE_INTERVAL_MS;
}

//////////
// LOOP //
//////////

void loop() {
  // Time for next record.
  logTime += 1000UL*SAMPLE_INTERVAL_MS;

  // Wait for log time.
  int32_t diff;
  do {
    diff = micros() - logTime;
  } while (diff < 0);

  // Check for data rate too high.
  if (diff > 10) {
    error(5);
  }

  // LOG

  // Force data to SD and update the directory entry to avoid data loss.
  if (!file.sync() || file.getWriteError()) {
    error(6);
  }
}

//////////////////////////
// FUNCTION DEFINITIONS //
//////////////////////////

void error(uint8_t c) {
  printMode.print(F("ERROR: "));
  printMode.println(c);
}
