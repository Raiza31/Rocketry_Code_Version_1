// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added multiple output formats
//                 - added seamless Fastwire support
//      2011-10-07 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "Arduino.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Adafruit_BMP085.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "FFat.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;

Adafruit_BMP085 bmp;

// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO


#define LED_PIN 13
bool blinkState = false;

File dataFile;

void readFile(fs::FS &fs, const char * path){
    Serial.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if(!file){
        Serial.println("Failed to open file for reading");
        return;
    }

    Serial.print("Read from file: ");
    while(file.available()){
        Serial.write(file.read());
    }
    file.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("File written");
    } else {
        Serial.println("Write failed");
    }
    file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("Failed to open file for appending");
        return;
    }
    if(file.print(message)){
        Serial.println("Message appended");
    } else {
        Serial.println("Append failed");
    }
    file.close();
}

void testFileIO(fs::FS &fs, const char * path){
    File file = fs.open(path);
    static uint8_t buf[512];
    size_t len = 0;
    uint32_t start = millis();
    uint32_t end = start;
    if(file){
        len = file.size();
        size_t flen = len;
        start = millis();
        while(len){
            size_t toRead = len;
            if(toRead > 512){
                toRead = 512;
            }
            file.read(buf, toRead);
            len -= toRead;
        }
        end = millis() - start;
        Serial.printf("%u bytes read for %u ms\n", flen, end);
        file.close();
    } else {
        Serial.println("Failed to open file for reading");
    }


    file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }

    size_t i;
    start = millis();
    for(i=0; i<2048; i++){
        file.write(buf, 512);
    }
    end = millis() - start;
    Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
    file.close();
}



void setup() {
// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
  // it's really up to you depending on your project)
  Serial.begin(115200);

  // if (!bmp.begin()) {
  //   Serial.println("Could not find a valid BMP085 sensor, check wiring!");
  //   while (1) {}
  // } else {
  //   Serial.println("BMP180 connection successful");
  // }


  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // use the code below to change accel/gyro offset values
  /*
    Serial.println("Updating internal sensor offsets...");
    // -76	-2359	1688	0	0	0
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    accelgyro.setXGyroOffset(220);
    accelgyro.setYGyroOffset(76);
    accelgyro.setZGyroOffset(-85);
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    */

  // configure Arduino LED pin for output
  pinMode(LED_PIN, OUTPUT);

  if(!SD.begin()){
      Serial.println("Card Mount Failed");
      while(1);
  }
  uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE){
      Serial.println("No SD card attached");
      return;
  }

  Serial.print("SD Card Type: ");
  if(cardType == CARD_MMC){
      Serial.println("MMC");
  } else if(cardType == CARD_SD){
      Serial.println("SDSC");
  } else if(cardType == CARD_SDHC){
      Serial.println("SDHC");
  } else {
      Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);

  dataFile = SD.open("/Rocket001.txt", FILE_WRITE);

  if (!dataFile) {
    Serial.println("error opening datalog.txt");
  }

  String headerString = "";
  headerString += "T(ms)";
  headerString += "\t";
  headerString += "Alt(m)";
  headerString += "\t";
  headerString += "AX(m/s/s)";
  headerString += "\t";
  headerString += "AY(m/s/s)";
  headerString += "\t";
  headerString += "AZ(m/s/s)";
  headerString += "\t";
  headerString += "GyroX";
  headerString += "\t";
  headerString += "GyroY";
  headerString += "\t";
  headerString += "GyroZ";
  //headerString += "\n";

  dataFile.println(headerString);
  dataFile.flush();

  // writeFile(SD, "/Rocket001.txt", "T(ms)"); appendFile(SD, "/Rocket001.txt", "\t");
  // appendFile(SD, "/Rocket001.txt", "Alt(m)"); appendFile(SD, "/Rocket001.txt", "\t");
  // appendFile(SD, "/Rocket001.txt", "AX(m/s/s)"); appendFile(SD, "/Rocket001.txt", "\t");
  // appendFile(SD, "/Rocket001.txt", "AY(m/s/s)"); appendFile(SD, "/Rocket001.txt", "\t");
  // appendFile(SD, "/Rocket001.txt", "AZ(m/s/s)"); appendFile(SD, "/Rocket001.txt", "\t");
  // appendFile(SD, "/Rocket001.txt", "GyroX" ); appendFile(SD, "/Rocket001.txt", "\t");
  // appendFile(SD, "/Rocket001.txt", "GyroY"); appendFile(SD, "/Rocket001.txt", "\t");
  // appendFile(SD, "/Rocket001.txt", "GyroZ"); appendFile(SD, "/Rocket001.txt", "\t");
  // appendFile(SD, "/Rocket001.txt", "\n");
}

void loop() {
  //unsigned long currentMillis = millis();
  // read raw accel/gyro measurements from device
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // these methods (and a few others) are also available
  //accelgyro.getAcceleration(&ax, &ay, &az);
  //accelgyro.getRotation(&gx, &gy, &gz);

  // String millisStr = String(currentMillis);
  // String altStr = String(bmp.readAltitude(101500));
  // String accelX_m_s2Str = String(ax);
  // String accelY_m_s2Str = String(ay);
  // String accelZ_m_s2Str = String(az);
  // String gxStr = String(gx);
  // String gyStr = String(gy);
  // String gzStr = String(gz);

  // const char* currentMillisC = millisStr.c_str();
  // const char* altC = altStr.c_str();
  // const char* accelX_m_s2C = accelX_m_s2Str.c_str();
  // const char* accelY_m_s2C = accelY_m_s2Str.c_str();
  // const char* accelZ_m_s2C = accelZ_m_s2Str.c_str();
  // const char* gxC = gxStr.c_str();
  // const char* gyC = gyStr.c_str();
  // const char* gzC = gzStr.c_str();

  // appendFile(SD, "/Rocket001.txt", currentMillisC); appendFile(SD, "/Rocket001.txt", "\t");
  // appendFile(SD, "/Rocket001.txt", altC); appendFile(SD, "/Rocket001.txt", "\t");
  // appendFile(SD, "/Rocket001.txt", accelX_m_s2C); appendFile(SD, "/Rocket001.txt", "\t");
  // appendFile(SD, "/Rocket001.txt", accelY_m_s2C); appendFile(SD, "/Rocket001.txt", "\t");
  // appendFile(SD, "/Rocket001.txt", accelZ_m_s2C); appendFile(SD, "/Rocket001.txt", "\t");
  // appendFile(SD, "/Rocket001.txt", gxC); appendFile(SD, "/Rocket001.txt", "\t");
  // appendFile(SD, "/Rocket001.txt", gyC); appendFile(SD, "/Rocket001.txt", "\t");
  //appendFile(SD, "/Rocket001.txt", gzC); appendFile(SD, "/Rocket001.txt", "\n");

  String dataString = "";
  dataString += String(millis()).c_str();
  dataString += "\t";
  dataString += String(bmp.readAltitude(101500)).c_str();
  dataString += "\t";
  dataString += String(ax).c_str();
  dataString += "\t";
  dataString += String(ay).c_str();
  dataString += "\t";
  dataString += String(az).c_str();
  dataString += "\t";
  dataString += String(gx).c_str();
  dataString += "\t";
  dataString += String(gy).c_str();
  dataString += "\t";
  dataString += String(gz).c_str();
  //dataString += "\n";

  dataFile.println(dataString);

  dataFile.flush();



// #ifdef OUTPUT_READABLE_ACCELGYRO
//   // display tab-separated accel/gyro x/y/z values
//   //Serial.print("a/g:\t");
//   // Serial.print("ax");
//   Serial.print(ax);
//   Serial.print("\t");
//   //Serial.print("ay");
//   Serial.print(ay);
//   Serial.print("\t");
//   //Serial.print("az");
//   Serial.print(az);
//   Serial.print("\t");
//   //Serial.print("gx");
//   Serial.print(gx);
//   Serial.print("\t");
//   //Serial.print("gy");
//   Serial.print(gy);
//   Serial.print("\t");
//   //Serial.print("gz");
//   Serial.print(gz);
//   Serial.print("\t");
// #endif
  
// #ifdef OUTPUT_BINARY_ACCELGYRO
//   Serial.write((uint8_t)(ax >> 8));
//   Serial.write((uint8_t)(ax & 0xFF));
//   Serial.write((uint8_t)(ay >> 8));
//   Serial.write((uint8_t)(ay & 0xFF));
//   Serial.write((uint8_t)(az >> 8));
//   Serial.write((uint8_t)(az & 0xFF));
//   Serial.write((uint8_t)(gx >> 8));
//   Serial.write((uint8_t)(gx & 0xFF));
//   Serial.write((uint8_t)(gy >> 8));
//   Serial.write((uint8_t)(gy & 0xFF));
//   Serial.write((uint8_t)(gz >> 8));
//   Serial.write((uint8_t)(gz & 0xFF));
// #endif

// Serial.println(bmp.readAltitude(101500));

  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
}