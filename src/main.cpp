#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <esp_log.h>
#include <HardwareSerial.h>
#include <TaskScheduler.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SparkFun_SCD30_Arduino_Library.h>
#include <U8g2lib.h>
#include <LoRa.h>
#include <SdFat.h>
#include <Adafruit_GPS.h>

// PIN Connections ----------------------------------------------------------
#define grimm_RX 16       // Grimm RX
#define partector_RX 17   // Partector RX
//RFM95W LoRa Module
#define BAND 866E6
#define LORA_DIO0 2
#define LORA_RST 3
#define LORA_CS 8
//SD CS PIN (SPI)
#define SD_CS_PIN 46
//GPS RX & TX PINS
#define GPS_RX 4 // µC RX not GPS RX (GPS TX -> µC RX)
#define GPS_TX 5 // µC TX not GPS TX (GPS RX -> µC TX)
// BME280 Vin -> 3.3V; SCK -> SCL; SDI -> SDA; GND -> GND
//---------------------------------------------------------------------------

//GRIMM varaiables ----------------------------------------------------------
const int MAX_LINES = 4;
int dataIndex = 0;
bool newGrimmValuesReceived = false;
//---------------------------------------------------------------------------

// Struct for the sensor data------------------------------------------------
struct sensorData
{
  int grimmValues[31];
  int partectorNumber;  // Parts/cm³
  int partectorDiam;    // nm
  float partectorMass;  // µg/m³
  float temperature;    // °C
  float humidity;       // %
  float pressure;       // hPa
  float altitude;       // m
  uint16_t co2;         // ppm
};

sensorData data;
//----------------------------------------------------------------------------

// HW Serial for the Grimm sensor UART2 --------------------------------------
HardwareSerial grimmSerial(2);
//----------------------------------------------------------------------------
// SW Serial for the partector2 ----------------------------------------------
EspSoftwareSerial::UART partectorSerial;
//----------------------------------------------------------------------------

// SW Serial for the GPS and object creation ---------------------------------
EspSoftwareSerial::UART gpsSerial;
Adafruit_GPS GPS(&gpsSerial);
#define GPSECHO false
bool newNMEAreceived = false;
bool parsingFailed = false;
//----------------------------------------------------------------------------

// BME280 sensor -------------------------------------------------------------
Adafruit_BME280 bme; // I2C
#define SEALEVELPRESSURE_HPA (1013.25)
//----------------------------------------------------------------------------

// SCD30 sensor --------------------------------------------------------------
SCD30 co2;
//----------------------------------------------------------------------------

// OLED Display --------------------------------------------------------------
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R2); //rotation R2
//----------------------------------------------------------------------------

// SD Card -------------------------------------------------------------------
const int8_t DISABLE_CHIP_SELECT = 8;
#define SPI_CLOCK SD_SCK_MHZ(50);
SdFat32 sd;
File32 file;

// Try to select the best SD card configuration.
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SPI_CLOCK)

void disableLoRaSPI() {
  pinMode(LORA_CS, OUTPUT);
  digitalWrite(LORA_CS, HIGH);
}


String filename = "";
//----------------------------------------------------------------------------

// Function prototypes -------------------------------------------------------
void processGrimmData(void);
void processPartectorData(void);
void processBMEData(void);
void processCO2Data(void);
void drawData2OLED(void);
void sendLoRaData(void);
void writeData2SD(void);
float gpsDecimal(float gps, char sector);
void gpsRunFrequently(void);
void gpsWait4Fix(void);
String getFilename(void);
void oledLoraErr(void);                     // Display error message on OLED
void oledSDErr(void);
void oledBMEErr(void);
void oledCO2Err(void);
//----------------------------------------------------------------------------

// TaskScheduler setup -------------------------------------------------------
Task t_Grimm2Struct(TASK_IMMEDIATE, TASK_FOREVER, &processGrimmData);
Task t_Partector2(TASK_IMMEDIATE, TASK_FOREVER, &processPartectorData);
Task t_BME280(1000, TASK_FOREVER, &processBMEData);
Task t_CO2(1000, TASK_FOREVER, &processCO2Data); //CO2 Data is only available every 2 seconds
Task t_OLED(1000, TASK_FOREVER, &drawData2OLED);
Task t_LoRa(2000, TASK_FOREVER, &sendLoRaData);
Task t_SD(1000, TASK_FOREVER, &writeData2SD);
Task t_gpsRunFrequently(TASK_IMMEDIATE, TASK_FOREVER, &gpsRunFrequently);
Scheduler runner;
//----------------------------------------------------------------------------

void setup() {
  Serial.begin(9600);
  grimmSerial.begin(9600, SERIAL_8N1, grimm_RX, -1);
  partectorSerial.begin(38400, EspSoftwareSerial::SWSERIAL_8N1, partector_RX, -1, false, 256);
  gpsSerial.begin(9600, EspSoftwareSerial::SWSERIAL_8N1, GPS_RX, GPS_TX, false, 256);

  //OLED
  Wire.begin();
  u8g2.begin();
  u8g2.setFont(u8g2_font_tiny5_tr);
  u8g2.clearBuffer();

  //GPS
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
  delay(1000);
  gpsSerial.println(PMTK_Q_RELEASE);
  ESP_LOGD("GPS", "GPS initialized, waiting for fix!");

  //wait for GPS fix
  gpsWait4Fix();

  //LoRa init
  SPI.begin(SCK, MISO, MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(BAND)) {
    ESP_LOGD("LoRa init", "Starting LoRa failed!");
    oledLoraErr();
  }
  ESP_LOGD("LoRa init", "LoRa init done!");
  //set LoRa syncWord
  LoRa.setSyncWord(0xB2);
  //enable CRC check
  LoRa.enableCrc();
  LoRa.setTxPower(20, PA_OUTPUT_PA_BOOST_PIN);

  //init BME280
  if (!bme.begin(0x77)) {
    ESP_LOGD("BME280", "Could not find a valid BME280 sensor, check wiring!");
    oledBMEErr();
  }

  //scd30 init
  if (co2.begin() == false) {
    ESP_LOGD("SCD30", "Sensor not detected. Please check your wiring.");
    oledCO2Err();
  }

  //SD Card init
  filename = getFilename();
  if (!sd.begin(SD_CS_PIN, SPI_FULL_SPEED)) {
    ESP_LOGD("SD Card", "SD Card init failed!");
    oledSDErr();
    sd.initErrorHalt();
    return;
  }

  if (sd.exists(filename.c_str())) {
    ESP_LOGD("SD Card checks", "File exists!");
  }
  else {
    // create file
    if (!file.open(filename.c_str(), O_WRONLY | O_CREAT | O_EXCL)) {
      ESP_LOGD("SD Card File Creation", "File creation failed!");
    } else {
      ESP_LOGD("SD Card File Creation", "File created!");
      String csvHeader = "Datetime,Latitude,Longitude,Temperature,Humidity,Pressure,Altitude,CO2,PartNum,PartDiam,PartMass,";
      for (int i(0); i < 31; ++i) {
        if (i == 30) {
          csvHeader += "Grimm" + String(i+1);
        } else {
          csvHeader += "Grimm" + String(i+1) + ",";
        }
      }
      file.println(csvHeader);
    }

    if (!file.sync() || file.getWriteError()) {
      ESP_LOGD("SD Card File Sync", "File sync failed!");
      oledSDErr();
    }
    file.close();
  }

  // scheduler init
  runner.init();
  
  runner.addTask(t_Grimm2Struct);
  runner.addTask(t_Partector2);
  runner.addTask(t_BME280);
  runner.addTask(t_CO2);
  runner.addTask(t_OLED);
  runner.addTask(t_LoRa);
  runner.addTask(t_SD);
  runner.addTask(t_gpsRunFrequently);

  t_Grimm2Struct.enable();
  t_Partector2.enable();
  t_BME280.enable();
  t_CO2.enable();
  t_OLED.enable();
  t_LoRa.enable();
  t_SD.enable();
  t_gpsRunFrequently.enable();

}

void loop() {

  runner.execute();
  
}

void processGrimmData(void) {
  if (grimmSerial.available()) {
    // Array leeren
    dataIndex = 0;
    
    // Variablen zur Überprüfung der 4 Zeilen und des Index
    bool validSet = true;
    char currentIndex = '0';
    int linesRead = 0;
    
    while (linesRead < MAX_LINES && validSet) {
      if (grimmSerial.available()) {
        String line = grimmSerial.readStringUntil('\n');
        line.trim(); // Entfernt führende und nachgestellte Leerzeichen
        ESP_LOGD("Grimm RAW Outputline", "%s", line.c_str());

        if (linesRead == 0) {
          // Überprüfen, ob die erste Zeile einer neuen Indexgruppe korrekt ist
          if (line.charAt(0) == 'C' && line.charAt(2) == ':') {
            currentIndex = line.charAt(1);
          } else {
            validSet = false;
            break;
          }
        } else {
          // Überprüfen, ob die folgenden Zeilen zur gleichen Indexgruppe gehören
          if (!((linesRead == 1 && line.charAt(0) == 'C' && line.charAt(2) == ';') ||
                (linesRead == 2 && line.charAt(0) == 'c' && line.charAt(2) == ':') ||
                (linesRead == 3 && line.charAt(0) == 'c' && line.charAt(2) == ';')) ||
              line.charAt(1) != currentIndex) {
            validSet = false;
            break;
          }
        }

        line.remove(0, 3); // Entfernt das Präfix "C0:", "C0;", "c0:" oder "c0;"
        
        // Verarbeitet die Zahlen in der Zeile
        int startIndex = 0;
        int endIndex = 0;
        int numberCount = 0;
        
        // Durchlaufe die Zeile und zähle die Zahlen
        while (endIndex < line.length()) {
          // Finde das Ende der aktuellen Zahl
          while (endIndex < line.length() && line.charAt(endIndex) != ' ') {
            endIndex++;
          }
          
          // Extrahiere die Zahl
          String numberStr = line.substring(startIndex, endIndex);
          numberStr.trim(); // Entfernt führende und nachgestellte Leerzeichen
          if (numberStr.length() > 0) {
            numberCount++;
          }
          
          // Setze den Startindex für die nächste Zahl
          startIndex = endIndex + 1;
          endIndex = startIndex;
          
          // Überspringe die Leerzeichen
          while (endIndex < line.length() && line.charAt(endIndex) == ' ') {
            endIndex++;
          }
        }

        // Zahlen erneut durchlaufen und in das Array einfügen
        startIndex = 0;
        endIndex = 0;
        int currentNumberCount = 0;
        
        while (endIndex < line.length()) {
          // Finde das Ende der aktuellen Zahl
          while (endIndex < line.length() && line.charAt(endIndex) != ' ') {
            endIndex++;
          }
          
          // Extrahiere die Zahl
          String numberStr = line.substring(startIndex, endIndex);
          numberStr.trim(); // Entfernt führende und nachgestellte Leerzeichen
          if (numberStr.length() > 0) {
            currentNumberCount++;
            int number = numberStr.toInt();
            // Letzte Zahl in der dritten Zeile (immer 160) und in der vierten Zeile (immer 0) überspringen
            if (!((linesRead == 2 && currentNumberCount == numberCount) ||
                  (linesRead == 3 && currentNumberCount == numberCount) ||
                  (linesRead == 2 && currentNumberCount == 1))) {
              data.grimmValues[dataIndex++] = number;
            }
          }
          
          // Setze den Startindex für die nächste Zahl
          startIndex = endIndex + 1;
          endIndex = startIndex;
          
          // Überspringe die Leerzeichen
          while (endIndex < line.length() && line.charAt(endIndex) == ' ') {
            endIndex++;
          }
        }
        
        // Überprüfe und entferne den doppelten Wert zwischen der zweiten und dritten Zeile
        //if (linesRead == 2 && dataIndex > 0 && data.grimmValues[dataIndex - 1] == data.grimmValues[dataIndex - 2]) {
        //  dataIndex--; // Entferne den doppelten ersten Wert der dritten Zeile
        //}
        
        linesRead++;
      }
    }
    // set flag for new data
    if (validSet && linesRead == MAX_LINES) {
      newGrimmValuesReceived = true;
    }
    // Debug-Ausgabe
    #if CORE_DEBUG_LEVEL >= 4
        if (validSet && linesRead == MAX_LINES) {
          // Ausgabe des Arrays über den seriellen Monitor
          for (int i = 0; i < dataIndex; i++) {
            Serial.printf("%d", data.grimmValues[i]);
            if (i < dataIndex - 1) {
              Serial.printf(", ");
            }
          }
          Serial.printf("\n");
          Serial.printf("Index: %d\n", dataIndex);
        } else {
          ESP_LOGD("Grimm Data", "Invalid data set\n");
        }
    #endif
  }
}

void processPartectorData(void) {
  if (partectorSerial.available()) {
/*     char c = partectorSerial.read();
    Serial.print(c, HEX);
    Serial.print(" ");
    if (c == '\n') {
      Serial.println();
    } */
    String line = partectorSerial.readStringUntil('\n');
    line.trim();
    ESP_LOGD("Partector RAW Outputline", "%s", line.c_str());

    //parse PartectorData
    int startIndex = 0;
    int endIndex = line.indexOf('\t');
    int index = 0;

    while(endIndex != -1 && index <=5) {
      String value = line.substring(startIndex, endIndex);
      value.trim();
      switch (index) {
        case 1: //number
          data.partectorNumber = value.toInt();
          break;
        case 2: //diameter
          data.partectorDiam = value.toInt();
          break;
        case 5: //mass
          data.partectorMass = value.toFloat();
          break;
      }
      startIndex = endIndex + 1;
      endIndex = line.indexOf('\t', startIndex);
      ++index;
    }
    #if CORE_DEBUG_LEVEL >= 4
      Serial.printf("Partector Number: %d\n", data.partectorNumber);
      Serial.printf("Partector Diameter: %d\n", data.partectorDiam);
      Serial.printf("Partector Mass: %f\n", data.partectorMass);
    #endif
  }
}

void processBMEData(void) {
  data.temperature = bme.readTemperature();
  data.humidity = bme.readHumidity();
  data.pressure = bme.readPressure() / 100.0F;
  //data.altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  data.altitude = GPS.altitude;
  #if CORE_DEBUG_LEVEL >= 4
    Serial.printf("Temperature: %f\n", data.temperature);
    Serial.printf("Humidity: %f\n", data.humidity);
    Serial.printf("Pressure: %f\n", data.pressure);
    Serial.printf("Altitude: %f\n", data.altitude);
  #endif
}

void processCO2Data(void) {
  if (co2.dataAvailable()) {
    data.co2 = co2.getCO2();
    ESP_LOGD("CO2", "CO2: %d ppm", data.co2);
  }
}

void drawData2OLED(void) {
  u8g2.clearBuffer();
  u8g2.setDisplayRotation(U8G2_R1);
  u8g2.setFont(u8g2_font_tiny5_tr);
  u8g2.setCursor(0, 10);
  u8g2.print("Temp:");
  u8g2.setCursor(0, 20);
  u8g2.print(data.temperature);
  u8g2.print("C");
  u8g2.setCursor(0, 30);
  u8g2.print("Humidity: ");
  u8g2.setCursor(0, 40);
  u8g2.print(data.humidity);
  u8g2.print("%");
  u8g2.setCursor(0, 50);
  u8g2.print("Partector");
  u8g2.setCursor(0, 60);
  u8g2.print(data.partectorNumber);
  u8g2.setCursor(0, 70);
  u8g2.print("CO2: ");
  u8g2.setCursor(0, 80);
  u8g2.print(data.co2);
  u8g2.print("ppm");
  u8g2.setCursor(0, 90);
  u8g2.print("Time:");
  u8g2.setCursor(0, 100);
  u8g2.print(GPS.hour);
  u8g2.print(":");
  u8g2.print(GPS.minute);
  u8g2.print(":");
  u8g2.print(GPS.seconds);
  u8g2.sendBuffer();
}

//send data via LoRa
void sendLoRaData(void) {
  LoRa.beginPacket();
  LoRa.write((uint8_t*)&data, sizeof(data));
  LoRa.endPacket();
}

void writeData2SD(void) {
  
  if (!file.open(filename.c_str(), O_WRONLY | O_CREAT | O_AT_END)) {
    ESP_LOGD("SDWrite Open File", "File open failed!");
  }

  // write data to file
  String sdData = String(GPS.year) + "." + String(GPS.month) + "." + String(GPS.day) + "T";
  sdData += String(GPS.hour) + "." + String(GPS.minute) + "." + String(GPS.seconds) + ",";
  sdData += String(gpsDecimal(GPS.latitude, GPS.lat), 6) + "," + String(gpsDecimal(GPS.longitude, GPS.lon), 6) + ",";
  sdData += String(data.temperature) + "," + String(data.humidity) + "," + String(data.pressure) + "," + String(data.altitude) + "," + String(data.co2) + "," + String(data.partectorNumber) + "," + String(data.partectorDiam) + "," + String(data.partectorMass) + ",";
  if (newGrimmValuesReceived) {
    for (int i(0); i < 31; ++i) {
      if (i == 30) {
        sdData += String(data.grimmValues[i]);
      } else {
        sdData += String(data.grimmValues[i]) + ",";
      }
    }
    newGrimmValuesReceived = false;
  } else {
    for (int i(0); i < 30; ++i) {
      sdData += ",";
    }
  }
  file.println(sdData);

  if (!file.sync() || file.getWriteError()) {
    ESP_LOGD("SDWrite Sync File", "File sync or write failed!");
  }

  file.close();
  ESP_LOGD("SDWrite", "Data written to SD Card!");

}

float gpsDecimal(float gps, char sector) {
  float decimal = 0;
  int degrees = int(gps / 100);
  float minutes = gps - (degrees * 100);
  decimal = degrees + (minutes / 60);
  if (sector == 'S' || sector == 'W') {
    decimal *= -1;
  }
  return decimal;
}

void gpsRunFrequently(void) {
  char c = GPS.read();
  if (GPS.newNMEAreceived()) {
    newNMEAreceived = true;
    if (!GPS.parse(GPS.lastNMEA())) {
      parsingFailed = true;
    }
    else {
      parsingFailed = false;
    }
  }
}

//wait for gps fix
void gpsWait4Fix(void) {
  u8g2.clearBuffer();
  u8g2.setDisplayRotation(U8G2_R2);
  u8g2.setFont(u8g2_font_logisoso28_tr);
  u8g2.drawStr(4, 29, "WAIT4FIX");
  u8g2.sendBuffer();
  while (!GPS.fix) {
    gpsRunFrequently();
  }
}

//display lora err
void oledLoraErr(void) {
  u8g2.clearBuffer();
  u8g2.setDisplayRotation(U8G2_R2);
  u8g2.setFont(u8g2_font_logisoso28_tr);
  u8g2.drawStr(4, 29, "LORA ERR");
  u8g2.sendBuffer();
}

//display sd err
void oledSDErr(void) {
  u8g2.clearBuffer();
  u8g2.setDisplayRotation(U8G2_R2);
  u8g2.setFont(u8g2_font_logisoso28_tr);
  u8g2.drawStr(4, 29, "SD ERR");
  u8g2.sendBuffer();
}

//display bme280 err
void oledBMEErr(void) {
  u8g2.clearBuffer();
  u8g2.setDisplayRotation(U8G2_R2);
  u8g2.setFont(u8g2_font_logisoso28_tr);
  u8g2.drawStr(4, 29, "BME ERR");
  u8g2.sendBuffer();
}

//display co2 err
void oledCO2Err(void) {
  u8g2.clearBuffer();
  u8g2.setDisplayRotation(U8G2_R2);
  u8g2.setFont(u8g2_font_logisoso28_tr);
  u8g2.drawStr(4, 29, "CO2 ERR");
  u8g2.sendBuffer();
}

//get datetime for filename
String getFilename(void) {
  String filename = "";
  filename += String(GPS.year)+ "-" + String(GPS.month)+ "-" + String(GPS.day) + "T" + String(GPS.hour) + "-" + String(GPS.minute) + "-" + String(GPS.seconds) + ".csv";
  return filename;
}