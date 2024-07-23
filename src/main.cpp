#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <esp_log.h>
#include <HardwareSerial.h>
#include <TaskScheduler.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

//Connections
#define grimm_RX 16
#define partector_RX 17
// BME280 Vin -> 3.3V; SCK -> SCL; SDI -> SDA; GND -> GND

//grimm varaiables
const int MAX_LINES = 4;
int dataIndex = 0;

// Struct for the sensor data
struct sensorData
{
  int grimmValues[34];
  int partectorNumber;  // Parts/cm³
  int partectorDiam;    // nm
  float partectorMass;  // µg/m³
  float temperature;    // °C
  float humidity;       // %
  float pressure;       // hPa
  float altitude;       // m
};

sensorData data;

// HW Serial for the Grimm sensor UART2
HardwareSerial grimmSerial(2);
// SW Serial for the partector2
EspSoftwareSerial::UART partectorSerial;

// BME280 sensor
Adafruit_BME280 bme; // I2C
#define SEALEVELPRESSURE_HPA (1013.25)

// Function prototypes
void processGrimmData(void);
void processPartectorData(void);
void processBMEData(void);

// TaskScheduler setup
Task t_Grimm2Struct(TASK_IMMEDIATE, TASK_FOREVER, &processGrimmData);
Task t_Partector2(TASK_IMMEDIATE, TASK_FOREVER, &processPartectorData);
Task t_BME280(1000, TASK_FOREVER, &processBMEData);
Scheduler runner;

void setup() {
  Serial.begin(9600);
  grimmSerial.begin(9600, SERIAL_8N1, grimm_RX, -1);
  partectorSerial.begin(38400, EspSoftwareSerial::SWSERIAL_8N1, partector_RX, -1, false, 256);

  //init BME280
  if (!bme.begin(0x77)) {
    ESP_LOGD("BME280", "Could not find a valid BME280 sensor, check wiring!");
  }

  // scheduler init
  runner.init();
  
  //runner.addTask(t_Grimm2Struct);
  runner.addTask(t_Partector2);
  runner.addTask(t_BME280);

  //t_Grimm2Struct.enable();
  t_Partector2.enable();
  t_BME280.enable();

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
  data.altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  #if CORE_DEBUG_LEVEL >= 4
    Serial.printf("Temperature: %f\n", data.temperature);
    Serial.printf("Humidity: %f\n", data.humidity);
    Serial.printf("Pressure: %f\n", data.pressure);
    Serial.printf("Altitude: %f\n", data.altitude);
  #endif
}