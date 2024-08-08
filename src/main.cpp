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
#include <RF24.h>
#include <RF24Network.h>
#include <RF24Mesh.h>

// PIN Connections ----------------------------------------------------------
#define grimm_RX 16
#define partector_RX 17
#define rf24_CE_PIN 40
#define rf24_CSN_PIN 39
// BME280 Vin -> 3.3V; SCK -> SCL; SDI -> SDA; GND -> GND
//---------------------------------------------------------------------------

//GRIMM varaiables ----------------------------------------------------------
const int MAX_LINES = 4;
int dataIndex = 0;
//---------------------------------------------------------------------------

// Struct for the sensor data------------------------------------------------
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

// RF24 Mesh Network ---------------------------------------------------------
RF24 radio(rf24_CE_PIN, rf24_CSN_PIN);
RF24Network network(radio);
RF24Mesh mesh(radio, network);

// Function prototypes -------------------------------------------------------
void processGrimmData(void);
void processPartectorData(void);
void processBMEData(void);
void processCO2Data(void);
void drawData2OLED(void);
void send2Mesh(void);
//----------------------------------------------------------------------------

// TaskScheduler setup -------------------------------------------------------
Task t_Grimm2Struct(TASK_IMMEDIATE, TASK_FOREVER, &processGrimmData);
Task t_Partector2(TASK_IMMEDIATE, TASK_FOREVER, &processPartectorData);
Task t_BME280(1000, TASK_FOREVER, &processBMEData);
Task t_CO2(1000, TASK_FOREVER, &processCO2Data); //CO2 Data is only available every 2 seconds
Task t_OLED(1000, TASK_FOREVER, &drawData2OLED);
Task t_Mesh(1000, TASK_FOREVER, &send2Mesh);
Scheduler runner;
//----------------------------------------------------------------------------

void setup() {
  Serial.begin(9600);
  grimmSerial.begin(9600, SERIAL_8N1, grimm_RX, -1);
  partectorSerial.begin(38400, EspSoftwareSerial::SWSERIAL_8N1, partector_RX, -1, false, 256);

  Wire.begin();

  u8g2.begin();
  u8g2.setFont(u8g2_font_tiny5_tr);
  u8g2.clearBuffer();

  //init BME280
  if (!bme.begin(0x77)) {
    ESP_LOGD("BME280", "Could not find a valid BME280 sensor, check wiring!");
  }

  //scd30 init
  if (co2.begin() == false) {
    ESP_LOGD("SCD30", "Sensor not detected. Please check your wiring.");
  }

  //init RF24
  mesh.setNodeID(1);
  radio.begin();
  radio.setPALevel(RF24_PA_MAX);
  if(!mesh.begin(MESH_DEFAULT_CHANNEL, RF24_250KBPS)) {
    if (radio.isChipConnected()) {
      do {
        ESP_LOGD("RF24MESH", "Could not connect to mesh network. Retrying...");
      } while (mesh.renewAddress() == MESH_DEFAULT_ADDRESS);
    } else {
      ESP_LOGD("RF24MESH", "Radio not connected. Please check your wiring.");
    }
  }

  // scheduler init
  runner.init();
  
  //runner.addTask(t_Grimm2Struct);
  runner.addTask(t_Partector2);
  runner.addTask(t_BME280);
  runner.addTask(t_CO2);
  runner.addTask(t_OLED);
  runner.addTask(t_Mesh);

  //t_Grimm2Struct.enable();
  t_Partector2.enable();
  t_BME280.enable();
  t_CO2.enable();
  t_OLED.enable();
  t_Mesh.enable();

}

void loop() {

  mesh.update();
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

void processCO2Data(void) {
  if (co2.dataAvailable()) {
    data.co2 = co2.getCO2();
    ESP_LOGD("CO2", "CO2: %d ppm", data.co2);
  }
}

void drawData2OLED(void) {
  u8g2.clearBuffer();
  u8g2.setDisplayRotation(U8G2_R1);
  u8g2.setCursor(0, 10);
  u8g2.print("Temperature:");
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
  u8g2.sendBuffer();
}

// Send data to the mesh network
void send2Mesh(void) {
  if (!mesh.write(&data, 'A', sizeof(data))) {
    if (!mesh.checkConnection()) {
      ESP_LOGD("send2Mesh", "Lost connection to mesh network. Trying to renew address...");
      if (mesh.renewAddress() == MESH_DEFAULT_ADDRESS) {
        mesh.begin(MESH_DEFAULT_CHANNEL, RF24_250KBPS);
      }
    } else {
      ESP_LOGD("send2Mesh", "Send fail, Test OK");
    }
  } else {
    ESP_LOGD("send2Mesh", "Send success");
  }
}