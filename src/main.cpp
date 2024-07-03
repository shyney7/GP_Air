#include <Arduino.h>
#include <SPI.h>
#include <HardwareSerial.h>

const int BUFFER_SIZE = 100;
const int MAX_LINES = 4;
int dataArray[BUFFER_SIZE];
int dataIndex = 0;

struct sensorData
{
  int values[34];
};

sensorData grimmData;
void printSensorData(const sensorData &data);


HardwareSerial grimmSerial(2);

void setup() {
  Serial.begin(9600);
  grimmSerial.begin(9600, SERIAL_8N1, 16, -1);
}

void loop() {

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
        Serial.println(line);

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
              dataArray[dataIndex++] = number;
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
        //if (linesRead == 2 && dataIndex > 0 && dataArray[dataIndex - 1] == dataArray[dataIndex - 2]) {
        //  dataIndex--; // Entferne den doppelten ersten Wert der dritten Zeile
        //}
        
        linesRead++;
      }
    }
    
    if (validSet && linesRead == MAX_LINES) {
      // Ausgabe des Arrays über den seriellen Monitor
      for (int i = 0; i < dataIndex; i++) {
        Serial.print(dataArray[i]);
        if (i < dataIndex - 1) {
          Serial.print(", ");
        }
      }
      Serial.println();
    } else {
      Serial.println("Ungültiges Datenset");
    }
  }
  
}

void printSensorData(const sensorData &data) {
  Serial.println("Sensor Data:");
  for (int i = 0; i < sizeof(data.values) / sizeof(data.values[0]); i++) {
    Serial.print(data.values[i]);
    Serial.print(", ");
  }
  Serial.println();
}