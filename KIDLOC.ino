#include <EEPROM.h>
#define EEPROM_SIZE 512  // EEPROM storage size
#define ADDR1 0          // Start address for first string
#define ADDR2 100        // Start address for second string (Ensure enough space)
#define ADDR3 150

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET -1
Adafruit_SSD1306 display(OLED_RESET);
#if (SSD1306_LCDHEIGHT != 32)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

#define SerialAT Serial2

#include <TinyGPSPlus.h>
TinyGPSPlus gps;
float latn = 0, longn = 0;

#define GPSRX 25
#define GPSTX 26
#define GSMRX 16
#define GSMTX 17
HardwareSerial SerialGSM(2);
HardwareSerial SerialGPS(1);

#include <millisDelay.h>
millisDelay readDelay;
millisDelay gpsDelay;

#include "BluetoothSerial.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"

BluetoothSerial SerialBT;
String device_name = "KidLoc";

esp_bd_addr_t remoteAddress = {0, 0, 0, 0, 0, 0};  // ✅ Dynamic MAC Address
const int btnSend = 19;                     

int setMtrs;
float distnce;
String names, phone, phone2, datasend, msg, devnum;

void setup() {

  Serial.begin(115200);
  //  SerialGSM.begin(9600, SERIAL_8N1, GSMRX, GSMTX);
  SerialAT.begin(115200, SERIAL_8N1, GSMRX, GSMTX);
  SerialGPS.begin(9600, SERIAL_8N1, GPSRX, GPSTX);
  SerialBT.begin(device_name);

  EEPROM.begin(EEPROM_SIZE);
  pinMode(btnSend, INPUT_PULLUP);

  esp_bt_gap_register_callback(bt_gap_callback);
  Serial.println("🔵 Waiting for Bluetooth connection...");

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(30, 1); display.print("Initializing");
  display.setCursor(35, 18); display.print("Please Wait");
  display.display(); delay(100);

  //  delay(10000);
  initGSM();

  //  tests();
  for (int x = 0; x < 5; x++) {
    getGPS();
  }
  distnce = -100.0;
  setMtrs = 100;
  readDelay.start(5000);

  // Read stored data from EEPROM
  names = EEPROM.readString(ADDR1);
  phone = EEPROM.readString(ADDR2);
  phone2 = EEPROM.readString(ADDR3);
  Serial.print("E-Name: "); Serial.println(names);
  Serial.print("E-Phone: "); Serial.println(phone);
  Serial.print("Phone3 : "); Serial.println(phone);

  display.clearDisplay();
  display.setCursor(5, 1); display.print(names);
  display.setCursor(5, 18); display.print(phone);
  display.display();

  // ✅ Get the MAC address of the bonded Android device
  getBondedDevices();

  msg = "KidLoc Alert: Your child is OUTSIDE the safezone. Check your child's location now!";
  gpsDelay.start(1000);
}

void loop() {
  if (Serial.available()) {
    String receivedData = Serial.readString();
    receivedData.trim();
    Serial.print("📩 Received: "); Serial.println(receivedData);
    SerialBT.print(receivedData);
  }

  if (SerialBT.available()) {
    String receivedData = SerialBT.readString();
    receivedData.trim();
    Serial.print("📩 Received: "); Serial.println(receivedData);

    if (receivedData.indexOf("SET") >= 0) {
      setMtrs = parseMeters(receivedData);
      Serial.print("Set Meters: "); Serial.println(setMtrs);
    }
    else if (receivedData.indexOf("INFO") >= 0) {
      parseInfo(receivedData, names, phone, phone2, devnum);
      Serial.print("Name: "); Serial.println(names);
      Serial.print("Phone: "); Serial.println(phone);
      Serial.print("Phone2: "); Serial.println(phone2);
      Serial.print("DevNum: "); Serial.println(devnum);

      display.clearDisplay();
      display.setCursor(5, 1); display.print(names);
      display.setCursor(5, 18); display.print(phone);
      display.display();

      EEPROM.writeString(ADDR1, names);
      EEPROM.writeString(ADDR2, phone);
      EEPROM.writeString(ADDR3, phone2);
      EEPROM.commit();
    }
    else if (receivedData.indexOf("GET_MAC") >= 0) {
      getBondedDevices();
    }
  }

  if (digitalRead(btnSend) == 0) {
    delay(50); while (digitalRead(btnSend) == 0);
    Serial.println("Pressed");
    Serial.println(msg);
    Serial.println(phone);
    String newMsg = "KidLoc Alert: Your child has pressed the SOS button. Immediate action needed! ";
    newMsg += String(latn, 6) + "" + String(longn, 6);
    smsSend(newMsg, phone); delay(5000);
    smsSend(newMsg, phone2); delay(5000);
  }

  if (readDelay.justFinished()) {
    getRSSI();
    readDelay.start(2000);
  }
  if (gpsDelay.justFinished()) {
    if (distnce > setMtrs) {
      latn = 0;
      while (latn == 0) {
        getGPS();
        datasend = "GPS_" + String(latn, 6) + "_" + String(longn, 6) + "_X";
        Serial.println(datasend);
        Serial.println(phone);
        smsSend(msg, phone); delay(5000);
        smsSend(datasend, phone); delay(2000);
      }
      gpsDelay.start(30000);
    }
    else {
      gpsDelay.start(2000);
    }
  }
}

// ✅ Function to get bonded Bluetooth devices and update remoteAddress
void getBondedDevices() {
  int dev_num = esp_bt_gap_get_bond_device_num();
  esp_bd_addr_t pairedDeviceList[dev_num];

  if (esp_bt_gap_get_bond_device_list(&dev_num, pairedDeviceList) == ESP_OK) {
    Serial.print("Bonded Devices: ");
    Serial.println(dev_num);

    if (dev_num > 0) {
      char macStr[18];
      sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
              pairedDeviceList[0][0], pairedDeviceList[0][1], pairedDeviceList[0][2],
              pairedDeviceList[0][3], pairedDeviceList[0][4], pairedDeviceList[0][5]);

      Serial.print("Connected MAC: ");
      Serial.println(macStr);

      SerialBT.print("MAC:");
      SerialBT.println(macStr);

      // ✅ Update remoteAddress for RSSI tracking
      memcpy(remoteAddress, pairedDeviceList[0], sizeof(esp_bd_addr_t));
    } else {
      Serial.println("No paired devices found.");
    }
  } else {
    Serial.println("Failed to get bonded devices.");
  }
}

int parseMeters(String data) {
  if (data.startsWith("SET_") && data.endsWith("_X")) {
    // Remove "SET_" and "_X"
    data = data.substring(4, data.length() - 2);
    return data.toInt(); // Convert to integer
  }
  return -1; // Return -1 if format is incorrect
}
bool parseInfo(String data, String &name, String &phone1, String &phone2 , String &phone3) {
  if (data.startsWith("INFO_") && data.endsWith("_X")) {
    data = data.substring(5, data.length() - 2); // Remove "INFO_" and "_X"

    int firstSep = data.indexOf('_'); // Find first underscore
    int secondSep = data.indexOf('_', firstSep + 1); // Find second underscore
    int thirdSep = data.indexOf('_', secondSep + 1);

    if (firstSep != -1 && secondSep != -1 && thirdSep != -1) {
      name = data.substring(0, firstSep); // Extract name
      phone1 = data.substring(firstSep + 1, secondSep); // Extract first phone number
      phone2 = data.substring(secondSep + 1, thirdSep); // Extract second phone number
      phone3 = data.substring(thirdSep + 1); // Extract device number
      return true;
    }
  }
  return false; // Return false if format is incorrect
}
// ✅ RSSI Callback
void bt_gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
  if (event == ESP_BT_GAP_READ_RSSI_DELTA_EVT) {
    int8_t rssi_value = param->read_rssi_delta.rssi_delta;
    Serial.print("📶 RSSI: "); Serial.println(rssi_value);
    distnce = calculateDistance(rssi_value);
    //    distnce = abs(float(rssi_value) / 5.5); // calaculate distance through rssi
    Serial.print("Distance: "); Serial.println(distnce);
  }
}
float calculateDistance(int rssi) {
  int txPower = -12; float n = 2.2;
  return pow(10, (txPower - rssi) / (10 * n));
}
// ✅ Function to Request RSSI using updated remoteAddress
void getRSSI() {
  esp_err_t result = esp_bt_gap_read_rssi_delta(remoteAddress);
  if (result != ESP_OK) {
    Serial.println("❌ Failed to get RSSI");
  }
}

void tests()
{
  while (1) {
    latn = 0; int ctr = 0;
    while (latn == 0) {
      getGPS();
    }
  }
}
