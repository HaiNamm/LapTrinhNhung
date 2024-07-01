#include <ESP8266WiFi.h>
#include <FirebaseESP8266.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>

const char* WIFI_SSID = "Hai Com Chim Non";
const char* WIFI_PASSWORD = "23032001";
const char* FIREBASE_HOST = "https://smarthome-555d2-default-rtdb.firebaseio.com";
const char* FIREBASE_AUTH = "AIzaSyCZu4mztRcXR8X7Mf_ccbZMPrG9hq8_jh4";

const byte RX = D6;
const byte TX = D7;
SoftwareSerial mySerial = SoftwareSerial(RX, TX);

FirebaseData firebaseData;
FirebaseConfig config;
FirebaseAuth auth;

long lastUART = 0;

void setup() {
  Serial.begin(9600);
  mySerial.begin(19200);

  // Kết nối WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected to WiFi with IP Address: ");
  Serial.println(WiFi.localIP());

  // Kết nối Firebase
  config.host = FIREBASE_HOST;
  config.signer.tokens.legacy_token = FIREBASE_AUTH;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  String path = "/";
  if (!Firebase.beginStream(firebaseData, path)) {
    Serial.print("REASON: ");
    Serial.println(firebaseData.errorReason());
  }

  Serial.println("UART Start");
  lastUART = millis();
}

void loop() {
  Read_Uart_STM32();           // Đọc dữ liệu nhận được từ STM32 và gửi lên Firebase
  Read_Firebase_Send_STM32();  // Lấy dữ liệu từ Firebase và gửi về STM32
}

void Read_Uart_STM32() {
  static String receivedData = "";
  while (mySerial.available()) {
    char inChar = (char)mySerial.read();
    if (inChar == '\n') {
      Serial.println("Data received: " + receivedData);  //Nhận dữ liệu từ STM32

      // Phân tích chuỗi để lấy giá trị độ ẩm
      int index = receivedData.indexOf("Soil Moisture: ");
      if (index != -1) {
        // Cắt chuỗi để lấy phần chứa số
        String moistureStr = receivedData.substring(index + strlen("Soil Moisture: "));
        moistureStr.trim();  // This modifies moistureStr in place and does not return a value
        // Gửi giá trị độ ẩm lên Firebase
        String path = "/hhainam/Plant/SoilMoisture";
        if (!Firebase.setInt(firebaseData, path, moistureStr.toInt())) {
          Serial.print("Firebase error: ");
          Serial.println(firebaseData.errorReason());
        }
      }

      // Phân tích chuỗi để thông báo cảm biến mưa
      if (receivedData.indexOf("Rain_DETECTED") != -1) {
        String rainPath = "/hhainam/Rain/Rained";
        Firebase.setString(firebaseData, rainPath, "Rain_DETECTED");
      }

      // Phân tích chuỗi để thông báo cảm biến lửa
      if (receivedData.indexOf("Fire_DETECTED") != -1) {
        String firePath = "/hhainam/Kitchen/Fire";
        Firebase.setString(firebaseData, firePath, "Rain_DETECTED");
      }

      // Phân tích chuỗi để thông báo cảm biến gas
      if (receivedData.indexOf("Gas_DETECTED") != -1) {
        String gasPath = "/hhainam/Kitchen/Gas";
        Firebase.setString(firebaseData, gasPath, "Gas_DETECTED");
      }

      // Phân tích chuỗi để thông báo cảm biến lưu lượng nước
      int flowIndex = receivedData.indexOf("Water Flow: ");
      if (flowIndex != -1) {
        String flowStr = receivedData.substring(flowIndex + strlen("Water Flow: "));
        flowStr.trim();
        String flowPath = "/hhainam/Plant/WaterFlow";
        Firebase.setFloat(firebaseData, flowPath, flowStr.toFloat());
      }

      //Lấy dữ liệu về
      receivedData = "";  // Reset chuỗi sau khi xử lý
    } else {
      receivedData += inChar;  // Thêm ký tự vào chuỗi
    }
  }
}

void Read_Firebase_Send_STM32() {
  String allValues = "";
  //Hệ thống cổng
  String relayPath = "/hhainam/Gate/Gate1";
  if (Firebase.getString(firebaseData, relayPath)) {
    allValues += firebaseData.stringData();
  }

  //Hệ thống tưới cây
  String relayPath1 = "/hhainam/Plant/Mode";
  String relayPath2 = "/hhainam/Plant/Relay1";
  if (Firebase.getString(firebaseData, relayPath1)) {
    allValues += firebaseData.stringData();
  }
  if (Firebase.getString(firebaseData, relayPath2)) {
    allValues += firebaseData.stringData();
  }

  //Hệ thống giàn phơi
  String relayPath3 = "/hhainam/Rain/Mode1";
  String relayPath4 = "/hhainam/Rain/SWD";
  String relayPath5 = "/hhainam/Rain/SWM";
  if (Firebase.getString(firebaseData, relayPath3)) {
    allValues += firebaseData.stringData();
  }
  if (Firebase.getString(firebaseData, relayPath4)) {
    allValues += firebaseData.stringData();
  }
  if (Firebase.getString(firebaseData, relayPath5)) {
    allValues += firebaseData.stringData();
  }

  //Hệ thống đèn
  String relayPath6 = "/hhainam/LED/Garden";
  String relayPath7 = "/hhainam/LED/Lobby";
  String relayPath8 = "/hhainam/LED/Stair";
  String relayPath9 = "/hhainam/LED/Kitchen";
  String relayPath10 = "/hhainam/LED/PN";
  if (Firebase.getString(firebaseData, relayPath6)) {
    allValues += firebaseData.stringData();
  }
  if (Firebase.getString(firebaseData, relayPath7)) {
    allValues += firebaseData.stringData();
  }
  if (Firebase.getString(firebaseData, relayPath8)) {
    allValues += firebaseData.stringData();
  }
  if (Firebase.getString(firebaseData, relayPath9)) {
    allValues += firebaseData.stringData();
  }
  if (Firebase.getString(firebaseData, relayPath10)) {
    allValues += firebaseData.stringData();
  }

  mySerial.print(allValues);
  Serial.print("Truyen :");
  Serial.println(allValues);
}