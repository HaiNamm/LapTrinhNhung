#define BLYNK_PRINT Serial
#define BLYNK_TEMPLATE_ID "TMPL6Gm-Q4VNU"
#define BLYNK_TEMPLATE_NAME "Nam"
#define BLYNK_AUTH_TOKEN "PqMSQD9TUdDPV1CeGX8pGhvKXQLQR_lA"

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "Hai Com Chim Non";
char pass[] = "23032001";

const int relayPin = D2; // Update this with the correct GPIO pin
int button;

void setup() {
  Serial.begin(9600);
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  pinMode(relayPin, OUTPUT);
}

BLYNK_WRITE(V1) {
  button = param.asInt();
  if(button == 1){
    digitalWrite(relayPin, HIGH);
  } 
  else{
    digitalWrite(relayPin, LOW);
  }
}

void loop() {
  Blynk.run();
}