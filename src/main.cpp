#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
// Complete Instructions: https://RandomNerdTutorials.com/esp32-digital-inputs-outputs-arduino/
const char *ssid = "iot";
const char *password = "iotisis;";

const char *mqtt_server = "192.168.3.151";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

// set pin numbers
const int RedbuttonPin = 26; // the number of the pushbutton pin
const int RedledPin = 9;     // the number of the LED pin
const int YellowButtonPin = 6;
const int YellowledPin = 7;
// variable for storing the pushbutton status
int RbuttonState = 0;
int YbuttonState = 0;
void setup()
{
  Serial.begin(115200);
  // initialize the pushbutton pin as an input
  pinMode(RedbuttonPin, INPUT);
  pinMode(YellowButtonPin, INPUT);
  // initialize the LED pin as an output
  pinMode(RedledPin, OUTPUT);
  pinMode(YellowledPin, OUTPUT);
}

void loop()
{
  delay(1000);
  Serial.println("Bip");
  // read the state of the pushbutton value
  RbuttonState = digitalRead(RedbuttonPin);
  Serial.println(RbuttonState);

  YbuttonState = digitalRead(YellowButtonPin);
  Serial.println(YbuttonState);
  // check if the pushbutton is pressed.
  // if it is, the buttonState is HIGH
  if (RbuttonState == HIGH)
  {
    // turn LED on
    digitalWrite(RedledPin, HIGH);
  }
  else
  {
    // turn LED off
    digitalWrite(RedledPin, LOW);
  }

  if (YbuttonState == HIGH)
  {
    // turn LED on
    digitalWrite(YellowledPin, HIGH);
  }
  else
  {
    // turn LED off
    digitalWrite(YellowledPin, LOW);
  }
}