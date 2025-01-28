#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <esp_now.h>
// Complete Instructions: https://RandomNerdTutorials.com/esp32-digital-inputs-outputs-arduino/
const char *ssid = "iot";
const char *password = "iotisis;";

const char *mqtt_server = "192.168.3.151";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

// Adresse MAC du Sink
uint8_t broadcastAddress[] = {0xec, 0x62, 0x60, 0x5a, 0x45, 0xa0};
esp_now_peer_info_t peerInfo;

// Structure pour l'échange des données : Mote -> Sink
typedef struct struct_mote2sinkMessage
{
  int boardId;
  int readingId;
  int timeTag;
  char text[64];
  bool redLedState;    // Nouvel état pour la LED rouge
  bool yellowLedState; // Nouvel état pour la LED Jaune
} struct_mote2sinkMessage;

// Structure pour l'échange des données : Sink -> Mote
typedef struct struct_sink2moteMessage
{
  int boardId;
  bool bool0;
  char text[64];
  bool redLedState; // Nouvel état pour la LED rouge
  bool yellowLedState;
} struct_sink2moteMessage;

struct_sink2moteMessage espNow_incomingMessage;

// Variables pour la gestion du temps
unsigned long previousMillis = 0;
const long interval = 2000;
unsigned int readingId = 0;

// set pin numbers
const int RedbuttonPin = 27; // the number of the pushbutton pin
const int RedledPin = 25;    // the number of the LED pin
const int YellowButtonPin = 26;
const int YellowledPin = 13;
// variable for storing the pushbutton status
int RbuttonState = 0;
int YbuttonState = 0;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("Message envoyé à : ");
  for (int i = 0; i < 6; i++)
  {
    Serial.printf("%02X", mac_addr[i]);
    if (i < 5)
      Serial.print(":");
  }
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? " Succès" : " Échec");
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len)
{
  memcpy(&espNow_incomingMessage, incomingData, sizeof(espNow_incomingMessage));
  Serial.println("Message reçu du Sink :");
  Serial.printf("Board ID: %d, Bool: %d, Texte: %s\n", espNow_incomingMessage.boardId, espNow_incomingMessage.bool0, espNow_incomingMessage.text);
}

void setup()
{
  Serial.begin(115200);
  // initialize the pushbutton pin as an input
  pinMode(RedbuttonPin, INPUT);
  pinMode(YellowButtonPin, INPUT);
  // initialize the LED pin as an output
  pinMode(RedledPin, OUTPUT);
  pinMode(YellowledPin, OUTPUT);

  // Initialisation du Wi-Fi
  WiFi.mode(WIFI_STA);
  Serial.print("Adresse MAC de cette carte : ");
  Serial.println(WiFi.macAddress());

  // Initialisation d'ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Erreur lors de l'initialisation d'ESP-NOW");
    return;
  }

  // Enregistrement des callbacks
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // Configuration du peer (sink)
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0; // Canal par défaut
  peerInfo.encrypt = false;

  // Ajouter le peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Erreur lors de l'ajout du sink");
    return;
  }
}

void loop()
{
  // delay(1000);
  // Serial.println("Bip");
  //  read the state of the pushbutton value
  RbuttonState = digitalRead(RedbuttonPin);
  Serial.println(RbuttonState);

  YbuttonState = digitalRead(YellowButtonPin);
  Serial.println(YbuttonState);
  // check if the pushbutton is pressed.
  // if it is, the buttonState is HIGH
  // Mettre à jour les LEDs
  digitalWrite(RedledPin, RbuttonState);
  digitalWrite(YellowledPin, YbuttonState);

  // Créer un message pour le sink
  struct_mote2sinkMessage outgoingMessage;
  outgoingMessage.boardId = 1;                 // ID de la carte
  outgoingMessage.readingId = millis() / 1000; // Identifiant basé sur le temps
  outgoingMessage.timeTag = millis();
  outgoingMessage.redLedState = RbuttonState;    // Ajouter l'état de la LED rouge
  outgoingMessage.yellowLedState = YbuttonState; // Ajouter l'état de la LED jaune
                                                 // Timestamp
  // if (RbuttonState == HIGH && YbuttonState == HIGH)
  // {
  //   strcpy(outgoingMessage.text, "Les deux boutons sont pressés !");

  // }
  // else if (RbuttonState == HIGH)
  // {
  //   strcpy(outgoingMessage.text, "Bouton rouge pressé.");
  // }
  // else if (YbuttonState == HIGH)
  // {
  //   strcpy(outgoingMessage.text, "Bouton jaune pressé.");
  // }
  // else
  // {
  //   strcpy(outgoingMessage.text, "Aucun bouton pressé.");
  // }

  // Envoyer le message au sink
  esp_now_send(broadcastAddress, (uint8_t *)&outgoingMessage, sizeof(outgoingMessage));

  // Attendre 500 ms
  delay(500);
}