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
  float temperature;
  float humidity;
  float distance;
  bool redLedState;    // Nouvel état pour la LED rouge
  bool yellowLedState; // Nouvel état pour la LED Jaune
  char text[64];
} struct_mote2sinkMessage;

// Structure pour l'échange des données : Sink -> Mote
typedef struct struct_sink2moteMessage
{
  int boardId;
  int readingId;
  int timeTag;
  float temperature;
  float humidity;
  float distance;
  bool redLedState;    // Nouvel état pour la LED rouge
  bool yellowLedState; // Nouvel état pour la LED Jaune
  char text[64];
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
bool RedLedState = false;
bool YellowLedState = false;
bool LastRedButtonState = false;
bool LastYellowButtonState = false;

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
  if (len == sizeof(espNow_incomingMessage))
  {
    memcpy(&espNow_incomingMessage, incomingData, sizeof(espNow_incomingMessage));
    Serial.println("Message reçu du Sink :");
    Serial.printf("Board ID: %d, Texte: %s\n", espNow_incomingMessage.boardId, espNow_incomingMessage.text);
  }
  else
  {
    Serial.println("Données reçues de taille incorrecte !");
  }
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
  bool currentRedButtonState = digitalRead(RedbuttonPin);
  bool currentYellowButtonState = digitalRead(YellowButtonPin);

  if (currentRedButtonState && !LastRedButtonState)
  {
    RedLedState = !RedLedState; // Toggle de l'état de la LED
    Serial.println(RedLedState ? "LED Rouge allumée" : "LED Rouge éteinte");
  }
  if (currentYellowButtonState && !LastYellowButtonState)
  {
    YellowLedState = !YellowLedState; // Toggle de l'état de la LED
    Serial.println(YellowLedState ? "LED Jaune allumée" : "LED Jaune éteinte");
  }
  // delay(1000);
  // Serial.println("Bip");

  digitalWrite(RedledPin, RedLedState);
  digitalWrite(YellowledPin, YellowLedState);

  // Créer un message pour le sink
  struct_mote2sinkMessage outgoingMessage;
  outgoingMessage.boardId = 1;                 // ID de la carte
  outgoingMessage.readingId = millis() / 1000; // Identifiant basé sur le temps
  outgoingMessage.timeTag = millis();
  outgoingMessage.redLedState = RedLedState;
  outgoingMessage.yellowLedState = YellowLedState; // Ajouter l'état de la LED jaune
  // Envoyer le message au sink
  esp_now_send(broadcastAddress, (uint8_t *)&outgoingMessage, sizeof(outgoingMessage));
  // Mise à jour des états précédents des boutons
  LastRedButtonState = currentRedButtonState;
  LastYellowButtonState = currentYellowButtonState;

  // Attendre 500 ms
  delay(500);
}