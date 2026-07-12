/*
 * TFG - Nodo IoT 1 (Entrada)
 * Sensores: PIR (movimiento) + KY-037 (sonido)
 * Publica eventos por MQTT cuando se supera un umbral.
 *
 * Topics:
 *   sensors/<ZONA>/pir     -> "1" al detectar movimiento
 *   sensors/<ZONA>/sonido  -> "1" al detectar ruido
 */

#include <WiFi.h>
#include <PubSubClient.h>

// ---------- CONFIGURACION (cambiar por nodo) ----------
const char* WIFI_SSID     = " "; //"WIFI"
const char* WIFI_PASSWORD = " "; //Contraseña
const char* MQTT_BROKER   =  " "; // IP
const int   MQTT_PORT     = 1883;
const char* ZONA          = "inicio"; //Se puede cambiar por cualquier punto de navegación
const char* CLIENT_ID     = "nodo1_entrada";
// ------------------------------------------------------

// Pines de sensores
const int PIN_PIR    = 13;
const int PIN_SONIDO = 14;

// Anti-rebote: tiempo minimo entre eventos del mismo sensor (ms)
const unsigned long COOLDOWN = 60000;

WiFiClient espClient;
PubSubClient mqtt(espClient);

unsigned long ultimoPir = 0;
unsigned long ultimoSonido = 0;

void conectarWiFi() {
  Serial.print("Conectando a WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" conectado!");
  Serial.print("IP local: ");
  Serial.println(WiFi.localIP());
}

void conectarMQTT() {
  while (!mqtt.connected()) {
    Serial.print("Conectando a MQTT...");
    if (mqtt.connect(CLIENT_ID)) {
      Serial.println(" conectado!");
    } else {
      Serial.print(" fallo, rc=");
      Serial.print(mqtt.state());
      Serial.println(" reintento en 2s");
      delay(2000);
    }
  }
}

void publicarEvento(const char* tipo) {
  char topic[64];
  snprintf(topic, sizeof(topic), "sensors/%s/%s", ZONA, tipo);
  mqtt.publish(topic, "1");
  Serial.print("Publicado: ");
  Serial.println(topic);
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_PIR, INPUT);
  pinMode(PIN_SONIDO, INPUT);

  conectarWiFi();
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  conectarMQTT();

  Serial.println("Nodo 1 (entrada) listo.");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) conectarWiFi();
  if (!mqtt.connected()) conectarMQTT();
  mqtt.loop();

  unsigned long ahora = millis();

  // PIR (movimiento)
  if (digitalRead(PIN_PIR) == HIGH) {
    if (ahora - ultimoPir > COOLDOWN) {
      publicarEvento("pir");
      ultimoPir = ahora;
    }
  }

  // KY-037 (sonido)
  if (digitalRead(PIN_SONIDO) == HIGH) {
    if (ahora - ultimoSonido > COOLDOWN) {
      publicarEvento("sonido");
      ultimoSonido = ahora;
    }
  }

  delay(50);
}
