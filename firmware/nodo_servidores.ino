/*
 * TFG - Nodo IoT 2 (Servidores)
 * Sensores: DHT11 (temp/humedad) + MQ-2 (gas/humo) + Flame (llama)
 *
 * Publica:
 *   sensors/<ZONA>/temperatura   -> valor cuando supera umbral
 *   sensors/<ZONA>/humedad       -> valor cuando supera umbral
 *   sensors/<ZONA>/gas           -> "1" cuando MQ-2 supera umbral
 *   sensors/<ZONA>/flame         -> "1" cuando flame detecta llama
 *   sensors/<ZONA>/fuego         -> "1" cuando MQ-2 Y Flame se disparan juntos (fusion local)
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>

// ---------- CONFIGURACION ----------
const char* WIFI_SSID     = " "; //"WIFI"
const char* WIFI_PASSWORD = " "; //Contraseña
const char* MQTT_BROKER   =  " "; // IP de la jetson
const int   MQTT_PORT     = 1883;
const char* ZONA          = "centro";
const char* CLIENT_ID     = "nodo2_servidores";
// -----------------------------------

// Pines
const int PIN_DHT   = 4;
const int PIN_MQ2   = 34;   // entrada analogica (ADC)
const int PIN_FLAME = 16;   // entrada digital

// Configuracion DHT
#define DHT_TYPE DHT11
DHT dht(PIN_DHT, DHT_TYPE);

// Umbrales
const float UMBRAL_TEMP_ALTA = 32.0;     // Celsius
const float UMBRAL_HUM_ALTA  = 85.0;     // %
const int   UMBRAL_GAS       = 600;     // valor ADC 0-4095 (ajustar segun ambiente)

// Cooldowns (ms)
const unsigned long COOLDOWN_AMBIENTE = 120000;  // temp/hum cada 30s si supera umbral
const unsigned long COOLDOWN_ALARMA   = 60000;   // gas/llama/fuego: 5s

// Estado
WiFiClient espClient;
PubSubClient mqtt(espClient);

unsigned long ultimoTemp = 0;
unsigned long ultimoHum  = 0;
unsigned long ultimoGas  = 0;
unsigned long ultimoFlame = 0;
unsigned long ultimoFuego = 0;

void conectarWiFi() {
  Serial.print("Conectando a WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" conectado!");
}

void conectarMQTT() {
  while (!mqtt.connected()) {
    Serial.print("Conectando a MQTT...");
    if (mqtt.connect(CLIENT_ID)) {
      Serial.println(" conectado!");
    } else {
      Serial.print(" fallo rc=");
      Serial.print(mqtt.state());
      Serial.println(", reintento en 2s");
      delay(2000);
    }
  }
}

void publicar(const char* tipo, const char* valor) {
  char topic[64];
  snprintf(topic, sizeof(topic), "sensors/%s/%s", ZONA, tipo);
  mqtt.publish(topic, valor);
  Serial.print("Publicado: ");
  Serial.print(topic);
  Serial.print(" = ");
  Serial.println(valor);
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_FLAME, INPUT);
  // PIN_MQ2 es analogico, no requiere pinMode

  dht.begin();
  conectarWiFi();
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  conectarMQTT();

  Serial.println("Nodo 2 (servidores) listo. El MQ-2 necesita 1-2 min de calentamiento.");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) conectarWiFi();
  if (!mqtt.connected()) conectarMQTT();
  mqtt.loop();

  unsigned long ahora = millis();

  // --- DHT11: temperatura y humedad ---
  float temp = dht.readTemperature();
  float hum  = dht.readHumidity();

  if (!isnan(temp) && temp > UMBRAL_TEMP_ALTA) {
    if (ahora - ultimoTemp > COOLDOWN_AMBIENTE) {
      char buf[16];
      dtostrf(temp, 0, 1, buf);
      publicar("temperatura", buf);
      ultimoTemp = ahora;
    }
  }

  if (!isnan(hum) && hum > UMBRAL_HUM_ALTA) {
    if (ahora - ultimoHum > COOLDOWN_AMBIENTE) {
      char buf[16];
      dtostrf(hum, 0, 1, buf);
      publicar("humedad", buf);
      ultimoHum = ahora;
    }
  }

  // --- MQ-2: gas/humo (lectura analogica) ---
  int gasValor = analogRead(PIN_MQ2);
  bool gasAlarma = (gasValor > UMBRAL_GAS);

  if (gasAlarma && (ahora - ultimoGas > COOLDOWN_ALARMA)) {
    publicar("gas", "1");
    ultimoGas = ahora;
  }

  // --- Flame: llama (digital) ---
  // El flame sensor normalmente da LOW cuando detecta llama (logica inversa)
  bool flameAlarma = (digitalRead(PIN_FLAME) == LOW);

  if (flameAlarma && (ahora - ultimoFlame > COOLDOWN_ALARMA)) {
    publicar("flame", "1");
    ultimoFlame = ahora;
  }

  // --- FUSION LOCAL: gas + llama = fuego confirmado ---
  if (gasAlarma && flameAlarma && (ahora - ultimoFuego > COOLDOWN_ALARMA)) {
    publicar("fuego", "1");
    ultimoFuego = ahora;
  }

  // Debug: imprimir valores ADC cada 2s (util para calibrar)
  static unsigned long ultDebug = 0;
  if (ahora - ultDebug > 2000) {
    Serial.print("Gas ADC: ");
    Serial.print(gasValor);
    Serial.print(" | Flame: ");
    Serial.print(flameAlarma ? "SI" : "no");
    Serial.print(" | Temp: ");
    Serial.print(isnan(temp) ? -1 : temp);
    Serial.print(" | Hum: ");
    Serial.println(isnan(hum) ? -1 : hum);
    ultDebug = ahora;
  }

  delay(100);
}
