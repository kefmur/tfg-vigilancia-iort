/*
 * TFG - Nodo IoT 3 (Zona comun / Equipos de valor)
 * Sensores: DHT11 (temp/humedad) + Vibration SW-420 (golpes)
 *
 * Publica:
 *   sensors/<ZONA>/temperatura  -> valor cuando supera umbral
 *   sensors/<ZONA>/humedad      -> valor cuando supera umbral
 *   sensors/<ZONA>/vibracion    -> "1" cuando hay golpe/movimiento del sensor
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>

// ---------- CONFIGURACION ----------
const char* WIFI_SSID     = "DIGIFIBRA-56NC";
const char* WIFI_PASSWORD = "RUHYkQ2z3kdG"; 
const char* MQTT_BROKER   = "192.168.1.149";
const int   MQTT_PORT     = 1883;
const char* ZONA          = "esquina_superior_derecha";   // ajusta a tu waypoint
const char* CLIENT_ID     = "nodo3_zonacomun";
// -----------------------------------

// Pines
const int PIN_DHT       = 4;
const int PIN_VIBRACION = 15;

// Configuracion DHT
#define DHT_TYPE DHT11
DHT dht(PIN_DHT, DHT_TYPE);

// Umbrales
const float UMBRAL_TEMP_ALTA = 32.0;
const float UMBRAL_HUM_ALTA  = 85.0;

// Cooldowns (ms)
const unsigned long COOLDOWN_AMBIENTE = 120000;   // 2 min
const unsigned long COOLDOWN_VIBRACION = 60000;   // 1 min

// Estado
WiFiClient espClient;
PubSubClient mqtt(espClient);

unsigned long ultimoTemp = 0;
unsigned long ultimoHum  = 0;
unsigned long ultimoVibracion = 0;


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
  pinMode(PIN_VIBRACION, INPUT);

  dht.begin();
  conectarWiFi();
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  conectarMQTT();

  Serial.println("Nodo 3 (zona comun) listo.");
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

  // --- Vibration sensor SW-420 ---
  // Salida HIGH cuando detecta vibracion/golpe
  if (digitalRead(PIN_VIBRACION) == HIGH) {
    if (ahora - ultimoVibracion > COOLDOWN_VIBRACION) {
      publicar("vibracion", "1");
      ultimoVibracion = ahora;
    }
  }

  // Debug cada 2s
  static unsigned long ultDebug = 0;
  if (ahora - ultDebug > 2000) {
    Serial.print("Temp: ");
    Serial.print(isnan(temp) ? -1 : temp);
    Serial.print(" | Hum: ");
    Serial.print(isnan(hum) ? -1 : hum);
    Serial.print(" | Vibracion: ");
    Serial.println(digitalRead(PIN_VIBRACION) == HIGH ? "SI" : "no");
    ultDebug = ahora;
  }

  delay(50);
}