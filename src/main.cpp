#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>
#include <ArduinoJson.h>
#include <time.h>
#include "EmonLib.h"

// Incluir el archivo de configuración
#include "config.h"

// Pega aquí el contenido de tu archivo .cer
static const char *root_ca PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)EOF";

// Instancias
WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);
OneWire oneWire(ONE_WIRE_BUS_PIN);
DallasTemperature ds18b20_sensors(&oneWire);
DHT dht(DHT_PIN, DHT_TYPE);
EnergyMonitor emon1;
EnergyMonitor emon2;

// --- REFACTORIZADO: Topic dinámico ---
char mqttTopicPub[128];

// Variables para temporización
unsigned long lastPublishTime = 0;
unsigned long lastWifiReconnectAttempt = 0;
unsigned long lastMqttReconnectAttempt = 0;
unsigned long currentReconnectDelay = INITIAL_RECONNECT_DELAY_MS;

// Declaraciones de funciones
void setupWifi();
void reconnectMqtt();
void readAllSensorsAndPublish();
void callback(char *topic, byte *payload, unsigned int length);
void configureTime();

void setup()
{
    Serial.begin(115200);
    Serial.println("\n\nIniciando dispositivo...");
    Serial.print("Firmware v");
    Serial.println(FIRMWARE_VERSION);

    // Inicializar sensores
    ds18b20_sensors.begin();
    dht.begin();
    emon1.current(SCT013_1_PIN, EMON_CALIBRATION_1);
    emon2.current(SCT013_2_PIN, EMON_CALIBRATION_2);

    setupWifi();
    configureTime();

    // --- REFACTORIZADO: Construir el topic de publicación dinámicamente ---
    snprintf(mqttTopicPub, sizeof(mqttTopicPub), "clientes/%s/sensores/%s", CLIENT_ID, NODE_ID);
    Serial.print("Topic de publicación MQTT: ");
    Serial.println(mqttTopicPub);

    wifiClient.setCACert(root_ca);
    mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
    mqttClient.setCallback(callback);
}

void loop()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        if (millis() - lastWifiReconnectAttempt > 15000)
        { // Reintentar WiFi cada 15s
            lastWifiReconnectAttempt = millis();
            Serial.println("WiFi desconectado. Reintentando conexión...");
            WiFi.reconnect();
        }
        return; // No hacer nada más si no hay WiFi
    }

    if (!mqttClient.connected())
    {
        reconnectMqtt(); // Intentará reconectar con retroceso exponencial
    }

    mqttClient.loop(); // Mantener la conexión MQTT activa

    if (mqttClient.connected() && (millis() - lastPublishTime > PUBLISH_INTERVAL))
    {
        lastPublishTime = millis();
        readAllSensorsAndPublish();
    }
}

void setupWifi()
{
    Serial.print("Conectando a WiFi: ");
    Serial.println(WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    int retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < 30)
    { // ~15 segundos de espera
        delay(500);
        Serial.print(".");
        retries++;
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.println("\nWiFi conectado!");
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());
    }
    else
    {
        Serial.println("\nNo se pudo conectar al WiFi. Reiniciando en 10 segundos...");
        delay(10000);
        ESP.restart();
    }
}

void configureTime()
{
    Serial.println("Sincronizando hora del sistema vía NTP...");
    configTime(-3 * 3600, 0, "pool.ntp.org"); // GMT-3

    struct tm timeinfo;
    if (!getLocalTime(&timeinfo, 10000))
    { // 10 segundos de timeout
        Serial.println("Error al obtener la hora del NTP. La conexión SSL puede fallar. Reiniciando...");
        delay(5000);
        ESP.restart();
    }
    Serial.println("¡Hora del sistema sincronizada!");
    Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

void reconnectMqtt()
{
    if (millis() - lastMqttReconnectAttempt > currentReconnectDelay)
    {
        lastMqttReconnectAttempt = millis();
        Serial.print("Intentando conexión MQTT...");

        if (mqttClient.connect(NODE_ID, MQTT_USER, MQTT_PASSWORD))
        {
            Serial.println(" ¡Conectado!");
            currentReconnectDelay = INITIAL_RECONNECT_DELAY_MS; // Reiniciar delay al conectar
        }
        else
        {
            Serial.print(" falló, rc=");
            Serial.print(mqttClient.state());
            Serial.print(". Reintentando en ");
            Serial.print(currentReconnectDelay / 1000);
            Serial.println(" segundos.");

            // --- REFACTORIZADO: Lógica de Retroceso Exponencial ---
            currentReconnectDelay *= RECONNECT_MULTIPLIER;
            if (currentReconnectDelay > MAX_RECONNECT_DELAY_MS)
            {
                currentReconnectDelay = MAX_RECONNECT_DELAY_MS;
            }
        }
    }
}

void readAllSensorsAndPublish()
{
    Serial.println("Leyendo sensores para publicar...");

    // --- REFACTORIZADO: Nuevo payload alineado con el backend ---
    StaticJsonDocument<512> doc;

    // 1. Añadir el ID del nodo, que es la clave principal
    doc["node_id"] = NODE_ID;

    // 2. Añadir el timestamp en formato ISO 8601 UTC
    time_t now;
    time(&now);
    char timestampBuffer[32];
    strftime(timestampBuffer, sizeof(timestampBuffer), "%Y-%m-%dT%H:%M:%SZ", gmtime(&now));
    doc["timestamp"] = timestampBuffer;

    // 3. Leer y añadir temperatura y humedad ambiente (DHT11)
    float humidity = dht.readHumidity();
    float temperature_dht = dht.readTemperature();
    if (!isnan(humidity))
    {
        doc["ambiente_hum"] = humidity;
    }
    if (!isnan(temperature_dht))
    {
        doc["ambiente_temp"] = temperature_dht;
    }

    // 4. Leer y añadir corrientes (SCT-013)
    // --- REFACTORIZADO: Reducir el número de muestras para evitar bloqueo ---
    // 250ms de muestreo es un buen compromiso. 1480 muestras ~ 250ms a 60Hz.
    double Irms1 = emon1.calcIrms(250);
    double Irms2 = emon2.calcIrms(250);
    doc["corriente_a"] = Irms1;
    doc["corriente_b"] = Irms2;

    // 5. Leer y añadir temperaturas de las sondas (DS18B20)
    ds18b20_sensors.requestTemperatures();
    JsonArray sondas_temp = doc.createNestedArray("sondas_temp");
    for (int i = 0; i < DS18B20_COUNT; i++)
    {
        float tempC = ds18b20_sensors.getTempCByIndex(i);
        // --- REFACTORIZADO: Añadir al array solo si la lectura es válida ---
        if (tempC != DEVICE_DISCONNECTED_C)
        {
            sondas_temp.add(tempC);
        }
        else
        {
            sondas_temp.add(nullptr); // Añadir 'null' para mantener el orden si una sonda falla
            Serial.printf("Error: No se pudo leer del sensor DS18B20 #%d\n", i);
        }
    }

    // Serializar y publicar
    char buffer[512];
    size_t n = serializeJson(doc, buffer);

    Serial.print("Publicando payload: ");
    Serial.println(buffer);

    if (mqttClient.publish(mqttTopicPub, buffer, n))
    {
        Serial.println("Mensaje publicado con éxito.");
    }
    else
    {
        Serial.println("Error al publicar el mensaje.");
    }
}

void callback(char *topic, byte *payload, unsigned int length)
{
    // Aquí podrías implementar la lógica para recibir comandos, como forzar una lectura.
    Serial.print("Mensaje recibido en topic: ");
    Serial.println(topic);
}