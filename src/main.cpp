// Librerías estándar
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <time.h>

#include <esp_task_wdt.h>

// Librerías de Sensores
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>
#include "EmonLib.h"

// Librerías para OTA y Persistencia
#include <HTTPClient.h>
#include <Update.h>
#include <LittleFS.h>

// Incluir el archivo de configuración
#include "config.h"

// --- Timeout para el Watchdog en segundos ---
#define WDT_TIMEOUT_S 30

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
unsigned long lastOtaCheckTime = 0;
unsigned long currentReconnectDelay = INITIAL_RECONNECT_DELAY_MS;

// Declaraciones de funciones
void setupWifi();
void configureTime();
void reconnectMqtt();
void processSensors(bool publishNow);
void callback(char *topic, byte *payload, unsigned int length);
void saveReadingToDisk(const char *payload);
void sendBufferedReadings();
void checkForUpdates();
void setupWatchdog();
// void readAllSensorsAndPublish();

// =================================================================
// FUNCIÓN SETUP
// =================================================================
void setup()
{
    Serial.begin(115200);
    Serial.println("\n\nIniciando dispositivo...");
    Serial.print("Firmware v");
    Serial.println(FIRMWARE_VERSION);

    // Configurar e iniciar el Watchdog Timer ---
    setupWatchdog();

    if (!LittleFS.begin(true, "/littlefs"))
    {
        Serial.println("Error fatal al montar LittleFS. El dispositivo se reiniciará.");
        delay(5000);
        ESP.restart();
    }
    Serial.println("Sistema de archivos LittleFS montado.");

    if (!LittleFS.exists("/data"))
    {
        if (LittleFS.mkdir("/data"))
        {
            Serial.println("Directorio /data creado.");
        }
        else
        {
            Serial.println("Error fatal al crear el directorio /data.");
        }
    }

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

// =================================================================
// FUNCIÓN LOOP PRINCIPAL
// =================================================================
void loop()
{
    // --- Alimentar al perro guardián en cada iteración ---
    esp_task_wdt_reset();

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

    // El cliente MQTT debe procesar mensajes incluso si no estamos publicando
    if (mqttClient.connected())
    {
        mqttClient.loop();
    }

    // --- Lógica de publicación y guardado desacoplada de la conexión ---
    if (millis() - lastPublishTime > PUBLISH_INTERVAL)
    {
        lastPublishTime = millis();
        // Intentar publicar si estamos conectados, si no, la función guardará en disco.
        processSensors(mqttClient.connected());
    }

    // --- Lógica de envío de datos bufferizados y OTA solo si hay conexión ---
    if (mqttClient.connected())
    {
        sendBufferedReadings();

        if (millis() - lastOtaCheckTime > OTA_CHECK_INTERVAL)
        {
            lastOtaCheckTime = millis();
            checkForUpdates();
        }
    }
}

// =================================================================
// FUNCIONES AUXILIARES
// =================================================================

// --- Función para configurar el Watchdog Timer ---
void setupWatchdog()
{
    Serial.printf("Configurando Watchdog Timer con timeout de %d segundos.\n", WDT_TIMEOUT_S);
    esp_task_wdt_init(WDT_TIMEOUT_S, true); // Habilitar panic WDT
    esp_task_wdt_add(NULL);                 // Añadir el task actual (loop) al WDT
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
        esp_task_wdt_reset(); // Alimentar WDT durante esperas largas
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
    // ---  Añadir servidores NTP de respaldo ---
    configTime(-3 * 3600, 0, "pool.ntp.org", "time.google.com"); // GMT-3

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

// void readAllSensorsAndPublish()
// {
//     Serial.println("Leyendo sensores para publicar...");

//     // --- REFACTORIZADO: Nuevo payload alineado con el backend ---
//     StaticJsonDocument<512> doc;

//     // 1. Añadir el ID del nodo, que es la clave principal
//     doc["node_id"] = NODE_ID;

//     // 2. Añadir el timestamp en formato ISO 8601 UTC
//     time_t now;
//     time(&now);
//     char timestampBuffer[32];
//     strftime(timestampBuffer, sizeof(timestampBuffer), "%Y-%m-%dT%H:%M:%SZ", gmtime(&now));
//     doc["timestamp"] = timestampBuffer;

//     // 3. Leer y añadir temperatura y humedad ambiente (DHT11)
//     float humidity = dht.readHumidity();
//     float temperature_dht = dht.readTemperature();
//     if (!isnan(humidity))
//     {
//         doc["ambiente_hum"] = humidity;
//     }
//     if (!isnan(temperature_dht))
//     {
//         doc["ambiente_temp"] = temperature_dht;
//     }

//     // 4. Leer y añadir corrientes (SCT-013)
//     // --- REFACTORIZADO: Reducir el número de muestras para evitar bloqueo ---
//     // 250ms de muestreo es un buen compromiso. 1480 muestras ~ 250ms a 60Hz.
//     double Irms1 = emon1.calcIrms(250);
//     double Irms2 = emon2.calcIrms(250);
//     doc["corriente_a"] = Irms1;
//     doc["corriente_b"] = Irms2;

//     // 5. Leer y añadir temperaturas de las sondas (DS18B20)
//     ds18b20_sensors.requestTemperatures();
//     JsonArray sondas_temp = doc.createNestedArray("sondas_temp");
//     for (int i = 0; i < DS18B20_COUNT; i++)
//     {
//         float tempC = ds18b20_sensors.getTempCByIndex(i);
//         // --- REFACTORIZADO: Añadir al array solo si la lectura es válida ---
//         if (tempC != DEVICE_DISCONNECTED_C)
//         {
//             sondas_temp.add(tempC);
//         }
//         else
//         {
//             sondas_temp.add(nullptr); // Añadir 'null' para mantener el orden si una sonda falla
//             Serial.printf("Error: No se pudo leer del sensor DS18B20 #%d\n", i);
//         }
//     }

//     // Serializar y publicar
//     char buffer[512];
//     size_t n = serializeJson(doc, buffer);

//     Serial.print("Publicando payload: ");
//     Serial.println(buffer);

//     if (mqttClient.publish(mqttTopicPub, buffer, n))
//     {
//         Serial.println("Mensaje publicado con éxito.");
//     }
//     else
//     {
//         Serial.println("Error al publicar el mensaje.");
//     }
// }

void processSensors(bool publishNow)
{
    if (publishNow)
    {
        Serial.println("Leyendo sensores para publicar...");
    }
    else
    {
        Serial.println("Sin conexión. Leyendo sensores para guardar en disco...");
    }

    StaticJsonDocument<512> doc;
    doc["node_id"] = NODE_ID;

    time_t now;
    time(&now);
    char timestampBuffer[32];
    strftime(timestampBuffer, sizeof(timestampBuffer), "%Y-%m-%dT%H:%M:%SZ", gmtime(&now));
    doc["timestamp"] = timestampBuffer;

    float humidity = dht.readHumidity();
    float temperature_dht = dht.readTemperature();
    if (!isnan(humidity))
        doc["ambiente_hum"] = humidity;
    if (!isnan(temperature_dht))
        doc["ambiente_temp"] = temperature_dht;

    double Irms1 = emon1.calcIrms(250);
    double Irms2 = emon2.calcIrms(250);
    doc["corriente_a"] = Irms1;
    doc["corriente_b"] = Irms2;

    ds18b20_sensors.requestTemperatures();
    JsonArray sondas_temp = doc.createNestedArray("sondas_temp");
    for (int i = 0; i < DS18B20_COUNT; i++)
    {
        float tempC = ds18b20_sensors.getTempCByIndex(i);
        if (tempC > -127.0)
        {
            sondas_temp.add(tempC);
        }
        else
        {
            sondas_temp.add(nullptr);
            Serial.printf("Error: No se pudo leer del sensor DS18B20 #%d\n", i);
        }
    }

    char buffer[512];
    size_t n = serializeJson(doc, buffer);

    if (publishNow)
    {
        if (!mqttClient.publish(mqttTopicPub, buffer, n))
        {
            Serial.println("Error al publicar. El mensaje se guardará.");
            saveReadingToDisk(buffer);
        }
        else
        {
            Serial.println("Mensaje publicado con éxito.");
        }
    }
    else
    {
        saveReadingToDisk(buffer);
    }
}

void saveReadingToDisk(const char *payload)
{
    char filename[32];
    sprintf(filename, "/data/%ld.json", time(nullptr));

    File file = LittleFS.open(filename, "w");
    if (!file)
    {
        Serial.println("Error al abrir el archivo para escritura");
        return;
    }
    if (file.print(payload))
    {
        Serial.printf("Lectura guardada en disco: %s\n", filename);
    }
    else
    {
        Serial.println("Error al escribir en el archivo");
    }
    file.close();
}

void sendBufferedReadings()
{
    File root = LittleFS.open("/data");
    if (!root || !root.isDirectory())
    {
        return;
    }

    File file = root.openNextFile();
    // Enviar solo un archivo por ciclo para no bloquear el loop por mucho tiempo
    if (file)
    {
        Serial.printf("Encontrado mensaje en búfer: %s\n", file.name());
        String payload = file.readString();
        String fullPath = file.name();
        file.close();

        if (mqttClient.publish(mqttTopicPub, payload.c_str(), payload.length()))
        {
            Serial.printf("Mensaje en búfer %s enviado. Eliminando archivo.\n", fullPath.c_str());
            if (!LittleFS.remove(fullPath.c_str()))
            {
                Serial.printf("Error al eliminar el archivo: %s\n", fullPath.c_str());
            }
        }
        else
        {
            Serial.println("Fallo al enviar mensaje en búfer. Se reintentará.");
        }
    }
    root.close();
}

void checkForUpdates()
{
    Serial.println("Buscando actualizaciones de firmware...");
    HTTPClient http;

    http.begin(OTA_VERSION_URL);
    int httpCode = http.GET();
    if (httpCode != HTTP_CODE_OK)
    {
        Serial.printf("Error al buscar versión, código: %d\n", httpCode);
        http.end(); // Asegurar limpieza
        return;
    }

    StaticJsonDocument<128> doc;
    DeserializationError error = deserializeJson(doc, http.getString());
    http.end(); // Limpiar después de usar el stream/string

    if (error)
    {
        Serial.print(F("deserializeJson() falló: "));
        Serial.println(error.c_str());
        return;
    }

    const char *serverVersion = doc["version"];
    Serial.printf("Versión actual: %s, Versión del servidor: %s\n", FIRMWARE_VERSION, serverVersion);

    if (strcmp(serverVersion, FIRMWARE_VERSION) > 0)
    {
        Serial.println("¡Nueva versión disponible! Iniciando actualización...");
        http.begin(OTA_FIRMWARE_URL);
        int httpCodeFw = http.GET();
        if (httpCodeFw != HTTP_CODE_OK)
        {
            Serial.printf("Error al descargar firmware, código: %d\n", httpCodeFw);
            http.end();
            return;
        }

        int contentLength = http.getSize();
        if (contentLength <= 0)
        {
            Serial.println("Error: Content-Length es inválido.");
            http.end();
            return;
        }

        if (!Update.begin(contentLength))
        {
            Update.printError(Serial);
            http.end();
            return;
        }

        // Realizar la actualización
        size_t written = Update.writeStream(*http.getStreamPtr());

        if (written != contentLength)
        {
            Serial.printf("Error de escritura: %d de %d bytes.\n", written, contentLength);
        }

        http.end(); // Limpiar la conexión HTTP tan pronto como sea posible

        if (Update.end(true)) // true para aplicar la actualización
        {
            Serial.println("¡Actualización OTA completada! Reiniciando...");
            ESP.restart();
        }
        else
        {
            Serial.println("Error en Update.end(). La actualización falló.");
            Update.printError(Serial);
        }
    }
    else
    {
        Serial.println("El firmware ya está actualizado.");
    }
}

void callback(char *topic, byte *payload, unsigned int length)
{
    // Aquí podrías implementar la lógica para recibir comandos, como forzar una lectura.
    Serial.print("Mensaje recibido en topic: ");
    Serial.println(topic);
}