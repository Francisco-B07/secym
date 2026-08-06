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

int detected_ds18b20_count = 0;

// --- REFACTORIZADO: Topic dinámico ---
char mqttTopicPub[128];

// Variables para temporización
unsigned long lastPublishTime = 0;
unsigned long lastWifiReconnectAttempt = 0;
unsigned long lastMqttReconnectAttempt = 0;
unsigned long lastOtaCheckTime = 0;
bool otaCheckedOnce = false;
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

// --- Variables para control de temperatura asíncrona ---
bool conversionInProgress = false;
unsigned long conversionStartTime = 0;
const unsigned long CONVERSION_TIME_MS = 800;

// Variable global para evitar buscar archivos si sabemos que no hay
bool filesToSync = true;

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
    ds18b20_sensors.setWaitForConversion(false);
    dht.begin();
    emon1.current(SCT013_1_PIN, EMON_CALIBRATION_1);
    emon2.current(SCT013_2_PIN, EMON_CALIBRATION_2);

    detected_ds18b20_count = ds18b20_sensors.getDeviceCount();
    Serial.printf("Se han detectado %d sensores DS18B20.\n", detected_ds18b20_count);

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
    // 1. Alimentar al perro guardián (Watchdog) en cada iteración
    esp_task_wdt_reset();

    unsigned long now = millis();

    // 2. Gestión de Conectividad WiFi
    if (WiFi.status() != WL_CONNECTED)
    {
        if (now - lastWifiReconnectAttempt > 15000)
        {
            lastWifiReconnectAttempt = now;
            Serial.println("Conexión WiFi perdida. Ejecutando limpieza y renegociando IP...");

            // Forzar desconexión profunda para evitar que el router nos bloquee
            WiFi.disconnect(true);
            delay(1000);

            // Volver a iniciar el ciclo de autenticación
            WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        }
        // No salimos con return para permitir que otras tareas locales (como leer sondas) sigan funcionando
    }

    // 3. Gestión de Conectividad MQTT
    if (WiFi.status() == WL_CONNECTED && !mqttClient.connected())
    {
        reconnectMqtt();
    }

    // 4. Procesar mensajes entrantes (Callback)
    if (mqttClient.connected())
    {
        mqttClient.loop();
    }

    // 5. LÓGICA DE SENSORES ASÍNCRONA (MÁQUINA DE ESTADOS)

    // FASE A: Iniciar conversión de sensores DS18B20 (1 segundo antes de publicar)
    if (!conversionInProgress && (now - lastPublishTime > (PUBLISH_INTERVAL - 1000)))
    {
        // === RUTINA DE AUTO-REPARACIÓN (PLUG & PLAY) ===
        // Si el conteo es 0 (fallo en el arranque), intentamos reiniciar el bus
        if (detected_ds18b20_count == 0)
        {
            Serial.println("Re-escaneando bus 1-Wire...");
            ds18b20_sensors.begin();
            // CRÍTICO: Al hacer begin() se pierden las configuraciones, debemos volver a setear el asincronismo
            ds18b20_sensors.setWaitForConversion(false);
            detected_ds18b20_count = ds18b20_sensors.getDeviceCount();
            if (detected_ds18b20_count > 0)
            {
                Serial.printf("¡Éxito! Se recuperaron %d sensores DS18B20.\n", detected_ds18b20_count);
            }
        }

        // Si hay sondas (ya sea desde el arranque o recuperadas), pedimos temperaturas
        if (detected_ds18b20_count > 0)
        {
            ds18b20_sensors.requestTemperatures();
        }

        conversionStartTime = now;
        conversionInProgress = true;
    }

    // FASE B: Procesar y Publicar cuando se cumple el intervalo
    if (now - lastPublishTime > PUBLISH_INTERVAL)
    {
        // Solo procedemos si la conversión ha tenido tiempo suficiente
        if (conversionInProgress && (now - conversionStartTime >= CONVERSION_TIME_MS))
        {
            lastPublishTime = now;

            // Intentar publicar si hay MQTT, de lo contrario guarda en disco (lógica interna de processSensors)
            processSensors(mqttClient.connected());

            // Liberar la bandera para la próxima lectura
            conversionInProgress = false;
        }
        else if (!conversionInProgress)
        {
            // Caso de seguridad: si llegamos al tiempo de publicación y no se inició conversión (ej. primer arranque)
            ds18b20_sensors.requestTemperatures();
            conversionStartTime = now;
            conversionInProgress = true;
        }
    }

    // 6. Tareas secundarias (Solo si hay conexión para no saturar el bus interno)
    if (mqttClient.connected())
    {
        // Enviar archivos pendientes de LittleFS uno por uno
        sendBufferedReadings();

        // Verificar actualizaciones OTA cada 24hs
        if (!otaCheckedOnce || (now - lastOtaCheckTime > OTA_CHECK_INTERVAL))
        {
            lastOtaCheckTime = now;
            otaCheckedOnce = true;
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

    // 1. Limpieza profunda preventiva del chip de radio
    WiFi.disconnect(true);
    delay(1000);

    // 2. Configurar modo Estación y encender Auto-Reconexión nativa a bajo nivel
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);

    // 3. Iniciar conexión
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    int retries = 0;
    // ~15 segundos de espera máxima (30 intentos * 500ms)
    while (WiFi.status() != WL_CONNECTED && retries < 30)
    {
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
        // En el primer arranque, si falla, es mejor reiniciar todo el hardware
        Serial.println("\nFallo crítico de WiFi en el arranque. Reiniciando el nodo...");
        delay(5000);
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

        // 1. Creamos un topic exclusivo para el estado de vida del nodo
        char willTopic[128];
        snprintf(willTopic, sizeof(willTopic), "clientes/%s/estado/%s", CLIENT_ID, NODE_ID);

        // 2. Conectamos inyectando el LWT (QoS 1, Retain True, Mensaje "OFFLINE")
        if (mqttClient.connect(NODE_ID, MQTT_USER, MQTT_PASSWORD, willTopic, 1, true, "OFFLINE"))
        {
            Serial.println(" ¡Conectado!");

            // 3. Apenas nos conectamos, pisamos el testamento publicando "ONLINE"
            mqttClient.publish(willTopic, "ONLINE", true);

            currentReconnectDelay = INITIAL_RECONNECT_DELAY_MS;
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

// =================================================================
// Agregar para identificar sondas por ID
// =================================================================

String getAddressStr(DeviceAddress deviceAddress)
{
    String str = "";
    for (uint8_t i = 0; i < 8; i++)
    {
        if (deviceAddress[i] < 16)
            str += "0";
        str += String(deviceAddress[i], HEX);
    }
    str.toUpperCase();
    return str;
}

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

    StaticJsonDocument<1024> doc;
    doc["node_id"] = NODE_ID;

    time_t now;
    time(&now);
    char timestampBuffer[32];
    strftime(timestampBuffer, sizeof(timestampBuffer), "%Y-%m-%dT%H:%M:%SZ", gmtime(&now));
    doc["timestamp"] = timestampBuffer;
    doc["firmware_version"] = FIRMWARE_VERSION;

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

    // =================================================================
    // Cambiar para identificar sondas por ID
    // =================================================================

    // JsonArray sondas_temp = doc.createNestedArray("sondas_temp");
    // for (int i = 0; i < detected_ds18b20_count; i++)
    // {
    //     float tempC = ds18b20_sensors.getTempCByIndex(i);
    //     if (tempC > -127.0)
    //     {
    //         sondas_temp.add(tempC);
    //     }
    //     else
    //     {
    //         sondas_temp.add(nullptr);
    //         Serial.printf("Error: No se pudo leer del sensor DS18B20 #%d\n", i);
    //     }
    // }
    // Cambiamos el array simple por un array de objetos con ID y Temperatura
    JsonArray sondas_data = doc.createNestedArray("sondas_raw");

    for (int i = 0; i < detected_ds18b20_count; i++)
    {
        DeviceAddress tempDeviceAddress;
        if (ds18b20_sensors.getAddress(tempDeviceAddress, i))
        {
            float tempC = ds18b20_sensors.getTempC(tempDeviceAddress);

            JsonObject s = sondas_data.createNestedObject();
            s["address"] = getAddressStr(tempDeviceAddress); // ID Único

            if (tempC > -127.0)
            {
                s["t"] = tempC;
            }
            else
            {
                s["t"] = nullptr;
                Serial.printf("Error de lectura en sonda: %s\n", getAddressStr(tempDeviceAddress).c_str());
            }
        }
    }

    // =================================================================
    // Hasta aquí
    // =================================================================

    char buffer[1024];
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

    // 1. Verificar espacio total (Opcional pero recomendado)
    size_t totalBytes = LittleFS.totalBytes();
    size_t usedBytes = LittleFS.usedBytes();
    if ((usedBytes * 100 / totalBytes) > MAX_FS_USAGE_PERCENT)
    {
        Serial.println("LittleFS: Espacio insuficiente. Abortando guardado.");
        return;
    }

    // 2. Contar archivos y borrar el más antiguo si excedemos el límite
    File root = LittleFS.open("/data");
    int fileCount = 0;
    String oldestFile = "";

    File file = root.openNextFile();
    while (file)
    {
        fileCount++;
        if (oldestFile == "")
            oldestFile = "/data/" + String(file.name());
        file = root.openNextFile();
    }
    root.close();

    if (fileCount >= MAX_BUFFERED_FILES)
    {
        Serial.printf("Límite de archivos alcanzado (%d). Borrando el más antiguo: %s\n", fileCount, oldestFile.c_str());
        LittleFS.remove(oldestFile);
    }

    // 3. Proceder a guardar el nuevo archivo
    char filename[32];
    sprintf(filename, "/data/%ld.json", time(nullptr));

    File newFile = LittleFS.open(filename, "w");
    if (!newFile)
    {
        Serial.println("Error al abrir archivo para escritura");
        return;
    }

    if (newFile.print(payload))
    {
        Serial.printf("Lectura guardada: %s\n", filename);
    }
    newFile.close();
}

void sendBufferedReadings()
{
    if (!filesToSync)
        return;

    File root = LittleFS.open("/data");

    File file = root.openNextFile();

    if (!file)
    {
        filesToSync = false; // No hay más archivos, dejar de buscar hasta el próximo fallo de envío
        root.close();
        return;
    }

    // Si hay un archivo, procesarlo
    String fullPath = "/data/" + String(file.name());
    String payload = file.readString();
    file.close();

    if (mqttClient.publish(mqttTopicPub, payload.c_str(), payload.length()))
    {
        Serial.printf("Buffer: %s enviado con éxito.\n", fullPath.c_str());
        LittleFS.remove(fullPath);
    }
    else
    {
        Serial.println("Buffer: Fallo al enviar, se reintentará luego.");
        // Si falla el envío, no intentamos con el siguiente para no saturar
    }
    root.close();
}

// =================================================================
// FUNCIÓN AUXILIAR: Comparar versiones semánticas
// =================================================================
bool isNewerVersion(const char *serverVer, const char *currentVer)
{
    int sMajor = 0, sMinor = 0, sPatch = 0;
    int cMajor = 0, cMinor = 0, cPatch = 0;

    // Parsear versión del servidor
    sscanf(serverVer, "%d.%d.%d", &sMajor, &sMinor, &sPatch);
    // Parsear versión actual
    sscanf(currentVer, "%d.%d.%d", &cMajor, &cMinor, &cPatch);

    // Comparar versiones
    if (sMajor > cMajor)
        return true;
    if (sMajor < cMajor)
        return false;

    if (sMinor > cMinor)
        return true;
    if (sMinor < cMinor)
        return false;

    if (sPatch > cPatch)
        return true;

    return false;
}

void checkForUpdates()
{
    Serial.println("=== INICIO VERIFICACIÓN OTA ===");
    Serial.printf("Versión actual: %s\n", FIRMWARE_VERSION);

    HTTPClient http;
    http.setTimeout(15000); // 15 segundos de timeout

    // --- PASO 1: Obtener versión disponible ---
    Serial.println("Descargando version.json...");
    http.begin(OTA_VERSION_URL);
    int httpCode = http.GET();

    if (httpCode != HTTP_CODE_OK)
    {
        Serial.printf("❌ Error al buscar versión, código HTTP: %d\n", httpCode);
        http.end();
        return;
    }

    // Aumentar tamaño del buffer y procesar respuesta
    String payload = http.getString();
    http.end(); // Liberar INMEDIATAMENTE después de obtener el string

    Serial.printf("Respuesta del servidor: %s\n", payload.c_str());

    StaticJsonDocument<256> doc; // Aumentado a 256 bytes
    DeserializationError error = deserializeJson(doc, payload);

    if (error)
    {
        Serial.print("❌ Error al parsear JSON: ");
        Serial.println(error.c_str());
        return;
    }

    const char *serverVersion = doc["version"];
    if (!serverVersion)
    {
        Serial.println("❌ Campo 'version' no encontrado en el JSON");
        return;
    }

    Serial.printf("Versión del servidor: %s\n", serverVersion);

    // --- PASO 2: Comparar versiones correctamente ---
    if (!isNewerVersion(serverVersion, FIRMWARE_VERSION))
    {
        Serial.println("✅ El firmware ya está actualizado.");
        return;
    }

    Serial.println("🔄 ¡Nueva versión disponible! Iniciando actualización...");

    // --- PASO 3: Descargar firmware ---
    http.begin(OTA_FIRMWARE_URL);
    httpCode = http.GET();

    if (httpCode != HTTP_CODE_OK)
    {
        Serial.printf("❌ Error al descargar firmware, código HTTP: %d\n", httpCode);
        http.end();
        return;
    }

    int contentLength = http.getSize();
    Serial.printf("Tamaño del firmware: %d bytes\n", contentLength);

    if (contentLength <= 0 || contentLength > 2000000)
    { // Límite de 2MB por seguridad
        Serial.println("❌ Content-Length inválido.");
        http.end();
        return;
    }

    // --- PASO 4: Iniciar actualización OTA ---
    if (!Update.begin(contentLength))
    {
        Serial.println("❌ Error al iniciar Update:");
        Update.printError(Serial);
        http.end();
        return;
    }

    Serial.println("Descargando firmware...");

    // --- PASO 5: Escribir firmware con feedback ---
    WiFiClient *stream = http.getStreamPtr();
    size_t written = 0;
    uint8_t buff[512];
    int lastPercent = 0;

    while (http.connected() && (written < contentLength))
    {
        size_t available = stream->available();

        if (available)
        {
            int c = stream->readBytes(buff, min(available, sizeof(buff)));

            if (c > 0)
            {
                Update.write(buff, c);
                written += c;

                // Mostrar progreso cada 10%
                int percent = (written * 100) / contentLength;
                if (percent >= lastPercent + 10)
                {
                    Serial.printf("Progreso: %d%% (%d/%d bytes)\n", percent, written, contentLength);
                    lastPercent = percent;
                    esp_task_wdt_reset(); // Alimentar watchdog durante descarga larga
                }
            }
        }
        delay(1);
        esp_task_wdt_reset(); // Alimentar watchdog
    }

    http.end(); // Liberar conexión HTTP

    if (written != contentLength)
    {
        Serial.printf("❌ Error: Solo se escribieron %d de %d bytes\n", written, contentLength);
        Update.abort();
        return;
    }

    // --- PASO 6: Finalizar actualización ---
    if (Update.end(true))
    {
        Serial.println("✅ ¡Actualización OTA completada exitosamente!");
        Serial.printf("Nueva versión: %s\n", serverVersion);
        Serial.println("Reiniciando en 3 segundos...");
        delay(3000);
        ESP.restart();
    }
    else
    {
        Serial.println("❌ Error al finalizar Update:");
        Update.printError(Serial);
        Update.abort();
    }
}

void callback(char *topic, byte *payload, unsigned int length)
{
    // Aquí podrías implementar la lógica para recibir comandos, como forzar una lectura.
    Serial.print("Mensaje recibido en topic: ");
    Serial.println(topic);
}