#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "EmonLib.h"

#include "config.h"

// --- Client Initialization ---
WiFiClientSecure espClient;
PubSubClient client(espClient);

// --- Sensor Initialization ---
OneWire oneWire(ONEWIRE_BUS_PIN);
DallasTemperature tempSensors(&oneWire);
EnergyMonitor emon;

// --- Non-Blocking Timers ---
unsigned long lastPublish = 0;
unsigned long lastWifiReconnectAttempt = 0;
unsigned long lastMqttReconnectAttempt = 0;

// --- Function Prototypes ---
void setup_wifi();
void handle_wifi();
void handle_mqtt();
void publish_sensor_data();
String get_utc_timestamp();
float read_current(int adc_pin); // <-- CAMBIO: Ahora recibe el pin ADC

void setup()
{
    Serial.begin(115200);

    // --- Initialize Sensors ---
    tempSensors.begin();

    // Configurar pines de puerta con pull-up interno
    for (int i = 0; i < NUM_ASSETS; i++)
    {
        pinMode(door_sensor_pins[i], INPUT_PULLUP);
    }

    // <-- CAMBIO: Se elimina la configuración de pines del multiplexor.
    // <-- CAMBIO: Se elimina la calibración inicial de EmonLib, se hará dinámicamente.

    // --- Network Setup ---
    setup_wifi();
    espClient.setCACert(root_ca);
    client.setServer(MQTT_BROKER, MQTT_PORT);
    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
}

void loop()
{
    handle_wifi();
    handle_mqtt();

    if (client.connected())
    {
        client.loop();
    }

    unsigned long currentMillis = millis();
    if (currentMillis - lastPublish >= PUBLISH_INTERVAL)
    {
        lastPublish = currentMillis;
        if (WiFi.status() == WL_CONNECTED && client.connected())
        {
            publish_sensor_data();
        }
        else
        {
            Serial.println("Skipping publish: network not ready.");
        }
    }
}

void setup_wifi() { /* ... sin cambios ... */ }
void handle_wifi() { /* ... sin cambios ... */ }
void handle_mqtt() { /* ... sin cambios ... */ }

void publish_sensor_data()
{
    StaticJsonDocument<1024> doc;

    doc["node_id"] = NODE_ID;
    doc["timestamp"] = get_utc_timestamp();

    JsonArray sensor_data = doc.createNestedArray("sensors");

    tempSensors.requestTemperatures();

    for (int i = 0; i < NUM_ASSETS; i++)
    {
        JsonObject asset = sensor_data.createNestedObject();
        asset["asset_id"] = asset_ids[i];

        float tempC = tempSensors.getTempC(ds18b20_addresses[i]);
        if (tempC == DEVICE_DISCONNECTED_C)
        {
            asset["temperature_c"] = nullptr;
        }
        else
        {
            asset["temperature_c"] = tempC;
        }

        // <-- CAMBIO: Se llama a read_current con el pin ADC del array de configuración.
        asset["current_a"] = read_current(current_sensor_adc_pins[i]);

        asset["door_open"] = (digitalRead(door_sensor_pins[i]) == HIGH);
    }

    char payload[1024];
    serializeJson(doc, payload);

    Serial.print("Publishing message: ");
    Serial.println(payload);

    if (client.publish(MQTT_TOPIC_DATA, payload))
    {
        Serial.println("Message published successfully.");
    }
    else
    {
        Serial.println("Failed to publish message.");
    }
}

// <-- CAMBIO: La función ahora acepta un pin ADC y no un canal de MUX.
float read_current(int adc_pin)
{
    // <-- CAMBIO: Se configura EmonLib para el pin correcto antes de cada lectura.
    // El valor de calibración (60.6) es para un SCT-013-000 50A/1V con una resistencia burden de 33ohm a 3.3V. Ajústalo si es necesario.
    emon.current(adc_pin, 60.6);

    // EmonLib necesita calcular sobre varias muestras. 1480 es un buen número para 50Hz.
    double Irms = emon.calcIrms(1480);

    // Devolver como float
    return (float)Irms;
}

String get_utc_timestamp() { /* ... sin cambios ... */ }