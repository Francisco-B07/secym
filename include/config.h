#ifndef CONFIG_H
#define CONFIG_H

// --- INFORMACIÓN DEL FIRMWARE ---
#define FIRMWARE_VERSION "1.0.0"

// --- Configuración WiFi ---
#ifndef WIFI_SSID
#define WIFI_SSID "TU_SSID_AQUI"
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "TU_PASSWORD_AQUI"
#endif

// --- Configuración MQTT (HiveMQ Cloud) ---
#define MQTT_BROKER "d7cc57e848fc4effa97498abc32223fc.s1.eu.hivemq.cloud"
#define MQTT_PORT 8883
#ifndef MQTT_USER
#define MQTT_USER "usuario_ejemplo"
#endif
#ifndef MQTT_PASSWORD
#define MQTT_PASSWORD "password_ejemplo"
#endif

// Identificadores para el Topic Dinámico ---
// CLIENT_ID: El identificador de tu cliente final (ej. el hospital, la farmacia).
#define CLIENT_ID "htalRawson"
// NODE_ID: El identificador único de este dispositivo ESP32. Debe coincidir
// con el 'node_id' en tu tabla 'devices' de Supabase.
#define NODE_ID "ESP32_SECYM_01"

// Parámetros para Reconexión con Retroceso Exponencial ---
#define INITIAL_RECONNECT_DELAY_MS 5000 // Empezar con 5 segundos
#define MAX_RECONNECT_DELAY_MS 120000   // Máximo de 2 minutos de espera
#define RECONNECT_MULTIPLIER 2          // Duplicar el tiempo de espera en cada fallo

// --- Configuración de Sensores ---
// Pines GPIO
#define ONE_WIRE_BUS_PIN 4
#define DHT_PIN 5
#define SCT013_1_PIN 34
#define SCT013_2_PIN 35

// Tipo de sensor DHT
#define DHT_TYPE DHT11

// Número de sensores DS18B20
#define DS18B20_COUNT 5

// Calibración para EmonLib (ajustar según tu resistencia de carga)
// Este valor se calcula a partir de la resistencia de carga (100Ω) y la relación de espiras del CT (2000:1)
// Calibración = (2000 / 100) = 20.0

#define EMON_CALIBRATION_1 20.0
#define EMON_CALIBRATION_2 20.0

// --- Configuración del Sistema ---
// Intervalo de publicación en milisegundos (ej. 1 minutos)
#define PUBLISH_INTERVAL 60000

#endif