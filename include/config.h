// -- WiFi Configuration
#define WIFI_SSID "SSID_Hospital_IoT"
#define WIFI_PASSWORD "Password_Seguro_WiFi"

// -- MQTT Broker Configuration
#define MQTT_BROKER "xxxx.supabase.co"
#define MQTT_PORT 8883
#define MQTT_USER "service_role"
#define MQTT_PASSWORD "SUPABASE_SERVICE_ROLE_KEY"
#define MQTT_TOPIC_DATA "realtime/topic/esp32/sensor_data"

// -- Node Configuration
#define NODE_ID "ESP32_NODO_LABORATORIO_01"

// -- Timing Configuration (en milisegundos)
#define PUBLISH_INTERVAL 300000       // 5 minutos
#define WIFI_RECONNECT_INTERVAL 20000 // Intentar reconectar WiFi cada 20 segundos
#define MQTT_RECONNECT_INTERVAL 10000 // Intentar reconectar MQTT cada 10 segundos

// -- Hardware Pin Configuration
#define ONEWIRE_BUS_PIN 4 // Pin para el bus de sensores DS18B20

// <-- CAMBIO: Se eliminan los pines del multiplexor.
// <-- CAMBIO: Se define un array para los pines ADC de cada sensor de corriente.
// ¡IMPORTANTE! En ESP32, usa pines del ADC1 (ej. 32-39) cuando uses WiFi.
const int current_sensor_adc_pins[] = {36, 39}; // Usando GPIO36 (ADC1_CH0) y GPIO39 (ADC1_CH3)

// -- Asset Mapping (¡CRUCIAL!)
// Mapea la dirección física del sensor a un ID de activo lógico.
const char *asset_ids[] = {"HELADERA_VACUNAS_01", "FREEZER_MUESTRAS_02"};
const uint8_t ds18b20_addresses[][8] = {
    {0x28, 0xFF, 0x64, 0x1E, 0x54, 0x3F, 0x2A, 0x9B}, // Dirección del sensor de la Heladera 01
    {0x28, 0xAA, 0x7E, 0x2B, 0x6, 0x0, 0x0, 0x81}     // Dirección del sensor del Freezer 02
};

// Mapea el pin del sensor de puerta al mismo ID de activo
const int door_sensor_pins[] = {25, 26}; // Pines GPIO
const int NUM_ASSETS = sizeof(asset_ids) / sizeof(asset_ids[0]);

// -- NTP Server for UTC Timestamp
#define NTP_SERVER "pool.ntp.org"
#define GMT_OFFSET_SEC -10800 // Para Argentina (GMT-3)
#define DAYLIGHT_OFFSET_SEC 0

// -- Root CA Certificate for Supabase/MQTT Broker (TLS)
const char *root_ca =
    "-----BEGIN CERTIFICATE-----\n" // ... (Pega aquí el certificado CA de tu broker) ...
    "-----END CERTIFICATE-----\n";