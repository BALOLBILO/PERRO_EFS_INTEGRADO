
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <Preferences.h>
#include <WiFi.h>
#include <ESP32Time.h>
#include <Adafruit_AHTX0.h>

#include "AsyncMqttClient.h"
#include "time.h"
#include "Arduino.h"


////////////////////////////////////////////////////////////////
// BOTONES Y ESTADOS ORIGINALES
#define BOTON_MAS 33
#define BOTON_MENOS 32
#define BOTON_SENSOR 26
#define BOTON_GMT 25
#define BOTON_ENVIO 27
///////////////////////////////////////////////////////////////
#define P1 2
#define SENSOR 3
#define SENSOR2 34
#define SENSOR_MAS 346
#define SENSOR_MENOS 345

#define SENSOR_ESPERA 45
#define SENSOR_ESPERA2 455
#define CAMBIAR_SENSOR 398
#define CAMBIAR_SENSOR_ESPERA 399

#define EEPROM_ADDR_VU_MQ9 4
#define EEPROM_ADDR_VU_MQ2 8
#define EEPROM_ADDR_VU_LDR 12
#define EEPROM_ADDR_VU_TEMP 16
#define EEPROM_ADDR_VU_HUMEDAD 20

int mq4Pin = 34;  // Pin ADC donde conectaste el MQ-4
int valorMap = 0;

int cambiarSensor = 0;
unsigned long tiempo = 0;
unsigned long tiempoActual = 0;
int vuHumedad = 1;
int vuTemp = 1;
int vuLdr = 1;
int vuMQ9 = 1;
int vuMQ2 = 1;
unsigned long tiempo1 = 0;
unsigned long tiempoActual1 = 0;

int HUMEDAD = 0;
int MQ9 = 0;
int TEMP = 0;
String FECHA = "";
#define GMT 4
#define GMT_ESPERA 44
#define GMT_ESPERA2 444

#define GMT_MAS 8
#define GMT_MENOS 9

#define ENVIO 5
#define ENVIO_ESPERA 55
#define ENVIO_ESPERA2 555

#define ENVIO_MENOS 6
#define ENVIO_MAS 7

#define DELAY_LOOP 100  // ms entre lecturas

int estado = P1;
void programa(void);
///////////////////////////////////////////////////////////////
// LIBRERÍAS (UNIÓN DE AMBOS CÓDIGOS)

////////////////////////////////////////////////////////////////
// OBJETOS ORIGINALES
LiquidCrystal_I2C lcd(0x27, 16, 2);
Preferences preferences;

// *** WIFI ORIGINAL (COMENTADO PARA EVITAR CHOQUE CON MQTT) ***
// const char* ssid = "viano";
// const char* password = "sisa111000";
// const char* ntpServer = "pool.ntp.org";

// === WIFI / NTP VERSIÓN MQTT (OPCIÓN B) ===
const char* ssid = "MECA-IoT";
const char* password = "IoT$2026";
const char* ntpServer = "south-america.pool.ntp.org";

ESP32Time rtc;
int gmtOffset;
int contadoEnvio;

TaskHandle_t Tarea1;
TaskHandle_t Tarea2;

#define EEPROM_SIZE 64
#define EEPROM_ADDR_ENVIO 0

Adafruit_AHTX0 aht;

unsigned long time1 = 0;
unsigned long actualTime = 0;

////////////////////////////////////////////////////////////////
// ====== CÓDIGO MQTT INTEGRADO "COMO ESTÁ AHÍ" ======

// wifi (ya están ssid/password arriba)
const char name_device = 11;  ////device numero de grupo

// Timers auxiliar variables//////////////////////////
unsigned long now = millis();    ///valor actual
unsigned long lastMeasure1 = 0;  ///variable para contar el tiempo actual
unsigned long lastMeasure2 = 0;  ///variable para contar el tiempo actual

unsigned long interval_envio = 30000;  // valor por defecto
unsigned long interval_leeo = 60000;   // valor por defecto
int i = 0;

///time
long unsigned int timestamp;  // hora
// ntpServer ya está arriba, usamos el de MQTT
const long gmtOffset_sec = -10800;
const int daylightOffset_sec = 0;

///variables ingresar a la cola struct
int indice_entra = 0;
int indice_saca = 0;
bool flag_vacio = 1;

/////mqqtt
#define MQTT_HOST IPAddress(192, 168, 5, 123)
#define MQTT_PORT 1884
#define MQTT_USERNAME "esp32"
#define MQTT_PASSWORD "mirko15"
char mqtt_payload[150];  /////
// Test MQTT Topic
#define MQTT_PUB "/esp32/datos_sensores"
AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

typedef struct
{
  long time;
  float T1;     ///temp en grados
  float H1;     ///valor entre 0 y 99 // mojado es cercano al 100
  float luz;    ///valor entre 0 y 99 . si hay luz es cercano al 100
  float G1;     ///valor entre 0 y 99
  float G2;     ///valor entre 0 y 99
  bool oct;     ///Lectura del octoacoplador
  bool Alarma;  //
} estructura;

const int valor_max_struct = 1000;
estructura datos_struct[valor_max_struct];
estructura aux2;

////////////////////////////////////////////////////////////////
// PROTOTIPOS MQTT
void setupmqtt();
void fun_envio_mqtt();
void connectToWifi();
void connectToMqtt();
void WiFiEvent(WiFiEvent_t event);
void onMqttConnect(bool sessionPresent);
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason);
void onMqttPublish(uint16_t packetId);
void fun_saca();
void fun_entra(void);

////////////////////////////////////////////////////////////////
// SETUP INTEGRADO
void setup() {
  tiempo1 = millis();
  time1 = millis();
  Serial.begin(115200);

  Wire.begin(21, 22);
  lcd.init();
  lcd.backlight();

  pinMode(BOTON_MAS, INPUT_PULLUP);
  pinMode(BOTON_MENOS, INPUT_PULLUP);
  pinMode(BOTON_SENSOR, INPUT_PULLUP);
  pinMode(BOTON_GMT, INPUT_PULLUP);
  pinMode(BOTON_ENVIO, INPUT_PULLUP);

  // --- EEPROM ---
  EEPROM.begin(EEPROM_SIZE);
  contadoEnvio = EEPROM.readUInt(EEPROM_ADDR_ENVIO);
  if (contadoEnvio == 0xFFFFFFFF || contadoEnvio == 0) {
    contadoEnvio = 30;  // valor por defecto si está vacío
  }
  interval_envio = (unsigned long)contadoEnvio * 1000UL;
  interval_leeo = (unsigned long)contadoEnvio * 2000UL;

  vuMQ9 = EEPROM.readUInt(EEPROM_ADDR_VU_MQ9);
  if (vuMQ9 == 0xFFFFFFFF || vuMQ9 == 0) vuMQ9 = 1;
  vuMQ2 = EEPROM.readUInt(EEPROM_ADDR_VU_MQ2);
  if (vuMQ2 == 0xFFFFFFFF || vuMQ2 == 0) vuMQ2 = 1;
  vuLdr = EEPROM.readUInt(EEPROM_ADDR_VU_LDR);
  if (vuLdr == 0xFFFFFFFF || vuLdr == 0) vuLdr = 1;
  vuTemp = EEPROM.readUInt(EEPROM_ADDR_VU_TEMP);
  if (vuTemp == 0xFFFFFFFF || vuTemp == 0) vuTemp = 1;
  vuHumedad = EEPROM.readUInt(EEPROM_ADDR_VU_HUMEDAD);
  if (vuHumedad == 0xFFFFFFFF || vuHumedad == 0) vuHumedad = 1;

  // --- Preferences (para GMT) ---
  preferences.begin("memoria", false);
  gmtOffset = preferences.getInt("gmtOffset", -3);

  // ====== WIFI + MQTT COMO EN TU CÓDIGO MQTT ======
  setupmqtt();  // configura timers, WiFi events y hace connectToWifi()

  // Setup de time para código MQTT
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  // --- Sincronización NTP para tu RTC y menú GMT ---
  configTime(gmtOffset * 3600, 0, ntpServer);
  sincronizarHora();

  // --- Tareas FreeRTOS originales ---
  xTaskCreatePinnedToCore(loop1, "Loop1", 10000, NULL, 1, &Tarea1, 0);
  xTaskCreatePinnedToCore(loop2, "Loop2", 10000, NULL, 1, &Tarea2, 1);

  if (!aht.begin()) {
    Serial.println("No se encontró el AHT10/AHT20, revisa conexiones!");
    while (1) delay(10);
  }
  Serial.println("Sensor AHT10 inicializado correctamente");
  Serial.println(contadoEnvio);
}

////////////////////////////////////////////////////////////////
// LOOP INTEGRADO (MQTT "COMO ESTÁ AHÍ")
void loop() {
  now = millis();
  if (now - lastMeasure1 > interval_envio) {  ////envio el doble de lectura por si falla algun envio
    lastMeasure1 = now;                       /// cargo el valor actual de millis
    fun_envio_mqtt();                         ///envio los valores por mqtt
  }
  if (now - lastMeasure2 > interval_leeo) {
    lastMeasure2 = now;  /// cargo el valor actual de millis
    fun_entra();         ///ingreso los valores a la cola struct
  }
}

////////////////////////////////////////////////////////////////
// TAREA 1: LECTURAS Y ALERTAS (ANTES ERA LOOP1)
void loop1(void* pvParameters) {
  for (;;) {
    actualTime = millis();
    if (actualTime - time1 >= (unsigned long)contadoEnvio * 1000UL) {

      HUMEDAD = valorHumedad();
      MQ9 = valorMQ9();
      TEMP = valorTemp();
      FECHA = obtenerFechaHoraLocal();

      Serial.print("HUMEDAD: ");
      Serial.println(HUMEDAD);
      Serial.print("VU DE HUMEDAD: ");
      Serial.println(vuHumedad);

      Serial.print("MQ9: ");
      Serial.println(MQ9);
      Serial.print("VU DE MQ9: ");
      Serial.println(vuMQ9);

      Serial.print("TEMPERATURA: ");
      Serial.println(TEMP);
      Serial.print("VU TEMPERATURA: ");
      Serial.println(vuTemp);

      Serial.println(FECHA);

      // (ANTES AQUÍ IBA TELEGRAM — ELIMINADO)

      time1 = millis();
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

////////////////////////////////////////////////////////////////
// TAREA 2: MENÚ Y MÁQUINA DE ESTADOS (ANTES ERA LOOP2)
void loop2(void* pvParameters) {
  for (;;) {
    tiempoActual1 = millis();
    if (tiempoActual1 - tiempo1 >= 15) {
      programa();
      tiempo1 = millis();
    }

    vTaskDelay(DELAY_LOOP / portTICK_PERIOD_MS);
  }
}

////////////////////////////////////////////////////////////////
// PROGRAMA (MENÚ LCD)
void programa(void) {
  switch (estado) {

    case P1:
      lcd.setCursor(0, 0);
      lcd.print("MENU ");

      // Mostrar TEMP y HUMEDAD
      lcd.setCursor(0, 1);
      lcd.print("T:");
      lcd.print(TEMP);
      lcd.print(" H:");
      lcd.print(HUMEDAD);

      // Mostrar MQ9 en la esquina final
      lcd.print(" M:");
      lcd.print(MQ9);

      // Botones que ya estaban
      if (digitalRead(BOTON_GMT) == LOW) {
        lcd.clear();
        Serial.println("BOTON_GMT presionado");
        estado = GMT_ESPERA;
      }

      if (digitalRead(BOTON_SENSOR) == LOW) {
        lcd.clear();
        Serial.println("BOTON_SENSOR presionado");
        estado = SENSOR_ESPERA;
      }

      if (digitalRead(BOTON_ENVIO) == LOW) {
        lcd.clear();
        Serial.println("BOTON_ENVIO presionado");
        estado = ENVIO_ESPERA;
      }

      break;


    case SENSOR_ESPERA:
      if (digitalRead(BOTON_SENSOR) == HIGH) {
        estado = SENSOR;
      }
      break;

    case SENSOR:
      Serial.println("sensor");
      if (digitalRead(BOTON_MAS) == LOW) {
        Serial.println("+");
        estado = SENSOR_MAS;
      }
      if (digitalRead(BOTON_MENOS) == LOW) {
        Serial.println("-");
        estado = SENSOR_MENOS;
      }
      if (cambiarSensor == 0) {
        lcd.setCursor(0, 0);
        lcd.print("MQ 9: ");
        lcd.print(vuMQ9);
      }
      if (cambiarSensor == 1) {
        lcd.setCursor(0, 0);
        lcd.print("MQ2: ");
        lcd.print(vuMQ2);
      }
      if (cambiarSensor == 2) {
        lcd.setCursor(0, 0);
        lcd.print("LDR: ");
        lcd.print(vuLdr);
      }
      if (cambiarSensor == 3) {
        lcd.setCursor(0, 0);
        lcd.print("TEMP: ");
        lcd.print(vuTemp);
      }
      if (cambiarSensor == 4) {
        lcd.setCursor(0, 0);
        lcd.print("HUMEDAD: ");
        lcd.print(vuHumedad);
      }
      if (digitalRead(BOTON_SENSOR) == LOW) {
        tiempo = millis();
        estado = CAMBIAR_SENSOR;
      }
      break;

    case CAMBIAR_SENSOR:
      tiempoActual = millis();
      if (digitalRead(BOTON_SENSOR) == HIGH) {
        Serial.println("+");
        cambiarSensor = cambiarSensor + 1;
        if (cambiarSensor >= 5) {
          cambiarSensor = 0;
        }
        lcd.clear();

        estado = SENSOR;
      }

      if (tiempoActual - tiempo >= 3000) {
        lcd.clear();
        estado = SENSOR_ESPERA2;
      }

      break;

    case SENSOR_MAS:

      if (digitalRead(BOTON_MAS) == HIGH) {

        if (cambiarSensor == 0) {
          vuMQ9 = vuMQ9 + 1;
        }
        if (cambiarSensor == 1) {
          vuMQ2 = vuMQ2 + 1;
        }
        if (cambiarSensor == 2) {
          vuLdr = vuLdr + 1;
        }
        if (cambiarSensor == 3) {
          vuTemp = vuTemp + 1;
        }
        if (cambiarSensor == 4) {
          vuHumedad = vuHumedad + 1;
        }
        lcd.clear();
        EEPROM.writeUInt(EEPROM_ADDR_VU_MQ9, vuMQ9);
        EEPROM.writeUInt(EEPROM_ADDR_VU_MQ2, vuMQ2);
        EEPROM.writeUInt(EEPROM_ADDR_VU_LDR, vuLdr);
        EEPROM.writeUInt(EEPROM_ADDR_VU_TEMP, vuTemp);
        EEPROM.writeUInt(EEPROM_ADDR_VU_HUMEDAD, vuHumedad);
        EEPROM.commit();

        estado = SENSOR;
      }
      break;

    case SENSOR_MENOS:

      if (digitalRead(BOTON_MENOS) == HIGH) {

        if (cambiarSensor == 0) {
          if (vuMQ9 >= 2) {
            vuMQ9 = vuMQ9 - 1;
          }
        }
        if (cambiarSensor == 1) {
          if (vuMQ2 >= 2) {
            vuMQ2 = vuMQ2 - 1;
          }
        }
        if (cambiarSensor == 2) {
          if (vuLdr >= 2) {
            vuLdr = vuLdr - 1;
          }
        }
        if (cambiarSensor == 3) {
          if (vuTemp >= 2) {
            vuTemp = vuTemp - 1;
          }
        }
        if (cambiarSensor == 4) {
          if (vuHumedad >= 2) {
            vuHumedad = vuHumedad - 1;
          }
        }

        lcd.clear();
        EEPROM.writeUInt(EEPROM_ADDR_VU_MQ9, vuMQ9);
        EEPROM.writeUInt(EEPROM_ADDR_VU_MQ2, vuMQ2);
        EEPROM.writeUInt(EEPROM_ADDR_VU_LDR, vuLdr);
        EEPROM.writeUInt(EEPROM_ADDR_VU_TEMP, vuTemp);
        EEPROM.writeUInt(EEPROM_ADDR_VU_HUMEDAD, vuHumedad);
        EEPROM.commit();

        estado = SENSOR;
      }
      break;

    case SENSOR_ESPERA2:

      if (digitalRead(BOTON_SENSOR) == HIGH) {
        estado = P1;
      }
      break;

    // ======== ENVÍO ========
    case ENVIO_ESPERA:
      if (digitalRead(BOTON_ENVIO) == HIGH) estado = ENVIO;
      break;

    case ENVIO:
      lcd.setCursor(0, 0);
      lcd.print("Tiempo Envio:");
      lcd.setCursor(0, 1);
      lcd.print(contadoEnvio);
      lcd.print(" seg   ");
      Serial.println("envio");

      if (digitalRead(BOTON_MAS) == LOW) estado = ENVIO_MAS;
      if (digitalRead(BOTON_MENOS) == LOW) estado = ENVIO_MENOS;
      if (digitalRead(BOTON_ENVIO) == LOW) {
        lcd.clear();

        EEPROM.writeUInt(EEPROM_ADDR_ENVIO, contadoEnvio);
        EEPROM.commit();
        Serial.print("Guardado EEPROM envio: ");
        Serial.println(contadoEnvio);
        interval_envio = (unsigned long)contadoEnvio * 1000UL;  // <-- AQUI
        interval_leeo = (unsigned long)contadoEnvio * 2000UL;

        estado = ENVIO_ESPERA2;
      }
      break;

    case ENVIO_MAS:
      if (digitalRead(BOTON_MAS) == HIGH) {
        contadoEnvio = contadoEnvio + 30;
        if (contadoEnvio > 600) contadoEnvio = 600;
        lcd.clear();
        estado = ENVIO;
      }
      break;

    case ENVIO_MENOS:
      if (digitalRead(BOTON_MENOS) == HIGH) {
        if (contadoEnvio > 30) contadoEnvio = contadoEnvio - 30;
        lcd.clear();
        estado = ENVIO;
      }
      break;

    case ENVIO_ESPERA2:
      if (digitalRead(BOTON_ENVIO) == HIGH) {
        estado = P1;
      }
      break;

    // ======== GMT ========
    case GMT_ESPERA:
      if (digitalRead(BOTON_GMT) == HIGH) {
        estado = GMT;
      }
      break;

    case GMT:
      lcd.setCursor(0, 0);
      lcd.print("Ajuste GMT:");
      lcd.setCursor(0, 1);
      lcd.print(gmtOffset);
      Serial.println("gmt");

      if (digitalRead(BOTON_GMT) == LOW) estado = GMT_ESPERA2;
      if (digitalRead(BOTON_MAS) == LOW) estado = GMT_MAS;
      if (digitalRead(BOTON_MENOS) == LOW) estado = GMT_MENOS;
      break;

    case GMT_MAS:
      if (digitalRead(BOTON_MAS) == HIGH) {
        Serial.println("gmt_mas");
        gmtOffset++;
        if (gmtOffset > 12) gmtOffset = -12;
        configTime(gmtOffset * 3600, 0, ntpServer);
        sincronizarHora();
        preferences.putInt("gmtOffset", gmtOffset);
        lcd.clear();
        estado = GMT;
      }
      break;

    case GMT_MENOS:
      if (digitalRead(BOTON_MENOS) == HIGH) {
        Serial.println("gmt_menos");
        gmtOffset--;
        if (gmtOffset < -12) gmtOffset = 12;
        configTime(gmtOffset * 3600, 0, ntpServer);
        sincronizarHora();
        preferences.putInt("gmtOffset", gmtOffset);
        lcd.clear();
        estado = GMT;
      }
      break;

    case GMT_ESPERA2:
      if (digitalRead(BOTON_GMT) == HIGH) {
        lcd.clear();
        estado = P1;
      }
      break;
  }
}

////////////////////////////////////////////////////////////////
void sincronizarHora() {
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) rtc.setTimeStruct(timeinfo);
  else Serial.println("Error al sincronizar hora");
}

/////////////////////////////////////////////////////////////
int valorMQ9(void) {
  int valor = analogRead(mq4Pin);          // Lee el valor del sensor
  valorMap = map(valor, 0, 4095, 0, 100);  // Mapea de 0-4095 a 0-100%
  return valorMap;
}

int valorHumedad(void) {
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);  // siempre se leen los dos
  return (int)humidity.relative_humidity;
}

int valorTemp(void) {
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);
  return (int)temp.temperature;
}

String obtenerFechaHoraLocal(void) {
  struct tm timeinfo;
  configTime(gmtOffset * 3600, 0, ntpServer);

  if (getLocalTime(&timeinfo)) {
    char buffer[25];
    strftime(buffer, sizeof(buffer), "%d/%m/%Y %H:%M:%S", &timeinfo);
    return String(buffer);
  } else {
    return "Error obteniendo hora";
  }
}

////////////////////////////////////////////////////////////////
// ============ IMPLEMENTACIÓN FUNCIONES MQTT ============

void setupmqtt() {
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0,
                                    reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0,
                                    reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));
  WiFi.onEvent(WiFiEvent);
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials(MQTT_USERNAME, MQTT_PASSWORD);
  connectToWifi();
}

void fun_envio_mqtt() {
  fun_saca();           ////veo si hay valores nuevos
  if (flag_vacio == 0)  ////si hay los envio
  {
    Serial.print("enviando");
    snprintf(mqtt_payload, 150, "%u&%ld&%.2f&%.2f&%.2f&%.2f&%.2f&%u&%u",
             name_device,
             aux2.time,
             aux2.T1,
             aux2.H1,
             aux2.luz,
             aux2.G1,
             aux2.G2,
             aux2.oct,
             aux2.Alarma);

    aux2.time = 0;
    aux2.T1 = 0;
    aux2.H1 = 0;
    aux2.luz = 0;
    aux2.G1 = 0;
    aux2.G2 = 0;
    aux2.oct = 0;
    aux2.Alarma = 0;

    Serial.print("Publish message: ");
    Serial.println(mqtt_payload);
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB, 1, true, mqtt_payload);
    (void)packetIdPub1;
  } else {
    Serial.println("no hay valores nuevos");
  }
}

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0);
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void fun_saca() {
  if (indice_saca != indice_entra) {
    aux2.time = datos_struct[indice_saca].time;
    aux2.T1 = datos_struct[indice_saca].T1;
    aux2.H1 = datos_struct[indice_saca].H1;
    aux2.luz = datos_struct[indice_saca].luz;
    aux2.G1 = datos_struct[indice_saca].G1;
    aux2.G2 = datos_struct[indice_saca].G2;
    aux2.oct = datos_struct[indice_saca].oct;
    aux2.Alarma = datos_struct[indice_saca].Alarma;

    flag_vacio = 0;

    Serial.println(indice_saca);
    if (indice_saca >= (valor_max_struct - 1)) {
      indice_saca = 0;
    } else {
      indice_saca++;
    }
    Serial.print("saco valores de la struct isaca:");
    Serial.println(indice_saca);
  } else {
    flag_vacio = 1;  ///// no hay datos
  }
}

void fun_entra(void) {
  if (indice_entra >= valor_max_struct) {
    indice_entra = 0;  ///si llego al maximo de la cola se vuelve a cero
  }

  Serial.print("> NTP Time:");
  timestamp = time(NULL);
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");

  datos_struct[indice_entra].time = timestamp;
  // SENSORES REALES
  datos_struct[indice_entra].T1 = TEMP;     // temperatura real
  datos_struct[indice_entra].H1 = HUMEDAD;  // humedad real
  datos_struct[indice_entra].G1 = MQ9;      // MQ9 real

  // SENSORES HARDCODEADOS QUE QUERÉS MANTENER
  datos_struct[indice_entra].luz = 30;
  datos_struct[indice_entra].G2 = 1.5;
  datos_struct[indice_entra].oct = 1.5;
  datos_struct[indice_entra].Alarma = 1;

  indice_entra++;
  Serial.print("saco valores de la struct ientra");
  Serial.println(indice_entra);
}
