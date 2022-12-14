/**
    Header que contiene constantes relevantes para el accionar del programa principal.
    @file constants.h
    @author Franco Abosso
    @author Julio Donadello
    @version 1.0 29/03/2021
*/

/// Comunicación serial.
#define DEBUG_LEVEL 1   // Nivel de debug (0 inhabilita el puerto serial).
#define SERIAL_BPS 9600 // Bitrate de las comunicaciones por puerto serial (físico).
#define GPS_BPS 9600    // Bitrate de las comunicaciones por puerto serial del GPS (virtual).

/// Watchdog
#define USE_WATCHDOG_TMR FALSE
#define WATCHDOG_TMR 8

/// LoRa.
#define LORA_FREQ 433175000                                                         // Frecuencia de la transmisión LoRa (en Hz).
#define DEVICE_ID 20009                                                             // Identificador de este nodo.
#define BROADCAST_ID (DEVICE_ID - DEVICE_ID % 10000 + 9999)                         // ID broadcast para este tipo de nodo.
#define DEVICE_ID_MAX_SIZE 6                                                        // Tamaño máximo que se espera para cada DEVICE_ID entrante.
#define INCOMING_PAYLOAD_MAX_SIZE 100                                               // Tamaño máximo esperado del payload LoRa entrante.
#define INCOMING_FULL_MAX_SIZE (INCOMING_PAYLOAD_MAX_SIZE + DEVICE_ID_MAX_SIZE + 2) // Tamaño máximo esperado del mensaje entrante.
#define MAX_SIZE_OUTCOMING_LORA_REPORT 200                                          // Tamaño máximo esperado del payload LoRa saliente.
#define KNOWN_COMMANDS_SIZE 1                                                       // Cantidad de comandos LoRa conocidos.
#define LORA_TIMEOUT 20                                                             // Tiempo entre cada mensaje LoRa.
#define LORA_SYNC_WORD 0x34                                                         // Palabra de sincronización LoRa.

/// Arrays.
#define SENSORS_QTY 2          // Cantidad de sensores conectados.
#define TIMEOUT_READ_SENSORS 2 // Tiempo entre mediciones.
#define ARRAY_SIZE (LORA_TIMEOUT / TIMEOUT_READ_SENSORS + 3)
#define TIMING_SLOTS 4 // Cantidad de slots necesarios de timing (ver timing_helpers.h)

// Sensor de combustible.
#define TIME_VACIO 1200          // Tiempo de retorno de eco ultrasónico cuando el tanque está vacío (en us).
#define TIME_LLENO 500           // Tiempo de retorno de eco ultrasónico cuando el tanque está lleno (en us).
#define CAPACIDAD_COMBUSTIBLE 12 // Capacidad del tanque (en L).
#define PING_SAMPLES 5           // Cantidad de muestras ultrasónicos.
#define ULTRASONICO_DIST_MAX 300 // Distancia máxima medible por el ultrasónico (en cm).

// Sensor de corriente.
#define TRANSFORMER_RATIO 100 / 0.05
#define BURDEN_RESISTOR 33
#define IDEAL_CALIBRATION TRANSFORMER_RATIO / BURDEN_RESISTOR
#define MEASURED_CURRENT 5.03
#define REAL_CURRENT 5.23
#define EMON_CALIBRATION IDEAL_CALIBRATION * (REAL_CURRENT / MEASURED_CURRENT)
#define THRESHOLD_NOISE_CURRENT 0.5
#define EMON_CROSSINGS 20 // Cantidad de semi-ondas muestreadas para medir tensión y/o corriente.
#define EMON_TIMEOUT 1000 // Timeout de la rutina calcVI (en ms).

// Sensor de lluvia.
#define LLUVIA_THRESHOLD_VOLTAGE 2.5 // Tensión threshold cuando llueve.
#define LLUVIA_THRESHOLD_10BIT ((int)(LLUVIA_THRESHOLD_VOLTAGE * (1024 / 5.0)))
#define LLUVIA_ACTIVO LOW

// Sensor GPS.
#define GPS_DECIMAL_POSITIONS 5 // Cantidad de posiciones decimales para medir la longitud y latitud del GPS.

// Actuador buzzer.
#define BUZZER_ACTIVO HIGH
#define BUZZER_INACTIVO LOW

/// Valores mock.
// #define CORRIENTE_MOCK 0.26        // Corriente falsa.
// #define RAINDROP_MOCK 0            // Lluvia falsa.
// #define GAS_MOCK 10.11             // Nafta falsa.
// #define GPS_MOCK (const float[]){-34.57475, -58.43552, 15} // GPS falso.
