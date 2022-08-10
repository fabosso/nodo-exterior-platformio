/**
    Rutina principal que obtiene los valores sensados y los modula
    y transmite sobre LoRa@433MHz en una string especialmente diseñada.
    @file nodo-sisicic.ino
    @author Franco Abosso
    @author Julio Donadello
    @version 1.1 29/03/2021
*/

/// Headers iniciales (preceden a la declaración de variables).

// Header que contiene constantes relevantes al accionar de este programa.
#include "constants.h"          // Biblioteca propia.

// Bibliotecas necesarias para manejar al SX1278.
#include <SPI.h>                // https://www.arduino.cc/en/reference/SPI
#include <LoRa.h>               // https://github.com/sandeepmistry/arduino-LoRa

// Biblioteca utilizada para manejar al HC-SR04.
#include <NewPing.h>            // https://bitbucket.org/teckel12/arduino-new-ping/wiki/Home

// Biblioteca necesaria para calcular los valores RMS de los sensores de tensión y corriente.
#include <EmonLib.h>            // https://learn.openenergymonitor.org/electricity-monitoring/ctac/how-to-build-an-arduino-energy-monitor

// Biblioteca necesaria para reservar espacios de memoria fijos para las Strings utilizadas.
#include <StringReserveCheck.h> // https://www.forward.com.au/pfod/ArduinoProgramming/ArduinoStrings/index.html

// Biblioteca necesaria para manejar el GPS.
#include <TinyGPS++.h>          // https://github.com/mikalhart/TinyGPSPlus

// Biblioteca necesaria para emular otro puerto serie.
#include <SoftwareSerial.h>     // https://www.arduino.cc/en/Reference/SoftwareSerial

/// Declaración de variables globales.

/**
    currents es un array de floats que contiene los valores de corriente medidos entre cada
    transmisión LoRa. El tamaño del array depende del intervalo de tiempo entre cada transmisión
    LoRa y el intervalo de tiempo entre cada medición.
    El valor que se transmite por LoRa en realidad es el valor promedio de este array.
    Una vez realizada la transmisión, todos los valores de este array vuelven a ponerse en 0.
*/
float currents[ARRAY_SIZE] = {0.0};

/**
    raindrops es un array de enteros con signo que contiene los resultados del polleo del pin de lluvia
    efectuados entre cada transmisión LoRa.
    El tamaño del array depende del intervalo de tiempo entre cada transmisión 
    LoRa y el intervalo de tiempo entre cada medición.
    El valor que se transmite por LoRa en realidad es el resultado de una votación para evitar falsos positivos.
    Una vez realizada la transmisión, todos los valores de este array vuelven a ponerse en -1.
*/
int raindrops[ARRAY_SIZE] = {-1};

/**
    gas es un float que almacena la cantidad de litros de combustible presentes
    en el grupo electrógeno.
*/
float gas = 0.0;

/**
    index es una variable de tipo int utilizado para recorrer los arrays de medición.
    Una vez realizada la transmisión, vuelve a ponerse en 0.
*/
int index = 0;

/**
    refreshRequested contiene SENSORS_QTY variables booleanas que representan la necesidad
    inmediata de refrescar los valores de los arrays de medición. Estos tienen un orden arbitrario:
    { Corriente, Lluvia  }
    Una vez refrescado, cada uno de estos booleanos vuelve a ponerse en false.
*/
bool refreshRequested[SENSORS_QTY] = {false};

/**
    gasRequested es un flag que representa la necesidad inmediata de volver a medir el nivel
    de combustible del grupo electrógeno.
    Se diferencia de los otros sensores (los controlados por refreshRequested) porque esta medición
    se realiza una vez cada TIMEOUT_LORA segundos (no una vez cada TIMEOUT_READ_SENSORS segundos).
*/
bool gasRequested = true;

/**
    GPSRequested es un flag que representa la necesidad inmediata de volver a leer la posición
    del GPS.
    Se diferencia de los otros sensores (los controlados por refreshRequested) porque esta medición
    se realiza una vez cada TIMEOUT_LORA segundos (no una vez cada TIMEOUT_READ_SENSORS segundos).
*/
bool GPSRequested = true;

/**
    outcomingFull es una string que contiene el mensaje LoRa de salida preformateado especialmente
    para que, posteriormente, el concentrador LoRa pueda decodificarla.
*/
String outcomingFull;

/**
    incomingFull es una string que contiene el mensaje LoRa de entrada, incluyendo
    el identificador de nodo.
*/
String incomingFull;

/**
    greaterThanStr (>) es el delimitador entre el identificador de nodo y la carga útil
    del mensaje de entrada LoRa. Se carga en memoria una única vez en una String constante
    para evitar problemas de memoria heap.
*/
const String greaterThanStr = ">";

/**
    receiverStr es una string que sólo contiene el identificador de nodo
    recibido en un mensaje LoRa entrante.
*/
String receiverStr;

/**
    incomingPayload es una string que contiene sólo la carga útil del mensaje LoRa de entrada,
    utilizada sólo cuando el identificador de nodo coincide con DEVICE_ID o con BROADCAST_ID.
*/
String incomingPayload;

/**
    knownCommands es un array de Strings que contiene los comandos LoRa que
    se pueden ejecutar.
*/
const String knownCommands[KNOWN_COMMANDS_SIZE] = {
    "startAlert"    // inicia una alerta con el siguiente llamado a función: startAlert(750, 10);
};

/**
    latStr es una String que almacena temporalmente el valor de latitud devuelto por el GPS.
*/
String latStr;

/**
    lngStr es una String que almacena temporalmente el valor de longitud devuelto por el GPS.
*/
String lngStr;

/**
    altStr es una String que almacena temporalmente el valor de altitud devuelto por el GPS.
*/
String altStr;

/// Headers finales (proceden a la declaración de variables).

#include "pinout.h"             // Biblioteca propia.
#include "alerts.h"             // Biblioteca propia.
#include "timing_helpers.h"     // Biblioteca propia.
#include "sensors.h"            // Biblioteca propia.
#include "actuators.h"          // Biblioteca propia.
#include "decimal_helpers.h"    // Biblioteca propia.
#include "array_helpers.h"      // Biblioteca propia.
#include "LoRa_helpers.h"       // Biblioteca propia.

/// Funciones principales.

/**
    setup() lleva a cabo las siguientes tareas:
        - setea el pinout,
        - inicializa el periférico serial (real),
        - reserva espacios de memoria para las Strings,
        - inicializa el periférico serial del GPS (virtual),
        - inicializa el módulo LoRa.
    Si después de realizar estas tareas no se "cuelga", da inicio
    a una alerta "exitosa".
*/
void setup() {
    setupPinout();
    #if DEBUG_LEVEL >= 1
        Serial.begin(SERIAL_BPS);
    #endif
    reserveMemory();
    LoRaInitialize();
    ssGPS.begin(GPS_BPS);
    startAlert(133, 3);
}

/**
    loop() determina las tareas que cumple el programa:
        - cada TIMEOUT_LORA segundos, envía un payload LoRa.
        - si no está ocupado con eso:
            - se ocupa de disparar las alertas preestablecidas.
            - cada TIMEOUT_READ_SENSORS segundos, refresca el estado de los sensores.
    Esta función se repite hasta que se le dé un reset al programa.
*/
void loop() {
    if (runEvery(sec2ms(TIMEOUT_LORA), 1)) {
        // Deja de refrescar TODOS los sensores.
        stopRefreshingAllSensors();

        // Compone la carga útil de LoRa.
        composeLoRaPayload(currents, raindrops, gas, outcomingFull);

        #if DEBUG_LEVEL >= 1
            Serial.print("Payload LoRa encolado!: ");
            Serial.println(outcomingFull);
        #endif

        // Compone y envía el paquete LoRa.
        LoRa.beginPacket();
        LoRa.print(outcomingFull);
        LoRa.endPacket();

        // Pone al módulo LoRa en modo recepción.
        LoRa.receive();

        // Inicia la alerta preestablecida.
        startAlert(133, 3);

        // Reestablece los arrays de medición.
        cleanupArray(currents, ARRAY_SIZE);
        cleanupArray(raindrops, ARRAY_SIZE);

        // Reestablece el index de los arrays de medición.
        index = 0;

        // Vuelve a pedir que se refresque el estado del nivel de combustible.
        gasRequested = true;
    }

    // Chequea la necesidad de inicializar alertas.
    callbackAlert();

    // Chequea la necesidad de realizar comandos remotos.
    callbackLoRaCommand();

    if(runEvery(sec2ms(TIMEOUT_READ_SENSORS), 2)) {
        // Refresca TODOS los sensores dependientes de refreshRequested.
        refreshAllSensors();
        // Avanza el índice de TODOS los arrays de medición.
        index++;
        // Vuelve a pedir que se refresque el estado del GPS.
        GPSRequested = true;
    }

    if (!resetAlert && !pitidosRestantes) {
        if (refreshRequested[0]) {
            // Obtiene un nuevo valor de corriente.
            #ifndef CORRIENTE_MOCK
                eMon.calcVI(EMON_CROSSINGS, EMON_TIMEOUT);
            #endif
            getNewCurrent();
        }
        if (refreshRequested[1]) {
            // Obtiene un nuevo valor de lluvia.
            getNewRaindrop();
        }

        if (gasRequested) {
            // Obtiene un nuevo valor de combustible.
            getNewGas();
        }

        if (GPSRequested) {
            // Obtiene un nuevo valor de GPS.
            getNewGPS();
        }
    }

}
