/**
    Header que indica el pinout de los nodos LoRa.
    @file pinout.h
    @author Franco Abosso
    @author Julio Donadello
    @version 1.6 29/03/2021
*/

/*
        Pinout simplificado del nodo.
        Fuente: http://busyducks.com/ascii-art-arduinos
                         +-----+
            +------------| USB |------------+
            |            +-----+            |
            | [ ]D13/SCK        MISO/D12[ ] | - Reservado para RA-02.
            | [ ]3.3V           MOSI/D11[ ]~| - Reservado para RA-02.
            | [ ]V.ref     ___    SS/D10[ ]~| - Reservado para RA-02.
  DB9 (1) - | [ ]A0       / N \       D9[ ]~| - DB9 (1).
  DB9 (1) - | [ ]A1      /  A  \      D8[ ] | - DB9 (1).
  DB9 (1) - | [ ]A2      \  N  /      D7[ ] | - DB9 (1).
  DB9 (2) - | [ ]A3       \_0_/       D6[ ]~| - Cable USB / SparkON.
  DB9 (2) - | [ ]A4/SDA               D5[ ]~| - Cable USB / SparkON.
  DB9 (2) - | [ ]A5/SCL               D4[ ] | - Reservado para RA-02.
  DB9 (2) - | [ ]A6              INT1/D3[ ]~| - Reservado para RA-02.
  DB9 (2) - | [ ]A7              INT0/D2[ ] | - Reservado para RA-02.
            | [ ]5V                  GND[ ] |
            | [ ]RST                 RST[ ] |
            | [ ]GND   5V MOSI GND   TX1[ ] |  - DB9 (2).
            | [ ]Vin   [ ] [ ] [ ]   RX1[ ] |  - DB9 (2).
            |          [ ] [ ] [ ]          |
            |          MISO SCK RST         |
            |                               |
            +-------------------------------+
*/

// Pinout RA-02.
#define NSS_PIN 10
#define RESET_PIN -1
#define DIO0_PIN 2
#define DIO1_PIN 3
#define DIO2_PIN 4
#define RXTX_PIN -1

/*
    Sensores y actuadores del nodo interior:
        - Puerto RS232 (1):
            - Sensor de corriente = A1.
            - Sensor de lluvia = A0.
            - Sensor GPS = D8 (RX) + D9 (TX).
            - Actuador buzzer (y LED) = D7.
*/

// Pinout sensores y actuadores.
#define CORRIENTE_PIN A1
#define LLUVIA_PIN A0
#define BUZZER_PIN 7
#define RX_GPS_PIN 8
#define TX_GPS_PIN 9
#define COMBUSTIBLE_ECHO_PIN 6      // A través de cable SparkOn.
#define COMBUSTIBLE_TRIG_PIN 5      // A través de cable SparkOn.


// Instanciamiento de objetos relacionados al pinout.
EnergyMonitor eMon;
NewPing sonar(COMBUSTIBLE_TRIG_PIN, COMBUSTIBLE_ECHO_PIN, ULTRASONICO_DIST_MAX);
SoftwareSerial ssGPS(TX_GPS_PIN, RX_GPS_PIN); // hacemos cruce de señales por SW, respecto del método constructor (TX -> RX, RX -> TX)
TinyGPSPlus GPS;

/**
    setupPinout() determina las I/Os digitales y calibra el módulo sensor de corriente.
*/
void setupPinout() {
    #ifdef BUZZER_PIN
        pinMode(BUZZER_PIN, OUTPUT);
    #endif
    #ifdef COMBUSTIBLE_TRIG_PIN
        pinMode(COMBUSTIBLE_TRIG_PIN, OUTPUT);
    #endif
    #ifdef COMBUSTIBLE_ECHO_PIN
        pinMode(COMBUSTIBLE_ECHO_PIN, INPUT);
    #endif
    #ifdef CORRIENTE_PIN
        eMon.current(CORRIENTE_PIN, EMON_CALIBRATION); 
    #endif
}
