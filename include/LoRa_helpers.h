/**
    Header que contiene funcionalidades referidas al módulo SX1278.
    @file LoRa_helpers.h
    @author Franco Abosso
    @author Julio Donadello
    @version 1.2 29/03/2021
*/

/*
    onRecieve() es la función por interrupción que se llama cuando
    existen datos en el buffer LoRa.
*/
void onReceive(int packetSize) {
    #if DEBUG_LEVEL >= 2
        Serial.println("Entering recieve mode");
    #endif

    // Si el tamaño del paquete entrante es nulo,
    // o si es superior al tamaño reservado para la string incomingFull
    // salir de la subrutina.
    if (packetSize == 0 || packetSize > INCOMING_FULL_MAX_SIZE) {
        return;
    }

    // No se puede utilizar readString() en un callback.
    // Se añaden los bytes uno por uno.
    while (LoRa.available()) {
        incomingFull += (char)LoRa.read();
    }

    // Se levanta un flag de finalización de lectura LoRa.
    incomingFullComplete = true;
}

void downlinkObserver() {
    if (incomingFullComplete) {
        // Extraer el delimitador ">" para diferenciar el ID del payload.
        int delimiter = incomingFull.indexOf(greaterThanStr);

        // Obtener el ID de receptor.
        receiverStr = incomingFull.substring(1, delimiter);
        int receiverID = receiverStr.toInt();
        #if DEBUG_LEVEL >= 1
            Serial.print("Receiver: ");
            Serial.println(receiverID);
        #endif

        // Si el ID del receptor coincide con nuestro ID o si es un broadcast:
        if (receiverID == DEVICE_ID || receiverID == BROADCAST_ID) {
            // Obtiene el payload entrante.
            incomingPayload = incomingFull.substring(delimiter + 1);
            #if DEBUG_LEVEL >= 1
                Serial.println("ID coincide!");
            #endif
        } else {
            #if DEBUG_LEVEL >= 2
                Serial.println("Descartado por ID!");
            #endif
        }

        // Limpiar variables.
        incomingFullComplete = false;
        incomingFull = "";
        receiverStr = "";
    }
}

/**
    LoRaInitialize() inicializa el módulo SX1278 con:
        - la frecuencia (LORA_FREQ) y la palabra de sincronización (LORA_SYNC_WORD) indicados en constants.h
        - los pines (NSS_PIN, RESET_PIN, DIO0_PIN) indicados en pinout.h,
    Además, define la función onRecieve como callback del evento onRecieve.
    Si por algún motivo fallara, "cuelga" al programa.
*/
void LoRaInitialize() {
    LoRa.setPins(NSS_PIN, RESET_PIN, DIO0_PIN);

    if (!LoRa.begin(LORA_FREQ)) {
        Serial.println("Starting LoRa failed!");
        blockingAlert(2000, 10);
        while (1);
    }
    LoRa.setSyncWord(LORA_SYNC_WORD);
    LoRa.onReceive(onReceive);
    LoRa.receive();

    #if DEBUG_LEVEL >= 1
        Serial.println("LoRa initialized OK.");
    #endif
}

/**
    reserveMemory() reserva memoria para las Strings.
    En caso de quedarse sin memoria, alerta por puerto serial
    e inicia una alerta de falla
*/
void reserveMemory() {
    receiverStr.reserve(DEVICE_ID_MAX_SIZE);
    incomingPayload.reserve(INCOMING_PAYLOAD_MAX_SIZE);
    incomingFull.reserve(INCOMING_FULL_MAX_SIZE);
    latStr.reserve(5 + GPS_DECIMAL_POSITIONS);
    lngStr.reserve(5 + GPS_DECIMAL_POSITIONS);
    altStr.reserve(5);

    if (!outcomingFull.reserve(MAX_SIZE_OUTCOMING_LORA_REPORT)) {
        #if DEBUG_LEVEL >= 1
            Serial.println("Strings out of memory!");
        #endif
        blockingAlert(133, 50);
        while (1);
    }
}

/**
    composeLoRaPayload() se encarga de crear la string de carga útil de LoRa,
    a partir de los estados actuales de los sensores.
    Por ejemplo, si:
        DEVICE_ID = 20009
        cts = {0.50, 0.80, 0.65}
        rain = {1, 0, 1, -1}
        gas = 123.5187
        CAPACIDAD_COMBUSTIBLE = 150
        GPS.location.lat() = -34.574749127
        GPS.location.lng() = 58.43552318
        GPS.location.alt() = 15.62
    Entonces, esta función sobreescribe la String a retornar con:
        "<20009>current=0.65&raindrops=1&gas=6.21/12&lat=-34.57475&lng=58.43552&alt=15"
    @param cts Array con los valores de medición de corriente.
    @param rain Array con los valores de medición de lluvia.
    @param gas Número con coma flotante con la medición de combustible.
    @param &rtn Dirección de memoria de la String a componer.
*/
void composeLoRaPayload(float cts[], int rain[], float gas, String& rtn) {
    // Payload LoRA = vector de bytes transmitidos en forma FIFO.
    // | Dev ID | Corriente | Lluvia | Combustible/capacidad | Latitud | Longitud | Altitud |
    rtn = "<";
    #ifdef DEVICE_ID
        rtn += ((int)DEVICE_ID);
    #else
        rtn += "***";
    #endif
    rtn += ">";

    rtn += "current";
    rtn += "=";
    rtn += compressArray(cts, ARRAY_SIZE);

    rtn += "&";
    rtn += "raindrops";
    rtn += "=";
    #ifndef RAINDROP_MOCK
        rtn += compressArray(rain, ARRAY_SIZE);
    #else
        rtn += ((int)RAINDROP_MOCK);
    #endif

    rtn += "&";
    rtn += "gas";
    rtn += "=";
    #ifndef GAS_MOCK
        rtn += round2decimals(gas);
    #else
        rtn += round2decimals(GAS_MOCK);
    #endif

    rtn += "/";
    #ifdef CAPACIDAD_COMBUSTIBLE
        rtn += ((int)CAPACIDAD_COMBUSTIBLE);
    #else
        rtn += "***";
    #endif

    #ifndef GPS_MOCK
        if (GPS.location.isValid()) {
            latStr = String(GPS.location.lat(), GPS_DECIMAL_POSITIONS);
            lngStr = String(GPS.location.lng(), GPS_DECIMAL_POSITIONS);
            altStr = String((int)GPS.altitude.meters());
        } else {
            latStr = "***";
            lngStr = "***";
            altStr = "***";
        }
    #else
        latStr = String(GPS_MOCK[0], GPS_DECIMAL_POSITIONS);
        lngStr = String(GPS_MOCK[1], GPS_DECIMAL_POSITIONS);
        altStr = String((int)GPS_MOCK[2]);
    #endif

    rtn += "&";
    rtn += "lat";
    rtn += "=";
    rtn += latStr;

    rtn += "&";
    rtn += "lng";
    rtn += "=";
    rtn += lngStr;

    rtn += "&";
    rtn += "alt";
    rtn += "=";
    rtn += altStr;
}
