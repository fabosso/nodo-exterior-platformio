/**
    Header que contiene funcionalidades referidas a los sensores conectados.
    @file sensors.h
    @author Franco Abosso
    @author Julio Donadello
    @version 1.0 29/03/2021
*/

/**
    refreshAllSensors() se encarga de pedir el refresco de todos los sensores,
    poniendo cada uno de los flags del vector refreshRequested en true.
*/
void refreshAllSensors() {
    for (int i = 0; i < SENSORS_QTY; i++) {
        refreshRequested[i] = true;
    }
    #if DEBUG_LEVEL >= 2
        Serial.println("Refrescando sensores!");
    #endif
}

/**
    stopRefreshingAllSensors() se encarga de parar el refresco de todos los sensores,
    poniendo cada uno de los flags del vector refreshRequested en false.
*/
void stopRefreshingAllSensors() {
    for (int i = 0; i < SENSORS_QTY; i++) {
        refreshRequested[i] = false;
    }
    #if DEBUG_LEVEL >= 2
        Serial.println("Abandonando refrescos!");
    #endif
}

/**
    getNewCurrent() se encarga de agregar un nuevo valor en el array de medición de tensión.
    Luego de hacerlo, baja el flag correspondiente en refreshRequested.
*/
void getNewCurrent() {
    float newCurrent = 0.0;
    if (index < ARRAY_SIZE) {
        #ifndef CORRIENTE_MOCK
            newCurrent = eMon.Irms;
            currents[index] = newCurrent;
        #else
            newCurrent = CORRIENTE_MOCK + random(30) / 100.0;
            currents[index] = newCurrent;
        #endif
        #if DEBUG_LEVEL >= 3
            Serial.print("Nueva corriente: ");
            Serial.println(newCurrent);
        #endif
    }
    refreshRequested[0] = false;
}

/**
    getNewRaindrop() se encarga de agregar un nuevo valor en el array de lluvia,
    basándose en la medición actual del puerto analógico LLUVIA_PIN y en el umbral 
    LLUVIA_THRESHOLD_10BIT configurado.
    Luego de hacerlo, baja el flag correspondiente en refreshRequested.
*/
void getNewRaindrop() {
    #ifndef RAINDROP_MOCK
        if (index < ARRAY_SIZE) {
            #if LLUVIA_ACTIVO == HIGH
                if (analogRead(LLUVIA_PIN) >= LLUVIA_THRESHOLD_10BIT) {
                    raindrops[index] = 1;
                } else {
                    raindrops[index] = 0;
                }
            #else
                if (analogRead(LLUVIA_PIN) < LLUVIA_THRESHOLD_10BIT) {
                    raindrops[index] = 1;
                } else {
                    raindrops[index] = 0;
                }
            #endif
        }
    #endif
    refreshRequested[1] = false;
}

/**
    getNewGas() se encarga de obtener el nivel de combustible actual,
    luego de promediar la cantidad de tiempos de eco ultrasónico definidos por PING_SAMPLES,
    basándose en la relación entre:
    - la diferencia de tiempos entre el eco ultrasónico actual (timeUltrasonic) y el tiempo 
    medido en vacío (T_VACIO), y
    - la diferencia de tiempos entre el tiempo medido en vacío y el tiempo medido en lleno (T_LLENO).
    multiplicada por una constante, la capacidad del tanque en litros (CAPACIDAD_COMBUSTIBLE).
    Luego de hacerlo, baja el flag gasRequested correspondiente.
*/
void getNewGas() {
    float timeUltrasonic = 0.0;
    #ifndef GAS_MOCK
        timeUltrasonic = sonar.ping_median(PING_SAMPLES);
        if (timeUltrasonic < TIME_LLENO) {
            gas = float(CAPACIDAD_COMBUSTIBLE);
        } else if (timeUltrasonic > TIME_VACIO) {
            gas = 0.0;
        } else if (timeUltrasonic > TIME_VACIO) {
            gas = 0.0;
        } else {
            gas = float(CAPACIDAD_COMBUSTIBLE) * (TIME_VACIO - timeUltrasonic) / (TIME_VACIO - TIME_LLENO);
        }
    #else
        gas = GAS_MOCK;
    #endif
    #if DEBUG_LEVEL >= 4
        Serial.println(String(timeUltrasonic) + " us");
        Serial.println(String(gas) + " litros");
    #endif
    gasRequested = false;
}

/**
    getNewGPS() se encarga de leer la información proveniente del puerto
    serial correspondiente al GPS (ssGPS) y encodear esa información a un
    objeto que organiza esos datos (GPS).
*/
void getNewGPS() {
    #ifndef GPS_MOCK
        while (ssGPS.available() > 0) {
            GPS.encode(ssGPS.read());
        }
    #endif
}