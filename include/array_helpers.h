/**
    Header que contiene funciones relevantes para el accionar de los arrays.
    @file constants.h
    @author Franco Abosso
    @author Julio Donadello
    @version 1.0 29/03/2021
*/

/**
    compressArray() obtiene el promedio de los valores no nulos de un array de floats.
    Por ejemplo:
        int size = 2;
        float array[size] = {10.00, 11.00};
        compressArray(array, size);
    Devuelve: 10.50.
    @param array Arreglo de números que se quiere promediar.
    @param size Cantidad de elementos del array.
    @return Promedio de los valores no nulos del array.
*/
float compressArray(float array[], int size) {
    float average = 0;
    int zerosFound = 0;
    for (int i = 0; i < size; i++) {
        average += array[i];
        if (array[i] == 0.0) {
            zerosFound++;
        }
        #if DEBUG_LEVEL >= 5
            Serial.print(array[i]);
            Serial.print(" ");
        #endif
    }
    int nonZeroValues = size - zerosFound;
    average /= nonZeroValues;
    #if DEBUG_LEVEL >= 5
        Serial.print("Average of array: ");
        Serial.println(average);
    #endif

    return average;
}

/**
    compressArray() obtiene el resultado la "votación" comprendida dentro de un array de enteros, donde:
        - 0 es un voto negativo,
        - 1 es un voto positivo,
        - (-1) es un voto en blanco.
    Por ejemplo:
        int size = 8;
        float array[size] = {1, 0, 0, 0, -1, -1, -1, -1};
        compressArray(array, size);
    Devuelve: 0.
    @param array Arreglo de votos.
    @param size Cantidad de elementos del array.
    @return Resultado de la votación (0 ó 1). En caso de empate, defaultea 0. En caso de abstención total de votos, devuelve -1.
*/
int compressArray(int array[], int size) {
    int zerosFound = 0;
    int onesFound = 0;
    for (int i = 0; i < size; i++) {
        if (array[i] == 0) {
            zerosFound++;
        }
        if (array[i] == 1) {
            onesFound++;
        }
        #if DEBUG_LEVEL >= 5
            Serial.print(array[i]);
            Serial.print(" ");
        #endif
    }
    if (zerosFound == 0 && onesFound == 0) {
        return -1;
    }
    if (zerosFound > onesFound) {
        return 0;
    } else {
        return 1;
    }
}

/**
    cleanupArray() resetea todos los elementos de un array.
    @param array Arreglo de números que se quiere limpiar.
    @param size Cantidad de elementos del array.
*/
void cleanupArray(float array[], int size) {
    for (int i = 0; i < size; i++) {
        array[i] = 0.0;
    }
}

/**
    cleanupArray() resetea todos los elementos de un array.
    @param array Arreglo de números que se quiere limpiar.
    @param size Cantidad de elementos del array.
*/
void cleanupArray(int array[], int size) {
    for (int i = 0; i < size; i++) {
        array[i] = -1;
    }
}
