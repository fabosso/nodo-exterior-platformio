/**
    Header que contiene funcionalidades referidas a los actuadores conectados.
    @file actuators.h
    @author Franco Abosso
    @author Julio Donadello
    @version 1.0 29/03/2021
*/

/**
    alertObserver() se encarga de consultar el estado de la variable resetAlert:
    si existe un pedido de iniciar la alerta, actualiza pitidosRestantes
    en base a totalPitidos (configurado por startAlert()) y baja el flag de pedido.
    Luego, si existen pedidos restantes, los realiza en base a tiempoPitido (configurado
    por startAlert()).
*/
void alertObserver() {
    if (resetAlert && pitidosRestantes == 0) {
        pitidosRestantes = totalPitidos;
        resetAlert = false;
    }
    if (pitidosRestantes > 0) {
        if(runEvery(tiempoPitido, 3)) {
            digitalWrite(BUZZER_PIN, !digitalRead(BUZZER_PIN));
            if (digitalRead(BUZZER_PIN) == BUZZER_INACTIVO) {
                pitidosRestantes--;
            }
        }
    } else {
        digitalWrite(BUZZER_PIN, BUZZER_INACTIVO);
    }
}

/**
    LoRaCmdObserver() se encarga de consultar el estado de la variable incomingPayload.
    Si la variable está vacía, sale de la función.
    Si el comando existe dentro del array de comandos conocidos, ejecuta cierta acción.
    Incluso si no existiera, limpia incomingPayload.
*/
void LoRaCmdObserver() {
    if (incomingPayload == "") {
        return;
    } else {
        #if DEBUG_LEVEL >= 1
            Serial.print("Quiero hacer esto >> ");
            Serial.println(incomingPayload);
        #endif
        if (incomingPayload == knownCommands[0]) {          // knownCommands[0]: startAlert
            startAlert(750, 10);
        } else {
            #if DEBUG_LEVEL >= 1
                Serial.println("Descartado por payload incorrecto!");
            #endif
        }
        incomingPayload = "";
    }
}
