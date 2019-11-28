#include "N_panda.h"
#include "Arduino.h"
#include "N_defines.h"

TaskHandle_t Task4;

bool pandaOnline;

void pandaInit(){
    pinMode(PANDA_POWER_PIN,OUTPUT);
}

void Task4code(void * parameter) {                                                             
    digitalWrite(PANDA_POWER_PIN,HIGH);                                                         // electronically press the lattepanda power button
    digitalWrite(DEBUG_LED_PIN_1,HIGH);
    vTaskDelay(pdMS_TO_TICKS(5000));                                                            // wait 3000ms to initialize the boot process
    digitalWrite(PANDA_POWER_PIN,LOW);   
    digitalWrite(DEBUG_LED_PIN_1,LOW);      
    pandaOnline = true;                                             
    vTaskDelete(NULL);                                                                          // end this task
}

void pandaPowerOn(){
    xTaskCreatePinnedToCore(Task4code, "Task4", 10000, NULL, 1, &Task4, 0);                     // create task for powering up the lattepanda board
}

bool pandaIsOnline(){
    return pandaOnline;
}