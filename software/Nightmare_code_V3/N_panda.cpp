#include "N_panda.h"
#include "Arduino.h"
#include "N_defines.h"
#include "N_sensors.h"
#include "N_math.h"
#include "N_Octapod.h"
#include "N_structs.h"

TaskHandle_t Task4;
TaskHandle_t Task5;

bool pandaOnline;

void pandaInit(){
    Serial.begin(460800);
    pinMode(PANDA_POWER_PIN,OUTPUT);
    digitalWrite(DEBUG_LED_PIN_2,HIGH);
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

void Task5code( void * parameter) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(50);
    xLastWakeTime = xTaskGetTickCount();
    for (;;) {
        pandaWriteData();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void pandaPowerOn(){
    xTaskCreatePinnedToCore(Task4code, "Task4", 10000, NULL, 1, &Task4, 0);                     // create task for powering up the lattepanda board
    xTaskCreatePinnedToCore(Task5code, "Task5", 10000, NULL, 1, &Task4, 1);                     // create task for sending data to the ros node
}

bool pandaIsOnline(){
    return pandaOnline;
}

void pandaWriteData(){
    const struct Dleg * dlegs = Nightmare::dlegs;

    Serial.write(0xAA);                                                                          //start byte
    Serial.write(0xAA);                                                                          //start byte
    Serial.write((byte)(fmap(read_battery_voltage(),5.0,10.0,0,255)));                           //byte containing the voltage value
    Serial.write((byte)(fmap(read_battery_current(),0.0,15.0,0,255)));                           //byte containing the current value
    for(int i=0;i<8;i++){                                                                        //bytes containing the servos' raw angles
        Serial.write((byte)(fmap(dlegs[i].CX_ANGLE,-120,120,0,255)));
        Serial.write((byte)(fmap(dlegs[i].FM_ANGLE,-120,120,0,255)));
        Serial.write((byte)(fmap(dlegs[i].TB_ANGLE,-120,120,0,255)));
    }
}